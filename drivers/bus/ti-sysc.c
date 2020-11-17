// SPDX-License-Identifier: GPL-2.0
/*
 * ti-sysc.c - Texas Instruments sysc interconnect target driver
 */

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/iopoll.h>

#include <linux/platform_data/ti-sysc.h>

#include <dt-bindings/bus/ti-sysc.h>

#define MAX_MODULE_SOFTRESET_WAIT		10000

static const char * const reg_names[] = { "rev", "sysc", "syss", };

enum sysc_clocks {
	SYSC_FCK,
	SYSC_ICK,
	SYSC_OPTFCK0,
	SYSC_OPTFCK1,
	SYSC_OPTFCK2,
	SYSC_OPTFCK3,
	SYSC_OPTFCK4,
	SYSC_OPTFCK5,
	SYSC_OPTFCK6,
	SYSC_OPTFCK7,
	SYSC_MAX_CLOCKS,
};

static const char * const clock_names[SYSC_MAX_CLOCKS] = {
	"fck", "ick", "opt0", "opt1", "opt2", "opt3", "opt4",
	"opt5", "opt6", "opt7",
};

#define SYSC_IDLEMODE_MASK		3
#define SYSC_CLOCKACTIVITY_MASK		3

/**
 * struct sysc - TI sysc interconnect target module registers and capabilities
 * @dev: struct device pointer
 * @module_pa: physical address of the interconnect target module
 * @module_size: size of the interconnect target module
 * @module_va: virtual address of the interconnect target module
 * @offsets: register offsets from module base
 * @mdata: ti-sysc to hwmod translation data for a module
 * @clocks: clocks used by the interconnect target module
 * @clock_roles: clock role names for the found clocks
 * @nr_clocks: number of clocks used by the interconnect target module
 * @rsts: resets used by the interconnect target module
 * @legacy_mode: configured for legacy mode if set
 * @cap: interconnect target module capabilities
 * @cfg: interconnect target module configuration
 * @cookie: data used by legacy platform callbacks
 * @name: name if available
 * @revision: interconnect target module revision
 * @enabled: sysc runtime enabled status
 * @needs_resume: runtime resume needed on resume from suspend
 * @child_needs_resume: runtime resume needed for child on resume from suspend
 * @disable_on_idle: status flag used for disabling modules with resets
 * @idle_work: work structure used to perform delayed idle on a module
 * @clk_enable_quirk: module specific clock enable quirk
 * @clk_disable_quirk: module specific clock disable quirk
 * @reset_done_quirk: module specific reset done quirk
 * @module_enable_quirk: module specific enable quirk
 * @module_disable_quirk: module specific disable quirk
 */
struct sysc {
	struct device *dev;
	u64 module_pa;
	u32 module_size;
	void __iomem *module_va;
	int offsets[SYSC_MAX_REGS];
	struct ti_sysc_module_data *mdata;
	struct clk **clocks;
	const char **clock_roles;
	int nr_clocks;
	struct reset_control *rsts;
	const char *legacy_mode;
	const struct sysc_capabilities *cap;
	struct sysc_config cfg;
	struct ti_sysc_cookie cookie;
	const char *name;
	u32 revision;
	unsigned int enabled:1;
	unsigned int needs_resume:1;
	unsigned int child_needs_resume:1;
	struct delayed_work idle_work;
	void (*clk_enable_quirk)(struct sysc *sysc);
	void (*clk_disable_quirk)(struct sysc *sysc);
	void (*reset_done_quirk)(struct sysc *sysc);
	void (*module_enable_quirk)(struct sysc *sysc);
	void (*module_disable_quirk)(struct sysc *sysc);
};

static void sysc_parse_dts_quirks(struct sysc *ddata, struct device_node *np,
				  bool is_child);

static void sysc_write(struct sysc *ddata, int offset, u32 value)
{
	if (ddata->cfg.quirks & SYSC_QUIRK_16BIT) {
		writew_relaxed(value & 0xffff, ddata->module_va + offset);

		/* Only i2c revision has LO and HI register with stride of 4 */
		if (ddata->offsets[SYSC_REVISION] >= 0 &&
		    offset == ddata->offsets[SYSC_REVISION]) {
			u16 hi = value >> 16;

			writew_relaxed(hi, ddata->module_va + offset + 4);
		}

		return;
	}

	writel_relaxed(value, ddata->module_va + offset);
}

static u32 sysc_read(struct sysc *ddata, int offset)
{
	if (ddata->cfg.quirks & SYSC_QUIRK_16BIT) {
		u32 val;

		val = readw_relaxed(ddata->module_va + offset);

		/* Only i2c revision has LO and HI register with stride of 4 */
		if (ddata->offsets[SYSC_REVISION] >= 0 &&
		    offset == ddata->offsets[SYSC_REVISION]) {
			u16 tmp = readw_relaxed(ddata->module_va + offset + 4);

			val |= tmp << 16;
		}

		return val;
	}

	return readl_relaxed(ddata->module_va + offset);
}

static bool sysc_opt_clks_needed(struct sysc *ddata)
{
	return !!(ddata->cfg.quirks & SYSC_QUIRK_OPT_CLKS_NEEDED);
}

static u32 sysc_read_revision(struct sysc *ddata)
{
	int offset = ddata->offsets[SYSC_REVISION];

	if (offset < 0)
		return 0;

	return sysc_read(ddata, offset);
}

static u32 sysc_read_sysconfig(struct sysc *ddata)
{
	int offset = ddata->offsets[SYSC_SYSCONFIG];

	if (offset < 0)
		return 0;

	return sysc_read(ddata, offset);
}

static u32 sysc_read_sysstatus(struct sysc *ddata)
{
	int offset = ddata->offsets[SYSC_SYSSTATUS];

	if (offset < 0)
		return 0;

	return sysc_read(ddata, offset);
}

static int sysc_add_named_clock_from_child(struct sysc *ddata,
					   const char *name,
					   const char *optfck_name)
{
	struct device_node *np = ddata->dev->of_node;
	struct device_node *child;
	struct clk_lookup *cl;
	struct clk *clock;
	const char *n;

	if (name)
		n = name;
	else
		n = optfck_name;

	/* Does the clock alias already exist? */
	clock = of_clk_get_by_name(np, n);
	if (!IS_ERR(clock)) {
		clk_put(clock);

		return 0;
	}

	child = of_get_next_available_child(np, NULL);
	if (!child)
		return -ENODEV;

	clock = devm_get_clk_from_child(ddata->dev, child, name);
	if (IS_ERR(clock))
		return PTR_ERR(clock);

	/*
	 * Use clkdev_add() instead of clkdev_alloc() to avoid the MAX_DEV_ID
	 * limit for clk_get(). If cl ever needs to be freed, it should be done
	 * with clkdev_drop().
	 */
	cl = kcalloc(1, sizeof(*cl), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;

	cl->con_id = n;
	cl->dev_id = dev_name(ddata->dev);
	cl->clk = clock;
	clkdev_add(cl);

	clk_put(clock);

	return 0;
}

static int sysc_init_ext_opt_clock(struct sysc *ddata, const char *name)
{
	const char *optfck_name;
	int error, index;

	if (ddata->nr_clocks < SYSC_OPTFCK0)
		index = SYSC_OPTFCK0;
	else
		index = ddata->nr_clocks;

	if (name)
		optfck_name = name;
	else
		optfck_name = clock_names[index];

	error = sysc_add_named_clock_from_child(ddata, name, optfck_name);
	if (error)
		return error;

	ddata->clock_roles[index] = optfck_name;
	ddata->nr_clocks++;

	return 0;
}

static int sysc_get_one_clock(struct sysc *ddata, const char *name)
{
	int error, i, index = -ENODEV;

	if (!strncmp(clock_names[SYSC_FCK], name, 3))
		index = SYSC_FCK;
	else if (!strncmp(clock_names[SYSC_ICK], name, 3))
		index = SYSC_ICK;

	if (index < 0) {
		for (i = SYSC_OPTFCK0; i < SYSC_MAX_CLOCKS; i++) {
			if (!ddata->clocks[i]) {
				index = i;
				break;
			}
		}
	}

	if (index < 0) {
		dev_err(ddata->dev, "clock %s not added\n", name);
		return index;
	}

	ddata->clocks[index] = devm_clk_get(ddata->dev, name);
	if (IS_ERR(ddata->clocks[index])) {
		dev_err(ddata->dev, "clock get error for %s: %li\n",
			name, PTR_ERR(ddata->clocks[index]));

		return PTR_ERR(ddata->clocks[index]);
	}

	error = clk_prepare(ddata->clocks[index]);
	if (error) {
		dev_err(ddata->dev, "clock prepare error for %s: %i\n",
			name, error);

		return error;
	}

	return 0;
}

static int sysc_get_clocks(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	struct property *prop;
	const char *name;
	int nr_fck = 0, nr_ick = 0, i, error = 0;

	ddata->clock_roles = devm_kcalloc(ddata->dev,
					  SYSC_MAX_CLOCKS,
					  sizeof(*ddata->clock_roles),
					  GFP_KERNEL);
	if (!ddata->clock_roles)
		return -ENOMEM;

	of_property_for_each_string(np, "clock-names", prop, name) {
		if (!strncmp(clock_names[SYSC_FCK], name, 3))
			nr_fck++;
		if (!strncmp(clock_names[SYSC_ICK], name, 3))
			nr_ick++;
		ddata->clock_roles[ddata->nr_clocks] = name;
		ddata->nr_clocks++;
	}

	if (ddata->nr_clocks < 1)
		return 0;

	if ((ddata->cfg.quirks & SYSC_QUIRK_EXT_OPT_CLOCK)) {
		error = sysc_init_ext_opt_clock(ddata, NULL);
		if (error)
			return error;
	}

	if (ddata->nr_clocks > SYSC_MAX_CLOCKS) {
		dev_err(ddata->dev, "too many clocks for %pOF\n", np);

		return -EINVAL;
	}

	if (nr_fck > 1 || nr_ick > 1) {
		dev_err(ddata->dev, "max one fck and ick for %pOF\n", np);

		return -EINVAL;
	}

	/* Always add a slot for main clocks fck and ick even if unused */
	if (!nr_fck)
		ddata->nr_clocks++;
	if (!nr_ick)
		ddata->nr_clocks++;

	ddata->clocks = devm_kcalloc(ddata->dev,
				     ddata->nr_clocks, sizeof(*ddata->clocks),
				     GFP_KERNEL);
	if (!ddata->clocks)
		return -ENOMEM;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		const char *name = ddata->clock_roles[i];

		if (!name)
			continue;

		error = sysc_get_one_clock(ddata, name);
		if (error)
			return error;
	}

	return 0;
}

static int sysc_enable_main_clocks(struct sysc *ddata)
{
	struct clk *clock;
	int i, error;

	if (!ddata->clocks)
		return 0;

	for (i = 0; i < SYSC_OPTFCK0; i++) {
		clock = ddata->clocks[i];

		/* Main clocks may not have ick */
		if (IS_ERR_OR_NULL(clock))
			continue;

		error = clk_enable(clock);
		if (error)
			goto err_disable;
	}

	return 0;

err_disable:
	for (i--; i >= 0; i--) {
		clock = ddata->clocks[i];

		/* Main clocks may not have ick */
		if (IS_ERR_OR_NULL(clock))
			continue;

		clk_disable(clock);
	}

	return error;
}

static void sysc_disable_main_clocks(struct sysc *ddata)
{
	struct clk *clock;
	int i;

	if (!ddata->clocks)
		return;

	for (i = 0; i < SYSC_OPTFCK0; i++) {
		clock = ddata->clocks[i];
		if (IS_ERR_OR_NULL(clock))
			continue;

		clk_disable(clock);
	}
}

static int sysc_enable_opt_clocks(struct sysc *ddata)
{
	struct clk *clock;
	int i, error;

	if (!ddata->clocks || ddata->nr_clocks < SYSC_OPTFCK0 + 1)
		return 0;

	for (i = SYSC_OPTFCK0; i < SYSC_MAX_CLOCKS; i++) {
		clock = ddata->clocks[i];

		/* Assume no holes for opt clocks */
		if (IS_ERR_OR_NULL(clock))
			return 0;

		error = clk_enable(clock);
		if (error)
			goto err_disable;
	}

	return 0;

err_disable:
	for (i--; i >= 0; i--) {
		clock = ddata->clocks[i];
		if (IS_ERR_OR_NULL(clock))
			continue;

		clk_disable(clock);
	}

	return error;
}

static void sysc_disable_opt_clocks(struct sysc *ddata)
{
	struct clk *clock;
	int i;

	if (!ddata->clocks || ddata->nr_clocks < SYSC_OPTFCK0 + 1)
		return;

	for (i = SYSC_OPTFCK0; i < SYSC_MAX_CLOCKS; i++) {
		clock = ddata->clocks[i];

		/* Assume no holes for opt clocks */
		if (IS_ERR_OR_NULL(clock))
			return;

		clk_disable(clock);
	}
}

static void sysc_clkdm_deny_idle(struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata;

	if (ddata->legacy_mode || (ddata->cfg.quirks & SYSC_QUIRK_CLKDM_NOAUTO))
		return;

	pdata = dev_get_platdata(ddata->dev);
	if (pdata && pdata->clkdm_deny_idle)
		pdata->clkdm_deny_idle(ddata->dev, &ddata->cookie);
}

static void sysc_clkdm_allow_idle(struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata;

	if (ddata->legacy_mode || (ddata->cfg.quirks & SYSC_QUIRK_CLKDM_NOAUTO))
		return;

	pdata = dev_get_platdata(ddata->dev);
	if (pdata && pdata->clkdm_allow_idle)
		pdata->clkdm_allow_idle(ddata->dev, &ddata->cookie);
}

/**
 * sysc_init_resets - init rstctrl reset line if configured
 * @ddata: device driver data
 *
 * See sysc_rstctrl_reset_deassert().
 */
static int sysc_init_resets(struct sysc *ddata)
{
	ddata->rsts =
		devm_reset_control_get_optional_shared(ddata->dev, "rstctrl");
	if (IS_ERR(ddata->rsts))
		return PTR_ERR(ddata->rsts);

	return 0;
}

/**
 * sysc_parse_and_check_child_range - parses module IO region from ranges
 * @ddata: device driver data
 *
 * In general we only need rev, syss, and sysc registers and not the whole
 * module range. But we do want the offsets for these registers from the
 * module base. This allows us to check them against the legacy hwmod
 * platform data. Let's also check the ranges are configured properly.
 */
static int sysc_parse_and_check_child_range(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	const __be32 *ranges;
	u32 nr_addr, nr_size;
	int len, error;

	ranges = of_get_property(np, "ranges", &len);
	if (!ranges) {
		dev_err(ddata->dev, "missing ranges for %pOF\n", np);

		return -ENOENT;
	}

	len /= sizeof(*ranges);

	if (len < 3) {
		dev_err(ddata->dev, "incomplete ranges for %pOF\n", np);

		return -EINVAL;
	}

	error = of_property_read_u32(np, "#address-cells", &nr_addr);
	if (error)
		return -ENOENT;

	error = of_property_read_u32(np, "#size-cells", &nr_size);
	if (error)
		return -ENOENT;

	if (nr_addr != 1 || nr_size != 1) {
		dev_err(ddata->dev, "invalid ranges for %pOF\n", np);

		return -EINVAL;
	}

	ranges++;
	ddata->module_pa = of_translate_address(np, ranges++);
	ddata->module_size = be32_to_cpup(ranges);

	return 0;
}

static struct device_node *stdout_path;

static void sysc_init_stdout_path(struct sysc *ddata)
{
	struct device_node *np = NULL;
	const char *uart;

	if (IS_ERR(stdout_path))
		return;

	if (stdout_path)
		return;

	np = of_find_node_by_path("/chosen");
	if (!np)
		goto err;

	uart = of_get_property(np, "stdout-path", NULL);
	if (!uart)
		goto err;

	np = of_find_node_by_path(uart);
	if (!np)
		goto err;

	stdout_path = np;

	return;

err:
	stdout_path = ERR_PTR(-ENODEV);
}

static void sysc_check_quirk_stdout(struct sysc *ddata,
				    struct device_node *np)
{
	sysc_init_stdout_path(ddata);
	if (np != stdout_path)
		return;

	ddata->cfg.quirks |= SYSC_QUIRK_NO_IDLE_ON_INIT |
				SYSC_QUIRK_NO_RESET_ON_INIT;
}

/**
 * sysc_check_one_child - check child configuration
 * @ddata: device driver data
 * @np: child device node
 *
 * Let's avoid messy situations where we have new interconnect target
 * node but children have "ti,hwmods". These belong to the interconnect
 * target node and are managed by this driver.
 */
static void sysc_check_one_child(struct sysc *ddata,
				 struct device_node *np)
{
	const char *name;

	name = of_get_property(np, "ti,hwmods", NULL);
	if (name)
		dev_warn(ddata->dev, "really a child ti,hwmods property?");

	sysc_check_quirk_stdout(ddata, np);
	sysc_parse_dts_quirks(ddata, np, true);
}

static void sysc_check_children(struct sysc *ddata)
{
	struct device_node *child;

	for_each_child_of_node(ddata->dev->of_node, child)
		sysc_check_one_child(ddata, child);
}

/*
 * So far only I2C uses 16-bit read access with clockactivity with revision
 * in two registers with stride of 4. We can detect this based on the rev
 * register size to configure things far enough to be able to properly read
 * the revision register.
 */
static void sysc_check_quirk_16bit(struct sysc *ddata, struct resource *res)
{
	if (resource_size(res) == 8)
		ddata->cfg.quirks |= SYSC_QUIRK_16BIT | SYSC_QUIRK_USE_CLOCKACT;
}

/**
 * sysc_parse_one - parses the interconnect target module registers
 * @ddata: device driver data
 * @reg: register to parse
 */
static int sysc_parse_one(struct sysc *ddata, enum sysc_registers reg)
{
	struct resource *res;
	const char *name;

	switch (reg) {
	case SYSC_REVISION:
	case SYSC_SYSCONFIG:
	case SYSC_SYSSTATUS:
		name = reg_names[reg];
		break;
	default:
		return -EINVAL;
	}

	res = platform_get_resource_byname(to_platform_device(ddata->dev),
					   IORESOURCE_MEM, name);
	if (!res) {
		ddata->offsets[reg] = -ENODEV;

		return 0;
	}

	ddata->offsets[reg] = res->start - ddata->module_pa;
	if (reg == SYSC_REVISION)
		sysc_check_quirk_16bit(ddata, res);

	return 0;
}

static int sysc_parse_registers(struct sysc *ddata)
{
	int i, error;

	for (i = 0; i < SYSC_MAX_REGS; i++) {
		error = sysc_parse_one(ddata, i);
		if (error)
			return error;
	}

	return 0;
}

/**
 * sysc_check_registers - check for misconfigured register overlaps
 * @ddata: device driver data
 */
static int sysc_check_registers(struct sysc *ddata)
{
	int i, j, nr_regs = 0, nr_matches = 0;

	for (i = 0; i < SYSC_MAX_REGS; i++) {
		if (ddata->offsets[i] < 0)
			continue;

		if (ddata->offsets[i] > (ddata->module_size - 4)) {
			dev_err(ddata->dev, "register outside module range");

				return -EINVAL;
		}

		for (j = 0; j < SYSC_MAX_REGS; j++) {
			if (ddata->offsets[j] < 0)
				continue;

			if (ddata->offsets[i] == ddata->offsets[j])
				nr_matches++;
		}
		nr_regs++;
	}

	if (nr_matches > nr_regs) {
		dev_err(ddata->dev, "overlapping registers: (%i/%i)",
			nr_regs, nr_matches);

		return -EINVAL;
	}

	return 0;
}

/**
 * syc_ioremap - ioremap register space for the interconnect target module
 * @ddata: device driver data
 *
 * Note that the interconnect target module registers can be anywhere
 * within the interconnect target module range. For example, SGX has
 * them at offset 0x1fc00 in the 32MB module address space. And cpsw
 * has them at offset 0x1200 in the CPSW_WR child. Usually the
 * the interconnect target module registers are at the beginning of
 * the module range though.
 */
static int sysc_ioremap(struct sysc *ddata)
{
	int size;

	if (ddata->offsets[SYSC_REVISION] < 0 &&
	    ddata->offsets[SYSC_SYSCONFIG] < 0 &&
	    ddata->offsets[SYSC_SYSSTATUS] < 0) {
		size = ddata->module_size;
	} else {
		size = max3(ddata->offsets[SYSC_REVISION],
			    ddata->offsets[SYSC_SYSCONFIG],
			    ddata->offsets[SYSC_SYSSTATUS]);

		if (size < SZ_1K)
			size = SZ_1K;

		if ((size + sizeof(u32)) > ddata->module_size)
			size = ddata->module_size;
	}

	ddata->module_va = devm_ioremap(ddata->dev,
					ddata->module_pa,
					size + sizeof(u32));
	if (!ddata->module_va)
		return -EIO;

	return 0;
}

/**
 * sysc_map_and_check_registers - ioremap and check device registers
 * @ddata: device driver data
 */
static int sysc_map_and_check_registers(struct sysc *ddata)
{
	int error;

	error = sysc_parse_and_check_child_range(ddata);
	if (error)
		return error;

	sysc_check_children(ddata);

	error = sysc_parse_registers(ddata);
	if (error)
		return error;

	error = sysc_ioremap(ddata);
	if (error)
		return error;

	error = sysc_check_registers(ddata);
	if (error)
		return error;

	return 0;
}

/**
 * sysc_show_rev - read and show interconnect target module revision
 * @bufp: buffer to print the information to
 * @ddata: device driver data
 */
static int sysc_show_rev(char *bufp, struct sysc *ddata)
{
	int len;

	if (ddata->offsets[SYSC_REVISION] < 0)
		return sprintf(bufp, ":NA");

	len = sprintf(bufp, ":%08x", ddata->revision);

	return len;
}

static int sysc_show_reg(struct sysc *ddata,
			 char *bufp, enum sysc_registers reg)
{
	if (ddata->offsets[reg] < 0)
		return sprintf(bufp, ":NA");

	return sprintf(bufp, ":%x", ddata->offsets[reg]);
}

static int sysc_show_name(char *bufp, struct sysc *ddata)
{
	if (!ddata->name)
		return 0;

	return sprintf(bufp, ":%s", ddata->name);
}

/**
 * sysc_show_registers - show information about interconnect target module
 * @ddata: device driver data
 */
static void sysc_show_registers(struct sysc *ddata)
{
	char buf[128];
	char *bufp = buf;
	int i;

	for (i = 0; i < SYSC_MAX_REGS; i++)
		bufp += sysc_show_reg(ddata, bufp, i);

	bufp += sysc_show_rev(bufp, ddata);
	bufp += sysc_show_name(bufp, ddata);

	dev_dbg(ddata->dev, "%llx:%x%s\n",
		ddata->module_pa, ddata->module_size,
		buf);
}

#define SYSC_IDLE_MASK	(SYSC_NR_IDLEMODES - 1)
#define SYSC_CLOCACT_ICK	2

/* Caller needs to manage sysc_clkdm_deny_idle() and sysc_clkdm_allow_idle() */
static int sysc_enable_module(struct device *dev)
{
	struct sysc *ddata;
	const struct sysc_regbits *regbits;
	u32 reg, idlemodes, best_mode;

	ddata = dev_get_drvdata(dev);
	if (ddata->offsets[SYSC_SYSCONFIG] == -ENODEV)
		return 0;

	regbits = ddata->cap->regbits;
	reg = sysc_read(ddata, ddata->offsets[SYSC_SYSCONFIG]);

	/* Set CLOCKACTIVITY, we only use it for ick */
	if (regbits->clkact_shift >= 0 &&
	    (ddata->cfg.quirks & SYSC_QUIRK_USE_CLOCKACT ||
	     ddata->cfg.sysc_val & BIT(regbits->clkact_shift)))
		reg |= SYSC_CLOCACT_ICK << regbits->clkact_shift;

	/* Set SIDLE mode */
	idlemodes = ddata->cfg.sidlemodes;
	if (!idlemodes || regbits->sidle_shift < 0)
		goto set_midle;

	if (ddata->cfg.quirks & (SYSC_QUIRK_SWSUP_SIDLE |
				 SYSC_QUIRK_SWSUP_SIDLE_ACT)) {
		best_mode = SYSC_IDLE_NO;
	} else {
		best_mode = fls(ddata->cfg.sidlemodes) - 1;
		if (best_mode > SYSC_IDLE_MASK) {
			dev_err(dev, "%s: invalid sidlemode\n", __func__);
			return -EINVAL;
		}

		/* Set WAKEUP */
		if (regbits->enwkup_shift >= 0 &&
		    ddata->cfg.sysc_val & BIT(regbits->enwkup_shift))
			reg |= BIT(regbits->enwkup_shift);
	}

	reg &= ~(SYSC_IDLE_MASK << regbits->sidle_shift);
	reg |= best_mode << regbits->sidle_shift;
	sysc_write(ddata, ddata->offsets[SYSC_SYSCONFIG], reg);

set_midle:
	/* Set MIDLE mode */
	idlemodes = ddata->cfg.midlemodes;
	if (!idlemodes || regbits->midle_shift < 0)
		goto set_autoidle;

	best_mode = fls(ddata->cfg.midlemodes) - 1;
	if (best_mode > SYSC_IDLE_MASK) {
		dev_err(dev, "%s: invalid midlemode\n", __func__);
		return -EINVAL;
	}

	if (ddata->cfg.quirks & SYSC_QUIRK_SWSUP_MSTANDBY)
		best_mode = SYSC_IDLE_NO;

	reg &= ~(SYSC_IDLE_MASK << regbits->midle_shift);
	reg |= best_mode << regbits->midle_shift;
	sysc_write(ddata, ddata->offsets[SYSC_SYSCONFIG], reg);

set_autoidle:
	/* Autoidle bit must enabled separately if available */
	if (regbits->autoidle_shift >= 0 &&
	    ddata->cfg.sysc_val & BIT(regbits->autoidle_shift)) {
		reg |= 1 << regbits->autoidle_shift;
		sysc_write(ddata, ddata->offsets[SYSC_SYSCONFIG], reg);
	}

	if (ddata->module_enable_quirk)
		ddata->module_enable_quirk(ddata);

	return 0;
}

static int sysc_best_idle_mode(u32 idlemodes, u32 *best_mode)
{
	if (idlemodes & BIT(SYSC_IDLE_SMART_WKUP))
		*best_mode = SYSC_IDLE_SMART_WKUP;
	else if (idlemodes & BIT(SYSC_IDLE_SMART))
		*best_mode = SYSC_IDLE_SMART;
	else if (idlemodes & BIT(SYSC_IDLE_FORCE))
		*best_mode = SYSC_IDLE_FORCE;
	else
		return -EINVAL;

	return 0;
}

/* Caller needs to manage sysc_clkdm_deny_idle() and sysc_clkdm_allow_idle() */
static int sysc_disable_module(struct device *dev)
{
	struct sysc *ddata;
	const struct sysc_regbits *regbits;
	u32 reg, idlemodes, best_mode;
	int ret;

	ddata = dev_get_drvdata(dev);
	if (ddata->offsets[SYSC_SYSCONFIG] == -ENODEV)
		return 0;

	if (ddata->module_disable_quirk)
		ddata->module_disable_quirk(ddata);

	regbits = ddata->cap->regbits;
	reg = sysc_read(ddata, ddata->offsets[SYSC_SYSCONFIG]);

	/* Set MIDLE mode */
	idlemodes = ddata->cfg.midlemodes;
	if (!idlemodes || regbits->midle_shift < 0)
		goto set_sidle;

	ret = sysc_best_idle_mode(idlemodes, &best_mode);
	if (ret) {
		dev_err(dev, "%s: invalid midlemode\n", __func__);
		return ret;
	}

	if (ddata->cfg.quirks & (SYSC_QUIRK_SWSUP_MSTANDBY) ||
	    ddata->cfg.quirks & (SYSC_QUIRK_FORCE_MSTANDBY))
		best_mode = SYSC_IDLE_FORCE;

	reg &= ~(SYSC_IDLE_MASK << regbits->midle_shift);
	reg |= best_mode << regbits->midle_shift;
	sysc_write(ddata, ddata->offsets[SYSC_SYSCONFIG], reg);

set_sidle:
	/* Set SIDLE mode */
	idlemodes = ddata->cfg.sidlemodes;
	if (!idlemodes || regbits->sidle_shift < 0)
		return 0;

	if (ddata->cfg.quirks & SYSC_QUIRK_SWSUP_SIDLE) {
		best_mode = SYSC_IDLE_FORCE;
	} else {
		ret = sysc_best_idle_mode(idlemodes, &best_mode);
		if (ret) {
			dev_err(dev, "%s: invalid sidlemode\n", __func__);
			return ret;
		}
	}

	reg &= ~(SYSC_IDLE_MASK << regbits->sidle_shift);
	reg |= best_mode << regbits->sidle_shift;
	if (regbits->autoidle_shift >= 0 &&
	    ddata->cfg.sysc_val & BIT(regbits->autoidle_shift))
		reg |= 1 << regbits->autoidle_shift;
	sysc_write(ddata, ddata->offsets[SYSC_SYSCONFIG], reg);

	return 0;
}

static int __maybe_unused sysc_runtime_suspend_legacy(struct device *dev,
						      struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata;
	int error;

	pdata = dev_get_platdata(ddata->dev);
	if (!pdata)
		return 0;

	if (!pdata->idle_module)
		return -ENODEV;

	error = pdata->idle_module(dev, &ddata->cookie);
	if (error)
		dev_err(dev, "%s: could not idle: %i\n",
			__func__, error);

	reset_control_assert(ddata->rsts);

	return 0;
}

static int __maybe_unused sysc_runtime_resume_legacy(struct device *dev,
						     struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata;
	int error;

	reset_control_deassert(ddata->rsts);

	pdata = dev_get_platdata(ddata->dev);
	if (!pdata)
		return 0;

	if (!pdata->enable_module)
		return -ENODEV;

	error = pdata->enable_module(dev, &ddata->cookie);
	if (error)
		dev_err(dev, "%s: could not enable: %i\n",
			__func__, error);

	return 0;
}

static int __maybe_unused sysc_runtime_suspend(struct device *dev)
{
	struct sysc *ddata;
	int error = 0;

	ddata = dev_get_drvdata(dev);

	if (!ddata->enabled)
		return 0;

	sysc_clkdm_deny_idle(ddata);

	if (ddata->legacy_mode) {
		error = sysc_runtime_suspend_legacy(dev, ddata);
		if (error)
			goto err_allow_idle;
	} else {
		error = sysc_disable_module(dev);
		if (error)
			goto err_allow_idle;
	}

	sysc_disable_main_clocks(ddata);

	if (sysc_opt_clks_needed(ddata))
		sysc_disable_opt_clocks(ddata);

	ddata->enabled = false;

err_allow_idle:
	reset_control_assert(ddata->rsts);

	sysc_clkdm_allow_idle(ddata);

	return error;
}

static int __maybe_unused sysc_runtime_resume(struct device *dev)
{
	struct sysc *ddata;
	int error = 0;

	ddata = dev_get_drvdata(dev);

	if (ddata->enabled)
		return 0;


	sysc_clkdm_deny_idle(ddata);

	reset_control_deassert(ddata->rsts);

	if (sysc_opt_clks_needed(ddata)) {
		error = sysc_enable_opt_clocks(ddata);
		if (error)
			goto err_allow_idle;
	}

	error = sysc_enable_main_clocks(ddata);
	if (error)
		goto err_opt_clocks;

	if (ddata->legacy_mode) {
		error = sysc_runtime_resume_legacy(dev, ddata);
		if (error)
			goto err_main_clocks;
	} else {
		error = sysc_enable_module(dev);
		if (error)
			goto err_main_clocks;
	}

	ddata->enabled = true;

	sysc_clkdm_allow_idle(ddata);

	return 0;

err_main_clocks:
	sysc_disable_main_clocks(ddata);
err_opt_clocks:
	if (sysc_opt_clks_needed(ddata))
		sysc_disable_opt_clocks(ddata);
err_allow_idle:
	sysc_clkdm_allow_idle(ddata);

	return error;
}

static int __maybe_unused sysc_noirq_suspend(struct device *dev)
{
	struct sysc *ddata;

	ddata = dev_get_drvdata(dev);

	if (ddata->cfg.quirks & SYSC_QUIRK_LEGACY_IDLE)
		return 0;

	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused sysc_noirq_resume(struct device *dev)
{
	struct sysc *ddata;

	ddata = dev_get_drvdata(dev);

	if (ddata->cfg.quirks & SYSC_QUIRK_LEGACY_IDLE)
		return 0;

	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops sysc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(sysc_noirq_suspend, sysc_noirq_resume)
	SET_RUNTIME_PM_OPS(sysc_runtime_suspend,
			   sysc_runtime_resume,
			   NULL)
};

/* Module revision register based quirks */
struct sysc_revision_quirk {
	const char *name;
	u32 base;
	int rev_offset;
	int sysc_offset;
	int syss_offset;
	u32 revision;
	u32 revision_mask;
	u32 quirks;
};

#define SYSC_QUIRK(optname, optbase, optrev, optsysc, optsyss,		\
		   optrev_val, optrevmask, optquirkmask)		\
	{								\
		.name = (optname),					\
		.base = (optbase),					\
		.rev_offset = (optrev),					\
		.sysc_offset = (optsysc),				\
		.syss_offset = (optsyss),				\
		.revision = (optrev_val),				\
		.revision_mask = (optrevmask),				\
		.quirks = (optquirkmask),				\
	}

static const struct sysc_revision_quirk sysc_revision_quirks[] = {
	/* These drivers need to be fixed to not use pm_runtime_irq_safe() */
	SYSC_QUIRK("gpio", 0, 0, 0x10, 0x114, 0x50600801, 0xffff00ff,
		   SYSC_QUIRK_LEGACY_IDLE | SYSC_QUIRK_OPT_CLKS_IN_RESET),
	SYSC_QUIRK("mmu", 0, 0, 0x10, 0x14, 0x00000020, 0xffffffff,
		   SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("mmu", 0, 0, 0x10, 0x14, 0x00000030, 0xffffffff,
		   SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("sham", 0, 0x100, 0x110, 0x114, 0x40000c03, 0xffffffff,
		   SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("smartreflex", 0, -1, 0x24, -1, 0x00000000, 0xffffffff,
		   SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("smartreflex", 0, -1, 0x38, -1, 0x00000000, 0xffffffff,
		   SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("timer", 0, 0, 0x10, 0x14, 0x00000015, 0xffffffff,
		   0),
	/* Some timers on omap4 and later */
	SYSC_QUIRK("timer", 0, 0, 0x10, -1, 0x50002100, 0xffffffff,
		   0),
	SYSC_QUIRK("timer", 0, 0, 0x10, -1, 0x4fff1301, 0xffff00ff,
		   0),
	SYSC_QUIRK("uart", 0, 0x50, 0x54, 0x58, 0x00000046, 0xffffffff,
		   SYSC_QUIRK_SWSUP_SIDLE | SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("uart", 0, 0x50, 0x54, 0x58, 0x00000052, 0xffffffff,
		   SYSC_QUIRK_SWSUP_SIDLE | SYSC_QUIRK_LEGACY_IDLE),
	/* Uarts on omap4 and later */
	SYSC_QUIRK("uart", 0, 0x50, 0x54, 0x58, 0x50411e03, 0xffff00ff,
		   SYSC_QUIRK_SWSUP_SIDLE_ACT | SYSC_QUIRK_LEGACY_IDLE),
	SYSC_QUIRK("uart", 0, 0x50, 0x54, 0x58, 0x47422e03, 0xffffffff,
		   SYSC_QUIRK_SWSUP_SIDLE_ACT | SYSC_QUIRK_LEGACY_IDLE),

	/* Quirks that need to be set based on the module address */
	SYSC_QUIRK("mcpdm", 0x40132000, 0, 0x10, -1, 0x50000800, 0xffffffff,
		   SYSC_QUIRK_EXT_OPT_CLOCK | SYSC_QUIRK_NO_RESET_ON_INIT |
		   SYSC_QUIRK_SWSUP_SIDLE),

	/* Quirks that need to be set based on detected module */
	SYSC_QUIRK("aess", 0, 0, 0x10, -1, 0x40000000, 0xffffffff,
		   SYSC_MODULE_QUIRK_AESS),
	SYSC_QUIRK("dcan", 0x48480000, 0x20, -1, -1, 0xa3170504, 0xffffffff,
		   SYSC_QUIRK_CLKDM_NOAUTO),
	SYSC_QUIRK("dwc3", 0x48880000, 0, 0x10, -1, 0x500a0200, 0xffffffff,
		   SYSC_QUIRK_CLKDM_NOAUTO),
	SYSC_QUIRK("dwc3", 0x488c0000, 0, 0x10, -1, 0x500a0200, 0xffffffff,
		   SYSC_QUIRK_CLKDM_NOAUTO),
	SYSC_QUIRK("hdq1w", 0, 0, 0x14, 0x18, 0x00000006, 0xffffffff,
		   SYSC_MODULE_QUIRK_HDQ1W),
	SYSC_QUIRK("hdq1w", 0, 0, 0x14, 0x18, 0x0000000a, 0xffffffff,
		   SYSC_MODULE_QUIRK_HDQ1W),
	SYSC_QUIRK("i2c", 0, 0, 0x20, 0x10, 0x00000036, 0x000000ff,
		   SYSC_MODULE_QUIRK_I2C),
	SYSC_QUIRK("i2c", 0, 0, 0x20, 0x10, 0x0000003c, 0x000000ff,
		   SYSC_MODULE_QUIRK_I2C),
	SYSC_QUIRK("i2c", 0, 0, 0x20, 0x10, 0x00000040, 0x000000ff,
		   SYSC_MODULE_QUIRK_I2C),
	SYSC_QUIRK("i2c", 0, 0, 0x10, 0x90, 0x5040000a, 0xfffff0f0,
		   SYSC_MODULE_QUIRK_I2C),
	SYSC_QUIRK("gpu", 0x50000000, 0x14, -1, -1, 0x00010201, 0xffffffff, 0),
	SYSC_QUIRK("gpu", 0x50000000, 0xfe00, 0xfe10, -1, 0x40000000 , 0xffffffff,
		   SYSC_MODULE_QUIRK_SGX),
	SYSC_QUIRK("usb_otg_hs", 0, 0x400, 0x404, 0x408, 0x00000050,
		   0xffffffff, SYSC_QUIRK_SWSUP_SIDLE | SYSC_QUIRK_SWSUP_MSTANDBY),
	SYSC_QUIRK("usb_otg_hs", 0, 0, 0x10, -1, 0x4ea2080d, 0xffffffff,
		   SYSC_QUIRK_SWSUP_SIDLE | SYSC_QUIRK_SWSUP_MSTANDBY),
	SYSC_QUIRK("wdt", 0, 0, 0x10, 0x14, 0x502a0500, 0xfffff0f0,
		   SYSC_MODULE_QUIRK_WDT),
	/* Watchdog on am3 and am4 */
	SYSC_QUIRK("wdt", 0x44e35000, 0, 0x10, 0x14, 0x502a0500, 0xfffff0f0,
		   SYSC_MODULE_QUIRK_WDT | SYSC_QUIRK_SWSUP_SIDLE),

#ifdef DEBUG
	SYSC_QUIRK("adc", 0, 0, 0x10, -1, 0x47300001, 0xffffffff, 0),
	SYSC_QUIRK("atl", 0, 0, -1, -1, 0x0a070100, 0xffffffff, 0),
	SYSC_QUIRK("cm", 0, 0, -1, -1, 0x40000301, 0xffffffff, 0),
	SYSC_QUIRK("control", 0, 0, 0x10, -1, 0x40000900, 0xffffffff, 0),
	SYSC_QUIRK("cpgmac", 0, 0x1200, 0x1208, 0x1204, 0x4edb1902,
		   0xffff00f0, 0),
	SYSC_QUIRK("dcan", 0, 0x20, -1, -1, 0xa3170504, 0xffffffff, 0),
	SYSC_QUIRK("dcan", 0, 0x20, -1, -1, 0x4edb1902, 0xffffffff, 0),
	SYSC_QUIRK("dmic", 0, 0, 0x10, -1, 0x50010000, 0xffffffff, 0),
	SYSC_QUIRK("dwc3", 0, 0, 0x10, -1, 0x500a0200, 0xffffffff, 0),
	SYSC_QUIRK("d2d", 0x4a0b6000, 0, 0x10, 0x14, 0x00000010, 0xffffffff, 0),
	SYSC_QUIRK("d2d", 0x4a0cd000, 0, 0x10, 0x14, 0x00000010, 0xffffffff, 0),
	SYSC_QUIRK("epwmss", 0, 0, 0x4, -1, 0x47400001, 0xffffffff, 0),
	SYSC_QUIRK("gpu", 0, 0x1fc00, 0x1fc10, -1, 0, 0, 0),
	SYSC_QUIRK("gpu", 0, 0xfe00, 0xfe10, -1, 0x40000000 , 0xffffffff, 0),
	SYSC_QUIRK("hsi", 0, 0, 0x10, 0x14, 0x50043101, 0xffffffff, 0),
	SYSC_QUIRK("iss", 0, 0, 0x10, -1, 0x40000101, 0xffffffff, 0),
	SYSC_QUIRK("lcdc", 0, 0, 0x54, -1, 0x4f201000, 0xffffffff, 0),
	SYSC_QUIRK("mcasp", 0, 0, 0x4, -1, 0x44306302, 0xffffffff, 0),
	SYSC_QUIRK("mcasp", 0, 0, 0x4, -1, 0x44307b02, 0xffffffff, 0),
	SYSC_QUIRK("mcbsp", 0, -1, 0x8c, -1, 0, 0, 0),
	SYSC_QUIRK("mcspi", 0, 0, 0x10, -1, 0x40300a0b, 0xffff00ff, 0),
	SYSC_QUIRK("mcspi", 0, 0, 0x110, 0x114, 0x40300a0b, 0xffffffff, 0),
	SYSC_QUIRK("mailbox", 0, 0, 0x10, -1, 0x00000400, 0xffffffff, 0),
	SYSC_QUIRK("m3", 0, 0, -1, -1, 0x5f580105, 0x0fff0f00, 0),
	SYSC_QUIRK("ocp2scp", 0, 0, 0x10, 0x14, 0x50060005, 0xfffffff0, 0),
	SYSC_QUIRK("ocp2scp", 0, 0, -1, -1, 0x50060007, 0xffffffff, 0),
	SYSC_QUIRK("padconf", 0, 0, 0x10, -1, 0x4fff0800, 0xffffffff, 0),
	SYSC_QUIRK("padconf", 0, 0, -1, -1, 0x40001100, 0xffffffff, 0),
	SYSC_QUIRK("prcm", 0, 0, -1, -1, 0x40000100, 0xffffffff, 0),
	SYSC_QUIRK("prcm", 0, 0, -1, -1, 0x00004102, 0xffffffff, 0),
	SYSC_QUIRK("prcm", 0, 0, -1, -1, 0x40000400, 0xffffffff, 0),
	SYSC_QUIRK("scm", 0, 0, 0x10, -1, 0x40000900, 0xffffffff, 0),
	SYSC_QUIRK("scm", 0, 0, -1, -1, 0x4e8b0100, 0xffffffff, 0),
	SYSC_QUIRK("scm", 0, 0, -1, -1, 0x4f000100, 0xffffffff, 0),
	SYSC_QUIRK("scm", 0, 0, -1, -1, 0x40000900, 0xffffffff, 0),
	SYSC_QUIRK("scrm", 0, 0, -1, -1, 0x00000010, 0xffffffff, 0),
	SYSC_QUIRK("sdio", 0, 0, 0x10, -1, 0x40202301, 0xffff0ff0, 0),
	SYSC_QUIRK("sdio", 0, 0x2fc, 0x110, 0x114, 0x31010000, 0xffffffff, 0),
	SYSC_QUIRK("sdma", 0, 0, 0x2c, 0x28, 0x00010900, 0xffffffff, 0),
	SYSC_QUIRK("slimbus", 0, 0, 0x10, -1, 0x40000902, 0xffffffff, 0),
	SYSC_QUIRK("slimbus", 0, 0, 0x10, -1, 0x40002903, 0xffffffff, 0),
	SYSC_QUIRK("spinlock", 0, 0, 0x10, -1, 0x50020000, 0xffffffff, 0),
	SYSC_QUIRK("rng", 0, 0x1fe0, 0x1fe4, -1, 0x00000020, 0xffffffff, 0),
	SYSC_QUIRK("rtc", 0, 0x74, 0x78, -1, 0x4eb01908, 0xffff00f0, 0),
	SYSC_QUIRK("timer32k", 0, 0, 0x4, -1, 0x00000060, 0xffffffff, 0),
	SYSC_QUIRK("usbhstll", 0, 0, 0x10, 0x14, 0x00000004, 0xffffffff, 0),
	SYSC_QUIRK("usbhstll", 0, 0, 0x10, 0x14, 0x00000008, 0xffffffff, 0),
	SYSC_QUIRK("usb_host_hs", 0, 0, 0x10, 0x14, 0x50700100, 0xffffffff, 0),
	SYSC_QUIRK("usb_host_hs", 0, 0, 0x10, -1, 0x50700101, 0xffffffff, 0),
	SYSC_QUIRK("vfpe", 0, 0, 0x104, -1, 0x4d001200, 0xffffffff, 0),
#endif
};

/*
 * Early quirks based on module base and register offsets only that are
 * needed before the module revision can be read
 */
static void sysc_init_early_quirks(struct sysc *ddata)
{
	const struct sysc_revision_quirk *q;
	int i;

	for (i = 0; i < ARRAY_SIZE(sysc_revision_quirks); i++) {
		q = &sysc_revision_quirks[i];

		if (!q->base)
			continue;

		if (q->base != ddata->module_pa)
			continue;

		if (q->rev_offset >= 0 &&
		    q->rev_offset != ddata->offsets[SYSC_REVISION])
			continue;

		if (q->sysc_offset >= 0 &&
		    q->sysc_offset != ddata->offsets[SYSC_SYSCONFIG])
			continue;

		if (q->syss_offset >= 0 &&
		    q->syss_offset != ddata->offsets[SYSC_SYSSTATUS])
			continue;

		ddata->name = q->name;
		ddata->cfg.quirks |= q->quirks;
	}
}

/* Quirks that also consider the revision register value */
static void sysc_init_revision_quirks(struct sysc *ddata)
{
	const struct sysc_revision_quirk *q;
	int i;

	for (i = 0; i < ARRAY_SIZE(sysc_revision_quirks); i++) {
		q = &sysc_revision_quirks[i];

		if (q->base && q->base != ddata->module_pa)
			continue;

		if (q->rev_offset >= 0 &&
		    q->rev_offset != ddata->offsets[SYSC_REVISION])
			continue;

		if (q->sysc_offset >= 0 &&
		    q->sysc_offset != ddata->offsets[SYSC_SYSCONFIG])
			continue;

		if (q->syss_offset >= 0 &&
		    q->syss_offset != ddata->offsets[SYSC_SYSSTATUS])
			continue;

		if (q->revision == ddata->revision ||
		    (q->revision & q->revision_mask) ==
		    (ddata->revision & q->revision_mask)) {
			ddata->name = q->name;
			ddata->cfg.quirks |= q->quirks;
		}
	}
}

/* 1-wire needs module's internal clocks enabled for reset */
static void sysc_clk_enable_quirk_hdq1w(struct sysc *ddata)
{
	int offset = 0x0c;	/* HDQ_CTRL_STATUS */
	u16 val;

	val = sysc_read(ddata, offset);
	val |= BIT(5);
	sysc_write(ddata, offset, val);
}

/* AESS (Audio Engine SubSystem) needs autogating set after enable */
static void sysc_module_enable_quirk_aess(struct sysc *ddata)
{
	int offset = 0x7c;	/* AESS_AUTO_GATING_ENABLE */

	sysc_write(ddata, offset, 1);
}

/* I2C needs extra enable bit toggling for reset */
static void sysc_clk_quirk_i2c(struct sysc *ddata, bool enable)
{
	int offset;
	u16 val;

	/* I2C_CON, omap2/3 is different from omap4 and later */
	if ((ddata->revision & 0xffffff00) == 0x001f0000)
		offset = 0x24;
	else
		offset = 0xa4;

	/* I2C_EN */
	val = sysc_read(ddata, offset);
	if (enable)
		val |= BIT(15);
	else
		val &= ~BIT(15);
	sysc_write(ddata, offset, val);
}

static void sysc_clk_enable_quirk_i2c(struct sysc *ddata)
{
	sysc_clk_quirk_i2c(ddata, true);
}

static void sysc_clk_disable_quirk_i2c(struct sysc *ddata)
{
	sysc_clk_quirk_i2c(ddata, false);
}

/* 36xx SGX needs a quirk for to bypass OCP IPG interrupt logic */
static void sysc_module_enable_quirk_sgx(struct sysc *ddata)
{
	int offset = 0xff08;	/* OCP_DEBUG_CONFIG */
	u32 val = BIT(31);	/* THALIA_INT_BYPASS */

	sysc_write(ddata, offset, val);
}

/* Watchdog timer needs a disable sequence after reset */
static void sysc_reset_done_quirk_wdt(struct sysc *ddata)
{
	int wps, spr, error;
	u32 val;

	wps = 0x34;
	spr = 0x48;

	sysc_write(ddata, spr, 0xaaaa);
	error = readl_poll_timeout(ddata->module_va + wps, val,
				   !(val & 0x10), 100,
				   MAX_MODULE_SOFTRESET_WAIT);
	if (error)
		dev_warn(ddata->dev, "wdt disable step1 failed\n");

	sysc_write(ddata, spr, 0x5555);
	error = readl_poll_timeout(ddata->module_va + wps, val,
				   !(val & 0x10), 100,
				   MAX_MODULE_SOFTRESET_WAIT);
	if (error)
		dev_warn(ddata->dev, "wdt disable step2 failed\n");
}

static void sysc_init_module_quirks(struct sysc *ddata)
{
	if (ddata->legacy_mode || !ddata->name)
		return;

	if (ddata->cfg.quirks & SYSC_MODULE_QUIRK_HDQ1W) {
		ddata->clk_enable_quirk = sysc_clk_enable_quirk_hdq1w;

		return;
	}

	if (ddata->cfg.quirks & SYSC_MODULE_QUIRK_I2C) {
		ddata->clk_enable_quirk = sysc_clk_enable_quirk_i2c;
		ddata->clk_disable_quirk = sysc_clk_disable_quirk_i2c;

		return;
	}

	if (ddata->cfg.quirks & SYSC_MODULE_QUIRK_AESS)
		ddata->module_enable_quirk = sysc_module_enable_quirk_aess;

	if (ddata->cfg.quirks & SYSC_MODULE_QUIRK_SGX)
		ddata->module_enable_quirk = sysc_module_enable_quirk_sgx;

	if (ddata->cfg.quirks & SYSC_MODULE_QUIRK_WDT) {
		ddata->reset_done_quirk = sysc_reset_done_quirk_wdt;
		ddata->module_disable_quirk = sysc_reset_done_quirk_wdt;
	}
}

static int sysc_clockdomain_init(struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata = dev_get_platdata(ddata->dev);
	struct clk *fck = NULL, *ick = NULL;
	int error;

	if (!pdata || !pdata->init_clockdomain)
		return 0;

	switch (ddata->nr_clocks) {
	case 2:
		ick = ddata->clocks[SYSC_ICK];
		/* fallthrough */
	case 1:
		fck = ddata->clocks[SYSC_FCK];
		break;
	case 0:
		return 0;
	}

	error = pdata->init_clockdomain(ddata->dev, fck, ick, &ddata->cookie);
	if (!error || error == -ENODEV)
		return 0;

	return error;
}

/*
 * Note that pdata->init_module() typically does a reset first. After
 * pdata->init_module() is done, PM runtime can be used for the interconnect
 * target module.
 */
static int sysc_legacy_init(struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata = dev_get_platdata(ddata->dev);
	int error;

	if (!pdata || !pdata->init_module)
		return 0;

	error = pdata->init_module(ddata->dev, ddata->mdata, &ddata->cookie);
	if (error == -EEXIST)
		error = 0;

	return error;
}

/**
 * sysc_rstctrl_reset_deassert - deassert rstctrl reset
 * @ddata: device driver data
 * @reset: reset before deassert
 *
 * A module can have both OCP softreset control and external rstctrl.
 * If more complicated rstctrl resets are needed, please handle these
 * directly from the child device driver and map only the module reset
 * for the parent interconnect target module device.
 *
 * Automatic reset of the module on init can be skipped with the
 * "ti,no-reset-on-init" device tree property.
 */
static int sysc_rstctrl_reset_deassert(struct sysc *ddata, bool reset)
{
	int error;

	if (!ddata->rsts)
		return 0;

	if (reset) {
		error = reset_control_assert(ddata->rsts);
		if (error)
			return error;
	}

	reset_control_deassert(ddata->rsts);

	return 0;
}

/*
 * Note that the caller must ensure the interconnect target module is enabled
 * before calling reset. Otherwise reset will not complete.
 */
static int sysc_reset(struct sysc *ddata)
{
	int sysc_offset, syss_offset, sysc_val, rstval, error = 0;
	u32 sysc_mask, syss_done;

	sysc_offset = ddata->offsets[SYSC_SYSCONFIG];
	syss_offset = ddata->offsets[SYSC_SYSSTATUS];

	if (ddata->legacy_mode || sysc_offset < 0 ||
	    ddata->cap->regbits->srst_shift < 0 ||
	    ddata->cfg.quirks & SYSC_QUIRK_NO_RESET_ON_INIT)
		return 0;

	sysc_mask = BIT(ddata->cap->regbits->srst_shift);

	if (ddata->cfg.quirks & SYSS_QUIRK_RESETDONE_INVERTED)
		syss_done = 0;
	else
		syss_done = ddata->cfg.syss_mask;

	if (ddata->clk_disable_quirk)
		ddata->clk_disable_quirk(ddata);

	sysc_val = sysc_read_sysconfig(ddata);
	sysc_val |= sysc_mask;
	sysc_write(ddata, sysc_offset, sysc_val);

	if (ddata->cfg.srst_udelay)
		usleep_range(ddata->cfg.srst_udelay,
			     ddata->cfg.srst_udelay * 2);

	if (ddata->clk_enable_quirk)
		ddata->clk_enable_quirk(ddata);

	/* Poll on reset status */
	if (syss_offset >= 0) {
		error = readx_poll_timeout(sysc_read_sysstatus, ddata, rstval,
					   (rstval & ddata->cfg.syss_mask) ==
					   syss_done,
					   100, MAX_MODULE_SOFTRESET_WAIT);

	} else if (ddata->cfg.quirks & SYSC_QUIRK_RESET_STATUS) {
		error = readx_poll_timeout(sysc_read_sysconfig, ddata, rstval,
					   !(rstval & sysc_mask),
					   100, MAX_MODULE_SOFTRESET_WAIT);
	}

	if (ddata->reset_done_quirk)
		ddata->reset_done_quirk(ddata);

	return error;
}

/*
 * At this point the module is configured enough to read the revision but
 * module may not be completely configured yet to use PM runtime. Enable
 * all clocks directly during init to configure the quirks needed for PM
 * runtime based on the revision register.
 */
static int sysc_init_module(struct sysc *ddata)
{
	int error = 0;
	bool manage_clocks = true;

	error = sysc_rstctrl_reset_deassert(ddata, false);
	if (error)
		return error;

	if (ddata->cfg.quirks &
	    (SYSC_QUIRK_NO_IDLE | SYSC_QUIRK_NO_IDLE_ON_INIT))
		manage_clocks = false;

	error = sysc_clockdomain_init(ddata);
	if (error)
		return error;

	sysc_clkdm_deny_idle(ddata);

	/*
	 * Always enable clocks. The bootloader may or may not have enabled
	 * the related clocks.
	 */
	error = sysc_enable_opt_clocks(ddata);
	if (error)
		return error;

	error = sysc_enable_main_clocks(ddata);
	if (error)
		goto err_opt_clocks;

	if (!(ddata->cfg.quirks & SYSC_QUIRK_NO_RESET_ON_INIT)) {
		error = sysc_rstctrl_reset_deassert(ddata, true);
		if (error)
			goto err_main_clocks;
	}

	ddata->revision = sysc_read_revision(ddata);
	sysc_init_revision_quirks(ddata);
	sysc_init_module_quirks(ddata);

	if (ddata->legacy_mode) {
		error = sysc_legacy_init(ddata);
		if (error)
			goto err_main_clocks;
	}

	if (!ddata->legacy_mode) {
		error = sysc_enable_module(ddata->dev);
		if (error)
			goto err_main_clocks;
	}

	error = sysc_reset(ddata);
	if (error)
		dev_err(ddata->dev, "Reset failed with %d\n", error);

	if (!ddata->legacy_mode && manage_clocks)
		sysc_disable_module(ddata->dev);

err_main_clocks:
	if (manage_clocks)
		sysc_disable_main_clocks(ddata);
err_opt_clocks:
	/* No re-enable of clockdomain autoidle to prevent module autoidle */
	if (manage_clocks) {
		sysc_disable_opt_clocks(ddata);
		sysc_clkdm_allow_idle(ddata);
	}

	return error;
}

static int sysc_init_sysc_mask(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	int error;
	u32 val;

	error = of_property_read_u32(np, "ti,sysc-mask", &val);
	if (error)
		return 0;

	ddata->cfg.sysc_val = val & ddata->cap->sysc_mask;

	return 0;
}

static int sysc_init_idlemode(struct sysc *ddata, u8 *idlemodes,
			      const char *name)
{
	struct device_node *np = ddata->dev->of_node;
	struct property *prop;
	const __be32 *p;
	u32 val;

	of_property_for_each_u32(np, name, prop, p, val) {
		if (val >= SYSC_NR_IDLEMODES) {
			dev_err(ddata->dev, "invalid idlemode: %i\n", val);
			return -EINVAL;
		}
		*idlemodes |=  (1 << val);
	}

	return 0;
}

static int sysc_init_idlemodes(struct sysc *ddata)
{
	int error;

	error = sysc_init_idlemode(ddata, &ddata->cfg.midlemodes,
				   "ti,sysc-midle");
	if (error)
		return error;

	error = sysc_init_idlemode(ddata, &ddata->cfg.sidlemodes,
				   "ti,sysc-sidle");
	if (error)
		return error;

	return 0;
}

/*
 * Only some devices on omap4 and later have SYSCONFIG reset done
 * bit. We can detect this if there is no SYSSTATUS at all, or the
 * SYSTATUS bit 0 is not used. Note that some SYSSTATUS registers
 * have multiple bits for the child devices like OHCI and EHCI.
 * Depends on SYSC being parsed first.
 */
static int sysc_init_syss_mask(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	int error;
	u32 val;

	error = of_property_read_u32(np, "ti,syss-mask", &val);
	if (error) {
		if ((ddata->cap->type == TI_SYSC_OMAP4 ||
		     ddata->cap->type == TI_SYSC_OMAP4_TIMER) &&
		    (ddata->cfg.sysc_val & SYSC_OMAP4_SOFTRESET))
			ddata->cfg.quirks |= SYSC_QUIRK_RESET_STATUS;

		return 0;
	}

	if (!(val & 1) && (ddata->cfg.sysc_val & SYSC_OMAP4_SOFTRESET))
		ddata->cfg.quirks |= SYSC_QUIRK_RESET_STATUS;

	ddata->cfg.syss_mask = val;

	return 0;
}

/*
 * Many child device drivers need to have fck and opt clocks available
 * to get the clock rate for device internal configuration etc.
 */
static int sysc_child_add_named_clock(struct sysc *ddata,
				      struct device *child,
				      const char *name)
{
	struct clk *clk;
	struct clk_lookup *l;
	int error = 0;

	if (!name)
		return 0;

	clk = clk_get(child, name);
	if (!IS_ERR(clk)) {
		clk_put(clk);

		return -EEXIST;
	}

	clk = clk_get(ddata->dev, name);
	if (IS_ERR(clk))
		return -ENODEV;

	l = clkdev_create(clk, name, dev_name(child));
	if (!l)
		error = -ENOMEM;

	clk_put(clk);

	return error;
}

static int sysc_child_add_clocks(struct sysc *ddata,
				 struct device *child)
{
	int i, error;

	for (i = 0; i < ddata->nr_clocks; i++) {
		error = sysc_child_add_named_clock(ddata,
						   child,
						   ddata->clock_roles[i]);
		if (error && error != -EEXIST) {
			dev_err(ddata->dev, "could not add child clock %s: %i\n",
				ddata->clock_roles[i], error);

			return error;
		}
	}

	return 0;
}

static struct device_type sysc_device_type = {
};

static struct sysc *sysc_child_to_parent(struct device *dev)
{
	struct device *parent = dev->parent;

	if (!parent || parent->type != &sysc_device_type)
		return NULL;

	return dev_get_drvdata(parent);
}

static int __maybe_unused sysc_child_runtime_suspend(struct device *dev)
{
	struct sysc *ddata;
	int error;

	ddata = sysc_child_to_parent(dev);

	error = pm_generic_runtime_suspend(dev);
	if (error)
		return error;

	if (!ddata->enabled)
		return 0;

	return sysc_runtime_suspend(ddata->dev);
}

static int __maybe_unused sysc_child_runtime_resume(struct device *dev)
{
	struct sysc *ddata;
	int error;

	ddata = sysc_child_to_parent(dev);

	if (!ddata->enabled) {
		error = sysc_runtime_resume(ddata->dev);
		if (error < 0)
			dev_err(ddata->dev,
				"%s error: %i\n", __func__, error);
	}

	return pm_generic_runtime_resume(dev);
}

#ifdef CONFIG_PM_SLEEP
static int sysc_child_suspend_noirq(struct device *dev)
{
	struct sysc *ddata;
	int error;

	ddata = sysc_child_to_parent(dev);

	dev_dbg(ddata->dev, "%s %s\n", __func__,
		ddata->name ? ddata->name : "");

	error = pm_generic_suspend_noirq(dev);
	if (error) {
		dev_err(dev, "%s error at %i: %i\n",
			__func__, __LINE__, error);

		return error;
	}

	if (!pm_runtime_status_suspended(dev)) {
		error = pm_generic_runtime_suspend(dev);
		if (error) {
			dev_dbg(dev, "%s busy at %i: %i\n",
				__func__, __LINE__, error);

			return 0;
		}

		error = sysc_runtime_suspend(ddata->dev);
		if (error) {
			dev_err(dev, "%s error at %i: %i\n",
				__func__, __LINE__, error);

			return error;
		}

		ddata->child_needs_resume = true;
	}

	return 0;
}

static int sysc_child_resume_noirq(struct device *dev)
{
	struct sysc *ddata;
	int error;

	ddata = sysc_child_to_parent(dev);

	dev_dbg(ddata->dev, "%s %s\n", __func__,
		ddata->name ? ddata->name : "");

	if (ddata->child_needs_resume) {
		ddata->child_needs_resume = false;

		error = sysc_runtime_resume(ddata->dev);
		if (error)
			dev_err(ddata->dev,
				"%s runtime resume error: %i\n",
				__func__, error);

		error = pm_generic_runtime_resume(dev);
		if (error)
			dev_err(ddata->dev,
				"%s generic runtime resume: %i\n",
				__func__, error);
	}

	return pm_generic_resume_noirq(dev);
}
#endif

static struct dev_pm_domain sysc_child_pm_domain = {
	.ops = {
		SET_RUNTIME_PM_OPS(sysc_child_runtime_suspend,
				   sysc_child_runtime_resume,
				   NULL)
		USE_PLATFORM_PM_SLEEP_OPS
		SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(sysc_child_suspend_noirq,
					      sysc_child_resume_noirq)
	}
};

/**
 * sysc_legacy_idle_quirk - handle children in omap_device compatible way
 * @ddata: device driver data
 * @child: child device driver
 *
 * Allow idle for child devices as done with _od_runtime_suspend().
 * Otherwise many child devices will not idle because of the permanent
 * parent usecount set in pm_runtime_irq_safe().
 *
 * Note that the long term solution is to just modify the child device
 * drivers to not set pm_runtime_irq_safe() and then this can be just
 * dropped.
 */
static void sysc_legacy_idle_quirk(struct sysc *ddata, struct device *child)
{
	if (ddata->cfg.quirks & SYSC_QUIRK_LEGACY_IDLE)
		dev_pm_domain_set(child, &sysc_child_pm_domain);
}

static int sysc_notifier_call(struct notifier_block *nb,
			      unsigned long event, void *device)
{
	struct device *dev = device;
	struct sysc *ddata;
	int error;

	ddata = sysc_child_to_parent(dev);
	if (!ddata)
		return NOTIFY_DONE;

	switch (event) {
	case BUS_NOTIFY_ADD_DEVICE:
		error = sysc_child_add_clocks(ddata, dev);
		if (error)
			return error;
		sysc_legacy_idle_quirk(ddata, dev);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block sysc_nb = {
	.notifier_call = sysc_notifier_call,
};

/* Device tree configured quirks */
struct sysc_dts_quirk {
	const char *name;
	u32 mask;
};

static const struct sysc_dts_quirk sysc_dts_quirks[] = {
	{ .name = "ti,no-idle-on-init",
	  .mask = SYSC_QUIRK_NO_IDLE_ON_INIT, },
	{ .name = "ti,no-reset-on-init",
	  .mask = SYSC_QUIRK_NO_RESET_ON_INIT, },
	{ .name = "ti,no-idle",
	  .mask = SYSC_QUIRK_NO_IDLE, },
};

static void sysc_parse_dts_quirks(struct sysc *ddata, struct device_node *np,
				  bool is_child)
{
	const struct property *prop;
	int i, len;

	for (i = 0; i < ARRAY_SIZE(sysc_dts_quirks); i++) {
		const char *name = sysc_dts_quirks[i].name;

		prop = of_get_property(np, name, &len);
		if (!prop)
			continue;

		ddata->cfg.quirks |= sysc_dts_quirks[i].mask;
		if (is_child) {
			dev_warn(ddata->dev,
				 "dts flag should be at module level for %s\n",
				 name);
		}
	}
}

static int sysc_init_dts_quirks(struct sysc *ddata)
{
	struct device_node *np = ddata->dev->of_node;
	int error;
	u32 val;

	ddata->legacy_mode = of_get_property(np, "ti,hwmods", NULL);

	sysc_parse_dts_quirks(ddata, np, false);
	error = of_property_read_u32(np, "ti,sysc-delay-us", &val);
	if (!error) {
		if (val > 255) {
			dev_warn(ddata->dev, "bad ti,sysc-delay-us: %i\n",
				 val);
		}

		ddata->cfg.srst_udelay = (u8)val;
	}

	return 0;
}

static void sysc_unprepare(struct sysc *ddata)
{
	int i;

	if (!ddata->clocks)
		return;

	for (i = 0; i < SYSC_MAX_CLOCKS; i++) {
		if (!IS_ERR_OR_NULL(ddata->clocks[i]))
			clk_unprepare(ddata->clocks[i]);
	}
}

/*
 * Common sysc register bits found on omap2, also known as type1
 */
static const struct sysc_regbits sysc_regbits_omap2 = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = 12,
	.sidle_shift = 3,
	.clkact_shift = 8,
	.emufree_shift = 5,
	.enwkup_shift = 2,
	.srst_shift = 1,
	.autoidle_shift = 0,
};

static const struct sysc_capabilities sysc_omap2 = {
	.type = TI_SYSC_OMAP2,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY | SYSC_OMAP2_EMUFREE |
		     SYSC_OMAP2_ENAWAKEUP | SYSC_OMAP2_SOFTRESET |
		     SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap2,
};

/* All omap2 and 3 timers, and timers 1, 2 & 10 on omap 4 and 5 */
static const struct sysc_capabilities sysc_omap2_timer = {
	.type = TI_SYSC_OMAP2_TIMER,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY | SYSC_OMAP2_EMUFREE |
		     SYSC_OMAP2_ENAWAKEUP | SYSC_OMAP2_SOFTRESET |
		     SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap2,
	.mod_quirks = SYSC_QUIRK_USE_CLOCKACT,
};

/*
 * SHAM2 (SHA1/MD5) sysc found on omap3, a variant of sysc_regbits_omap2
 * with different sidle position
 */
static const struct sysc_regbits sysc_regbits_omap3_sham = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 4,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = 1,
	.autoidle_shift = 0,
	.emufree_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap3_sham = {
	.type = TI_SYSC_OMAP3_SHAM,
	.sysc_mask = SYSC_OMAP2_SOFTRESET | SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap3_sham,
};

/*
 * AES register bits found on omap3 and later, a variant of
 * sysc_regbits_omap2 with different sidle position
 */
static const struct sysc_regbits sysc_regbits_omap3_aes = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 6,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = 1,
	.autoidle_shift = 0,
	.emufree_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap3_aes = {
	.type = TI_SYSC_OMAP3_AES,
	.sysc_mask = SYSC_OMAP2_SOFTRESET | SYSC_OMAP2_AUTOIDLE,
	.regbits = &sysc_regbits_omap3_aes,
};

/*
 * Common sysc register bits found on omap4, also known as type2
 */
static const struct sysc_regbits sysc_regbits_omap4 = {
	.dmadisable_shift = 16,
	.midle_shift = 4,
	.sidle_shift = 2,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.emufree_shift = 1,
	.srst_shift = 0,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4 = {
	.type = TI_SYSC_OMAP4,
	.sysc_mask = SYSC_OMAP4_DMADISABLE | SYSC_OMAP4_FREEEMU |
		     SYSC_OMAP4_SOFTRESET,
	.regbits = &sysc_regbits_omap4,
};

static const struct sysc_capabilities sysc_omap4_timer = {
	.type = TI_SYSC_OMAP4_TIMER,
	.sysc_mask = SYSC_OMAP4_DMADISABLE | SYSC_OMAP4_FREEEMU |
		     SYSC_OMAP4_SOFTRESET,
	.regbits = &sysc_regbits_omap4,
};

/*
 * Common sysc register bits found on omap4, also known as type3
 */
static const struct sysc_regbits sysc_regbits_omap4_simple = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = 2,
	.sidle_shift = 0,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_simple = {
	.type = TI_SYSC_OMAP4_SIMPLE,
	.regbits = &sysc_regbits_omap4_simple,
};

/*
 * SmartReflex sysc found on omap34xx
 */
static const struct sysc_regbits sysc_regbits_omap34xx_sr = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = -ENODEV,
	.clkact_shift = 20,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_34xx_sr = {
	.type = TI_SYSC_OMAP34XX_SR,
	.sysc_mask = SYSC_OMAP2_CLOCKACTIVITY,
	.regbits = &sysc_regbits_omap34xx_sr,
	.mod_quirks = SYSC_QUIRK_USE_CLOCKACT | SYSC_QUIRK_UNCACHED |
		      SYSC_QUIRK_LEGACY_IDLE,
};

/*
 * SmartReflex sysc found on omap36xx and later
 */
static const struct sysc_regbits sysc_regbits_omap36xx_sr = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 24,
	.clkact_shift = -ENODEV,
	.enwkup_shift = 26,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_36xx_sr = {
	.type = TI_SYSC_OMAP36XX_SR,
	.sysc_mask = SYSC_OMAP3_SR_ENAWAKEUP,
	.regbits = &sysc_regbits_omap36xx_sr,
	.mod_quirks = SYSC_QUIRK_UNCACHED | SYSC_QUIRK_LEGACY_IDLE,
};

static const struct sysc_capabilities sysc_omap4_sr = {
	.type = TI_SYSC_OMAP4_SR,
	.regbits = &sysc_regbits_omap36xx_sr,
	.mod_quirks = SYSC_QUIRK_LEGACY_IDLE,
};

/*
 * McASP register bits found on omap4 and later
 */
static const struct sysc_regbits sysc_regbits_omap4_mcasp = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 0,
	.clkact_shift = -ENODEV,
	.enwkup_shift = -ENODEV,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_mcasp = {
	.type = TI_SYSC_OMAP4_MCASP,
	.regbits = &sysc_regbits_omap4_mcasp,
	.mod_quirks = SYSC_QUIRK_OPT_CLKS_NEEDED,
};

/*
 * McASP found on dra7 and later
 */
static const struct sysc_capabilities sysc_dra7_mcasp = {
	.type = TI_SYSC_OMAP4_SIMPLE,
	.regbits = &sysc_regbits_omap4_simple,
	.mod_quirks = SYSC_QUIRK_OPT_CLKS_NEEDED,
};

/*
 * FS USB host found on omap4 and later
 */
static const struct sysc_regbits sysc_regbits_omap4_usb_host_fs = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = 24,
	.clkact_shift = -ENODEV,
	.enwkup_shift = 26,
	.srst_shift = -ENODEV,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_omap4_usb_host_fs = {
	.type = TI_SYSC_OMAP4_USB_HOST_FS,
	.sysc_mask = SYSC_OMAP2_ENAWAKEUP,
	.regbits = &sysc_regbits_omap4_usb_host_fs,
};

static const struct sysc_regbits sysc_regbits_dra7_mcan = {
	.dmadisable_shift = -ENODEV,
	.midle_shift = -ENODEV,
	.sidle_shift = -ENODEV,
	.clkact_shift = -ENODEV,
	.enwkup_shift = 4,
	.srst_shift = 0,
	.emufree_shift = -ENODEV,
	.autoidle_shift = -ENODEV,
};

static const struct sysc_capabilities sysc_dra7_mcan = {
	.type = TI_SYSC_DRA7_MCAN,
	.sysc_mask = SYSC_DRA7_MCAN_ENAWAKEUP | SYSC_OMAP4_SOFTRESET,
	.regbits = &sysc_regbits_dra7_mcan,
	.mod_quirks = SYSS_QUIRK_RESETDONE_INVERTED,
};

static int sysc_init_pdata(struct sysc *ddata)
{
	struct ti_sysc_platform_data *pdata = dev_get_platdata(ddata->dev);
	struct ti_sysc_module_data *mdata;

	if (!pdata)
		return 0;

	mdata = devm_kzalloc(ddata->dev, sizeof(*mdata), GFP_KERNEL);
	if (!mdata)
		return -ENOMEM;

	if (ddata->legacy_mode) {
		mdata->name = ddata->legacy_mode;
		mdata->module_pa = ddata->module_pa;
		mdata->module_size = ddata->module_size;
		mdata->offsets = ddata->offsets;
		mdata->nr_offsets = SYSC_MAX_REGS;
		mdata->cap = ddata->cap;
		mdata->cfg = &ddata->cfg;
	}

	ddata->mdata = mdata;

	return 0;
}

static int sysc_init_match(struct sysc *ddata)
{
	const struct sysc_capabilities *cap;

	cap = of_device_get_match_data(ddata->dev);
	if (!cap)
		return -EINVAL;

	ddata->cap = cap;
	if (ddata->cap)
		ddata->cfg.quirks |= ddata->cap->mod_quirks;

	return 0;
}

static void ti_sysc_idle(struct work_struct *work)
{
	struct sysc *ddata;

	ddata = container_of(work, struct sysc, idle_work.work);

	/*
	 * One time decrement of clock usage counts if left on from init.
	 * Note that we disable opt clocks unconditionally in this case
	 * as they are enabled unconditionally during init without
	 * considering sysc_opt_clks_needed() at that point.
	 */
	if (ddata->cfg.quirks & (SYSC_QUIRK_NO_IDLE |
				 SYSC_QUIRK_NO_IDLE_ON_INIT)) {
		sysc_disable_main_clocks(ddata);
		sysc_disable_opt_clocks(ddata);
		sysc_clkdm_allow_idle(ddata);
	}

	/* Keep permanent PM runtime usage count for SYSC_QUIRK_NO_IDLE */
	if (ddata->cfg.quirks & SYSC_QUIRK_NO_IDLE)
		return;

	/*
	 * Decrement PM runtime usage count for SYSC_QUIRK_NO_IDLE_ON_INIT
	 * and SYSC_QUIRK_NO_RESET_ON_INIT
	 */
	if (pm_runtime_active(ddata->dev))
		pm_runtime_put_sync(ddata->dev);
}

static const struct of_device_id sysc_match_table[] = {
	{ .compatible = "simple-bus", },
	{ /* sentinel */ },
};

static int sysc_probe(struct platform_device *pdev)
{
	struct ti_sysc_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct sysc *ddata;
	int error;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = &pdev->dev;
	platform_set_drvdata(pdev, ddata);

	error = sysc_init_match(ddata);
	if (error)
		return error;

	error = sysc_init_dts_quirks(ddata);
	if (error)
		return error;

	error = sysc_map_and_check_registers(ddata);
	if (error)
		return error;

	error = sysc_init_sysc_mask(ddata);
	if (error)
		return error;

	error = sysc_init_idlemodes(ddata);
	if (error)
		return error;

	error = sysc_init_syss_mask(ddata);
	if (error)
		return error;

	error = sysc_init_pdata(ddata);
	if (error)
		return error;

	sysc_init_early_quirks(ddata);

	error = sysc_get_clocks(ddata);
	if (error)
		return error;

	error = sysc_init_resets(ddata);
	if (error)
		goto unprepare;

	error = sysc_init_module(ddata);
	if (error)
		goto unprepare;

	pm_runtime_enable(ddata->dev);
	error = pm_runtime_get_sync(ddata->dev);
	if (error < 0) {
		pm_runtime_put_noidle(ddata->dev);
		pm_runtime_disable(ddata->dev);
		goto unprepare;
	}

	/* Balance reset counts */
	if (ddata->rsts)
		reset_control_assert(ddata->rsts);

	sysc_show_registers(ddata);

	ddata->dev->type = &sysc_device_type;
	error = of_platform_populate(ddata->dev->of_node, sysc_match_table,
				     pdata ? pdata->auxdata : NULL,
				     ddata->dev);
	if (error)
		goto err;

	INIT_DELAYED_WORK(&ddata->idle_work, ti_sysc_idle);

	/* At least earlycon won't survive without deferred idle */
	if (ddata->cfg.quirks & (SYSC_QUIRK_NO_IDLE |
				 SYSC_QUIRK_NO_IDLE_ON_INIT |
				 SYSC_QUIRK_NO_RESET_ON_INIT)) {
		schedule_delayed_work(&ddata->idle_work, 3000);
	} else {
		pm_runtime_put(&pdev->dev);
	}

	return 0;

err:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
unprepare:
	sysc_unprepare(ddata);

	return error;
}

static int sysc_remove(struct platform_device *pdev)
{
	struct sysc *ddata = platform_get_drvdata(pdev);
	int error;

	cancel_delayed_work_sync(&ddata->idle_work);

	error = pm_runtime_get_sync(ddata->dev);
	if (error < 0) {
		pm_runtime_put_noidle(ddata->dev);
		pm_runtime_disable(ddata->dev);
		goto unprepare;
	}

	of_platform_depopulate(&pdev->dev);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	reset_control_assert(ddata->rsts);

unprepare:
	sysc_unprepare(ddata);

	return 0;
}

static const struct of_device_id sysc_match[] = {
	{ .compatible = "ti,sysc-omap2", .data = &sysc_omap2, },
	{ .compatible = "ti,sysc-omap2-timer", .data = &sysc_omap2_timer, },
	{ .compatible = "ti,sysc-omap4", .data = &sysc_omap4, },
	{ .compatible = "ti,sysc-omap4-timer", .data = &sysc_omap4_timer, },
	{ .compatible = "ti,sysc-omap4-simple", .data = &sysc_omap4_simple, },
	{ .compatible = "ti,sysc-omap3430-sr", .data = &sysc_34xx_sr, },
	{ .compatible = "ti,sysc-omap3630-sr", .data = &sysc_36xx_sr, },
	{ .compatible = "ti,sysc-omap4-sr", .data = &sysc_omap4_sr, },
	{ .compatible = "ti,sysc-omap3-sham", .data = &sysc_omap3_sham, },
	{ .compatible = "ti,sysc-omap-aes", .data = &sysc_omap3_aes, },
	{ .compatible = "ti,sysc-mcasp", .data = &sysc_omap4_mcasp, },
	{ .compatible = "ti,sysc-dra7-mcasp", .data = &sysc_dra7_mcasp, },
	{ .compatible = "ti,sysc-usb-host-fs",
	  .data = &sysc_omap4_usb_host_fs, },
	{ .compatible = "ti,sysc-dra7-mcan", .data = &sysc_dra7_mcan, },
	{  },
};
MODULE_DEVICE_TABLE(of, sysc_match);

static struct platform_driver sysc_driver = {
	.probe		= sysc_probe,
	.remove		= sysc_remove,
	.driver         = {
		.name   = "ti-sysc",
		.of_match_table	= sysc_match,
		.pm = &sysc_pm_ops,
	},
};

static int __init sysc_init(void)
{
	bus_register_notifier(&platform_bus_type, &sysc_nb);

	return platform_driver_register(&sysc_driver);
}
module_init(sysc_init);

static void __exit sysc_exit(void)
{
	bus_unregister_notifier(&platform_bus_type, &sysc_nb);
	platform_driver_unregister(&sysc_driver);
}
module_exit(sysc_exit);

MODULE_DESCRIPTION("TI sysc interconnect target driver");
MODULE_LICENSE("GPL v2");
