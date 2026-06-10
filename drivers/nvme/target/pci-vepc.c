// SPDX-License-Identifier: GPL-2.0
/*
 * Virtual PCIe Endpoint Controller
 */
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/compiler_types.h>
#include <linux/kthread.h>
#include <linux/pci-epc.h>
#include <linux/configfs.h>
#include <linux/kstrtox.h>
#include <linux/nodemask_types.h>
#include <linux/log2.h>

#define MSI_NR_VECTORS 64

struct reg_entry;
struct vepc_dev;
struct reg_state;

typedef u32 (*reg_read_fn)(struct vepc_dev *vepc, struct reg_entry *self, struct reg_state *state);
typedef void (*reg_write_fn)(struct vepc_dev *vepc, struct reg_entry *self, struct reg_state *state, u32 val);

/* single reg entry */
struct reg_entry {
    u8 size;    //1, 2, 4 bytes only
    u16 offset;
    u32 default_val;
    u32 ro_mask;
    u32 rw1c_mask;
    u32 sticky_mask;
    reg_read_fn read_handler;
    reg_write_fn write_handler;
};

/* single mutable reg value */
struct reg_state {
    u32 val;
};

/* container for device's registers space */
struct reg_space {
    const struct reg_entry *entries;
    size_t n_entries;
    struct reg_state *states;
    struct vepc_dev *dev;
};

struct vepc_dev {
	/* PCIe endpoint */
	u16 ep_vid;
	u16 ep_did;
	u64 bar0_phys;
	u64 bar0_size;
	//void __iomem *bar0_virt;
	struct reg_space ep_regs;
	struct pci_epc *epc;
	bool enabled;


	/* PCIe switch */
	u16 rc_vid;
	u16 rc_did;
	struct pci_host_bridge *bridge;
	struct platform_device *plat_dev;
	struct pci_sysdata sysdata;
	struct resource bus_res;
	struct resource mem_res;
	struct reg_space rc_regs;
	u64 ob_win_phys;
	u64 ob_win_size;

	struct irq_domain    *msi_domain;
	struct fwnode_handle *msi_fwnode;
	DECLARE_BITMAP(msi_used, MSI_NR_VECTORS);
	struct mutex msi_lock;
	int slot_irq;
	struct notifier_block pci_nb;

	/* for both */
	u32 access_filter;
};

enum reset_type {
	RESET_POWER_ON,
	RESET_PCIE_LINK_RESET,
	RESET_PCIE_FUNCTION_RESET,
};

enum acc_flags {
	ACC_F_EP_CRS	= (1 << 0),
	ACC_F_RC_CRS	= (1 << 1),
	ACC_F_EP_UR	= (1 << 2),
	ACC_F_RC_UR	= (1 << 3),
};

struct vepc_dev *vepc_dev;

/* ---- PCI EPC ---- */
static const struct pci_epc_features epc_features = {
	.linkup_notifier = 1,
	.msi_capable = 0,
	.msix_capable = 1,
	.intx_capable = 0,
	.align = SZ_4K,
	.bar[BAR_0] = {
		.type = BAR_PROGRAMMABLE,
		.only_64bit = true,
	},
	.bar[BAR_2] = {.type = BAR_DISABLED},
	.bar[BAR_3] = {.type = BAR_DISABLED},
	.bar[BAR_4] = {.type = BAR_DISABLED},
	.bar[BAR_5] = {.type = BAR_DISABLED},
};

static const struct pci_epc_features *epc_get_features(struct pci_epc *epc, u8 func_no, 
							u8 vfunc_no)
{
	return &epc_features;
}

static int epc_start(struct pci_epc *epc)
{
	pr_info("epc_start()\n");
	return 0;
}

static void epc_stop(struct pci_epc *epc)
{
	pr_info("epc_stop()\n");
}

static const struct pci_epc_ops epc_ops = {
	.get_features = epc_get_features,
	.start = epc_start,
	.stop = epc_stop,
	.owner = THIS_MODULE,
};



static void epc_unregister(struct vepc_dev *vepc)
{
	if(!vepc->epc)
		return;
	pci_epc_mem_exit(vepc->epc);
}

static int epc_register(struct vepc_dev *vepc)
{
	struct device *parent = &vepc->plat_dev->dev;
	int rc = dma_coerce_mask_and_coherent(parent, DMA_BIT_MASK(64));
	if(rc)
	{
		pr_err("dma_coerce_mask_and_coherent() failed: %d\n", rc);
		return rc;
	}

	struct pci_epc *epc = devm_pci_epc_create(parent, &epc_ops);
	if(IS_ERR(epc))
	{
		rc = PTR_ERR(epc);
		pr_err("devm_pci_epc_create() failed: %d\n", rc);
		return rc;
	}
	epc_set_drvdata(epc, vepc);
	vepc->epc = epc;

	epc->max_functions = 1;
	epc->max_vfs = devm_kcalloc(parent, epc->max_functions,
				    sizeof(*epc->max_vfs), GFP_KERNEL);
	if(!epc->max_vfs)
	{
		pr_err("devm_kcalloc() failed\n");
		return -ENOMEM;
	}

	rc = pci_epc_mem_init(epc, vepc->ob_win_phys, vepc->ob_win_size, SZ_4K);
	if(rc)
	{
		pr_err("pci_epc_mem_init() failed: %d\n", rc);
		return rc;
	}

	pr_info("epc: '%s' registered\n", dev_name(&epc->dev));
	return 0;
}

static int rc_init(struct vepc_dev *vepc);
static int rc_exit(struct vepc_dev *vepc);
static int rc_hotplug(struct vepc_dev *vepc);
static int rc_hotremove(struct vepc_dev *vepc);
static int rc_reset(enum reset_type reset, struct vepc_dev *vepc);

static int ep_init(struct vepc_dev *vepc);
static int ep_exit(struct vepc_dev *vepc);
static int ep_hotplug(struct vepc_dev *vepc);
static int ep_hotremove(struct vepc_dev *vepc);
static int ep_reset(enum reset_type reset, struct vepc_dev *vepc);

static int reg_space_init(struct reg_space *space, const struct reg_entry *entries, size_t n_entries);
static int reg_space_destroy(struct reg_space *space);
static int reg_read(struct reg_space *space, u32 offset, u32 size, u32 *val);
static int reg_write(struct reg_space *space, u32 offset, u32 size, u32 val);
static int reg_write_direct(struct reg_space *space, u32 offset, u32 size, u32 val);
static int reg_set_default_values(struct reg_space *space);

static int ep_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val);
static int ep_reg_write(struct vepc_dev *vepc, int where, int size, u32 val);
static int rc_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val);
static int rc_reg_write(struct vepc_dev *vepc, int where, int size, u32 val);
static void set_access_filter(struct vepc_dev *vepc, enum acc_flags flags);
static void clear_access_filter(struct vepc_dev *vepc, enum acc_flags flags);

static int msi_domain_create(struct vepc_dev *vepc, struct device *bridge);
static void msi_domain_destroy(struct vepc_dev *vepc);
static void msi_hotplug_irq(struct vepc_dev *vepc);
static int bus_notify(struct notifier_block *nb, unsigned long action, void *data);
static void patch_resources(struct vepc_dev *vepc, struct pci_dev *pdev);



static u32 did_read(struct vepc_dev *vepc, struct reg_entry *self,
		    struct reg_state *state)
{
	return vepc->rc_did;
}

static u32 vid_read(struct vepc_dev *vepc, struct reg_entry *self,
		    struct reg_state *state)
{
	return vepc->rc_vid;
}

static u32 pref_window_read(struct vepc_dev *vepc, struct reg_entry *self,
			    struct reg_state *state)
{
	const u64 base  = vepc->bar0_phys & ~0xFFFFFull;                          /* 1MiB-aligned base */
	const u64 limit = (vepc->bar0_phys + vepc->bar0_size - 1) & ~0xFFFFFull; /* top, 1MiB units  */
	const u16 pref_base_lo  = (u16)(((base  >> 16) & 0xFFF0) | 0x1);
	const u16 pref_limit_lo = (u16)(((limit >> 16) & 0xFFF0) | 0x1);

	return ((u32)pref_limit_lo << 16) | pref_base_lo;
}

/* Prefetchable Base Upper 32 Bits */
static u32 pref_base_upper_read(struct vepc_dev *vepc, struct reg_entry *self,
				struct reg_state *state)
{
	const u64 base = vepc->bar0_phys & ~0xFFFFFull; /* 1MiB-aligned base */

	return (u32)(base >> 32);
}

/* Prefetchable Limit Upper 32 Bits */
static u32 pref_limit_upper_read(struct vepc_dev *vepc, struct reg_entry *self,
				 struct reg_state *state)
{
	const u64 limit = (vepc->bar0_phys + vepc->bar0_size - 1) & ~0xFFFFFull; /* top, 1MiB units */

	return (u32)(limit >> 32);
}

const struct reg_entry rc_regs_layout[] = {
/* ---- Type 1 header, PCI-compatible region (0x00-0x3F) ---- */

/* Vendor ID */
   {  .offset = 0x00, .size = 2, .ro_mask = 0xFFFF, .read_handler = vid_read },
/* Device ID */
   {  .offset = 0x02, .size = 2, .ro_mask = 0xFFFF, .read_handler = did_read },
/* Command */
   {  .offset = 0x04, .size = 2, .ro_mask = 0xFAB9 },
/* Status */
   {  .offset = 0x06, .size = 2, .default_val = 0x0010,
      .ro_mask = 0x06FF, .rw1c_mask = 0xF900 },
/* Revision ID */
   {  .offset = 0x08, .size = 1, .default_val = 0x01, .ro_mask = 0xFF },
/* Programming Interface */
   {  .offset = 0x09, .size = 1, .default_val = 0x00, .ro_mask = 0xFF },
/* Sub-Class Code */
   {  .offset = 0x0A, .size = 1, .default_val = 0x04, .ro_mask = 0xFF },
/* Base Class Code */
   {  .offset = 0x0B, .size = 1, .default_val = 0x06, .ro_mask = 0xFF },
/* Cache Line Size */
   {  .offset = 0x0C, .size = 1 },
/* Primary Latency Timer */
   {  .offset = 0x0D, .size = 1, .ro_mask = 0xFF },
/* Header Type */
   {  .offset = 0x0E, .size = 1, .default_val = 0x01, .ro_mask = 0xFF },
/* BIST */
   {  .offset = 0x0F, .size = 1, .ro_mask = 0xFF },
/* BAR0 - not implemented */
   {  .offset = 0x10, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* BAR1 - not implemented */
   {  .offset = 0x14, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Primary Bus Number */
   {  .offset = 0x18, .size = 1 },
/* Secondary Bus Number */
   {  .offset = 0x19, .size = 1 },
/* Subordinate Bus Number */
   {  .offset = 0x1A, .size = 1 },
/* Secondary Latency Timer */
   {  .offset = 0x1B, .size = 1, .ro_mask = 0xFF },
/* I/O Base - disabled */
   {  .offset = 0x1C, .size = 1, .default_val = 0x00u, .ro_mask = 0xFFu },
/* I/O Limit - disabled */
   {  .offset = 0x1D, .size = 1, .default_val = 0x00u, .ro_mask = 0xFFu },
/* Secondary Status */
   {  .offset = 0x1E, .size = 2, .ro_mask = 0x06FF, .rw1c_mask = 0xF900 },
/* Non-prefetchable Memory Base/Limit - disabled */
   {  .offset = 0x20, .size = 4, .default_val = 0x0000FFF0u, .ro_mask = 0xFFFFFFFF},
/* Prefetchable Memory Base/Limit */
   {  .offset = 0x24, .size = 4, .ro_mask = 0xFFFFFFFF, .read_handler = pref_window_read},
/* Prefetchable Base Upper 32 Bits */
   {  .offset = 0x28, .size = 4, .ro_mask = 0xFFFFFFFF, .read_handler = pref_base_upper_read },
/* Prefetchable Limit Upper 32 Bits */
   {  .offset = 0x2C, .size = 4, .ro_mask = 0xFFFFFFFF, .read_handler = pref_limit_upper_read},
/* I/O Base Upper 16 Bits + I/O Limit Upper 16 Bits */
   {  .offset = 0x30, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Capabilities Pointer */
   {  .offset = 0x34, .size = 4, .default_val = 0x00000040u,
      .ro_mask = 0xFFFFFFFFu },
/* Expansion ROM Base Address */
   {  .offset = 0x38, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Interrupt Line */
   {  .offset = 0x3C, .size = 1 },
/* Interrupt Pin - no legacy INTx */
   {  .offset = 0x3D, .size = 1, .default_val = 0x00, .ro_mask = 0xFF },
/* Bridge Control */
   {  .offset = 0x3E, .size = 2, .ro_mask = 0xFFA0 },

/* ---- PCI Express Capability */

/* PCI Express Capability List */
   {  .offset = 0x40, .size = 2, .default_val = 0x8010, .ro_mask = 0xFFFF },
/* PCI Express Capabilities Register */
   {  .offset = 0x42, .size = 2, .default_val = 0x0142, .ro_mask = 0xFFFF },
/* Device Capabilities */
   {  .offset = 0x44, .size = 4, .default_val = 0x00008001u, .ro_mask = 0xFFFFFFFFu },
/* Device Control + Device Status */
   {  .offset = 0x48, .size = 4, .rw1c_mask = 0x000F0000u, .ro_mask = 0x00200000u },
/* Link Capabilities */
   {  .offset = 0x4C, .size = 4, .default_val = 0x00100045u, .ro_mask = 0xFFFFFFFFu },
/* Link Control */
   {  .offset = 0x50, .size = 2, .default_val = 0x0000, .ro_mask = 0x0000 },
/* Link Status */
   {  .offset = 0x52, .size = 2, .default_val = 0x0045, .ro_mask = 0xFFFF },

/* Slot Capabilities */
   {  .offset = 0x54, .size = 4, .default_val = 0x000C005Au, .ro_mask = 0xFFFFFFFFu },
/* Slot Control */
   {  .offset = 0x58, .size = 2, .default_val = 0x0000, .ro_mask = 0x0000 },
/* Slot Status */
   { .offset = 0x5A, .size = 2, .default_val = 0x0000, .rw1c_mask = 0x011F, .ro_mask = 0x00E0 },
/* Root Control + Root Capabilities */
   {  .offset = 0x5C, .size = 4, .default_val = 0x00010000u, .ro_mask = 0xFFFF0000u },
/* Root Status */
   {  .offset = 0x60, .size = 4, .rw1c_mask = 0x00010000u, .ro_mask = 0xFFFEFFFFu },
/* Device Capabilities 2 */
   {  .offset = 0x64, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Device Control 2 + Device Status 2 */
   {  .offset = 0x68, .size = 4, .ro_mask = 0xFFFF0000u },
/* Link Capabilities 2 */
   {  .offset = 0x6C, .size = 4, .default_val = 0x0000003Eu,
      .ro_mask = 0xFFFFFFFFu },
/* Link Control 2 + Link Status 2 */
   {  .offset = 0x70, .size = 4, .ro_mask = 0xFFFF0000u },

/* ---- MSI Capability */

/* MSI Capability Header + Message Control */
   {  .offset = 0x80, .size = 4, .default_val = 0x00800005u, .ro_mask = 0xFF8EFFFFu },
/* Message Address */
   {  .offset = 0x84, .size = 4, .ro_mask = 0x00000003u },
/* Message Upper Address */
   {  .offset = 0x88, .size = 4 },
/* Message Data */
   {  .offset = 0x8C, .size = 4, .ro_mask = 0xFFFF0000u },

/* ---- AER Extended Capability */

/* AER Extended Capability Header */
   {  .offset = 0x100, .size = 4, .default_val = 0x00020001u, .ro_mask = 0xFFFFFFFFu },
/* Uncorrectable Error Status */
   {  .offset = 0x104, .size = 4, .rw1c_mask = 0x03FF7030u, .sticky_mask = 0x03FF7030u },
/* Uncorrectable Error Mask */
   {  .offset = 0x108, .size = 4, .sticky_mask = 0x03FF7030u },
/* Uncorrectable Error Severity */
   {  .offset = 0x10C, .size = 4, .sticky_mask = 0x03FF7030u },
/* Correctable Error Status */
   {  .offset = 0x110, .size = 4,
      .rw1c_mask = 0x0000F1C1u, .sticky_mask = 0x0000F1C1u },
/* Correctable Error Mask */
   {  .offset = 0x114, .size = 4, .sticky_mask = 0x0000F1C1u },
/* Advanced Error Capabilities and Control */
   {  .offset = 0x118, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW0 */
   {  .offset = 0x11C, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW1 */
   {  .offset = 0x120, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW2 */
   {  .offset = 0x124, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW3 */
   {  .offset = 0x128, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Root Error Command */
   {  .offset = 0x12C, .size = 4, .ro_mask = 0xFFFFFFF8u },
/* Root Error Status */
   {  .offset = 0x130, .size = 4, .rw1c_mask = 0x0000007Fu, .ro_mask = 0xFFFFFF80u, .sticky_mask = 0x0000007Fu },
/* Error Source Identification */
   {  .offset = 0x134, .size = 4, .ro_mask = 0xFFFFFFFFu },
};
const size_t rc_regs_layout_size = ARRAY_SIZE(rc_regs_layout);


#define BAR0_TYPE 0xCu   /* memory space, 64-bit, prefetchable */

static void bar0_write_lo(struct vepc_dev *vepc, struct reg_entry *self, struct reg_state *state, u32 val)
{
	const u32 ro = (u32)(vepc->bar0_size - 1);

	state->val = (val & ~ro) | (BAR0_TYPE & ro);
}

static void bar0_write_hi(struct vepc_dev *vepc, struct reg_entry *self,
			    struct reg_state *state, u32 val)
{
	const u32 ro = (u32)((vepc->bar0_size - 1) >> 32);

	state->val = val & ~ro;
}

static void command_write(struct vepc_dev *vnvme, struct reg_entry *self,
			  struct reg_state *state, u32 val)
{
	const u32 old = state->val;
	const u32 changed = old ^ val;

	if (changed & PCI_COMMAND_MASTER)
	{
		if(val & PCI_COMMAND_MASTER)
			pr_info("BME ON\n");
		else
			pr_info("BME OFF\n");

	}

	if (changed & PCI_COMMAND_MEMORY)
	{
		if(val & PCI_COMMAND_MEMORY)
			pr_info("MSE ON\n");
		else
			pr_info("MSE OFF\n");
	}

	state->val = val;
}

static void pmcsr_write(struct vepc_dev *vepc, struct reg_entry *self,
			struct reg_state *state, u32 val)
{
	const u32 old_ps = state->val & PCI_PM_CTRL_STATE_MASK; /* bits [1:0], still pre-write */
	const u32 new_ps = val & PCI_PM_CTRL_STATE_MASK;        /* val already ro/rw1c-filtered by reg.c */

	if (new_ps == 0x3 && old_ps != 0x3)
		pr_info("D3Hot ON\n");
	else if(new_ps == 0x0 && old_ps == 0x3)
		pr_info("D3Hot OFF\n");

	state->val = val;
}

static u32 ep_did_read(struct vepc_dev *vepc, struct reg_entry *self,
		    struct reg_state *state)
{
	return vepc->ep_did;
}

static u32 ep_vid_read(struct vepc_dev *vepc, struct reg_entry *self,
		    struct reg_state *state)
{
	return vepc->ep_vid;
}

const struct reg_entry ep_regs_layout[] = {
/* ---- Type 0 header, PCI-compatible region (0x00-0x3F) */

/* Vendor ID */
   { .offset = 0x00, .size = 2, .ro_mask = 0xFFFF, .read_handler = ep_vid_read },
/* Device ID */
   { .offset = 0x02, .size = 2, .ro_mask = 0xFFFF, .read_handler = ep_did_read },
/* Command */
   { .offset = 0x04, .size = 2, .ro_mask = 0xFAB9, .write_handler = command_write },
/* Status  */
   { .offset = 0x06, .size = 2, .default_val = 0x0010, .ro_mask = 0x06FF,
.rw1c_mask = 0xF900 },
/* Revision ID  */
   { .offset = 0x08, .size = 1, .default_val = 0x01, .ro_mask = 0xFF },
/* Programing Interface */
   { .offset = 0x09, .size = 1, .default_val = 0x02, .ro_mask = 0xFF },
/* Sub-Class  */
   { .offset = 0x0A, .size = 1, .default_val = 0x08, .ro_mask = 0xFF },
/* Base Class */
   { .offset = 0x0B, .size = 1, .default_val = 0x01, .ro_mask = 0xFF },
/* Cache Line Size */
   { .offset = 0x0C, .size = 1 },
/* Latency Timer */
   { .offset = 0x0D, .size = 1, .ro_mask = 0xFF },
/* Header Type */
   { .offset = 0x0E, .size = 1, .default_val = 0x00, .ro_mask = 0xFF },
/* BIST */
   { .offset = 0x0F, .size = 1, .ro_mask = 0xFF },
/* BAR0 - special case, dedicated write handler overrides all masks */
   { .offset = 0x10, .size = 4, .default_val = BAR0_TYPE, .write_handler = bar0_write_lo},
/* BAR1 - special case, dedicated write handler overrides all masks */
   { .offset = 0x14, .size = 4, .write_handler = bar0_write_hi },
/* BAR2 */
   { .offset = 0x18, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* BAR3 */
   { .offset = 0x1C, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* BAR4 */
   { .offset = 0x20, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* BAR5 */
   { .offset = 0x24, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Cardbus CIS Pointer */
   { .offset = 0x28, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Subsystem Vendor ID */
   { .offset = 0x2C, .size = 2, .ro_mask = 0xFFFF, .read_handler = ep_vid_read },
/* Subsystem ID */
   { .offset = 0x2E, .size = 2, .ro_mask = 0xFFFF, .read_handler = ep_did_read },
/* Expansion ROM Base Address */
   { .offset = 0x30, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Capabilities Pointer */
   { .offset = 0x34, .size = 4, .default_val = 0x00000040u, .ro_mask = 0xFFFFFFFFu },
/* Reserved */
   { .offset = 0x38, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Interrupt Line */
   {  .offset = 0x3C, .size = 1 },
/* Interrupt Pin - no legacy INTx */
   {  .offset = 0x3D, .size = 1, .default_val = 0x00, .ro_mask = 0xFF },
/* Min_Gnt */
   {  .offset = 0x3E, .size = 1, .ro_mask = 0xFF },
/* Max_Lat */
   {  .offset = 0x3F, .size = 1, .ro_mask = 0xFF },

/* ---- PCI Power Management Capability  */
/* Required for all PCI Express Functions */

/* Power Management Capabilities (PMC) */
   {  .offset = 0x40, .size = 4, .default_val = 0x00035001u, .ro_mask = 0xFFFFFFFFu },
/* Power Management Control/Status (PMCSR) */
   {  .offset = 0x44, .size = 4, .ro_mask = 0xFFFF60FCu, .rw1c_mask = 0x00008000u, .write_handler = pmcsr_write },

/* MSI-X Capability */

/* MSI-X Message Control + Capability Header (table size = 17 (16 IO + 1 admin)) */
   {  .offset = 0x50, .size = 4, .default_val = 0x00106011u, .ro_mask = 0x3FFFFFFFu },
/* MSI-X Table Offset / Table BIR */
   {  .offset = 0x54, .size = 4, .default_val = 0x00002000u, .ro_mask = 0xFFFFFFFFu },
/* MSI-X PBA Offset / PBA BIR */
   {  .offset = 0x58, .size = 4, .default_val = 0x00003000u, .ro_mask = 0xFFFFFFFFu },

/* ---- PCI Express Capability */

/* PCI Express Capability List */
   {  .offset = 0x60, .size = 2, .default_val = 0x0010, .ro_mask = 0xFFFF },
/* PCI Express Capabilities Register */
   {  .offset = 0x62, .size = 2, .default_val = 0x0002, .ro_mask = 0xFFFF },
/* Device Capabilities */
   {  .offset = 0x64, .size = 4, .default_val = 0x00008001u, .ro_mask = 0xFFFFFFFFu },
/* Device Control + Device Status */
   {  .offset = 0x68, .size = 4, .rw1c_mask = 0x000F0000u, .ro_mask = 0x00208000u },
/* Link Capabilities */
   {  .offset = 0x6C, .size = 4, .default_val = 0x00000045u, .ro_mask = 0xFFFFFFFFu },
/* Link Control + Link Status */
   {  .offset = 0x70, .size = 4, .default_val = 0x00450000u, .ro_mask = 0xFFFF0000u },
/* Device Capabilities 2 */
   {  .offset = 0x74, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Device Control 2 + Device Status 2 */
   {  .offset = 0x78, .size = 4, .ro_mask = 0xFFFF0000u },
/* Link Capabilities 2 */
   {  .offset = 0x7C, .size = 4, .default_val = 0x0000003Eu, .ro_mask = 0xFFFFFFFFu },
/* Link Control 2 + Link Status 2 */
   {  .offset = 0x80, .size = 4, .ro_mask = 0xFFFF0000u },

/* ---- AER Extended Capability */

/* AER Extended Capability Header */
   {  .offset = 0x100, .size = 4, .default_val = 0x15010001u, .ro_mask = 0xFFFFFFFFu },
/* Uncorrectable Error Status */
   {  .offset = 0x104, .size = 4, .rw1c_mask = 0x03FF7030u, .sticky_mask = 0x03FF7030u },
/* Uncorrectable Error Mask */
   {  .offset = 0x108, .size = 4, .sticky_mask = 0x03FF7030u },
/* Uncorrectable Error Severity */
   {  .offset = 0x10C, .size = 4, .sticky_mask = 0x03FF7030u },
/* Correctable Error Status */
   {  .offset = 0x110, .size = 4, .rw1c_mask = 0x0000F1C1u, .sticky_mask = 0x0000F1C1u },
/* Correctable Error Mask */
   {  .offset = 0x114, .size = 4, .sticky_mask = 0x0000F1C1u },
/* Advanced Error Capabilities and Control */
   {  .offset = 0x118, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW0 */
   {  .offset = 0x11C, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW1 */
   {  .offset = 0x120, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW2 */
   {  .offset = 0x124, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Header Log DW3 */
   {  .offset = 0x128, .size = 4, .ro_mask = 0xFFFFFFFFu },

/* ---- Secondary PCI Express Extended Capability */

/* Secondary PCI Express Extended Capability Header */
   {  .offset = 0x150, .size = 4, .default_val = 0x18010019u, .ro_mask = 0xFFFFFFFFu },
/* Link Control 3 Register */
   {  .offset = 0x154, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Lane Error Status Register */
   {  .offset = 0x158, .size = 4, .rw1c_mask = 0x0000000Fu,
      .sticky_mask = 0x0000000Fu, .ro_mask = 0xFFFFFFF0u },
/* Lane Equalization Control - Lanes 0,1 */
   {  .offset = 0x15C, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Lane Equalization Control - Lanes 2,3 */
   {  .offset = 0x160, .size = 4, .ro_mask = 0xFFFFFFFFu },

/* ---- Physical Layer 16.0 GT/s Extended Capability */

/* Physical Layer 16.0 GT/s Extended Capability Header */
   {  .offset = 0x180, .size = 4, .default_val = 0x1B010026u, .ro_mask = 0xFFFFFFFFu },
/* 16.0 GT/s Capabilities Register */
   {  .offset = 0x184, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* 16.0 GT/s Control Register */
   {  .offset = 0x188, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* 16.0 GT/s Status Register */
   {  .offset = 0x18C, .size = 4, .rw1c_mask = 0x00000010u,
      .sticky_mask = 0x00000010u, .ro_mask = 0xFFFFFFEFu },
/* 16.0 GT/s Local Data Parity Mismatch Status */
   {  .offset = 0x190, .size = 4, .rw1c_mask = 0x0000000Fu,
      .sticky_mask = 0x0000000Fu, .ro_mask = 0xFFFFFFF0u },
/* 16.0 GT/s First Retimer Data Parity Mismatch Status */
   {  .offset = 0x194, .size = 4, .rw1c_mask = 0x0000000Fu,
      .sticky_mask = 0x0000000Fu, .ro_mask = 0xFFFFFFF0u },
/* 16.0 GT/s Second Retimer Data Parity Mismatch Status */
   {  .offset = 0x198, .size = 4, .rw1c_mask = 0x0000000Fu,
      .sticky_mask = 0x0000000Fu, .ro_mask = 0xFFFFFFF0u },
/* Physical Layer 16.0 GT/s Reserved */
   {  .offset = 0x19C, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* 16.0 GT/s Lane Equalization Control Register */
   {  .offset = 0x1A0, .size = 4, .ro_mask = 0xFFFFFFFFu },

/* ---- Physical Layer 32.0 GT/s Extended Capability */

/* Physical Layer 32.0 GT/s Extended Capability Header */
   {  .offset = 0x1B0, .size = 4, .default_val = 0x1E01002Au, .ro_mask = 0xFFFFFFFFu },
/* 32.0 GT/s Capabilities Register */
   {  .offset = 0x1B4, .size = 4, .default_val = 0x00000101u, .ro_mask = 0xFFFFFFFFu },
/* 32.0 GT/s Control Register */
   {  .offset = 0x1B8, .size = 4, .ro_mask = 0xFFFFFFFEu },
/* 32.0 GT/s Status Register */
   {  .offset = 0x1BC, .size = 4, .rw1c_mask = 0x00000010u,
      .sticky_mask = 0x00000010u, .ro_mask = 0xFFFFFFEFu },
/* Received Modified TS Data 1 */
   {  .offset = 0x1C0, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Received Modified TS Data 2 */
   {  .offset = 0x1C4, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Transmitted Modified TS Data 1 */
   {  .offset = 0x1C8, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Transmitted Modified TS Data 2 */
   {  .offset = 0x1CC, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* 32.0 GT/s Lane Equalization Control Register */
   {  .offset = 0x1D0, .size = 4, .ro_mask = 0xFFFFFFFFu },

/* ---- Lane Margining at the Receiver Extended Capability */

/* Lane Margining Extended Capability Header */
   {  .offset = 0x1E0, .size = 4, .default_val = 0x00010027u, .ro_mask = 0xFFFFFFFFu },
/* Margining Port Capabilities + Margining Port Status */
   {  .offset = 0x1E4, .size = 4, .ro_mask = 0xFFFFFFFFu },
/* Margining Lane Control/Status - Lane 0 */
   {  .offset = 0x1E8, .size = 4, .default_val = 0x00009C38u, .ro_mask = 0xFFFF0080u },
/* Margining Lane Control/Status - Lane 1 */
   {  .offset = 0x1EC, .size = 4, .default_val = 0x00009C38u, .ro_mask = 0xFFFF0080u },
/* Margining Lane Control/Status - Lane 2 */
   {  .offset = 0x1F0, .size = 4, .default_val = 0x00009C38u, .ro_mask = 0xFFFF0080u },
/* Margining Lane Control/Status - Lane 3 */
   {  .offset = 0x1F4, .size = 4, .default_val = 0x00009C38u, .ro_mask = 0xFFFF0080u },
};
const size_t ep_regs_layout_size = ARRAY_SIZE(ep_regs_layout);

static int pci_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	struct vepc_dev *vepc = container_of(bus->sysdata, struct vepc_dev, sysdata);
	*val = 0xFFFFFF; //TODO: better define needed
	
	if(size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;	//support only 1,2,4

	if(devfn != PCI_DEVFN(0,0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus->number == 0)
		return rc_reg_read(vepc, where, size, val);
	else if(bus->number == 1)
		return ep_reg_read(vepc, where, size, val);
	
	return PCIBIOS_DEVICE_NOT_FOUND;
}

static int pci_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	struct vepc_dev *vepc = container_of(bus->sysdata, struct vepc_dev, sysdata);
	
	if(size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;	//support only 1,2,4

	if(devfn != PCI_DEVFN(0,0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus->number == 0)
		rc_reg_write(vepc, where, size, val);
	else if(bus->number == 1)
		ep_reg_write(vepc, where, size, val);
	
	return PCIBIOS_DEVICE_NOT_FOUND;
}

static struct pci_ops pci_ops = {
	.read = pci_read,
	.write = pci_write,
};

/* ----   config fs ----*/
static DEFINE_MUTEX(cfg_lock);

static u16 rc_vid;
static u16 rc_did;
static u16 ep_vid;
static u16 ep_did;
static u64 mem_win_phys;
static u32 mem_win_size;

static ssize_t vepc_cfg_hotplug_store(struct config_item *item, const char *page, size_t len)
{
	bool plug;
	if(kstrtobool(page, &plug))
		return -EINVAL;
	if(!plug)
		return -EINVAL;

	if(vepc_dev->enabled)
		return -EPERM;

	pr_info("hotplug requested!\n");
	mutex_lock(&cfg_lock);
	ep_hotplug(vepc_dev);
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR_WO(vepc_cfg_, hotplug);

static ssize_t vepc_cfg_hotremove_store(struct config_item *item, const char *page, size_t len)
{
	bool remove;
	if(kstrtobool(page, &remove))
		return -EINVAL;
	if(!remove)
		return -EINVAL;

	if(!vepc_dev->enabled)
		return -EPERM;

	pr_info("hotremoval requested!\n");

	mutex_lock(&cfg_lock);
	ep_hotremove(vepc_dev);
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR_WO(vepc_cfg_, hotremove);


static bool verify_pids_vids(void)
{
	if(!rc_vid)
	{
		pr_err("rc_vid is empty!\n");
		return false;
	}

	if(!rc_did)
	{
		pr_err("rc_did is empty!\n");
		return false;
	}
	if(!ep_vid)
	{
		pr_err("ep_vid is empty!\n");
		return false;
	}
	if(!ep_did)
	{
		pr_err("ep_did is empty!\n");
		return false;
	}
	
	return true;
}

/*
 * mem window layout:
 * [ Hot-plug mem window (4M)  | translation window for EPC (28M)]
 * [BAR0 (64K)]
 *
 * BAR0 lives in the beginning of Hot-plug mem
 */
#define VEPC_BAR0_SIZE		SZ_64K
#define VEPC_HP_SIZE		SZ_4M
#define VEPC_MEM_WIN_MIN	SZ_32M
#define VEPC_MEM_WIN_MAX	SZ_256M

static bool verify_mem_win(void)
{
	if(!IS_ALIGNED(mem_win_size, SZ_1M))
	{
		pr_err("mem_win_size must be a multiple of 1MB!\n");
		return false;
	}
	if(mem_win_size  < VEPC_MEM_WIN_MIN || mem_win_size > VEPC_MEM_WIN_MAX)
	{
		pr_err("mem_win_size out or range [32MB, 256MB]!\n");
		return false;
	}
	if(!IS_ALIGNED(mem_win_phys, SZ_1M))
	{
		pr_err("mem_win_phys in NOT 1MB aligned!\n");
		return false;
	}

	return true;
}

static ssize_t vepc_cfg_enable_store(struct config_item *item, const char *page, size_t len)
{
	bool enable;
	if(kstrtobool(page, &enable))
		return -EINVAL;

	pr_info("enable = %d!\n", enable);
	mutex_lock(&cfg_lock);
	if(enable)
	{
		if(vepc_dev->bridge)
		{
			pr_info("already enabled\n");
			mutex_unlock(&cfg_lock);
			return -EPERM;
		}

		if(!verify_pids_vids() || !verify_mem_win())
		{
			mutex_unlock(&cfg_lock);
			return -EINVAL;
		}

		vepc_dev->rc_did = rc_did;
		vepc_dev->rc_vid = rc_vid;
		vepc_dev->ep_did = ep_did;
		vepc_dev->ep_vid = ep_vid;
		vepc_dev->bar0_phys = mem_win_phys;
		vepc_dev->bar0_size = VEPC_BAR0_SIZE;
		vepc_dev->ob_win_phys = mem_win_phys + VEPC_HP_SIZE;
		vepc_dev->ob_win_size = mem_win_phys - VEPC_HP_SIZE;

		//enable controller
		pr_info("enabling...\n");
		int rc = rc_hotplug(vepc_dev);
		if(rc)
		{
			mutex_unlock(&cfg_lock);
			return rc;
		}
	}
	else
	{
		if(!vepc_dev->bridge)
		{
			pr_info("already disabled\n");
			mutex_unlock(&cfg_lock);
			return -EPERM;
		}

		pr_info("disabling...\n");
		ep_hotremove(vepc_dev);
		rc_hotremove(vepc_dev);
	}
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR_WO(vepc_cfg_, enable);

static ssize_t vepc_cfg_rc_vid_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", rc_vid);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_rc_vid_store(struct config_item *item, const char *page, size_t len)
{
	u16 vid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &vid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	rc_vid = vid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, rc_vid);

static ssize_t vepc_cfg_rc_did_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", rc_did);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_rc_did_store(struct config_item *item, const char *page, size_t len)
{
	u16 pid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &pid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	rc_did = pid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, rc_did);

static ssize_t vepc_cfg_ep_vid_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", ep_vid);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_ep_vid_store(struct config_item *item, const char *page, size_t len)
{
	u16 vid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &vid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	ep_vid = vid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, ep_vid);

static ssize_t vepc_cfg_ep_did_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", ep_did);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_ep_did_store(struct config_item *item, const char *page, size_t len)
{
	u16 pid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &pid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	ep_did = pid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, ep_did);

static ssize_t vepc_cfg_mem_win_phys_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%llx\n", mem_win_phys);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_mem_win_phys_store(struct config_item *item, const char *page, size_t len)
{
	u64 mem_win;
	if(!len)
		return -EINVAL;
	if(kstrtou64(page, 0, &mem_win))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	mem_win_phys = mem_win;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, mem_win_phys);

static ssize_t vepc_cfg_mem_win_size_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", mem_win_size);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_mem_win_size_store(struct config_item *item, const char *page, size_t len)
{
	u32 mem_win;
	if(!len)
		return -EINVAL;
	if(kstrtou32(page, 0, &mem_win))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	mem_win_size = mem_win;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, mem_win_size);

static struct configfs_attribute *vepc_cfg_attrs[] = {
	&vepc_cfg_attr_hotplug,
	&vepc_cfg_attr_hotremove,
	&vepc_cfg_attr_enable,
	&vepc_cfg_attr_rc_vid,
	&vepc_cfg_attr_rc_did,
	&vepc_cfg_attr_ep_vid,
	&vepc_cfg_attr_ep_did,
	&vepc_cfg_attr_mem_win_phys,
	&vepc_cfg_attr_mem_win_size,
	NULL
};

static const struct config_item_type vepc_cfg_root_type = {
	.ct_owner = THIS_MODULE,
	.ct_attrs = vepc_cfg_attrs,
};

static struct configfs_subsystem vepc_cfg_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "vepc",
			.ci_type = &vepc_cfg_root_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(vepc_cfg_subsys.su_mutex),
};

/* ----   INIT ---- */
static int __init vepc_init_module(void)
{
	int rc;
	config_group_init(&vepc_cfg_subsys.su_group);
	rc = configfs_register_subsystem(&vepc_cfg_subsys);
	if(rc) {
		pr_err("configfs_register_subsystem() failed: %d\n", rc);
		return rc;
	}

	vepc_dev = kzalloc(sizeof(*vepc_dev), GFP_KERNEL);
	if(!vepc_dev)
	{
		rc = -ENOMEM;
		goto group_relese;
	}

	pr_info("vepc_init complete!\n");
	return 0;

group_relese:
	configfs_unregister_subsystem(&vepc_cfg_subsys);

	return rc;
}

static void __exit vepc_exit_module(void)
{
	if(vepc_dev)
	{
		configfs_unregister_subsystem(&vepc_cfg_subsys);
	}
}

/* 1MiB-aligned helper */
#define RC_MEM_BASE(phys) ((phys) & ~((1ULL << 20) - 1))

static int rc_init(struct vepc_dev *vepc)
{
	int rc;

	rc = reg_space_init(&vepc->rc_regs, rc_regs_layout, rc_regs_layout_size);
	if(rc)
	{
		pr_err("reg_space_init() failed: %d\n", rc);
		return rc;
	}
	vepc->rc_regs.dev = vepc;

	rc = rc_reset(RESET_POWER_ON, vepc);
	if(rc)
	{
		pr_err("rc_reset() failed!\n");
		goto destroy_regs;
	}

	vepc->plat_dev = platform_device_register_simple("vepc", -1, NULL, 0);
	if(IS_ERR(vepc->plat_dev))
	{
		rc = PTR_ERR(vepc->plat_dev);
		pr_err("platform_device_register_simple() failed: %d\n", rc);
		goto destroy_regs;
	}
	struct device *dev = &vepc->plat_dev->dev;
	pr_info("device succesfully registered\n");
	
	/* allocate bridge */
	struct pci_host_bridge *bridge = devm_pci_alloc_host_bridge(dev, 0);
	if(!bridge)
	{
		pr_err("devm_pci_alloc_host_bridge() failed\n");
		rc = -ENOMEM;
		goto destroy_platform;
	}

	/* add resources */
	vepc->bus_res	= (struct resource) {
		.name	= "vepc-busn",
		.start	= 0x00,
		.end	= 0xFF,
		.flags	= IORESOURCE_BUS,
	};
	pci_add_resource(&bridge->windows, &vepc->bus_res);

	vepc->mem_res	= (struct resource) {
		.name	= "vepc-mem",
		.start	= vepc->bar0_phys,
		.end	= vepc->bar0_phys + VEPC_HP_SIZE - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_64 | IORESOURCE_PREFETCH,
	};
	pci_add_resource(&bridge->windows, &vepc->mem_res);

	/* allocate private domain */
	const int d = pci_bus_find_emul_domain_nr(0, 0x12345, 0x12345);
	if(d < 0)
	{
		dev_err(dev, "pci_bus_find_emul_domain_nr() failed: %d\n", d);
		rc = d;
		goto free_resources;
	}
	vepc->sysdata.domain = d;
	vepc->sysdata.node = NUMA_NO_NODE;
	pr_info("got private domain: 0x%x\n", vepc->sysdata.domain);

	bridge->sysdata = (void *) &vepc->sysdata;
	bridge->ops = &pci_ops;
	bridge->busnr = 0x0;

	/* MSI */
	rc = msi_domain_create(vepc, &bridge->dev);
	if(rc)
	{
		pr_err("msi_domain_create() failed: %d\n", rc);
		goto release_domain;
	}

	set_access_filter(vepc, ACC_F_EP_UR);	//Don't allow endpoint enumeration
	
	vepc->pci_nb.notifier_call = bus_notify;
	rc = bus_register_notifier(&pci_bus_type, &vepc->pci_nb);
	if(rc)
	{
		pr_err("bus_register_notifier() failed: %d\n", rc);
		goto destroy_msi;
	}

	rc = pci_scan_root_bus_bridge(bridge);
	if(rc)
	{
		pr_err("pci_scan_root_bus_bridge() failed: %d\n", rc);
		goto release_notifier;
	}

	pci_bus_size_bridges(bridge->bus);
	pci_bus_assign_resources(bridge->bus);
	vepc->bridge = bridge;
	
	pci_bus_add_devices(bridge->bus);

	return 0;

release_notifier:
	bus_unregister_notifier(&pci_bus_type, &vepc->pci_nb);

destroy_msi:
	msi_domain_destroy(vepc);

release_domain:
	pci_bus_release_emul_domain_nr(vepc->sysdata.domain);

free_resources:
	pci_free_resource_list(&bridge->windows);

destroy_platform:
	platform_device_unregister(vepc->plat_dev);
	vepc->plat_dev = NULL;

destroy_regs:
	reg_space_destroy(&vepc->rc_regs);
	return rc;
}

static int rc_exit(struct vepc_dev *vepc)
{
	bus_unregister_notifier(&pci_bus_type, &vepc->pci_nb);
	if(vepc->bridge)
	{
		pci_stop_root_bus(vepc->bridge->bus);
		pci_remove_root_bus(vepc->bridge->bus);
	}
	msi_domain_destroy(vepc);
	if(vepc->sysdata.domain)
	{
		pci_bus_release_emul_domain_nr(vepc->sysdata.domain);
		vepc->sysdata.domain = 0x0;
	}

	if(vepc->plat_dev)
	{
		platform_device_unregister(vepc->plat_dev);
		vepc->plat_dev = NULL;
	}
	vepc->bridge = NULL;
	reg_space_destroy(&vepc->rc_regs);
	return 0;
}

static int rc_hotplug(struct vepc_dev *vepc)
{
	int rc = rc_init(vepc);
	if(rc)
	{
		pr_err("rc_init() failed: %d\n", rc);
		return rc;
	}
	rc = epc_register(vepc);
	if(rc)
	{
		pr_err("epc_register() failed: %d\n", rc);
		return rc;
	}
	return rc;
}

static int rc_hotremove(struct vepc_dev *vepc)
{
	epc_unregister(vepc);
	ep_hotremove(vepc);
	return rc_exit(vepc);
}

static size_t reg_find_first(const struct reg_space *space, u32 offset)
{
	size_t lo = 0, hi = space->n_entries;
	while(lo < hi) {
		size_t mid = lo + (hi - lo) / 2;
		const struct reg_entry *e = &space->entries[mid];

		if((u32) e->offset + e->size <= offset)
			lo = mid + 1;
		else
			hi = mid;

	}
	return lo;
}

static u32 reg_incoming(u32 access_start, u32 access_end,
			u32 reg_base, u32 reg_size, u32 write_val)
{
	const u32 reg_end = reg_base + reg_size;
	const u32 lo = max(reg_base, access_start);
	const u32 hi = min(reg_end, access_end);
	u32 incoming = 0, addr;

	for (addr = lo; addr < hi; ++addr)
		incoming |= (u32)((write_val >> (8 * (addr - access_start))) & 0xff)
			    << (8 * (addr - reg_base));
	return incoming;
}

static u32 reg_filter_write(const struct reg_state *state, u32 access_start,
			    u32 access_end, u32 reg_base, u32 reg_size,
			    u32 ro_mask, u32 rw1c_mask, u32 write_val)
{
	const u32 reg_end = reg_base + reg_size;
	const u32 copy_lo = max(reg_base, access_start);
	const u32 copy_hi = min(reg_end, access_end);
	u32 byte_mask = 0;
	u32 incoming;
	u32 rw1c_bits, rw_bits, old, new;
	u32 addr;

	for (addr = copy_lo; addr < copy_hi; ++addr)
		byte_mask |= (u32)0xff << (8 * (addr - reg_base));

	incoming = reg_incoming(access_start, access_end, reg_base, reg_size,
				write_val);

	rw1c_bits = rw1c_mask & byte_mask;
	rw_bits = ~ro_mask & ~rw1c_mask & byte_mask;

	old = state->val;
	new = (old & ~rw_bits) | (incoming & rw_bits);
	new &= ~(incoming & rw1c_bits);
	return new;
}

/*
 * validate provided reg space.
 * - check if size is 1,2,4
 * - natural alignment by the PCIe spec (offset % size == 0)
 * - acending + non-overlap - otherwise reg_find_first() will fail
 */
static int reg_space_validate(const struct reg_entry *entries, size_t n_entries)
{
	size_t i;

	for (i = 0; i < n_entries; ++i) {
		const struct reg_entry *e = &entries[i];

		if (e->size != 1 && e->size != 2 && e->size != 4) {
			pr_err("reg entry %zu (offset 0x%x) has invalid size %u\n",
			       i, (u32)e->offset, e->size);
			return -EINVAL;
		}

		if ((u32)e->offset % e->size != 0) {
			pr_err("reg entry %zu (offset 0x%x) not naturally aligned to size %u\n",
			       i, (u32)e->offset, e->size);
			return -EINVAL;
		}

		if (i > 0) {
			const u32 prev_end =
				(u32)entries[i - 1].offset + entries[i - 1].size;

			if ((u32)e->offset < prev_end) {
				pr_err("reg entry %zu (offset 0x%x) not ascending / overlaps previous (prev_end 0x%x)\n",
				       i, (u32)e->offset, prev_end);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int reg_space_init(struct reg_space *space, const struct reg_entry *entries, size_t n_entries)
{
	if (reg_space_validate(entries, n_entries))
		return -EINVAL;

	space->states = kcalloc(n_entries, sizeof(*space->states), GFP_KERNEL);
	if (!space->states)
		return -ENOMEM;

	space->entries = entries;
	space->n_entries = n_entries;

	reg_set_default_values(space);

	return 0;
}

static int reg_space_destroy(struct reg_space *space)
{
	kfree(space->states);
	space->states	 = NULL;
	space->entries	 = NULL;
	space->n_entries = 0;
	return 0;
}

static void reg_copy_overlap(u32 *result, u32 access_start, u32 access_end,
			     u32 reg_base, u32 reg_size, u32 reg_val)
{
	const u32 reg_end = reg_base + reg_size;
	const u32 copy_lo = max(reg_base, access_start);
	const u32 copy_hi = min(reg_end, access_end);
	u32 addr;

	for (addr = copy_lo; addr < copy_hi; ++addr) {
		const u32 shift_in_reg = 8 * (addr - reg_base);
		const u32 shift_in_result = 8 * (addr - access_start);
		const u8 byte = (reg_val >> shift_in_reg) & 0xff;

		*result |= (u32)byte << shift_in_result;
	}
}

static int reg_read(struct reg_space *space, u32 offset, u32 size, u32 *val)
{
	const u32 access_start = offset;
	const u32 access_end = offset + size;
	u32 result = 0x0;	//retrn 0 for unimplemented regs
	size_t i;

	for (i = reg_find_first(space, access_start); i < space->n_entries; ++i) {
		const struct reg_entry *e = &space->entries[i];
		const u32 reg_base = e->offset;

		if (reg_base >= access_end)
			break;

		//TODO: needs some locking? maybe on dispatcher level
		/* read handler (if any) supplies the value, bypassing state */
		u32 cur = e->read_handler
			? e->read_handler(space->dev, (struct reg_entry *)e,
					  &space->states[i])
			: space->states[i].val;

		reg_copy_overlap(&result, access_start, access_end,
				 reg_base, e->size, cur);
	}

	*val = result;
	return PCIBIOS_SUCCESSFUL;
}

static int reg_write(struct reg_space *space, u32 offset, u32 size, u32 val)
{
	const u32 access_start = offset;
	const u32 access_end = offset + size;
	size_t i;

	for (i = reg_find_first(space, access_start); i < space->n_entries; ++i) {
		const struct reg_entry *e = &space->entries[i];
		const u32 reg_base = e->offset;

		if (reg_base >= access_end)
			break;

		u32 filtered_val = reg_filter_write(&space->states[i], access_start,
						    access_end, reg_base, e->size,
						    e->ro_mask, e->rw1c_mask, val);

		if (e->write_handler)
			e->write_handler(space->dev, (struct reg_entry *)e,
					 &space->states[i], filtered_val);
		else
			space->states[i].val = filtered_val;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int reg_write_direct(struct reg_space *space, u32 offset, u32 size, u32 val)
{
	size_t i = reg_find_first(space, offset);

	if (i >= space->n_entries)
		return -EINVAL;

	const struct reg_entry *e = &space->entries[i];

	if ((u32)e->offset != offset || e->size != size)
		return -EINVAL;

	switch(size)
	{
	case 1: space->states[i].val = val & 0xFF; break;
	case 2: space->states[i].val = val & 0xFFFF; break;
	case 4: space->states[i].val = val; break;
	default: return EINVAL;
	}

	return 0;
}

static int reg_set_default_values(struct reg_space *space)
{
	for(size_t i=0; i < space->n_entries; ++i)
		space->states[i].val = space->entries[i].default_val;
	return 0;
}

static int rc_reset(enum reset_type reset, struct vepc_dev *vepc)
{
	switch(reset)
	{
	case RESET_POWER_ON:
		reg_set_default_values(&vepc->rc_regs);
		break;
	default:
		pr_err("Not supported reset!\n");
		return -EINVAL;
	}
	return 0;
}

static int ep_init(struct vepc_dev *vepc)
{
	int rc = reg_space_init(&vepc->ep_regs, ep_regs_layout, ep_regs_layout_size);
	if(rc)
	{
		pr_err("reg_space_init() failed: %d\n", rc);
		return -ENOMEM;
	}
	vepc->ep_regs.dev = vepc;

	/*
	vepc->bar0_virt = ioremap(vepc->bar0_phys, vepc->bar0_size);
	if(!vepc->bar0_virt)
	{
		pr_err("ioremap failed!\n");
		rc = -EPERM;	//TODO: find a better error code
		goto reg_destroy;
	}
	memset_io(vepc->bar0_virt, 0, vepc->bar0_size);
	*/

	ep_reset(RESET_POWER_ON, vepc);
	vepc->enabled = true;
	return 0;

/*	
reg_destroy:
	reg_space_destroy(&vepc->ep_regs);
	return rc;
	*/
}

static int ep_exit(struct vepc_dev *vepc)
{
	int rc = reg_space_destroy(&vepc->ep_regs);
	if(rc)
		pr_err("reg_space_destroy() failed: %d\n", rc);
	/*
	if(vepc->bar0_virt)
	{
		iounmap(vepc->bar0_virt);
		vepc->bar0_virt = NULL;
	}
	*/
	vepc->enabled = false;

	return 0;
}

static int ep_hotplug(struct vepc_dev *vepc)
{
	int rc = ep_init(vepc);
	if(rc)
	{
		pr_err("ep_init() failed\n");
		return rc;
	}
	
	clear_access_filter(vepc, ACC_F_EP_UR);

	u32 link;
	rc_reg_read(vepc, 0x52, 2, &link);	//TODO: rc check
	link |= PCI_EXP_LNKSTA_DLLLA;
	reg_write_direct(&vepc->rc_regs, 0x52, 2, link);

	u32 slot;
	rc_reg_read(vepc, 0x54, 2, &slot);
	slot |= PCI_EXP_SLTSTA_PDS | PCI_EXP_SLTSTA_PDC | PCI_EXP_SLTSTA_DLLSC;
	reg_write_direct(&vepc->rc_regs, 0x54, 2, slot);

	pci_lock_rescan_remove();
	pci_rescan_bus(vepc->bridge->bus);
	pci_unlock_rescan_remove();

	msi_hotplug_irq(vepc);
	return 0;
}

static int ep_hotremove(struct vepc_dev *vepc)
{
	struct pci_dev *rp = pci_get_slot(vepc->bridge->bus, PCI_DEVFN(0,0));
	struct pci_bus *bus1 = rp ? rp->subordinate : NULL;
	
	set_access_filter(vepc, ACC_F_EP_UR);

	struct pci_dev *ep = bus1 ? pci_get_slot(bus1, PCI_DEVFN(0,0)) : NULL;
	if(ep)
	{
		WRITE_ONCE(ep->error_state, pci_channel_io_perm_failure); //hack to drop io instantly
		pci_stop_and_remove_bus_device_locked(ep);
		pci_dev_put(ep);
	}
	pci_dev_put(rp);

	u32 link;
	rc_reg_read(vepc, 0x52, 2, &link);	//TODO: rc check
	link &= ~(PCI_EXP_LNKSTA_DLLLA);
	reg_write_direct(&vepc->rc_regs, 0x52, 2, link);

	u32 slot;
	rc_reg_read(vepc, 0x54, 2, &slot);
	slot &= ~(PCI_EXP_SLTSTA_PDS);
	slot |= PCI_EXP_SLTSTA_PDC | PCI_EXP_SLTSTA_DLLSC;
	reg_write_direct(&vepc->rc_regs, 0x54, 2, slot);

	ep_exit(vepc);

	msi_hotplug_irq(vepc);
	return 0;
}

static int ep_reset(enum reset_type reset, struct vepc_dev *vepc)
{
	switch(reset)
	{
	case RESET_POWER_ON:
		reg_set_default_values(&vepc->ep_regs);
		break;
		/*
	case RESET_PCIE_LINK_RESET:
		break;
	case RESET_PCIE_FUNCTION_RESET:
		break;
		*/
	default:
		pr_err("Not supported yet!\n");
		return -EINVAL;
		break;
	}
	return 0;
}

static int ep_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val)
{
	if(vepc->access_filter & ACC_F_EP_UR)
	{
		*val = 0xFFFFFFFF;
		return PCIBIOS_SUCCESSFUL;
	}
	if(vepc->access_filter & ACC_F_EP_CRS)
	{
		*val = 0xFFFF0001;
		return PCIBIOS_SUCCESSFUL;
	}
	return reg_read(&vepc->ep_regs, where, size, val);
}

static int ep_reg_write(struct vepc_dev *vepc, int where, int size, u32 val)
{
	if((vepc->access_filter & ACC_F_EP_UR) || (vepc->access_filter & ACC_F_EP_UR))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return reg_write(&vepc->ep_regs, where, size, val);
}

static int rc_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val)
{
	if(vepc->access_filter & ACC_F_RC_UR)
	{
		*val = 0xFFFFFFFF;
		return PCIBIOS_SUCCESSFUL;
	}
	if(vepc->access_filter & ACC_F_RC_CRS)
	{
		*val = 0xFFFF0001;
		return PCIBIOS_SUCCESSFUL;
	}
	return reg_read(&vepc->rc_regs, where, size, val);
}

static int rc_reg_write(struct vepc_dev *vepc, int where, int size, u32 val)
{
	if((vepc->access_filter & ACC_F_RC_UR) || (vepc->access_filter & ACC_F_RC_UR))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return reg_write(&vepc->rc_regs, where, size, val);
}

static void msi_compose_msg(struct irq_data *d, struct msi_msg *msg)
{
	/* token only — delivery is via generic_handle_irq_safe(), never a real MSI write */
	msg->address_lo = 0;
	msg->address_hi = 0;
	msg->data = d->hwirq;
}

static struct irq_chip msi_bottom_chip = {	/* NOT const: DEF_CHIP_OPS patches it in place */
	.name = "vEPC-MSI",
	.irq_compose_msi_msg = msi_compose_msg,
};

static int msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *arg)
{
	unsigned long bit;
	struct vepc_dev *vepc = domain->host_data;

	WARN_ON(nr_irqs != 1);

	mutex_lock(&vepc->msi_lock);
	bit = find_first_zero_bit(vepc->msi_used, MSI_NR_VECTORS);
	if (bit >= MSI_NR_VECTORS) {
		mutex_unlock(&vepc->msi_lock);
		return -ENOSPC;
	}
	set_bit(bit, vepc->msi_used);
	mutex_unlock(&vepc->msi_lock);

	irq_domain_set_info(domain, virq, bit, &msi_bottom_chip,
			    domain->host_data, handle_simple_irq, NULL, NULL);

	msi_alloc_info_t *info = arg;
        struct msi_desc *desc = info ? info->desc : NULL;

        if (desc && dev_is_pci(desc->dev)) {
                struct pci_dev *pdev = to_pci_dev(desc->dev);

                /* root port == bus 0; hot-plug vector == Interrupt Message Number 0 */
                if (pdev->bus->number == 0 && desc->msi_index == 0)
                        WRITE_ONCE(vepc->slot_irq, virq);
        }

	return 0;
}

static void msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct vepc_dev *vepc = domain->host_data;

	mutex_lock(&vepc->msi_lock);
	__clear_bit(d->hwirq, vepc->msi_used);
	mutex_unlock(&vepc->msi_lock);
}

static const struct irq_domain_ops msi_domain_ops = {
	.alloc = msi_domain_alloc,
	.free  = msi_domain_free,
};

#define VEPC_MSI_FLAGS_REQUIRED  (MSI_FLAG_USE_DEF_DOM_OPS | \
				   MSI_FLAG_USE_DEF_CHIP_OPS | \
				   MSI_FLAG_NO_AFFINITY)
#define VEPC_MSI_FLAGS_SUPPORTED (MSI_GENERIC_FLAGS_MASK | \
				   MSI_FLAG_PCI_MSIX)

static bool init_dev_msi_info(struct device *dev, struct irq_domain *domain,
				    struct irq_domain *real_parent,
				    struct msi_domain_info *info)
{
	const struct msi_parent_ops *pops = real_parent->msi_parent_ops;

	if (WARN_ON_ONCE(!pops))
		return false;

	/* We are the root MSI parent. */
	if (domain->bus_token == pops->bus_select_token) {
		if (WARN_ON_ONCE(domain != real_parent))
			return false;
	} else {
		WARN_ON_ONCE(1);
		return false;
	}

	/* Only PCI MSI / MSI-X child domains are expected. */
	switch (info->bus_token) {
	case DOMAIN_BUS_PCI_DEVICE_MSI:
	case DOMAIN_BUS_PCI_DEVICE_MSIX:
		if (WARN_ON_ONCE(!IS_ENABLED(CONFIG_PCI_MSI)))
			return false;
		break;
	default:
		WARN_ON_ONCE(1);
		return false;
	}

	/* Mask to parent-supported flags, then enforce the required ones. */
	info->flags &= pops->supported_flags;
	info->flags |= pops->required_flags;

	return true;
}

static const struct msi_parent_ops msi_parent_ops = {
	.required_flags    = VEPC_MSI_FLAGS_REQUIRED,
	.supported_flags   = VEPC_MSI_FLAGS_SUPPORTED,
	.bus_select_token  = DOMAIN_BUS_PCI_MSI,
	.prefix            = "vEPC-",
	.init_dev_msi_info = init_dev_msi_info,
};

static int msi_domain_create(struct vepc_dev *vepc, struct device *bridge_dev)
{
	struct irq_domain_info info = {
		.fwnode    = irq_domain_alloc_named_id_fwnode("vEPC-MSI", vepc->sysdata.domain),
		.ops       = &msi_domain_ops,
		.host_data = vepc,
		.size      = MSI_NR_VECTORS,
	};

	if (!info.fwnode)
		return -ENOMEM;

	mutex_init(&vepc->msi_lock);

	vepc->msi_fwnode = info.fwnode;
	vepc->msi_domain = msi_create_parent_irq_domain(&info, &msi_parent_ops);
	if (!vepc->msi_domain) {	/* returns NULL (not ERR_PTR) on failure */
		irq_domain_free_fwnode(vepc->msi_fwnode);
		vepc->msi_fwnode = NULL;
		return -ENOMEM;
	}

	dev_set_msi_domain(bridge_dev, vepc->msi_domain);
	return 0;
}

static void msi_domain_destroy(struct vepc_dev *vepc)
{
	if (vepc->msi_domain) {
		irq_domain_remove(vepc->msi_domain);
		vepc->msi_domain = NULL;
	}
	if (vepc->msi_fwnode) {
		irq_domain_free_fwnode(vepc->msi_fwnode);
		vepc->msi_fwnode = NULL;
	}
	mutex_destroy(&vepc->msi_lock);
}


static void msi_hotplug_irq(struct vepc_dev *vepc)
{
	int virq = READ_ONCE(vepc->slot_irq);

	if (!virq) {
		pr_warn("hotplug IRQ skipped: root-port MSI not enabled\n");
		return;
	}
	generic_handle_irq_safe(virq);
}

static void set_access_filter(struct vepc_dev *vepc, enum acc_flags flags)
{
	vepc->access_filter |= flags;
}

static void clear_access_filter(struct vepc_dev *vepc, enum acc_flags flags)
{
	vepc->access_filter &= ~(flags);
}

static int bus_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct vepc_dev *vepc = container_of(nb, struct vepc_dev, pci_nb);
	struct device *dev = data;
	struct pci_dev *pdev;

	if(action != BUS_NOTIFY_ADD_DEVICE || !dev_is_pci(dev))
		return NOTIFY_DONE;

	pdev = to_pci_dev(dev);

	if(pdev->bus->sysdata != &vepc->sysdata ||
	   pdev->bus->number != 1 ||
	   PCI_FUNC(pdev->devfn) != 0)
		return NOTIFY_DONE;

	patch_resources(vepc, pdev);
	return NOTIFY_OK;
}

/*
 *	it's needed to make sure EP will use provided BAR0
 */
static void patch_resources(struct vepc_dev *vepc, struct pci_dev *pdev)
{

	if(!pdev || PCI_FUNC(pdev->devfn) != 0)
		return;

	if(pdev->bus->number == 0x0)
	{
		//TODO:some patching for root complex
		return;
	}
	if(pdev->bus->number == 0x1)	//we have private domain, endpoint will always be 0x1 //TODO: magic num
	{
		struct resource *r = &pdev->resource[0];

		r->start = vepc->bar0_phys;
		r->end = vepc->bar0_phys + vepc->bar0_size - 1;
		r->flags &= ~IORESOURCE_UNSET;
		r->flags |= IORESOURCE_MEM | IORESOURCE_MEM_64 | IORESOURCE_PREFETCH | IORESOURCE_PCI_FIXED;
	}
}

module_init(vepc_init_module);
module_exit(vepc_exit_module);

MODULE_DESCRIPTION("Virtual PCI Endpoint Controller for NVMe target driver");
MODULE_AUTHOR("Mateusz Nowicki <mateusz.nowicki@posteo.net>");
MODULE_LICENSE("GPL");
