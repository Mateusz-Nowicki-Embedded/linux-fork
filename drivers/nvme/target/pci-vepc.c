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
	u32 bar0_size;
	void __iomem *bar0_virt;
	struct reg_space ep_regs;
	struct pci_epc *epc;


	/* PCIe switch */
	u16 rc_vid;
	u16 rc_did;
	struct pci_host_bridge *bridge;
	struct platform_device *plat_dev;
	struct pci_sysdata sysdata;
	struct resource bus_res;
	struct resource mem_res;
	struct reg_space rc_regs;

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

static int rc_int(struct vepc_dev *vepc);
static int rc_exit(struct vepc_dev *vepc);
static int rc_hotplug(struct vepc_dev *vepc);
static int rc_hotremove(struct vepc_dev *vepc);
static int rc_reset(enum reset_type reset, struct vepc_dev *vepc);

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


/* 1MiB-aligned helper */
#define RC_MEM_BASE(phys) ((phys) & ~((1ULL << 20) - 1))
#define RC_MEM_LIMIT(phys) (RC_MEM_BASE(phys) + (1ULL << 20) - 1)

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

static int pci_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	struct vepc_dev *vepc = container_of(bus->sysdata, struct vepc_dev, sysdata);
	*val = 0xFFFFFF; //TODO: better define needed
	
	if(size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;	//support only 1,2,4

	if(devfn != PCI_DEVFN(0,0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus->number == 0)
		rc_reg_read(vepc, where, size, val);
	else if(bus->number == 1)
		ep_reg_read(vepc, where, size, val);
	
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

static ssize_t vepc_cfg_hotplug_store(struct config_item *item, const char *page, size_t len)
{
	bool plug;
	if(kstrtobool(page, &plug))
		return -EINVAL;
	if(!plug)
		return -EINVAL;

	pr_info("hotplug requested!\n");

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

	pr_info("hotremoval requested!\n");

	return len;
}
CONFIGFS_ATTR_WO(vepc_cfg_, hotremove);


static DEFINE_MUTEX(cfg_lock);
static u16 rc_vid;
static u16 rc_pid;
static u16 ep_vid;
static u16 ep_pid;
static u64 bar0_phys;
static u32 bar0_size;

static bool verify_pids_vids(void)
{
	if(!rc_vid)
	{
		pr_err("rc_vid is empty!\n");
		return false;
	}

	if(!rc_pid)
	{
		pr_err("rc_pid is empty!\n");
		return false;
	}
	if(!ep_vid)
	{
		pr_err("ep_vid is empty!\n");
		return false;
	}
	if(!ep_pid)
	{
		pr_err("ep_pid is empty!\n");
		return false;
	}
	
	return true;
}

static bool verify_bar0(void)
{
	if(!is_power_of_2(bar0_size))
	{
		pr_err("bar0_size needs to be power of two!\n");
		return false;
	}
	if(bar0_size < 64*1024)	//TODO: magic value
	{
		pr_err("bar0_size needs at least 64K bytes!\n");
		return false;
	}
	if(bar0_size > 1 * 1024 * 1024) //TODO: magic value
	{
		pr_err("bar0_size can't be larger than 1MB!\n");
		return false;
	}
	if(!IS_ALIGNED(bar0_phys, bar0_size))
	{
		pr_err("bar0_phys is NOT aligned with bar0_size!\n");
		return false;
	}
	return true;
}

static ssize_t vepc_cfg_enable_store(struct config_item *item, const char *page, size_t len)
{
	bool enable;
	if(kstrtobool(page, &enable))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	pr_info("enable = %d!\n", enable);
	if(enable)
	{
		if(!verify_pids_vids())
			return -EINVAL;
		if(!verify_bar0())
			return -EINVAL;

		//enable controller
		pr_info("enabling...\n");
		rc_hotplug(vepc_dev);
	}
	else
	{
		pr_info("disabling...\n");
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

static ssize_t vepc_cfg_rc_pid_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", rc_pid);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_rc_pid_store(struct config_item *item, const char *page, size_t len)
{
	u16 pid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &pid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	rc_pid = pid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, rc_pid);

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

static ssize_t vepc_cfg_ep_pid_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", ep_pid);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_ep_pid_store(struct config_item *item, const char *page, size_t len)
{
	u16 pid;
	if(!len)
		return -EINVAL;
	if(kstrtou16(page, 0, &pid))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	ep_pid = pid;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, ep_pid);

static ssize_t vepc_cfg_bar0_phys_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%llx\n", bar0_phys);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_bar0_phys_store(struct config_item *item, const char *page, size_t len)
{
	u64 bar0;
	if(!len)
		return -EINVAL;
	if(kstrtou64(page, 0, &bar0))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	bar0_phys = bar0;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, bar0_phys);

static ssize_t vepc_cfg_bar0_size_show(struct config_item *item, char *page)
{
	ssize_t ret;
	mutex_lock(&cfg_lock);
	ret = sysfs_emit(page, "0x%x\n", bar0_size);
	mutex_unlock(&cfg_lock);

	return ret;
}

static ssize_t vepc_cfg_bar0_size_store(struct config_item *item, const char *page, size_t len)
{
	u32 bar0;
	if(!len)
		return -EINVAL;
	if(kstrtou32(page, 0, &bar0))
		return -EINVAL;

	mutex_lock(&cfg_lock);
	bar0_size = bar0;
	mutex_unlock(&cfg_lock);

	return len;
}
CONFIGFS_ATTR(vepc_cfg_, bar0_size);

static struct configfs_attribute *vepc_cfg_attrs[] = {
	&vepc_cfg_attr_hotplug,
	&vepc_cfg_attr_hotremove,
	&vepc_cfg_attr_enable,
	&vepc_cfg_attr_rc_vid,
	&vepc_cfg_attr_rc_pid,
	&vepc_cfg_attr_ep_vid,
	&vepc_cfg_attr_ep_pid,
	&vepc_cfg_attr_bar0_phys,
	&vepc_cfg_attr_bar0_size,
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

	rc = rc_int(vepc_dev);
	if(rc)
		goto vepc_dev_relese;

	return 0;

vepc_dev_relese:
	kfree(vepc_dev);
	vepc_dev = NULL;

group_relese:
	configfs_unregister_subsystem(&vepc_cfg_subsys);

	return rc;
}

static void __exit vepc_exit_module(void)
{
	if(vepc_dev)
	{
		configfs_unregister_subsystem(&vepc_cfg_subsys);
		rc_exit(vepc_dev);
	}
}

static int rc_int(struct vepc_dev *vepc)
{
	int rc;

	rc = reg_space_init(&vepc->rc_regs, rc_regs_layout, rc_regs_layout_size);
	if(rc)
		return rc;
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
		.name	= "vrc-busn",
		.start	= 0x00,
		.end	= 0xFF,
		.flags	= IORESOURCE_BUS,
	};
	pci_add_resource(&bridge->windows, &vepc->bus_res);

	vepc->mem_res	= (struct resource) {
		.name	= "vrc-mem",
		.start	= RC_MEM_BASE(vepc->bar0_phys),
		.end	= RC_MEM_LIMIT(vepc->bar0_phys),
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
	bus_register_notifier(&pci_bus_type, &vepc->pci_nb);

	rc = pci_scan_root_bus_bridge(bridge);
	if(rc)
	{
		pr_err("pci_scan_root_bus_bridge() failed: %d\n", rc);
		goto destroy_msi;
	}

	pci_bus_size_bridges(bridge->bus);
	pci_bus_assign_resources(bridge->bus);
	vepc->bridge = bridge;

	return 0;

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
	platform_device_unregister(vepc->plat_dev);
	vepc->plat_dev = NULL;
	reg_space_destroy(&vepc->rc_regs);
	return 0;
}

static int rc_hotplug(struct vepc_dev *vepc)
{
	return 0;
}

static int rc_hotremove(struct vepc_dev *vepc)
{
	return 0;
}

static int reg_space_init(struct reg_space *space, const struct reg_entry *entries, size_t n_entries)
{
	return 0;
}

static int reg_space_destroy(struct reg_space *space)
{
	return 0;
}

static int reg_read(struct reg_space *space, u32 offset, u32 size, u32 *val)
{
	return 0;
}

static int reg_write(struct reg_space *space, u32 offset, u32 size, u32 val)
{
	return 0;
}

static int reg_write_direct(struct reg_space *space, u32 offset, u32 size, u32 val)
{
	return 0;
}

static int reg_set_default_values(struct reg_space *space)
{
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

static int ep_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val)
{
	return 0;
}

static int ep_reg_write(struct vepc_dev *vepc, int where, int size, u32 val)
{
	return 0;
}

static int rc_reg_read(struct vepc_dev *vepc, int where, int size, u32 *val)
{
	return 0;
}

static int rc_reg_write(struct vepc_dev *vepc, int where, int size, u32 val)
{
	return 0;
}

static int msi_domain_create(struct vepc_dev *vepc, struct device *bridge)
{
	return 0;
}

static void msi_domain_destroy(struct vepc_dev *vepc)
{

}

static void msi_hotplug_irq(struct vepc_dev *vepc)
{

}

static void set_access_filter(struct vepc_dev *vepc, enum acc_flags flags)
{

}

static void clear_access_filter(struct vepc_dev *vepc, enum acc_flags flags)
{

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
