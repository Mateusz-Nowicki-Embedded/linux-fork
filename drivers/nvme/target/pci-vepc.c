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
	u16 endpoint_vid;
	u16 endpoint_did;
	u64 bar0_phys;
	u64 bar0_size;
	void __iomem *bar0_virt;
	struct reg_space endpoint_regs;
	struct pci_epc *epc;


	/* PCIe switch */
	u16 switch_vid;
	u16 switch_did;
	struct pci_host_bridge *bridge;
	struct platform_device *plat_dev;
	struct pci_sysdata sysdata;
	struct resource bus_res;
	struct resource mem_res;
	struct reg_space switch_regs;

	struct irq_domain    *msi_domain;
	struct fwnode_handle *msi_fwnode;
	DECLARE_BITMAP(msi_used, MSI_NR_VECTORS);
	struct mutex msi_lock;
	int slot_irq;
	struct notifier_block pci_nb;

	/* for both */
	u32 access_filter;
};

struct vepc_dev *vepc_dev;

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
	}
	else
	{
		pr_info("disabling...\n");
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
	return 0;
}

static void __exit vepc_exit_module(void)
{
	configfs_unregister_subsystem(&vepc_cfg_subsys);
}

module_init(vepc_init_module);
module_exit(vepc_exit_module);

MODULE_DESCRIPTION("Virtual PCI Endpoint Controller for NVMe target driver");
MODULE_AUTHOR("Mateusz Nowicki <mateusz.nowicki@posteo.net>");
MODULE_LICENSE("GPL");
