#include <linux/pci.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/compiler_types.h>
#include <linux/kthread.h>
#include <linux/pci-epc.h>

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
