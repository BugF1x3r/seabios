// PCI config space access functions.
//
// Copyright (C) 2008  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2002  MandrakeSoft S.A.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "output.h" // dprintf
#include "pci.h" // pci_config_writel
#include "pci_regs.h" // PCI_VENDOR_ID
#include "util.h" // udelay
#include "x86.h" // outl

#define PORT_PCI_CMD           0x0cf8
#define PORT_PCI_DATA          0x0cfc

static u32 mmconfig;

static void *mmconfig_addr(u16 bdf, u32 addr)
{
    return (void*)(mmconfig + ((u32)bdf << 12) + addr);
}

static u32 ioconfig_cmd(u16 bdf, u32 addr)
{
    return 0x80000000 | (bdf << 8) | (addr & 0xfc);
}

// Xbox MCPX: config reads of 00:00.1, 00:00.2, any bus 1 device other
// than 0 and any bus 2 and up hang the console.  Every config access
// (enumeration, INT 1Ah B1xx, BIOS32) ends in one of the six port
// accessors below, so filtering here makes those addresses read as an
// absent device and swallows writes to them.
static int pci_mcpx_hazard(u16 bdf)
{
    if (!CONFIG_XBOX)
        return 0;
    u8 bus = pci_bdf_to_bus(bdf);
    if (bus == 0)
        return pci_bdf_to_dev(bdf) == 0
            && (pci_bdf_to_fn(bdf) == 1 || pci_bdf_to_fn(bdf) == 2);
    if (bus == 1)
        return pci_bdf_to_dev(bdf) != 0;
    return 1;
}

static void pci_mcpx_hazard_log(u16 bdf, u32 addr, const char *op)
{
    dprintf(5, "pci: MCPX hazard %02x:%02x.%x+%02x %s filtered\n"
            , pci_bdf_to_bus(bdf), pci_bdf_to_dev(bdf), pci_bdf_to_fn(bdf)
            , addr, op);
}

void pci_ioconfig_writel(u16 bdf, u32 addr, u32 val)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "write");
        return;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    outl(val, PORT_PCI_DATA);
}

void pci_config_writel(u16 bdf, u32 addr, u32 val)
{
    if (!MODESEGMENT && mmconfig) {
        writel(mmconfig_addr(bdf, addr), val);
    } else {
        pci_ioconfig_writel(bdf, addr, val);
    }
}

void pci_ioconfig_writew(u16 bdf, u32 addr, u16 val)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "write");
        return;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    outw(val, PORT_PCI_DATA + (addr & 2));
}

void pci_config_writew(u16 bdf, u32 addr, u16 val)
{
    if (!MODESEGMENT && mmconfig) {
        writew(mmconfig_addr(bdf, addr), val);
    } else {
        pci_ioconfig_writew(bdf, addr, val);
    }
}

void pci_ioconfig_writeb(u16 bdf, u32 addr, u8 val)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "write");
        return;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    outb(val, PORT_PCI_DATA + (addr & 3));
}

void pci_config_writeb(u16 bdf, u32 addr, u8 val)
{
    if (!MODESEGMENT && mmconfig) {
        writeb(mmconfig_addr(bdf, addr), val);
    } else {
        pci_ioconfig_writeb(bdf, addr, val);
    }
}

u32 pci_ioconfig_readl(u16 bdf, u32 addr)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "read");
        return (u32)~0;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    return inl(PORT_PCI_DATA);
}

u32 pci_config_readl(u16 bdf, u32 addr)
{
    if (!MODESEGMENT && mmconfig) {
        return readl(mmconfig_addr(bdf, addr));
    } else {
        return pci_ioconfig_readl(bdf, addr);
    }
}

u16 pci_ioconfig_readw(u16 bdf, u32 addr)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "read");
        return (u16)~0;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    return inw(PORT_PCI_DATA + (addr & 2));
}

u16 pci_config_readw(u16 bdf, u32 addr)
{
    if (!MODESEGMENT && mmconfig) {
        return readw(mmconfig_addr(bdf, addr));
    } else {
        return pci_ioconfig_readw(bdf, addr);
    }
}

u8 pci_ioconfig_readb(u16 bdf, u32 addr)
{
    if (pci_mcpx_hazard(bdf)) {
        pci_mcpx_hazard_log(bdf, addr, "read");
        return (u8)~0;
    }
    outl(ioconfig_cmd(bdf, addr), PORT_PCI_CMD);
    return inb(PORT_PCI_DATA + (addr & 3));
}

u8 pci_config_readb(u16 bdf, u32 addr)
{
    if (!MODESEGMENT && mmconfig) {
        return readb(mmconfig_addr(bdf, addr));
    } else {
        return pci_ioconfig_readb(bdf, addr);
    }
}

void
pci_config_maskw(u16 bdf, u32 addr, u16 off, u16 on)
{
    u16 val = pci_config_readw(bdf, addr);
    val = (val & ~off) | on;
    pci_config_writew(bdf, addr, val);
}

void
pci_enable_mmconfig(u64 addr, const char *name)
{
    if (addr >= 0x100000000ll)
        return;
    dprintf(1, "PCIe: using %s mmconfig at 0x%llx\n", name, addr);
    mmconfig = addr;
}

u8 pci_find_capability(u16 bdf, u8 cap_id, u8 cap)
{
    int i;
    u16 status = pci_config_readw(bdf, PCI_STATUS);

    if (!(status & PCI_STATUS_CAP_LIST))
        return 0;

    if (cap == 0) {
        /* find first */
        cap = pci_config_readb(bdf, PCI_CAPABILITY_LIST);
    } else {
        /* find next */
        cap = pci_config_readb(bdf, cap + PCI_CAP_LIST_NEXT);
    }
    for (i = 0; cap && i <= 0xff; i++) {
        if (pci_config_readb(bdf, cap + PCI_CAP_LIST_ID) == cap_id)
            return cap;
        cap = pci_config_readb(bdf, cap + PCI_CAP_LIST_NEXT);
    }

    return 0;
}

// Helper function for pci_ioconfig_foreachbdf() macro - return next device
int pci_ioconfig_next(int bdf, int bus)
{
    if (pci_bdf_to_fn(bdf) == 0
        && (pci_ioconfig_readb(bdf, PCI_HEADER_TYPE) & 0x80) == 0)
        // Last found device wasn't a multi-function device - skip to
        // the next device.
        bdf += 8;
    else
        bdf += 1;

    for (;;) {
        if (pci_bdf_to_bus(bdf) != bus)
            return -1;

        u16 v = pci_ioconfig_readw(bdf, PCI_VENDOR_ID);
        if (v != 0x0000 && v != 0xffff)
            // Device is present.
            return bdf;

        if (pci_bdf_to_fn(bdf) == 0)
            bdf += 8;
        else
            bdf += 1;
    }
}

// Helper function for foreachbdf() macro - return next device
int
pci_next(int bdf, int bus)
{
    if (pci_bdf_to_fn(bdf) == 0
        && (pci_config_readb(bdf, PCI_HEADER_TYPE) & 0x80) == 0)
        // Last found device wasn't a multi-function device - skip to
        // the next device.
        bdf += 8;
    else
        bdf += 1;

    for (;;) {
        if (pci_bdf_to_bus(bdf) != bus)
            return -1;

        u16 v = pci_config_readw(bdf, PCI_VENDOR_ID);
        if (v != 0x0000 && v != 0xffff)
            // Device is present.
            return bdf;

        if (pci_bdf_to_fn(bdf) == 0)
            bdf += 8;
        else
            bdf += 1;
    }
}

// Check if PCI is available at all
int
pci_probe_host(void)
{
    outl(0x80000000, PORT_PCI_CMD);
    if (inl(PORT_PCI_CMD) != 0x80000000) {
        dprintf(1, "Detected non-PCI system\n");
        return -1;
    }
    return 0;
}

void
pci_reboot(void)
{
    u8 v = inb(PORT_PCI_REBOOT) & ~6;
    outb(v|2, PORT_PCI_REBOOT); /* Request hard reset */
    udelay(50);
    outb(v|6, PORT_PCI_REBOOT); /* Actually do the reset */
    udelay(50);
}
