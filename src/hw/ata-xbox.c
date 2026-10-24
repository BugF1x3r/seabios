// Xbox (MCPX, nForce IDE 10DE:01BC): transfer-mode negotiation for the OS.
//
// Copyright (C) 2026  XBIOS contributors
//
// This file may be distributed under the terms of the GNU LGPLv3 license.
//
// SeaBIOS itself transfers in PIO and never asks a drive for a mode. The
// operating systems XBIOS boots keep whatever the firmware set up: Win98's
// inbox ESDI_506 and Linux pata_amd both leave the MCPX timing registers
// alone, and ESDI_506 uses the mode the drive is in once its DMA box is
// ticked. So, once a drive answered IDENTIFY, pick the fastest mode the
// drive, the controller and the cable allow, tell the drive (SET FEATURES
// 03h, PIO 4 first, then the DMA mode), program the controller's UDMA
// timing byte for that drive to match, and read IDENTIFY again to confirm.
//
// Register map of the MCPX IDE function (the nForce layout, from Linux's
// pata_amd.c, 33 MHz clock, UDMA at half clocks; offset 0x50):
//   0x50  bit 1 primary channel enable, bit 0 secondary (XBIOS: 2)
//   0x58-0x5B  PIO/MWDMA drive timing, byte 3 - dn, dn = channel*2 + drive:
//              (active clocks - 1) << 4 | (recover clocks - 1); PIO 4 = 0x20
//   0x5C  address setup, 2 bits per drive (bits 7:6 = primary master)
//   0x5E/0x5F  8-bit timing secondary/primary, same encoding
//   0x60-0x63  UDMA timing, byte 3 - dn: 0xC0 | code enables UDMA, 0x03 off;
//              code 2 = UDMA0, 1 = UDMA1, 0 = UDMA2, 4 = UDMA3, 5 = UDMA4,
//              6 = UDMA5 (Linux limits the drive to the mode the firmware
//              left in this byte: nv_mode_filter)
// The retail kernel and Cromwell leave 0x60 = 0xC0C0C0C0 (UDMA2 on every
// slot); that stays the default until a drive negotiates something else.

#include "ata.h" // struct atadrive_s
#include "biosvar.h" // GET_GLOBALFLAT
#include "output.h" // dprintf
#include "pci.h" // pci_config_writeb
#include "string.h" // memset

// SET FEATURES 03h transfer-mode values
#define XFER_PIO_DEFAULT  0x00
#define XFER_PIO(n)       (0x08 | (n))
#define XFER_MWDMA(n)     (0x20 | (n))
#define XFER_UDMA(n)      (0x40 | (n))

#define MCPX_IDE_DRIVE_TIMING  0x58
#define MCPX_IDE_UDMA_TIMING   0x60
#define MCPX_IDE_UDMA_ENABLE   0xc0
#define MCPX_IDE_UDMA_OFF      0x03

// UDMA mode -> MCPX timing code (the nForce column of Linux's amd_cyc2udma
// at a 15 ns half clock: 120, 80, 60, 45, 30 ns cycles, UDMA5 special-cased)
static const u8 mcpx_udma_code[6] = { 2, 1, 0, 4, 5, 6 };

#define IDENTIFY_W53_W88_VALID   (1 << 2)
#define IDENTIFY_W53_W64_VALID   (1 << 1)
#define IDENTIFY_W93_80WIRE      (1 << 13)

static int
set_features_xfer(struct atadrive_s *adrive, u8 mode)
{
    struct ata_pio_command cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.feature = 0x03;
    cmd.sector_count = mode;
    cmd.command = ATA_CMD_SET_FEATURES;
    return ata_cmd_nondata(adrive, &cmd);
}

static int
highest_bit(u16 mask, int max)
{
    int n;
    for (n = max; n >= 0; n--)
        if (mask & (1 << n))
            return n;
    return -1;
}

// Called by ata_detect() once 'buffer' holds the drive's IDENTIFY data.
void
ata_xbox_set_mode(struct atadrive_s *adrive, u16 *buffer, int atapi)
{
    if (!CONFIG_XBOX)
        return;
    struct ata_channel_s *chan_gf = GET_GLOBALFLAT(adrive->chan_gf);
    int ataid = GET_GLOBALFLAT(chan_gf->ataid);
    int chanid = GET_GLOBALFLAT(chan_gf->chanid);
    int slave = GET_GLOBALFLAT(adrive->slave);
    int bdf = GET_GLOBALFLAT(chan_gf->pci_bdf);

    u16 w53 = buffer[53], w63 = buffer[63], w64 = buffer[64];
    u16 w88 = buffer[88], w93 = buffer[93];
    u16 udma_sup = (w53 & IDENTIFY_W53_W88_VALID) ? (w88 & 0x7f) : 0;
    u16 mwdma_sup = w63 & 0x07;
    int pio4 = (w53 & IDENTIFY_W53_W64_VALID) && (w64 & 0x02);
    // Word 93 bit 13 is the drive's own view of pin 34: set behind an
    // 80-conductor cable. ATAPI drives rarely report it; nothing on the
    // Xbox but the stock 40-wire ribbon sits between MCPX and drive, so
    // the slave inherits the master's reading when it has none.
    int cable80 = !!(w93 & IDENTIFY_W93_80WIRE);

    int cap = CONFIG_XBOX_IDE_UDMA_MAX;
    if (!cable80 && cap > CONFIG_XBOX_IDE_UDMA_40WIRE)
        cap = CONFIG_XBOX_IDE_UDMA_40WIRE;
    int udma = highest_bit(udma_sup, cap);
    int mwdma = udma < 0 ? highest_bit(mwdma_sup, 2) : -1;

    char *kind;
    int n;
    u8 mode;
    if (udma >= 0) {
        kind = "UDMA"; n = udma; mode = XFER_UDMA(udma);
    } else if (mwdma >= 0) {
        kind = "MWDMA"; n = mwdma; mode = XFER_MWDMA(mwdma);
    } else {
        kind = "PIO"; n = pio4 ? 4 : 0;
        mode = pio4 ? XFER_PIO(4) : XFER_PIO_DEFAULT;
    }

    dprintf(1, "xbox: ata%d-%d %s w53=%04x w63=%04x w64=%04x w88=%04x w93=%04x"
            " cable %d-wire, udma cap %d\n"
            , ataid, slave, atapi ? "atapi" : "ata", w53, w63, w64, w88, w93
            , cable80 ? 80 : 40, cap);

    // PIO 4 first (SeaBIOS's own transfers and the OS's fallback), then
    // the DMA mode: a device keeps one PIO and one DMA selection.
    // ata_cmd_nondata() returns the status byte on success, negative on error.
    int ret = 0;
    if (pio4 && mode != XFER_PIO(4))
        ret = set_features_xfer(adrive, XFER_PIO(4));
    if (ret >= 0)
        ret = set_features_xfer(adrive, mode);
    ret = ret < 0 ? ret : 0;

    // Controller side, this drive's slot: the UDMA timing byte, and the
    // PIO drive timing (active/recovery clocks of 30 ns) for a drive
    // without PIO 4: PIO 3 is 3+3 clocks (180 ns), PIO 0 10+11 (630 ns).
    int dn = chanid * 2 + slave;
    u8 timing = udma >= 0 ? (MCPX_IDE_UDMA_ENABLE | mcpx_udma_code[udma])
                          : MCPX_IDE_UDMA_OFF;
    u8 pio_timing = pio4 ? 0x20 : (w64 & 0x01) ? 0x22 : 0x9a;
    if (bdf >= 0 && !ret) {
        pci_config_writeb(bdf, MCPX_IDE_UDMA_TIMING + (3 - dn), timing);
        pci_config_writeb(bdf, MCPX_IDE_DRIVE_TIMING + (3 - dn), pio_timing);
    }

    // Confirm: IDENTIFY again and read the active-mode bits (word 88 bit
    // 8+n for UDMA, word 63 bit 8+n for MWDMA). PIO has no active bit.
    const char *confirm = "not confirmed";
    if (!ret) {
        u16 again[256];
        int cmd = atapi ? ATA_CMD_IDENTIFY_PACKET_DEVICE
                        : ATA_CMD_IDENTIFY_DEVICE;
        if (ata_identify(adrive, again, cmd) == 0) {
            int active;
            if (udma >= 0)
                active = (again[88] >> 8) & 0x7f;
            else if (mwdma >= 0)
                active = (again[63] >> 8) & 0x07;
            else
                active = -1;
            if (active < 0)
                confirm = "set";
            else if (active == (1 << n))
                confirm = "confirmed by IDENTIFY";
            else
                confirm = "IDENTIFY disagrees";
            dprintf(3, "xbox: ata%d-%d after SET FEATURES w63=%04x w88=%04x\n"
                    , ataid, slave, again[63], again[88]);
        }
    }
    dprintf(1, "xbox: ata%d-%d transfer mode %s%d %s, mcpx udma byte %02x"
            " pio byte %02x%s\n"
            , ataid, slave, kind, n
            , ret ? "SET FEATURES failed" : confirm
            , timing, pio_timing, bdf >= 0 && !ret ? "" : " (not written)");
}
