// Legacy VGA memory on a coreboot framebuffer.
//
// With CONFIG_VGA_FBSYNC the text modes (0-3, 7) and mode 13h keep their
// video memory where a real VGA has it: text cells at B8000 (B0000 for mode
// 7), 320x200 packed pixels at A0000. The vgabios writes that memory as the
// stdvga code would, and so can any program; this module renders it into
// the linear framebuffer coreboot describes. A hash per row of video memory
// finds the rows that changed; each changed row is drawn with the 8x16
// font into a staging line in low RAM and moved to the framebuffer with
// int 15h/87h (memcpy_high), as the rest of the cbvga code does. The render
// runs after every int 10h call that may have changed the screen and from
// the hooked timer irq (18.2 Hz), which is what shows direct writes.
//
// Mode 13h is line-doubled into a 640x400 picture centred on a 640x480
// framebuffer; the DAC is kept here (int 10h ah=10h), so palettes written
// through port 3C8h/3C9h are not seen. Mode 12h is the framebuffer itself
// (int 10h drawing works, direct A0000 writes do not).
//
// On the Xbox console (the init stage's XBOX_VGA coreboot table tag) mode
// 13h is native instead: the NV2A's VGA core scans the page at A0000 out of
// RAM itself (VGA format, byte mode, pixel and line doubling, centred on
// the 480-line raster) through its hardware DAC, so a program's direct
// writes and its own palette writes to ports 3C8h/3C9h show unchanged. The
// vgabios reaches the NV2A's registers through int 15h/87h one-word copies
// (16-bit MMIO writes) and puts the 32 bpp register set back when 13h ends.
// The ports (VSE2) are open only in 13h; text and VBE modes keep them shut.
//
// The renderer stays idle until the OS or a boot loader draws through int
// 10h, so a platform console drawn on the framebuffer before that survives.
// int 10h ax=4F14h bl=0 switches every framebuffer write of this vgabios
// off (bl=1 on, bl=2 reports in bh) for the time a Windows DOS box may run:
// there the BIOS writes the VM's own video memory and must not touch the
// screen, and no query tells the BIOS which VM it runs in (int 2Fh is not
// callable from a BIOS running on DOS's own stack).
//
// Copyright (C) 2026 Marco Janssen Groesbeek
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "bregs.h" // struct bregs
#include "output.h" // dprintf
#include "stdvga.h" // SEG_CTEXT
#include "string.h" // memset_far
#include "util.h" // find_cb_table
#include "hw/pci.h" // pci_config_readl
#include "x86.h" // outb
#include "vgabios.h" // get_current_mode
#include "vgafb.h" // memcpy_high
#include "vgautil.h" // VBE_framebuffer

#define FBSYNC_LINE 640          // pixels per staged scanline
#define FBSYNC_SRC_ROWS 200      // mode 13h source rows
#define FBSYNC_TEXT_ROWS 60      // text rows tracked (8x8 font on 480 lines)
#define FBSYNC_MARGIN13 40       // lines above and below the 13h picture

// Runtime state, in low RAM allocated through PMM at POST (the ROM image
// may be read-only once the OS runs).
struct fbsync_s {
    u32 line[FBSYNC_LINE * 2];   // two framebuffer scanlines (13h doubles)
    u32 pal[256];                // the DAC as 32 bpp xrgb
    u32 rowhash[FBSYNC_SRC_ROWS];// hash of each row as last rendered
    u8 dac[256 * 3];             // 6-bit DAC values
    u8 attr[16];                 // attribute palette: text colour -> DAC index
    u8 active;                   // the OS owns the screen: render
    u8 blink;                    // attribute bit 7 blinks (else 16 backgrounds)
    u8 busy;                     // a render is in progress
    u8 force;                    // repaint every row on the next render
    u8 below_cleared;            // framebuffer below the text rows is black
    u8 off;                      // int 10h 4F14h: no framebuffer writes at all
    u8 native;                   // the NV2A scans A0000 (native 13h is on)
    u8 dac_ports;                // DAC reads via 3C7h/3C9h: 0 untried, 1 ok, 2 not
    u32 io;                      // int 15h/87h source/destination for MMIO
};

u16 FbsyncSeg VAR16;

#define GET_FS(fs, var) GET_FARVAR((fs), ((struct fbsync_s *)0)->var)
#define SET_FS(fs, var, val) SET_FARVAR((fs), ((struct fbsync_s *)0)->var, (val))
#define FS_OFF(var) ((u32)&((struct fbsync_s *)0)->var)

// Mode records. Text modes at B8000/B0000 as on a VGA; 13h packed at A0000.
static struct vgamode_s fbsync_modes[] VAR16 = {
    { MM_TEXT, 40, 25, 4, 8, 16, SEG_CTEXT },    // 0, 1
    { MM_TEXT, 80, 25, 4, 8, 16, SEG_CTEXT },    // 2, 3
    { MM_TEXT, 80, 25, 4, 8, 16, SEG_MTEXT },    // 7
    { MM_PACKED, 320, 200, 8, 8, 8, SEG_GRAPH }, // 13h
};

struct vgamode_s *
fbsync_find_mode(int mode)
{
    switch (mode) {
    case 0x00: case 0x01: return &fbsync_modes[0];
    case 0x02: case 0x03: return &fbsync_modes[1];
    case 0x07: return &fbsync_modes[2];
    case 0x13: return &fbsync_modes[3];
    }
    return NULL;
}

static int
is_fbsync_mode(struct vgamode_s *vmode_g)
{
    return (vmode_g >= &fbsync_modes[0]
            && vmode_g < &fbsync_modes[ARRAY_SIZE(fbsync_modes)]);
}

// Bytes per row of video memory, -1 for modes this module does not own.
int
fbsync_linelength(struct vgamode_s *vmode_g)
{
    if (!is_fbsync_mode(vmode_g))
        return -1;
    if (GET_GLOBAL(vmode_g->memmodel) == MM_TEXT)
        return GET_GLOBAL(vmode_g->width) * 2;
    return GET_GLOBAL(vmode_g->width);
}


/****************************************************************
 * Native mode 13h on the Xbox's NV2A
 ****************************************************************/

// The init stage's tag: the NV2A's registers and the 32 bpp values of the
// ones the 13h switch changes. Absent (mmio 0) off the console: xemu shows
// VGA format as garbage and keeps the copy path.
#define CB_TAG_XBOX_VGA 0x5856
struct cb_xbox_vga {
    u32 tag, size;
    u32 mmio;                    // NV2A registers (BAR0)
    u32 start;                   // PCRTC_START
    u32 fp_v[7];                 // PRAMDAC flat-panel vertical timing
    u8 misc, sr01, sr02, sr04, gr05, gr06;
    u8 ar10, ar11, ar12, ar13, ar14;
    u8 cr07, cr09, cr12, cr13, cr14, cr15, cr17, cr19, cr25, cr28;
    u8 pad[3];
} PACKED;

static struct cb_xbox_vga NvVga VAR16;

#define NV_PRMVIO   0x0c0000     // 3C2-3CF: misc, VSE2, sequencer, graphics
#define NV_PCRTC    0x600000
#define NV_PRMCIO   0x601000     // 3B4-3DA: CRTC, attribute, status
#define NV_PRAMDAC  0x680000
#define NV_PRMDIO   0x681000     // 3C6-3C9: DAC

#define NV13_LINES 400           // 200 rows, line-doubled
#define NV13_HEIGHT 480          // the raster it is centred on

// One 16-bit (len 2) or 32-bit (len 4) MMIO write through int 15h/87h, the
// only way to BAR0 from real mode or V86. A one-word copy writes both bytes
// in one cycle: index and data for the CRTC, sequencer and graphics
// controller. Never two words to an 8-bit register pair: a dword at 3C8h
// corrupts the DAC entry.
static void
nv_write(u16 fs, u32 reg, u32 val, int len)
{
    SET_FS(fs, io, val);
    memcpy_high((void*)(GET_GLOBAL(NvVga.mmio) + reg)
                , MAKE_FLATPTR(fs, FS_OFF(io)), len);
}

static void
nv_read16(u16 fs, u32 reg)
{
    memcpy_high(MAKE_FLATPTR(fs, FS_OFF(io))
                , (void*)(GET_GLOBAL(NvVga.mmio) + reg), 2);
}

static void
nv_idx(u16 fs, u32 reg, u8 idx, u8 val)
{
    nv_write(fs, reg, idx | (val << 8), 2);
}

#define nv_crtc(fs, i, v) nv_idx((fs), NV_PRMCIO + 0x3d4, (i), (v))
#define nv_seq(fs, i, v) nv_idx((fs), NV_PRMVIO + 0x3c4, (i), (v))
#define nv_gr(fs, i, v) nv_idx((fs), NV_PRMVIO + 0x3ce, (i), (v))

// The attribute controller takes index and data at 3C0h as two words (one
// (index, data) word does not take). A status read first puts its
// flip-flop on the index; nv_attr_end turns palette access back on.
static void
nv_attr_begin(u16 fs)
{
    nv_read16(fs, NV_PRMCIO + 0x3da);
}

static void
nv_attr(u16 fs, u8 i, u8 v)
{
    nv_write(fs, NV_PRMCIO + 0x3c0, i, 2);
    nv_write(fs, NV_PRMCIO + 0x3c0, v, 2);
}

static void
nv_attr_end(u16 fs)
{
    nv_write(fs, NV_PRMCIO + 0x3c0, 0x20, 2);
}

// The hardware DAC (6 bits per component, as DOS expects): (index, red) at
// 3C8h, then green and blue as words at 3C9h.
static void
nv_dac_write(u16 fs, int i, u8 r, u8 g, u8 b)
{
    nv_idx(fs, NV_PRMDIO + 0x3c8, i, r);
    nv_write(fs, NV_PRMDIO + 0x3c9, g, 2);
    nv_write(fs, NV_PRMDIO + 0x3c9, b, 2);
}

// Through the legacy ports, which are open in native 13h.
static void
nv_dac_read(int i, u8 *r, u8 *g, u8 *b)
{
    outb(i, 0x3c7);
    *r = inb(0x3c9);
    *g = inb(0x3c9);
    *b = inb(0x3c9);
}

// Misc output, then VSE2 (3C3h) as a word of its own (its high byte sets
// the sequencer index, which every sequencer write sets again). Misc bit 0
// decodes the CRTC and status at 3D4h/3DAh (13h programs poll 3DAh for
// retrace); VSE2 enables the legacy ports (3C0h-3DFh), which then still
// need nv_open_ports.
static void
nv_misc(u16 fs, u8 misc, u8 vse2)
{
    nv_write(fs, NV_PRMVIO + 0x3c2, misc | (vse2 << 8), 2);
    nv_write(fs, NV_PRMVIO + 0x3c3, vse2, 2);
}

// Do the legacy ports reach the NV2A: misc read through port 3CCh equals
// misc read through MMIO, and a graphics index written to port 3CEh reads
// back through MMIO.
static int
nv_ports_ok(u16 fs, const char *step)
{
    nv_read16(fs, NV_PRMVIO + 0x3cc);
    u8 misc = GET_FS(fs, io), misc_p = inb(0x3cc);
    outb(0x05, 0x3ce);
    nv_read16(fs, NV_PRMVIO + 0x3ce);
    u8 gri = GET_FS(fs, io);
    nv_read16(fs, NV_PRMVIO + 0x3c2);
    u8 vse2 = GET_FS(fs, io) >> 8;
    int ok = misc_p == misc && gri == 0x05;
    dprintf(1, "fbsync: ports %s: %s (misc %x port %x, gr index %x, vse2 %x)\n"
            , step, ok ? "open" : "shut", misc, misc_p, gri, vse2);
    return ok;
}

// The legacy ports reach the NV2A only once the AGP bridge's VGA Enable is
// written off and on after VSE2 is set: the bit set by the init stage alone
// leaves them shut (port reads ff), as does VSE2 alone (1.4, ticket #38).
static void
nv_open_ports(void)
{
    u16 br = pci_to_bdf(0, 0x1e, 0);
    u32 ctl = pci_config_readl(br, 0x3c);
    pci_config_writel(br, 0x3c, ctl & ~(1u << 19));
    pci_config_writel(br, 0x3c, ctl | (1u << 19));
}

static void
nv_start(u16 fs, u32 start)
{
    nv_write(fs, NV_PCRTC + 0x800, start, 4);
}

static int
nv_available(u16 fs)
{
    return GET_GLOBAL(NvVga.mmio) && !GET_FS(fs, off);
}

static void load_soft_dac_to_hw(u16 fs);

// Mode 13h in VGA format: byte mode (the NV2A shifts a dword-mode address as
// an IBM VGA does), chain-4, 256-colour shift, pixel doubling (AR10 bit 6)
// and double scan (CR09) on the unchanged 640x480 raster, 400 lines centred
// through the CRTC display end and the flat-panel vertical timing.
static void
nv_enter_13h(u16 fs, int load_dac)
{
    u8 cr07 = GET_GLOBAL(NvVga.cr07), cr09 = GET_GLOBAL(NvVga.cr09);
    u32 vd = NV13_LINES - 1, top = (NV13_HEIGHT - NV13_LINES) / 2;
    int i;
    nv_seq(fs, 0x01, 0x01);             // 8-dot clock, display on
    nv_seq(fs, 0x02, 0x0f);
    nv_seq(fs, 0x04, 0x0e);             // chain-4, no odd/even
    nv_gr(fs, 0x05, 0x40);              // 256-colour shift
    nv_gr(fs, 0x06, 0x05);              // graphics, A0000 64 KiB
    nv_attr_begin(fs);
    for (i = 0; i < 16; i++)
        nv_attr(fs, i, i);
    nv_attr(fs, 0x10, 0x41);            // graphics, 8-bit colour
    nv_attr(fs, 0x11, 0x00);
    nv_attr(fs, 0x12, 0x0f);
    nv_attr(fs, 0x13, 0x00);
    nv_attr(fs, 0x14, 0x00);
    nv_attr_end(fs);
    nv_write(fs, NV_PRMDIO + 0x3c6, 0xff, 2);   // pel mask (3C7h: read index 0)
    nv_crtc(fs, 0x09, (cr09 & 0xe0) | 0x01);    // double scan
    nv_crtc(fs, 0x13, 0x28);            // 320 bytes per row in byte mode
    nv_crtc(fs, 0x25, GET_GLOBAL(NvVga.cr25) & ~0x20);
    nv_crtc(fs, 0x14, 0x00);            // byte mode
    nv_crtc(fs, 0x17, 0xe3);
    // Display end and blank start at line 400; the flat panel's display
    // end 80 lines earlier and its sync 40 lines earlier, so the picture
    // sits between 40 black lines above and below.
    nv_crtc(fs, 0x12, vd);
    nv_crtc(fs, 0x15, vd);
    nv_crtc(fs, 0x07, (cr07 & ~0x4a) | ((vd >> 7) & 0x02) | ((vd >> 5) & 0x08)
            | ((vd >> 3) & 0x40));
    nv_write(fs, NV_PRAMDAC + 0x800, GET_GLOBAL(NvVga.fp_v[0]) - 2 * top, 4);
    nv_write(fs, NV_PRAMDAC + 0x808, GET_GLOBAL(NvVga.fp_v[2]) - 2 * top, 4);
    nv_write(fs, NV_PRAMDAC + 0x80c, GET_GLOBAL(NvVga.fp_v[3]) - top, 4);
    nv_write(fs, NV_PRAMDAC + 0x810, GET_GLOBAL(NvVga.fp_v[4]) - top, 4);
    nv_write(fs, NV_PRAMDAC + 0x818, GET_GLOBAL(NvVga.fp_v[6]) - 2 * top, 4);
    nv_crtc(fs, 0x28, GET_GLOBAL(NvVga.cr28) & 0x80);   // VGA format
    nv_crtc(fs, 0x19, 0x00);            // pitch bits 10:8
    nv_misc(fs, GET_GLOBAL(NvVga.misc) | 0x01, 0x01);
    nv_open_ports();
    nv_start(fs, SEG_GRAPH << 4);       // last: CR19 moves it
    SET_FS(fs, native, 1);
    if (load_dac)
        load_soft_dac_to_hw(fs);
    dprintf(1, "fbsync: native 13h on%s\n", load_dac ? ", DAC loaded" : "");
    nv_ports_ok(fs, "in 13h");

    // Reading the DAC back through ports 3C7h/3C9h, once: int 10h 1015h and
    // 1017h use it, or the BIOS's own copy if it does not work.
    if (load_dac && !GET_FS(fs, dac_ports)) {
        u8 r, g, b, n = 0x29;   // default 13h palette: 3f 10 00
        nv_dac_read(n, &r, &g, &b);
        u8 wr = GET_FS(fs, dac[n * 3]) & 0x3f, wg = GET_FS(fs, dac[n * 3 + 1]) & 0x3f;
        u8 wb = GET_FS(fs, dac[n * 3 + 2]) & 0x3f;
        int ok = r == wr && g == wg && b == wb;
        SET_FS(fs, dac_ports, ok ? 1 : 2);
        dprintf(1, "fbsync: DAC read through ports %s (entry %x: %x %x %x, want %x %x %x)\n"
                , ok ? "ok" : "FAILS, int 10h reads the BIOS copy", n, r, g, b
                , wr, wg, wb);
    }
}

// Back to the 32 bpp scanout from the saved set, PCRTC_START last; the DAC
// does not touch 32 bpp, so it stays as 13h left it.
static void
nv_leave_13h(u16 fs)
{
    int i;
    for (i = 0; i < 7; i++)
        nv_write(fs, NV_PRAMDAC + 0x800 + 4 * i, GET_GLOBAL(NvVga.fp_v[i]), 4);
    nv_seq(fs, 0x02, GET_GLOBAL(NvVga.sr02));
    nv_seq(fs, 0x04, GET_GLOBAL(NvVga.sr04));
    nv_gr(fs, 0x05, GET_GLOBAL(NvVga.gr05));
    nv_gr(fs, 0x06, GET_GLOBAL(NvVga.gr06));
    nv_attr_begin(fs);
    nv_attr(fs, 0x10, GET_GLOBAL(NvVga.ar10));
    nv_attr(fs, 0x11, GET_GLOBAL(NvVga.ar11));
    nv_attr(fs, 0x12, GET_GLOBAL(NvVga.ar12));
    nv_attr(fs, 0x13, GET_GLOBAL(NvVga.ar13));
    nv_attr(fs, 0x14, GET_GLOBAL(NvVga.ar14));
    nv_attr_end(fs);
    nv_crtc(fs, 0x07, GET_GLOBAL(NvVga.cr07));
    nv_crtc(fs, 0x09, GET_GLOBAL(NvVga.cr09));
    nv_crtc(fs, 0x12, GET_GLOBAL(NvVga.cr12));
    nv_crtc(fs, 0x13, GET_GLOBAL(NvVga.cr13));
    nv_crtc(fs, 0x14, GET_GLOBAL(NvVga.cr14));
    nv_crtc(fs, 0x15, GET_GLOBAL(NvVga.cr15));
    nv_crtc(fs, 0x17, GET_GLOBAL(NvVga.cr17));
    nv_crtc(fs, 0x25, GET_GLOBAL(NvVga.cr25));
    nv_crtc(fs, 0x28, GET_GLOBAL(NvVga.cr28));
    nv_crtc(fs, 0x19, GET_GLOBAL(NvVga.cr19));
    nv_seq(fs, 0x01, GET_GLOBAL(NvVga.sr01));
    nv_misc(fs, GET_GLOBAL(NvVga.misc), 0x00);  // ports shut again
    nv_start(fs, GET_GLOBAL(NvVga.start));
    SET_FS(fs, native, 0);
    SET_FS(fs, force, 1);
    dprintf(1, "fbsync: native 13h off, 32 bpp scanout back\n");
    nv_ports_ok(fs, "after leaving 13h");
}

// Before every mode set (cbvga_set_mode): leave native 13h. With rendering
// off (4F14h) the hardware is not this BIOS's to touch.
void
fbsync_leave_native(void)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (fs && GET_FS(fs, native) && !GET_FS(fs, off))
        nv_leave_13h(fs);
}


/****************************************************************
 * Palette
 ****************************************************************/

static void
set_dac(u16 fs, int i, u8 r, u8 g, u8 b)
{
    SET_FS(fs, dac[i * 3 + 0], r);
    SET_FS(fs, dac[i * 3 + 1], g);
    SET_FS(fs, dac[i * 3 + 2], b);
    if (GET_FS(fs, native) && !GET_FS(fs, off))
        nv_dac_write(fs, i, r, g, b);
    r &= 0x3f; g &= 0x3f; b &= 0x3f;
    u32 px = (((r << 2) | (r >> 4)) << 16) | (((g << 2) | (g >> 4)) << 8)
        | ((b << 2) | (b >> 4));
    SET_FS(fs, pal[i], px);
}

// DAC entry i <- colour c of the EGA 64-colour set: bits 0-2 =
// blue/green/red (0x2a), bits 3-5 the secondary intensities (0x15). The text
// modes' default DAC is that set itself (i = c).
static void
set_dac_ega(u16 fs, int i, int c)
{
    set_dac(fs, i, 0x2a * ((c >> 2) & 1) + 0x15 * ((c >> 5) & 1)
            , 0x2a * ((c >> 1) & 1) + 0x15 * ((c >> 4) & 1)
            , 0x2a * (c & 1) + 0x15 * ((c >> 3) & 1));
}

// The 16 text colours' DAC indexes in the EGA set (6 is brown, 0x14).
static u8 attr_default[16] VAR16 = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07,
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
};

// Mode 13h default DAC: 16 EGA colours, 16 greys, 216 hues in three
// intensities of three saturations (24 hues each), 8 black.
static u8 grey_ramp[16] VAR16 = {
    0x00, 0x05, 0x08, 0x0b, 0x0e, 0x11, 0x14, 0x18,
    0x1c, 0x20, 0x24, 0x28, 0x2d, 0x32, 0x38, 0x3f,
};
static u8 hue_ramps[9][5] VAR16 = {
    { 0x00, 0x10, 0x1f, 0x2f, 0x3f }, { 0x1f, 0x27, 0x2f, 0x37, 0x3f },
    { 0x2d, 0x31, 0x36, 0x3a, 0x3f }, { 0x00, 0x07, 0x0e, 0x15, 0x1c },
    { 0x0e, 0x11, 0x15, 0x18, 0x1c }, { 0x14, 0x16, 0x18, 0x1a, 0x1c },
    { 0x00, 0x04, 0x08, 0x0c, 0x10 }, { 0x08, 0x0a, 0x0c, 0x0e, 0x10 },
    { 0x0b, 0x0c, 0x0d, 0x0f, 0x10 },
};
// Hue h of 24: ramp index of r, g, b (4 = full, 0 = minimum).
static u8 hue_rgb[24][3] VAR16 = {
    {0,0,4},{1,0,4},{2,0,4},{3,0,4},{4,0,4},{4,0,3},{4,0,2},{4,0,1},
    {4,0,0},{4,1,0},{4,2,0},{4,3,0},{4,4,0},{3,4,0},{2,4,0},{1,4,0},
    {0,4,0},{0,4,1},{0,4,2},{0,4,3},{0,4,4},{0,3,4},{0,2,4},{0,1,4},
};

static void
load_soft_dac_to_hw(u16 fs)
{
    int i;
    for (i = 0; i < 256; i++)
        nv_dac_write(fs, i, GET_FS(fs, dac[i * 3]), GET_FS(fs, dac[i * 3 + 1])
                     , GET_FS(fs, dac[i * 3 + 2]));
}

// One DAC entry for int 10h 1015h/1017h: the hardware's in native 13h
// (where programs write it through the ports), else the BIOS's copy.
static void
get_dac(u16 fs, int i, u8 *r, u8 *g, u8 *b)
{
    if (GET_FS(fs, native) && !GET_FS(fs, off) && GET_FS(fs, dac_ports) == 1) {
        nv_dac_read(i, r, g, b);
        return;
    }
    *r = GET_FS(fs, dac[i * 3 + 0]);
    *g = GET_FS(fs, dac[i * 3 + 1]);
    *b = GET_FS(fs, dac[i * 3 + 2]);
}

static void
load_default_dac(u16 fs, int mode13)
{
    int i;
    if (!mode13) {
        for (i = 0; i < 64; i++)
            set_dac_ega(fs, i, i);
        for (; i < 256; i++)
            set_dac(fs, i, 0, 0, 0);
        return;
    }
    for (i = 0; i < 16; i++)
        set_dac_ega(fs, i, GET_GLOBAL(attr_default[i]));
    for (i = 0; i < 16; i++) {
        u8 v = GET_GLOBAL(grey_ramp[i]);
        set_dac(fs, 16 + i, v, v, v);
    }
    for (i = 0; i < 216; i++) {
        int ramp = i / 24, h = i % 24;
        set_dac(fs, 32 + i
                , GET_GLOBAL(hue_ramps[ramp][GET_GLOBAL(hue_rgb[h][0])])
                , GET_GLOBAL(hue_ramps[ramp][GET_GLOBAL(hue_rgb[h][1])])
                , GET_GLOBAL(hue_ramps[ramp][GET_GLOBAL(hue_rgb[h][2])]));
    }
    for (i = 248; i < 256; i++)
        set_dac(fs, i, 0, 0, 0);
}

// int 10h ah=10h: palette and DAC functions, kept here instead of in VGA
// registers. The caller repaints afterwards.
void
fbsync_1010(struct bregs *regs)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (!fs)
        return;
    int i;
    switch (regs->al) {
    case 0x00:
        if (regs->bl < 16)
            SET_FS(fs, attr[regs->bl], regs->bh);
        break;
    case 0x02:
        for (i = 0; i < 16; i++)
            SET_FS(fs, attr[i], GET_FARVAR(regs->es, *(u8*)(regs->dx + i)));
        break;
    case 0x03:
        SET_FS(fs, blink, regs->bl & 1);
        break;
    case 0x07:
        if (regs->bl < 16)
            regs->bh = GET_FS(fs, attr[regs->bl]);
        break;
    case 0x09:
        for (i = 0; i < 16; i++)
            SET_FARVAR(regs->es, *(u8*)(regs->dx + i), GET_FS(fs, attr[i]));
        SET_FARVAR(regs->es, *(u8*)(regs->dx + 16), 0);
        break;
    case 0x10:
        set_dac(fs, regs->bl, regs->dh, regs->ch, regs->cl);
        break;
    case 0x12:
        for (i = 0; i < regs->cx && (regs->bx + i) < 256; i++) {
            u8 *p = (u8*)(regs->dx + i * 3);
            set_dac(fs, regs->bx + i, GET_FARVAR(regs->es, p[0])
                    , GET_FARVAR(regs->es, p[1]), GET_FARVAR(regs->es, p[2]));
        }
        break;
    case 0x15: {
        u8 r, g, b;
        get_dac(fs, regs->bl, &r, &g, &b);
        regs->dh = r;
        regs->ch = g;
        regs->cl = b;
        break;
    }
    case 0x17:
        for (i = 0; i < regs->cx && (regs->bx + i) < 256; i++) {
            u8 *p = (u8*)(regs->dx + i * 3), r, g, b;
            get_dac(fs, regs->bx + i, &r, &g, &b);
            SET_FARVAR(regs->es, p[0], r);
            SET_FARVAR(regs->es, p[1], g);
            SET_FARVAR(regs->es, p[2], b);
        }
        break;
    default:
        break;
    }
}


/****************************************************************
 * Rendering
 ****************************************************************/

// Framebuffer geometry (the coreboot mode).
struct fbinfo {
    u32 base, pitch;
    u16 width, height;
};

static void
get_fbinfo(struct fbinfo *fb)
{
    struct vgamode_s *cb = cbvga_fb_mode();
    fb->base = GET_GLOBAL(VBE_framebuffer);
    fb->pitch = cbvga_get_linelength(cb);
    fb->width = GET_GLOBAL(cb->width);
    fb->height = GET_GLOBAL(cb->height);
}

static u32
hash_mem(u16 seg, u32 off, int dwords, u32 *any)
{
    u32 h = 0x811c9dc5, a = 0;
    SET_SEG(ES, seg);
    for (; dwords; dwords--, off += 4) {
        u32 w = GET_VAR(ES, *(u32*)off);
        a |= w;
        h = (h ^ w) * 0x9e3779b1u;
        h ^= h >> 15;
    }
    if (any)
        *any = a;
    return h;
}

// Fill framebuffer lines [y0, y1) with black.
static void
clear_lines(u16 fs, struct fbinfo *fb, int y0, int y1)
{
    if (y1 > fb->height)
        y1 = fb->height;
    if (y0 >= y1)
        return;
    void *line = MAKE_FLATPTR(fs, FS_OFF(line));
    memset_far(fs, (void*)FS_OFF(line), 0, fb->width * 4);
    for (; y0 < y1; y0++)
        memcpy_high((void*)(fb->base + y0 * fb->pitch), line, fb->width * 4);
}

struct textinfo {
    u16 seg, pageoff;
    u8 cols, rows, cheight, cw;
    u8 blink, blinkon;
    u8 cur_row, cur_col, cur_start, cur_end; // cur_start > cur_end: hidden
    struct segoff_s font, font_hi;
};

static void
get_textinfo(u16 fs, struct vgamode_s *vmode_g, struct textinfo *ti)
{
    ti->seg = GET_GLOBAL(vmode_g->sstart);
    ti->pageoff = GET_BDA(video_pagestart);
    ti->cols = GET_BDA(video_cols);
    ti->rows = GET_BDA(video_rows) + 1;
    ti->cheight = GET_BDA(char_height);
    if (!ti->cheight || ti->cheight > 16)
        ti->cheight = 16;
    ti->cw = ti->cols <= 40 ? 16 : 8;
    u32 ticks = GET_BDA(timer_counter);
    ti->blink = GET_FS(fs, blink);
    ti->blinkon = (ticks >> 4) & 1;
    ti->font = GET_IVT(0x43);
    ti->font_hi = GET_IVT(0x1f);

    struct cursorpos cp = get_cursor_pos(GET_BDA(video_page));
    u16 shape = get_cursor_shape();
    ti->cur_row = cp.y;
    ti->cur_col = cp.x;
    ti->cur_start = (shape >> 8) & 0x1f;
    ti->cur_end = shape & 0x1f;
    if (shape & 0x2000 || !((ticks >> 3) & 1))
        ti->cur_start = 0xff; // off, or in the dark half of the blink
    if (ti->cur_end >= ti->cheight)
        ti->cur_end = ti->cheight - 1;
}

// Hash of a text row with everything that changes its picture mixed in.
static u32
hash_text_row(struct textinfo *ti, int r)
{
    u32 any;
    u32 h = hash_mem(ti->seg, ti->pageoff + r * ti->cols * 2
                     , ti->cols / 2, &any);
    if (ti->blink && (any & 0x80008000))
        h ^= ti->blinkon << 31;
    if (r == ti->cur_row)
        h = (h ^ (ti->cur_col | ti->cur_start << 8 | ti->cur_end << 16))
            * 0x9e3779b1u;
    return h;
}

static void
render_text_row(u16 fs, struct fbinfo *fb, struct textinfo *ti, int r)
{
    u32 linebytes = ti->cols * ti->cw * 4;
    void *line = MAKE_FLATPTR(fs, FS_OFF(line));
    int y, c, x;
    for (y = 0; y < ti->cheight; y++) {
        u32 off = ti->pageoff + r * ti->cols * 2;
        u32 *lp = (u32*)FS_OFF(line);
        for (c = 0; c < ti->cols; c++, off += 2) {
            u16 cell = GET_FARVAR(ti->seg, *(u16*)off);
            u8 ch = cell, at = cell >> 8;
            struct segoff_s font = ti->font;
            if (ti->cheight == 8 && ch >= 128) {
                font = ti->font_hi;
                ch -= 128;
            }
            u8 g = GET_FARVAR(font.seg
                              , *(u8*)(font.offset + ch * ti->cheight + y));
            u8 fgi = at & 0x0f, bgi = at >> 4;
            if (ti->blink) {
                bgi &= 7;
                if ((at & 0x80) && !ti->blinkon)
                    fgi = bgi;
            }
            u32 fg = GET_FS(fs, pal[GET_FS(fs, attr[fgi])]);
            u32 bg = GET_FS(fs, pal[GET_FS(fs, attr[bgi])]);
            if (r == ti->cur_row && c == ti->cur_col
                && y >= ti->cur_start && y <= ti->cur_end)
                g = 0xff;
            SET_SEG(ES, fs);
            for (x = 0; x < 8; x++) {
                u32 px = (g & (0x80 >> x)) ? fg : bg;
                SET_VAR(ES, *lp, px);
                lp++;
                if (ti->cw == 16) {
                    SET_VAR(ES, *lp, px);
                    lp++;
                }
            }
        }
        memcpy_high((void*)(fb->base + (r * ti->cheight + y) * fb->pitch)
                    , line, linebytes);
    }
}

static void
sync_text(u16 fs, struct vgamode_s *vmode_g, struct fbinfo *fb, int force)
{
    struct textinfo ti;
    get_textinfo(fs, vmode_g, &ti);
    if (ti.rows > FBSYNC_TEXT_ROWS || ti.rows * ti.cheight > fb->height
        || ti.cols * ti.cw > fb->width)
        return;
    if (!GET_FS(fs, below_cleared)) {
        dprintf(3, "fbsync: clearing lines %d-%d\n", ti.rows * ti.cheight
                , fb->height);
        clear_lines(fs, fb, ti.rows * ti.cheight, fb->height);
        SET_FS(fs, below_cleared, 1);
    }
    int r;
    for (r = 0; r < ti.rows; r++) {
        u32 h = hash_text_row(&ti, r);
        if (!force && h == GET_FS(fs, rowhash[r]))
            continue;
        render_text_row(fs, fb, &ti, r);
        SET_FS(fs, rowhash[r], h);
    }
}

// Mode 13h: row y of 320 pixels becomes two framebuffer lines of 640.
static void
render_13h_row(u16 fs, struct fbinfo *fb, int y)
{
    u32 off = y * 320;
    u32 *lp = (u32*)FS_OFF(line);
    int x;
    for (x = 0; x < 320; x++, off++) {
        u8 i = GET_FARVAR(SEG_GRAPH, *(u8*)off);
        u32 px = GET_FS(fs, pal[i]);
        SET_SEG(ES, fs);
        SET_VAR(ES, lp[0], px);
        SET_VAR(ES, lp[1], px);
        SET_VAR(ES, lp[FBSYNC_LINE], px);
        SET_VAR(ES, lp[FBSYNC_LINE + 1], px);
        lp += 2;
    }
    void *line = MAKE_FLATPTR(fs, FS_OFF(line));
    void *dest = (void*)(fb->base + (FBSYNC_MARGIN13 + 2 * y) * fb->pitch);
    if (fb->pitch == FBSYNC_LINE * 4) {
        memcpy_high(dest, line, FBSYNC_LINE * 4 * 2);
    } else {
        memcpy_high(dest, line, FBSYNC_LINE * 4);
        memcpy_high(dest + fb->pitch, line, FBSYNC_LINE * 4);
    }
}

static void
sync_13h(u16 fs, struct fbinfo *fb, int force)
{
    if (fb->width < FBSYNC_LINE
        || fb->height < FBSYNC_MARGIN13 + 2 * FBSYNC_SRC_ROWS)
        return;
    int y;
    for (y = 0; y < FBSYNC_SRC_ROWS; y++) {
        u32 h = hash_mem(SEG_GRAPH, y * 320, 80, NULL);
        if (!force && h == GET_FS(fs, rowhash[y]))
            continue;
        render_13h_row(fs, fb, y);
        SET_FS(fs, rowhash[y], h);
    }
}

// Framebuffer writes switched off through int 10h 4F14h (also honoured by
// the cbvga pixel path, vgafb.c).
int
fbsync_off(void)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    return fs && GET_FS(fs, off);
}

// int 10h ax=4F14h: bl=0 off, bl=1 on (repaints), bl=2 query (bh = 1 on).
// Off leaves the screen as it is, native 13h included: no framebuffer,
// register or DAC writes until on. On brings the NV2A back in line with the
// current mode (native 13h again, its DAC from the BIOS's copy, or the
// 32 bpp scanout), since whoever had the screen may have reprogrammed it.
void
fbsync_104f14(struct bregs *regs)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (!fs) {
        regs->ax = 0x014f;
        return;
    }
    switch (regs->bl) {
    case 0:
        SET_FS(fs, off, 1);
        break;
    case 1:
        SET_FS(fs, off, 0);
        SET_FS(fs, force, 1);
        if (GET_GLOBAL(NvVga.mmio) && get_current_mode() == &fbsync_modes[3])
            nv_enter_13h(fs, 1);
        else if (GET_FS(fs, native))
            nv_leave_13h(fs);
        break;
    default:
        break;
    }
    regs->bh = !GET_FS(fs, off);
    regs->ax = 0x004f;
}

// Render what changed since the last call (everything with force).
static void
fbsync_render(int force)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (!fs || !GET_FS(fs, active) || GET_FS(fs, busy))
        return;
    struct vgamode_s *vmode_g = get_current_mode();
    if (!is_fbsync_mode(vmode_g) || GET_FS(fs, off) || GET_FS(fs, native))
        return;
    SET_FS(fs, busy, 1);
    if (GET_FS(fs, force)) {
        force = 1;
        SET_FS(fs, force, 0);
    }
    struct fbinfo fb;
    get_fbinfo(&fb);
    if (GET_GLOBAL(vmode_g->memmodel) == MM_TEXT)
        sync_text(fs, vmode_g, &fb, force);
    else
        sync_13h(fs, &fb, force);
    SET_FS(fs, busy, 0);
}

// Record the current memory as rendered, so nothing is repainted until it
// changes (the screen stays as the platform left it).
static void
adopt_memory(u16 fs, struct vgamode_s *vmode_g)
{
    int r;
    if (GET_GLOBAL(vmode_g->memmodel) == MM_TEXT) {
        struct textinfo ti;
        get_textinfo(fs, vmode_g, &ti);
        if (ti.rows > FBSYNC_TEXT_ROWS)
            ti.rows = FBSYNC_TEXT_ROWS;
        for (r = 0; r < ti.rows; r++)
            SET_FS(fs, rowhash[r], hash_text_row(&ti, r));
    } else {
        for (r = 0; r < FBSYNC_SRC_ROWS; r++)
            SET_FS(fs, rowhash[r], hash_mem(SEG_GRAPH, r * 320, 80, NULL));
    }
}


/****************************************************************
 * Entry points
 ****************************************************************/

// Called from cbvga_set_mode for the modes this module owns.
void
fbsync_set_mode(struct vgamode_s *vmode_g, int flags)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (!fs)
        return;
    int text = GET_GLOBAL(vmode_g->memmodel) == MM_TEXT;
    int i;
    SET_FS(fs, blink, 1);
    for (i = 0; i < 16; i++)
        SET_FS(fs, attr[i], text ? GET_GLOBAL(attr_default[i]) : i);
    if (!(flags & MF_NOPALETTE))
        load_default_dac(fs, !text);
    if (!(flags & MF_NOCLEARMEM)) {
        if (text)
            memset16_far(GET_GLOBAL(vmode_g->sstart), 0, 0x0720, 32 * 1024);
        else
            memset_far(SEG_GRAPH, 0, 0, 320 * 200);
        // A mode set that clears the screen through the legacy entry is
        // the OS taking the screen.
        if ((flags & MF_LEGACY) && !GET_FS(fs, active)) {
            dprintf(1, "fbsync: mode set takes the screen\n");
            SET_FS(fs, active, 1);
        }
    }
    if (!text && nv_available(fs)) {
        // The NV2A shows A0000 itself; the framebuffer keeps the text
        // screen for the way back.
        SET_FS(fs, active, 1);
        nv_enter_13h(fs, !(flags & MF_NOPALETTE));
        return;
    }
    if (GET_FS(fs, active) && !(flags & MF_NOCLEARMEM) && !GET_FS(fs, off)) {
        struct fbinfo fb;
        get_fbinfo(&fb);
        clear_lines(fs, &fb, 0, fb.height);
        SET_FS(fs, below_cleared, 1);
        SET_FS(fs, force, 1);
    } else if (GET_FS(fs, active)) {
        SET_FS(fs, force, 1);
    } else {
        adopt_memory(fs, vmode_g);
    }
}

// After an int 10h call: render if it may have changed the screen. The
// first such call hands the screen from the platform to the OS.
void
fbsync_post_handle10(struct bregs *regs)
{
    u16 fs = GET_GLOBAL(FbsyncSeg);
    if (!fs)
        return;
    switch (regs->ah) {
    case 0x00:
        // fbsync_set_mode decided whether this mode set took the screen
        fbsync_render(0);
        break;
    case 0x10: case 0x11:
        SET_FS(fs, force, 1);
        // NO BREAK
    case 0x01: case 0x02: case 0x05: case 0x06: case 0x07:
    case 0x09: case 0x0a: case 0x0e: case 0x13:
        if (!GET_FS(fs, active)) {
            dprintf(1, "fbsync: int 10h ah=%02x takes the screen\n", regs->ah);
            SET_FS(fs, active, 1);
            SET_FS(fs, below_cleared, 0);
        }
        fbsync_render(0);
        break;
    default:
        break;
    }
}

// Called by the hooked timer irq.
void
fbsync_timer(void)
{
    fbsync_render(0);
}

void
fbsync_setup(void)
{
    if (!CONFIG_VGA_FBSYNC)
        return;
    u32 res = allocate_pmm(sizeof(struct fbsync_s), 0, 0);
    if (!res) {
        dprintf(1, "fbsync: no low memory, legacy modes render nothing\n");
        return;
    }
    u16 fs = res >> 4;
    memset_far(fs, 0, 0, sizeof(struct fbsync_s));
    SET_VGA(FbsyncSeg, fs);
    SET_FS(fs, blink, 1);
    int i;
    for (i = 0; i < 16; i++)
        SET_FS(fs, attr[i], GET_GLOBAL(attr_default[i]));
    load_default_dac(fs, 0);
    struct cb_header *cbh = find_cb_table();
    struct cb_xbox_vga *nv = cbh ? find_cb_subtable(cbh, CB_TAG_XBOX_VGA) : NULL;
    if (nv && GET_FARVAR(0, nv->size) >= sizeof(NvVga)) {
        memcpy_far(get_global_seg(), &NvVga, 0, nv, sizeof(NvVga));
        dprintf(1, "fbsync: native 13h on the NV2A at %x (32 bpp start %x)\n"
                , GET_GLOBAL(NvVga.mmio), GET_GLOBAL(NvVga.start));
    }
    // Text memory starts blank, so nothing is painted over the platform's
    // screen until something writes it; the first mode set (the BIOS's own,
    // without a clear) records it as rendered.
    memset16_far(SEG_CTEXT, 0, 0x0720, 32 * 1024);
    dprintf(1, "fbsync: %d bytes at %x, text at b8000 blanked\n"
            , (int)sizeof(struct fbsync_s), res);
}
