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
 * Palette
 ****************************************************************/

static void
set_dac(u16 fs, int i, u8 r, u8 g, u8 b)
{
    SET_FS(fs, dac[i * 3 + 0], r);
    SET_FS(fs, dac[i * 3 + 1], g);
    SET_FS(fs, dac[i * 3 + 2], b);
    r &= 0x3f; g &= 0x3f; b &= 0x3f;
    u32 px = (((r << 2) | (r >> 4)) << 16) | (((g << 2) | (g >> 4)) << 8)
        | ((b << 2) | (b >> 4));
    SET_FS(fs, pal[i], px);
}

// The EGA 64-colour set: bits 0-2 = blue/green/red (0x2a), bits 3-5 the
// secondary intensities (0x15). The default DAC of the text modes.
static void
set_dac_ega(u16 fs, int i)
{
    set_dac(fs, i, 0x2a * ((i >> 2) & 1) + 0x15 * ((i >> 5) & 1)
            , 0x2a * ((i >> 1) & 1) + 0x15 * ((i >> 4) & 1)
            , 0x2a * (i & 1) + 0x15 * ((i >> 3) & 1));
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
load_default_dac(u16 fs, int mode13)
{
    int i;
    if (!mode13) {
        for (i = 0; i < 64; i++)
            set_dac_ega(fs, i);
        for (; i < 256; i++)
            set_dac(fs, i, 0, 0, 0);
        return;
    }
    for (i = 0; i < 16; i++)
        set_dac_ega(fs, GET_GLOBAL(attr_default[i]));
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
    case 0x15:
        regs->dh = GET_FS(fs, dac[regs->bl * 3 + 0]);
        regs->ch = GET_FS(fs, dac[regs->bl * 3 + 1]);
        regs->cl = GET_FS(fs, dac[regs->bl * 3 + 2]);
        break;
    case 0x17:
        for (i = 0; i < regs->cx && (regs->bx + i) < 256; i++) {
            u8 *p = (u8*)(regs->dx + i * 3);
            int n = (regs->bx + i) * 3;
            SET_FARVAR(regs->es, p[0], GET_FS(fs, dac[n + 0]));
            SET_FARVAR(regs->es, p[1], GET_FS(fs, dac[n + 1]));
            SET_FARVAR(regs->es, p[2], GET_FS(fs, dac[n + 2]));
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
    if (!is_fbsync_mode(vmode_g) || GET_FS(fs, off))
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
    // Text memory starts blank, so nothing is painted over the platform's
    // screen until something writes it; the first mode set (the BIOS's own,
    // without a clear) records it as rendered.
    memset16_far(SEG_CTEXT, 0, 0x0720, 32 * 1024);
    dprintf(1, "fbsync: %d bytes at %x, text at b8000 blanked\n"
            , (int)sizeof(struct fbsync_s), res);
}
