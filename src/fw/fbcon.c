// Framebuffer debug console for coreboot payload mode.
//
// Renders the debug log as 8x16 text into the framebuffer coreboot
// describes (32 bpp), for boards without a serial port. The platform's init
// code may have drawn its own log on that framebuffer already: its cbmem
// console contents are replayed through the same layout rules to recover
// the cursor, so the BIOS output continues below it without clearing the
// screen. Layout rules: '\n' starts a new row (the text rows scroll up by
// one at the bottom), '\r' and other control bytes are dropped, a printable
// byte at the last column wraps first. printf() output is shown here too
// a boot sector is entered (output.c); fbcon_boot() then hands the cursor
// to the vgabios so the OS's int 10h output follows the log.
//
// Copyright (C) 2026 Marco Janssen Groesbeek
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_*
#include "output.h" // dprintf
#include "stacks.h" // call16_int
#include "string.h" // memset
#include "util.h" // fbcon_setup

#define CELL_W 8
#define CELL_H 16
#define MAX_COLS 128
#define MAX_ROWS 48
#define FG 0xe0e0e0
#define BG 0x000000

#define CB_TAG_FRAMEBUFFER 0x0012
struct cb_framebuffer {
    u32 tag;
    u32 size;
    u64 physical_address;
    u32 x_resolution;
    u32 y_resolution;
    u32 bytes_per_line;
    u8 bits_per_pixel;
    u8 red_mask_pos;
    u8 red_mask_size;
    u8 green_mask_pos;
    u8 green_mask_size;
    u8 blue_mask_pos;
    u8 blue_mask_size;
    u8 reserved_mask_pos;
    u8 reserved_mask_size;
};

extern u8 fbcon_font16[256 * 16];

static struct {
    u8 *fb;
    u32 pitch;
    u16 cols, rows, col, row;
    u8 active;
} Fbcon;
static u8 FbconShadow[MAX_ROWS][MAX_COLS]; // 0 = blank (glyph 0 is empty)

int
fbcon_active(void)
{
    return CONFIG_DEBUG_FRAMEBUFFER && Fbcon.active;
}

static void
draw_cell(int r, int c)
{
    const u8 *g = &fbcon_font16[FbconShadow[r][c] * CELL_H];
    int y, x;
    for (y = 0; y < CELL_H; y++) {
        u32 *px = (u32*)(Fbcon.fb + (r * CELL_H + y) * Fbcon.pitch
                         + c * CELL_W * 4);
        for (x = 0; x < CELL_W; x++)
            px[x] = (g[y] >> (7 - x)) & 1 ? FG : BG;
    }
}

static void
draw_rows(int from)
{
    int r, c;
    for (r = from; r < Fbcon.rows; r++)
        for (c = 0; c < Fbcon.cols; c++)
            draw_cell(r, c);
}

static void
newline(int draw)
{
    Fbcon.col = 0;
    if (++Fbcon.row < Fbcon.rows)
        return;
    int top = CONFIG_DEBUG_FRAMEBUFFER_TOP;
    Fbcon.row = Fbcon.rows - 1;
    memmove(FbconShadow[top], FbconShadow[top + 1]
            , (Fbcon.rows - top - 1) * MAX_COLS);
    memset(FbconShadow[Fbcon.rows - 1], 0, MAX_COLS);
    if (draw)
        draw_rows(top);
}

static void
layout(char c, int draw)
{
    if (c == '\n') {
        newline(draw);
        return;
    }
    if ((u8)c < 0x20)
        return;
    if (Fbcon.col == Fbcon.cols)
        newline(draw);
    FbconShadow[Fbcon.row][Fbcon.col] = c;
    if (draw)
        draw_cell(Fbcon.row, Fbcon.col);
    Fbcon.col++;
}

void
fbcon_putc(char c)
{
    if (!fbcon_active())
        return;
    layout(c, 1);
}

// Find the framebuffer and the cursor; log is the platform's log so far
// (NULL to clear the text rows and start at the top instead).
void
fbcon_setup(struct cb_header *cbh, const u8 *log, u32 loglen)
{
    if (!CONFIG_DEBUG_FRAMEBUFFER)
        return;
    struct cb_framebuffer *cbfb = find_cb_subtable(cbh, CB_TAG_FRAMEBUFFER);
    if (!cbfb || cbfb->bits_per_pixel != 32 || !cbfb->physical_address
        || cbfb->physical_address > 0xffffffff)
        return;
    int top = CONFIG_DEBUG_FRAMEBUFFER_TOP;
    int cols = cbfb->x_resolution / CELL_W, rows = cbfb->y_resolution / CELL_H;
    if (cols > MAX_COLS)
        cols = MAX_COLS;
    if (rows > MAX_ROWS)
        rows = MAX_ROWS;
    if (cols < 1 || rows <= top)
        return;
    Fbcon.fb = (u8*)(u32)cbfb->physical_address;
    Fbcon.pitch = cbfb->bytes_per_line;
    Fbcon.cols = cols;
    Fbcon.rows = rows;
    Fbcon.col = 0;
    Fbcon.row = top;
    memset(FbconShadow, 0, sizeof(FbconShadow));
    if (log) {
        u32 i;
        for (i = 0; i < loglen; i++)
            layout(log[i], 0);
    } else {
        draw_rows(top);
    }
    Fbcon.active = 1;
}

// Stop as the boot loader is entered and hand the cursor to the vgabios.
void
fbcon_boot(void)
{
    if (!fbcon_active())
        return;
    int row = Fbcon.row, col = Fbcon.col;
    if (row > GET_BDA(video_rows))
        row = GET_BDA(video_rows);
    if (col >= GET_BDA(video_cols))
        col = GET_BDA(video_cols) - 1;
    dprintf(3, "fbcon: cursor to the vgabios at row %d col %d\n", row, col);
    Fbcon.active = 0;
    if (GET_IVT(0x10).segoff == FUNC16(entry_10).segoff)
        return; // no vgabios
    struct bregs br;
    memset(&br, 0, sizeof(br));
    br.flags = F_IF;
    br.ah = 0x02;
    br.bh = 0;
    br.dh = row;
    br.dl = col;
    call16_int(0x10, &br);
}
