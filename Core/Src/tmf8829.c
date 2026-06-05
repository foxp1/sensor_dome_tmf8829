#include "tmf8829.h"
#include "tmf8829_fw.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// ---------------------------------------------------------------------------
// SPI buffers — must fit a whole result-frame payload in one DMA transfer
// (~2.3 KB) plus the 3-byte read header.
// ---------------------------------------------------------------------------
#define SPI_BUF_MAX 2560
#define TMF8829_CFG_PAGE_FIRST 0x22
#define TMF8829_CFG_PAGE_LAST  0xDF
#define TMF8829_CFG_PAGE_SIZE  (TMF8829_CFG_PAGE_LAST - TMF8829_CFG_PAGE_FIRST + 1)  // 190

static uint8_t spi_tx[SPI_BUF_MAX];
static uint8_t spi_rx[SPI_BUF_MAX];

// Internal frame buffer
static uint8_t frame_buf[TMF8829_FIFO_HW_SIZE];

// ---------------------------------------------------------------------------
// Diagnostic variables — add all to Expressions watch in debugger
// ---------------------------------------------------------------------------
volatile uint8_t  tmf8829_last_cmd_stat;   // CMD_STAT at point of failure
volatile uint8_t  tmf8829_last_app_id;     // APP_ID: 0x80=BL, 0x01=App
volatile int      tmf8829_last_bl_err;     // bl_download_firmware sub-step
volatile uint32_t tmf8829_fw_bytes_sent;   // bytes uploaded (watch progress)
volatile uint8_t  tmf8829_cs_pre;          // CMD_STAT before reset
volatile uint8_t  tmf8829_cs_post;         // CMD_STAT after reset
volatile uint32_t tmf8829_flush_bytes;     // bytes written to flush stuck FIFO
volatile uint8_t  tmf8829_dbg_cid;         // CID_RID after LOAD_48X32
volatile uint8_t  tmf8829_dbg_fpmode;      // FP_MODE after LOAD_48X32
volatile int      tmf8829_wpm_rc;          // last reg_read rc during WPM wait
volatile uint8_t  tmf8829_wpm_val;         // last CMD_STAT value during WPM wait
volatile uint32_t tmf8829_wpm_iters;       // WPM wait iterations

// ---------------------------------------------------------------------------
// Low-level SPI
// ---------------------------------------------------------------------------
static int reg_write(uint8_t reg, const uint8_t *data, uint16_t len)
{
    if ((uint16_t)(len + 2) > SPI_BUF_MAX) return -1;
    spi_tx[0] = TMF8829_SPI_WR;
    spi_tx[1] = reg;
    memcpy(spi_tx + 2, data, len);
    return (HAL_SPI_Transmit(&hspi1, spi_tx, (uint16_t)(len + 2), 50) == HAL_OK) ? 0 : -1;
}

static int reg_write_byte(uint8_t reg, uint8_t val)
{
    return reg_write(reg, &val, 1);
}

static int reg_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((uint16_t)(len + 3) > SPI_BUF_MAX) return -1;
    memset(spi_tx, 0, (uint16_t)(len + 3));
    spi_tx[0] = TMF8829_SPI_RD;
    spi_tx[1] = reg;
    spi_tx[2] = 0x00;
    if (HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, (uint16_t)(len + 3), 50) != HAL_OK)
        return -1;
    memcpy(data, spi_rx + 3, len);
    return 0;
}

static int reg_read_byte(uint8_t reg, uint8_t *val)
{
    return reg_read(reg, val, 1);
}

// ---------------------------------------------------------------------------
// Poll helpers
// ---------------------------------------------------------------------------
static int wait_cmd_stat(uint8_t expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t val;
    do {
        HAL_Delay(1);
        if (reg_read_byte(TMF8829_REG_CMD_STAT, &val) != 0) return -1;
        if (val == expected) return 0;
    } while ((HAL_GetTick() - start) < timeout_ms);
    tmf8829_last_cmd_stat = val;  // capture whatever it's stuck at
    return -2;
}

static int wait_cpu_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t val;
    do {
        HAL_Delay(1);
        if (reg_read_byte(TMF8829_REG_ENABLE, &val) != 0) return -1;
        if (val & TMF8829_ENABLE_CPU_READY) return 0;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return -2;
}

// ---------------------------------------------------------------------------
// I2C-bus idle helpers
// The TMF8829 datasheet requires SCL to be HIGH while BL_I2C_OFF is processed.
// To present a clean idle I2C bus, drive both lines high:
//   PB2  = SCL / GPIO4
//   PB10 = SDA / GPIO5
// Drive high before the command, restore to input after.
// ---------------------------------------------------------------------------
#define TMF8829_I2C_PINS (GPIO_PIN_2 | GPIO_PIN_10)

static void i2c_bus_drive_high(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = TMF8829_I2C_PINS;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, TMF8829_I2C_PINS, GPIO_PIN_SET);
}

static void i2c_bus_restore_input(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = TMF8829_I2C_PINS;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);
}

// ---------------------------------------------------------------------------
// Firmware download via BL_CMD_FIFO_BOTH
//
// Matches ams-OSRAM core driver tmf8829DownloadFirmware (useFifo=1):
//  1. Write [CMD, payload, addr(4), wsize(2)] as ONE auto-incrementing
//     transaction starting at CMD_STAT (0x08).  The command byte and all
//     parameters go out together — splitting them does NOT work.
//  2. Wait CMD_STAT == READY (0).
//  3. Stream the whole image to FIFO (0xFF) — no per-chunk polling.
//  4. Go straight to start-RAM-app (no final CMD_STAT wait).
// ---------------------------------------------------------------------------
static int bl_download_firmware(void)
{
    tmf8829_fw_bytes_sent = 0;

    uint16_t wsize = (TMF8829_FW_SIZE + 3) / 4;   // round up to 32-bit words

    // Setup command + parameters, written atomically from register 0x08
    uint8_t cmd[8] = {
        TMF8829_BL_FIFO_BOTH,                       // 0x08 CMD_STAT
        6,                                          // 0x09 PAYLOAD
        (uint8_t)(TMF8829_FW_ADDR >>  0),           // 0x0A ADDR0
        (uint8_t)(TMF8829_FW_ADDR >>  8),           // 0x0B ADDR1
        (uint8_t)(TMF8829_FW_ADDR >> 16),           // 0x0C ADDR2
        (uint8_t)(TMF8829_FW_ADDR >> 24),           // 0x0D ADDR3
        (uint8_t)(wsize & 0xFF),                    // 0x0E WSIZE LSB
        (uint8_t)((wsize >> 8) & 0xFF),             // 0x0F WSIZE MSB
    };
    if (reg_write(TMF8829_REG_CMD_STAT, cmd, 8) != 0)
        return tmf8829_last_bl_err = -1;

    if (wait_cmd_stat(TMF8829_STAT_OK, 50) != 0)
        return tmf8829_last_bl_err = -2;

    // Stream firmware to FIFO (0xFF) in 128-byte chunks.
    // 0xFF does not auto-increment; each write feeds the device FIFO.
    uint32_t offset = 0;
    while (offset < TMF8829_FW_SIZE) {
        uint16_t chunk = TMF8829_BL_CHUNK;
        if (offset + chunk > TMF8829_FW_SIZE)
            chunk = (uint16_t)(TMF8829_FW_SIZE - offset);

        spi_tx[0] = TMF8829_SPI_WR;
        spi_tx[1] = TMF8829_REG_FIFO;
        memcpy(spi_tx + 2, tmf8829_fw + offset, chunk);
        if (HAL_SPI_Transmit(&hspi1, spi_tx, (uint16_t)(chunk + 2), 100) != HAL_OK)
            return tmf8829_last_bl_err = -3;

        offset += chunk;
        tmf8829_fw_bytes_sent = offset;   // watch this in Expressions
    }

    return tmf8829_last_bl_err = 0;
}

// ---------------------------------------------------------------------------
// Recover from a stuck FIFO upload (CMD_STAT == ERR_FIFO == 4).
//
// If a previous, incomplete BL_FIFO_BOTH left the device expecting more FIFO
// bytes, it ignores all commands until the upload count is satisfied.  Since
// the module is always powered (no EN line) this state survives MCU resets.
// We drain it by writing zero bytes to the FIFO (0xFF) until CMD_STAT returns
// to READY (0).  Capped well above the max image size to stay bounded.
// ---------------------------------------------------------------------------
static int recover_stuck_fifo(void)
{
    uint8_t cs;
    tmf8829_flush_bytes = 0;

    if (reg_read_byte(TMF8829_REG_CMD_STAT, &cs) != 0) return -1;
    if (cs != 0x04) return 0;   // not stuck

    // Write zeros to FIFO in 128-byte chunks, re-checking CMD_STAT each round.
    // 20 KB cap > 14 KB image + word rounding — guaranteed to complete any
    // stale upload.
    memset(spi_tx + 2, 0x00, TMF8829_BL_CHUNK);
    spi_tx[0] = TMF8829_SPI_WR;
    spi_tx[1] = TMF8829_REG_FIFO;
    while (tmf8829_flush_bytes < 20480) {
        if (HAL_SPI_Transmit(&hspi1, spi_tx, TMF8829_BL_CHUNK + 2, 100) != HAL_OK)
            return -2;
        tmf8829_flush_bytes += TMF8829_BL_CHUNK;
        if (reg_read_byte(TMF8829_REG_CMD_STAT, &cs) != 0) return -3;
        if (cs == 0x00) return 0;   // recovered
    }
    return -4;   // still stuck after cap
}

// ---------------------------------------------------------------------------
// Public: Init
// ---------------------------------------------------------------------------
int TMF8829_Init(void)
{
    uint8_t app_id;

    // First SPI transaction also selects SPI as the active interface
    if (reg_read_byte(TMF8829_REG_APP_ID, &app_id) != 0) return -1;
    tmf8829_last_app_id = app_id;

    // If already in app mode (MCU reset/reflashed without power-cycling the
    // sensor), abort any running measurement first.
    if (app_id == TMF8829_APPID_APP) {
        reg_write_byte(TMF8829_REG_CMD_STAT, TMF8829_CMD_STOP);
        HAL_Delay(30);
    }

    // Capture CMD_STAT before reset (diagnostic)
    reg_read_byte(TMF8829_REG_CMD_STAT, (uint8_t *)&tmf8829_cs_pre);

    // Force the next boot into the bootloader WITHOUT powering the device down
    // (a poff/pon power-cycle leaves it unable to start a proper measurement).
    // Clear powerup_select to 0 (boot loader) while keeping the device powered,
    // then soft-reset to restart the CPUs into the bootloader.  This replicates
    // the clean soft-reset-into-bootloader entry that works.
    {
        uint8_t en;
        if (reg_read_byte(TMF8829_REG_ENABLE, &en) != 0) return -2;
        en &= ~0x70u;          // powerup_select = 0 (boot loader)
        reg_write_byte(TMF8829_REG_ENABLE, en);
    }
    reg_write_byte(TMF8829_REG_RESET, 0x40);   // soft reset -> bootloader
    HAL_Delay(20);

    if (wait_cpu_ready(500) != 0) return -2;

    // Confirm bootloader mode
    if (reg_read_byte(TMF8829_REG_APP_ID, &app_id) != 0) return -3;
    tmf8829_last_app_id = app_id;
    if (app_id != TMF8829_APPID_BOOTLOADER) return -4;

    // Capture CMD_STAT after reset (diagnostic), then clear any stuck FIFO
    // left by a previous incomplete download.
    reg_read_byte(TMF8829_REG_CMD_STAT, (uint8_t *)&tmf8829_cs_post);
    if (recover_stuck_fifo() != 0) return -20;

    // Drive SCL+SDA HIGH (idle I2C bus) — the datasheet requires SCL high
    // while the device processes BL_I2C_OFF.  Without this, the I2C bus
    // appears busy and the command fails with error 0x04.
    i2c_bus_drive_high();

    // Disable I2C to lock the interface to SPI
    if (reg_write_byte(TMF8829_REG_CMD_STAT, TMF8829_BL_I2C_OFF) != 0) {
        i2c_bus_restore_input();
        return -5;
    }
    if (wait_cmd_stat(TMF8829_STAT_OK, 200) != 0) {
        i2c_bus_restore_input();
        return -6;
    }
    i2c_bus_restore_input();

    // Download firmware
    if (bl_download_firmware() != 0) return -7;

    // Start the RAM application: write START_RAM to CMD_STAT, then poll the
    // APP_ID register until it reports the application is running (0x01).
    if (reg_write_byte(TMF8829_REG_CMD_STAT, TMF8829_BL_START_RAM) != 0) return -8;
    {
        uint32_t t0 = HAL_GetTick();
        do {
            HAL_Delay(1);
            if (reg_read_byte(TMF8829_REG_APP_ID, &app_id) != 0) return -9;
            tmf8829_last_app_id = app_id;
        } while (app_id != TMF8829_APPID_APP && (HAL_GetTick() - t0) < 100);
        if (app_id != TMF8829_APPID_APP) return -10;
    }

    // Configure ENABLE: select RAM for powerup and set PON (matches driver)
    {
        uint8_t en;
        if (reg_read_byte(TMF8829_REG_ENABLE, &en) != 0) return -11;
        en &= ~0x30u;          // clear powerup_select
        en |=  (2u << 4);      // powerup_select = RAM
        en |=  TMF8829_ENABLE_PON;
        if (reg_write_byte(TMF8829_REG_ENABLE, en) != 0) return -12;
    }

    // Verify chip ID
    uint8_t chip_id;
    if (reg_read_byte(TMF8829_REG_CHIP_ID, &chip_id) != 0) return -13;
    if (chip_id != TMF8829_CHIP_ID_VALUE) return -14;

    return 0;
}

// ---------------------------------------------------------------------------
// Public: Configure 48x32 at ~15 fps
// ---------------------------------------------------------------------------
int TMF8829_Configure_48x32_15fps(void)
{
    // Preconfigure 48x32 HIGH-ACCURACY mode.
    if (reg_write_byte(TMF8829_REG_CMD_STAT, TMF8829_CMD_LOAD_48X32_HA) != 0) return -1;
    if (wait_cmd_stat(TMF8829_STAT_OK, 100) != 0) return -2;

    // Capture device state after the preset load
    reg_read_byte(TMF8829_REG_CID_RID, (uint8_t *)&tmf8829_dbg_cid);
    reg_read_byte(TMF8829_REG_FP_MODE, (uint8_t *)&tmf8829_dbg_fpmode);

    uint8_t period[2] = {67, 0};
    if (reg_write(TMF8829_REG_PERIOD_LSB, period, 2) != 0) return -3;

    // Request 2 peaks per pixel (keep signal/noise/xtalk off) so the assembler
    // can skip the near cover-glass return.  Preserve the read-only sub_result
    // bit (6) the preset set.
    {
        uint8_t rf;
        if (reg_read_byte(TMF8829_REG_RESULT_FORMAT, &rf) != 0) return -7;
        rf = (uint8_t)((rf & ~0x07u) | (TMF8829_RESULT_NR_PEAKS & 0x07u));
        if (reg_write_byte(TMF8829_REG_RESULT_FORMAT, rf) != 0) return -8;
    }

    if (reg_write_byte(TMF8829_REG_INT_ENAB, TMF8829_INT_RESULT) != 0) return -4;

    // CMD_WRITE_PAGE_AND_MEASURE — instrumented wait.
    if (reg_write_byte(TMF8829_REG_CMD_STAT, TMF8829_CMD_W_PAGE_MEASURE) != 0) return -5;
    {
        uint32_t t0 = HAL_GetTick();
        tmf8829_wpm_iters = 0;
        do {
            HAL_Delay(1);
            uint8_t v = 0xAA;
            int rc = reg_read_byte(TMF8829_REG_CMD_STAT, &v);
            tmf8829_wpm_iters++;
            tmf8829_wpm_rc  = rc;
            tmf8829_wpm_val = v;
            if (rc == 0 && (v == TMF8829_STAT_OK || v == TMF8829_STAT_ACCEPTED))
                return 0;   // measurement started
        } while ((HAL_GetTick() - t0) < 2000);
        return -6;
    }
}

// ---------------------------------------------------------------------------
// Diagnostics for the poll path
// ---------------------------------------------------------------------------
volatile uint8_t  tmf8829_int_seen;     // OR of all INT_STATUS bits observed
volatile uint32_t tmf8829_poll_iters;   // poll calls made
volatile uint8_t  tmf8829_last_fid;     // frame ID of last header read
volatile uint16_t tmf8829_last_payload; // payload size from last header
volatile uint16_t tmf8829_last_eof;     // EOF marker of last frame

// Full 48x32 depth grid in mm, assembled from the two sub-frames.
// Host reads this directly for full-resolution visualisation.
#define TMF8829_GRID_W      48
#define TMF8829_GRID_H      32
#define TMF8829_GRID_N      (TMF8829_GRID_W * TMF8829_GRID_H)  // 1536
#define TMF8829_SUB_ROWS    16                                 // rows per sub-frame
#define TMF8829_PIX_OFF     16                                 // first pixel byte = after 16B header
static   uint16_t tmf8829_grid_work[TMF8829_GRID_N];          // accumulator
volatile uint16_t tmf8829_grid[TMF8829_GRID_N];              // stable snapshot (host reads)
volatile uint32_t tmf8829_grid_seq;                          // bumps per full snapshot

// ---------------------------------------------------------------------------
// Assemble a result frame (in frame_buf, `total` bytes) into the full grid.
//
// Layout authoritative from the ams Python driver (getPixelResultsFromFrame +
// getFullPixelResult):
//   - 16-byte frame header occupies frame_buf[0..15]; pixel data starts at
//     frame_buf[16] (NOT 12 — frame_buf[12..15] is the header's refPos field).
//   - Per-pixel size derived from the layout byte (frame_buf[1]):
//       size = nr_peaks*(3 + 2*useSignal) + 2*useNoise + 2*useXtalk
//     distance (u16 LE, 0.25 mm) is the first peak field, after the optional
//     noise/xtalk fields -> distOff = 2*useNoise + 2*useXtalk.
//   - 48x32 is split into two sub-frames of 16 rows x 48 cols (768 pixels),
//     row-major.  The sub_result bit (layout & 0x40) selects which: the two
//     are INTERLEAVED BY ROW (insert(row*2+1, ...) in the driver):
//       sub_result=0 -> image rows 0,2,4,...   sub_result=1 -> rows 1,3,5,...
//     So a sub-frame pixel i maps to image (row = 2*(i/48) + sub, col = i%48).
//   - A stable full-grid snapshot is published after the sub_result=1 frame.
// ---------------------------------------------------------------------------
static void tmf8829_assemble_frame(uint32_t total)
{
    uint8_t layout   = frame_buf[1];
    uint8_t sub      = (layout & 0x40u) ? 1 : 0;
    uint8_t numPeak  = layout & 0x07u;
    uint8_t useSig   = (layout & 0x08u) ? 1 : 0;
    uint8_t useNoise = (layout & 0x10u) ? 1 : 0;
    uint8_t useXtalk = (layout & 0x20u) ? 1 : 0;
    uint8_t pixSize  = (uint8_t)(numPeak * (3 + 2 * useSig) + 2 * useNoise + 2 * useXtalk);
    uint8_t distOff  = (uint8_t)(2 * useNoise + 2 * useXtalk);
    if (pixSize == 0) { pixSize = 3; distOff = 0; }   // safety: nr_peaks=1, no extras

    uint32_t pixbytes = (total > TMF8829_PIX_OFF) ? (total - TMF8829_PIX_OFF) : 0;
    uint32_t npix     = pixbytes / pixSize;
    uint32_t maxpix   = TMF8829_SUB_ROWS * TMF8829_GRID_W;   // 768
    if (npix > maxpix) npix = maxpix;

    uint8_t peakStride = (uint8_t)(3 + 2 * useSig);   // bytes per peak field
    for (uint32_t i = 0; i < npix; i++) {
        uint32_t base = TMF8829_PIX_OFF + i * pixSize + distOff;

        // Pick the first peak beyond the cover-glass cutoff.  The near return
        // (cover glass ~1 mm) is reported as peak 0; skip it and use the next
        // real object.  Distances are in 0.25 mm units.
        uint16_t d = 0;
        for (uint8_t p = 0; p < numPeak; p++) {
            uint32_t o = base + p * peakStride;
            uint16_t dp = (uint16_t)frame_buf[o] | ((uint16_t)frame_buf[o + 1] << 8);
            if (dp > TMF8829_COVER_CUTOFF_UQ) { d = dp; break; }
        }

        uint32_t y = i / TMF8829_GRID_W;              // 0..15 within the sub-frame
        uint32_t x = i % TMF8829_GRID_W;              // 0..47
        uint32_t row = 2 * y + sub;                   // interleave into full image
        uint32_t col = (TMF8829_GRID_W - 1) - x;      // optics mirror the scene L<->R
        tmf8829_grid_work[row * TMF8829_GRID_W + col] = (uint16_t)(d / 4);  // mm
    }

    // Publish a stable full-grid snapshot once the second sub-frame is in.
    if (sub == 1) {
        memcpy((void *)tmf8829_grid, tmf8829_grid_work, sizeof(tmf8829_grid_work));
        tmf8829_grid_seq++;
    }
}

// ---------------------------------------------------------------------------
// Public: Poll for a measurement result
//
// Frame read protocol (matches driver tmf8829ReadResults):
//   1. Read PRE_HEADER(5 @0xFA) + FRAME_HEADER(16 @0xFF) = 21 bytes.  The read
//      auto-increments 0xFA..0xFE then flows into the FIFO at 0xFF.
//   2. frame ID = hdr[5]; results frame if (fid & 0xF0)==0x10.
//   3. payload size = u16 at hdr[7]; remaining payload = payload - (16-4).
//   4. Read remaining payload from FIFO (0xFF), verify EOF marker 0xE0F7.
// ---------------------------------------------------------------------------
int TMF8829_Poll(const uint8_t **frame_out, uint32_t *frame_size)
{
    tmf8829_poll_iters++;

    uint8_t int_st;
    if (reg_read_byte(TMF8829_REG_INT_STATUS, &int_st) != 0) return -1;
    tmf8829_int_seen |= int_st;
    if (!(int_st & TMF8829_INT_RESULT)) return 0;

    // Clear the result interrupt
    reg_write_byte(TMF8829_REG_INT_STATUS, TMF8829_INT_RESULT);

    // 1. Read pre-header + frame header (21 bytes) starting at FIFOSTATUS
    uint8_t hdr[TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE];
    if (reg_read(TMF8829_REG_FIFOSTATUS, hdr, sizeof(hdr)) != 0) return -1;

    uint8_t  fid     = hdr[TMF8829_PRE_HEADER_SIZE];          // frame header byte 0
    uint16_t payload = (uint16_t)hdr[TMF8829_PRE_HEADER_SIZE + 2]
                     | ((uint16_t)hdr[TMF8829_PRE_HEADER_SIZE + 3] << 8);
    tmf8829_last_fid     = fid;
    tmf8829_last_payload = payload;

    if ((fid & TMF8829_FID_MASK) != TMF8829_FID_RESULTS) return 0;

    // Copy the 16-byte frame header into the output buffer
    memcpy(frame_buf, hdr + TMF8829_PRE_HEADER_SIZE, TMF8829_FRAME_HEADER_SIZE);
    uint32_t total = TMF8829_FRAME_HEADER_SIZE;

    // 3+4. Remaining payload after the header bytes already read
    int32_t to_read = (int32_t)payload
                    - (TMF8829_FRAME_HEADER_SIZE - TMF8829_FRAME_HEADER_OFFSET);
    while (to_read > 0 && total + 128 <= TMF8829_FIFO_HW_SIZE) {
        uint16_t chunk = (to_read > 128) ? 128 : (uint16_t)to_read;
        if (reg_read(TMF8829_REG_FIFO, frame_buf + total, chunk) != 0) break;
        total   += chunk;
        to_read -= chunk;
    }

    // EOF marker = last 2 bytes
    if (total >= 2)
        tmf8829_last_eof = (uint16_t)frame_buf[total - 2]
                         | ((uint16_t)frame_buf[total - 1] << 8);

    tmf8829_assemble_frame(total);

    *frame_out  = frame_buf;
    *frame_size = total;
    return 1;
}

// ===========================================================================
// Interrupt + DMA driven frame reception
// ===========================================================================

volatile uint8_t tmf8829_frame_pending;          // set by PB6 EXTI ISR
static volatile uint8_t spi_dma_done;            // set by SPI TxRx DMA complete

// Stage-B diagnostics
volatile int      tmf8829_rf_step;     // last step reached in ReadFrame
volatile int      tmf8829_rf_dma_rc;   // reg_read_dma return code
volatile int      tmf8829_dma_hal_rc;  // HAL_SPI_TransmitReceive_DMA HAL status
volatile uint32_t tmf8829_dma_err;     // hspi1.ErrorCode after DMA
volatile uint8_t  tmf8829_rf_int_st;   // INT_STATUS seen in ReadFrame
volatile uint8_t  tmf8829_rf_fid;      // FID seen
volatile uint16_t tmf8829_rf_payload;  // payload seen
volatile uint32_t tmf8829_rf_calls;    // ReadFrame invocations
volatile uint32_t tmf8829_edge_count;  // PB6 falling edges seen by EXTI
volatile uint32_t tmf8829_loop_iters;  // main-loop iterations (liveness)
volatile uint32_t tmf8829_dma_ok;      // successful DMA chunk reads (USE_DMA=1)
volatile uint32_t tmf8829_dma_fail;    // failed DMA chunk reads (USE_DMA=1)
volatile uint32_t tmf8829_dma_state;   // hspi1.State at a DMA failure

// GPDMA1 channels linked to SPI1 (configured by hand — no CubeMX DMA in .ioc)
static DMA_HandleTypeDef hdma_spi1_rx;           // GPDMA1_Channel0
static DMA_HandleTypeDef hdma_spi1_tx;           // GPDMA1_Channel1

// ---------------------------------------------------------------------------
// SPI complete callback (DMA) — fires for TransmitReceive_DMA on SPI1
// ---------------------------------------------------------------------------
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) spi_dma_done = 1;
}

// ---------------------------------------------------------------------------
// DMA-based register read: TX [0x03, reg, 0x00, zeros...] / RX into spi_rx,
// data lands at spi_rx[3..].  Waits for completion (DMA offloads the CPU).
// ---------------------------------------------------------------------------
// Fully reset the SPI + both GPDMA channels after a failed DMA transfer.
// HAL_SPI_Abort alone does not restore a GPDMA channel that was left mid-
// transfer, so HAL_DMA_Start_IT keeps failing forever.  Abort everything and
// re-establish the SPI<->DMA linkage so the next transfer can start clean.
static void dma_channel_init(DMA_HandleTypeDef *h, DMA_Channel_TypeDef *inst,
                             uint32_t request, uint32_t dir,
                             uint32_t srcInc, uint32_t dstInc);

static void spi_dma_recover(void)
{
    HAL_SPI_Abort(&hspi1);
    HAL_DMA_DeInit(&hdma_spi1_rx);
    HAL_DMA_DeInit(&hdma_spi1_tx);
    dma_channel_init(&hdma_spi1_rx, GPDMA1_Channel0, GPDMA1_REQUEST_SPI1_RX,
                     DMA_PERIPH_TO_MEMORY, DMA_SINC_FIXED, DMA_DINC_INCREMENTED);
    dma_channel_init(&hdma_spi1_tx, GPDMA1_Channel1, GPDMA1_REQUEST_SPI1_TX,
                     DMA_MEMORY_TO_PERIPH, DMA_SINC_INCREMENTED, DMA_DINC_FIXED);
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);
}

// Kept available behind TMF8829_FIFO_USE_DMA; see the header for why the
// blocking path is the default.
__attribute__((unused))
static int reg_read_dma(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint16_t n = (uint16_t)(len + 3);
    if (n > SPI_BUF_MAX) return -1;

    memset(spi_tx, 0, n);
    spi_tx[0] = TMF8829_SPI_RD;
    spi_tx[1] = reg;
    spi_dma_done = 0;
    tmf8829_dma_hal_rc = HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx, spi_rx, n);
    if (tmf8829_dma_hal_rc != HAL_OK) {
        tmf8829_dma_state = hspi1.State;
        tmf8829_dma_fail++;
        spi_dma_recover();
        return -1;
    }
    uint32_t t0 = HAL_GetTick();
    while (!spi_dma_done) {
        if ((HAL_GetTick() - t0) > 20) {
            tmf8829_dma_err = hspi1.ErrorCode;
            tmf8829_dma_fail++;
            spi_dma_recover();
            return -2;
        }
    }
    memcpy(data, spi_rx + 3, len);
    tmf8829_dma_ok++;
    return 0;
}

// ---------------------------------------------------------------------------
// Configure one GPDMA channel for SPI1 and link it to hspi1
// ---------------------------------------------------------------------------
static void dma_channel_init(DMA_HandleTypeDef *h, DMA_Channel_TypeDef *inst,
                             uint32_t request, uint32_t dir,
                             uint32_t srcInc, uint32_t dstInc)
{
    h->Instance                          = inst;
    h->Init.Request                      = request;
    h->Init.BlkHWRequest                 = DMA_BREQ_SINGLE_BURST;
    h->Init.Direction                    = dir;
    h->Init.SrcInc                       = srcInc;
    h->Init.DestInc                      = dstInc;
    h->Init.SrcDataWidth                 = DMA_SRC_DATAWIDTH_BYTE;
    h->Init.DestDataWidth                = DMA_DEST_DATAWIDTH_BYTE;
    h->Init.Priority                     = DMA_LOW_PRIORITY_LOW_WEIGHT;
    h->Init.SrcBurstLength               = 1;
    h->Init.DestBurstLength              = 1;
    h->Init.TransferAllocatedPort        = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    h->Init.TransferEventMode            = DMA_TCEM_BLOCK_TRANSFER;
    h->Init.Mode                         = DMA_NORMAL;
    HAL_DMA_Init(h);
    HAL_DMA_ConfigChannelAttributes(h, DMA_CHANNEL_NPRIV);
}

void TMF8829_DMA_Init(void)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    dma_channel_init(&hdma_spi1_rx, GPDMA1_Channel0, GPDMA1_REQUEST_SPI1_RX,
                     DMA_PERIPH_TO_MEMORY, DMA_SINC_FIXED, DMA_DINC_INCREMENTED);
    dma_channel_init(&hdma_spi1_tx, GPDMA1_Channel1, GPDMA1_REQUEST_SPI1_TX,
                     DMA_MEMORY_TO_PERIPH, DMA_SINC_INCREMENTED, DMA_DINC_FIXED);

    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

    // On STM32H5 the SPI HAL signals DMA completion via the SPI End-Of-Transfer
    // interrupt, so the SPI global IRQ must be enabled too.
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

// ---------------------------------------------------------------------------
// PB6 (TMF8829 INT, active-low) falling-edge EXTI
// ---------------------------------------------------------------------------
void TMF8829_EXTI_Init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_6;
    g.Mode = GPIO_MODE_IT_FALLING;   // INT is active-low / open-drain
    g.Pull = GPIO_PULLUP;            // idle-high via internal pull-up
    HAL_GPIO_Init(GPIOB, &g);

    HAL_NVIC_SetPriority(EXTI6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI6_IRQn);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_6) { tmf8829_frame_pending = 1; tmf8829_edge_count++; }
}

// ---------------------------------------------------------------------------
// IRQ handlers (weak HAL symbols overridden here)
// ---------------------------------------------------------------------------
void EXTI6_IRQHandler(void)            { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6); }
void GPDMA1_Channel0_IRQHandler(void)  { HAL_DMA_IRQHandler(&hdma_spi1_rx); }
void GPDMA1_Channel1_IRQHandler(void)  { HAL_DMA_IRQHandler(&hdma_spi1_tx); }
void SPI1_IRQHandler(void)             { HAL_SPI_IRQHandler(&hspi1); }

// ---------------------------------------------------------------------------
// Read + assemble the pending result frame.  Header via blocking SPI (small),
// the bulk payload via one DMA transfer.
// ---------------------------------------------------------------------------
int TMF8829_ReadFrame(void)
{
    tmf8829_rf_calls++;
    tmf8829_rf_step = 1;
    uint8_t int_st;
    if (reg_read_byte(TMF8829_REG_INT_STATUS, &int_st) != 0) return -1;
    tmf8829_rf_int_st = int_st;
    if (!(int_st & TMF8829_INT_RESULT)) return 0;
    reg_write_byte(TMF8829_REG_INT_STATUS, TMF8829_INT_RESULT);   // clear -> deasserts INT

    tmf8829_rf_step = 2;
    uint8_t hdr[TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE];
    if (reg_read(TMF8829_REG_FIFOSTATUS, hdr, sizeof(hdr)) != 0) return -1;

    uint8_t  fid     = hdr[TMF8829_PRE_HEADER_SIZE];
    uint16_t payload = (uint16_t)hdr[TMF8829_PRE_HEADER_SIZE + 2]
                     | ((uint16_t)hdr[TMF8829_PRE_HEADER_SIZE + 3] << 8);
    tmf8829_rf_fid     = fid;
    tmf8829_rf_payload = payload;
    if ((fid & TMF8829_FID_MASK) != TMF8829_FID_RESULTS) return 0;

    memcpy(frame_buf, hdr + TMF8829_PRE_HEADER_SIZE, TMF8829_FRAME_HEADER_SIZE);
    uint32_t total = TMF8829_FRAME_HEADER_SIZE;

    tmf8829_rf_step = 3;
    int32_t to_read = (int32_t)payload - (TMF8829_FRAME_HEADER_SIZE - TMF8829_FRAME_HEADER_OFFSET);
    // The device FIFO (0xFF) must be read in bounded transactions: a single
    // giant CS-held read desyncs its result engine and the sensor stops
    // raising INT after a variable number of frames.  Read in fixed chunks —
    // the same transaction pattern that streams indefinitely when blocking.
    while (to_read > 0 && total + TMF8829_FIFO_CHUNK <= TMF8829_FIFO_HW_SIZE) {
        uint16_t chunk = (to_read > TMF8829_FIFO_CHUNK)
                       ? TMF8829_FIFO_CHUNK : (uint16_t)to_read;
#if TMF8829_FIFO_USE_DMA
        tmf8829_rf_dma_rc = reg_read_dma(TMF8829_REG_FIFO, frame_buf + total, chunk);
        if (tmf8829_rf_dma_rc != 0) break;
#else
        if (reg_read(TMF8829_REG_FIFO, frame_buf + total, chunk) != 0) break;
#endif
        total   += chunk;
        to_read -= chunk;
    }

    tmf8829_rf_step = 4;
    tmf8829_assemble_frame(total);
    return 1;
}
