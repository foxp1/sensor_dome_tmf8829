#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Register addresses (from TMF8829 datasheet DS001140 v2-00)
// ---------------------------------------------------------------------------
#define TMF8829_REG_APP_ID           0x00
#define TMF8829_REG_MAJOR            0x01
#define TMF8829_REG_MINOR            0x02
#define TMF8829_REG_CMD_STAT         0x08
#define TMF8829_REG_BL_PAYLOAD       0x09  // bootloader: payload byte count
#define TMF8829_REG_BL_ADDR0         0x0A  // bootloader: dest address LSB
#define TMF8829_REG_BL_ADDR1         0x0B
#define TMF8829_REG_BL_ADDR2         0x0C
#define TMF8829_REG_BL_ADDR3         0x0D  // bootloader: dest address MSB
#define TMF8829_REG_BL_WSIZE0        0x0E  // bootloader: word count LSB
#define TMF8829_REG_BL_WSIZE1        0x0F  // bootloader: word count MSB
#define TMF8829_REG_LIVE_BEAT0       0x1A
#define TMF8829_REG_LIVE_BEAT1       0x1B
#define TMF8829_REG_CID_RID          0x20  // frame/page type ID
#define TMF8829_REG_PAYLOAD_SIZE     0x21  // payload byte count in page
#define TMF8829_REG_PERIOD_LSB       0x22  // measurement period ms (LSB)
#define TMF8829_REG_PERIOD_MSB       0x23  // measurement period ms (MSB)
#define TMF8829_REG_ITERATIONS_LSB   0x24
#define TMF8829_REG_ITERATIONS_MSB   0x25
#define TMF8829_REG_FP_MODE          0x26  // resolution / focal plane mode
#define TMF8829_REG_RESULT_FORMAT    0x2A  // nr_peaks, optional fields
#define TMF8829_REG_INT_STATUS       0xE1
#define TMF8829_REG_INT_ENAB         0xE2
#define TMF8829_REG_CHIP_ID          0xE3  // fixed 0x9E
#define TMF8829_REG_REVID            0xE4
#define TMF8829_REG_INTERFACE        0xE9
#define TMF8829_REG_RESET            0xF7
#define TMF8829_REG_ENABLE           0xF8
#define TMF8829_REG_FIFOSTATUS       0xFA
#define TMF8829_REG_FIFO             0xFF  // FIFO access; no auto-increment

// ---------------------------------------------------------------------------
// Application-mode commands  (write to CMD_STAT 0x08)
// ---------------------------------------------------------------------------
#define TMF8829_CMD_MEASURE          0x10
#define TMF8829_CMD_CLEAR_STATUS     0x11
#define TMF8829_CMD_W_PAGE_MEASURE   0x14  // write config page then start
#define TMF8829_CMD_WRITE_PAGE       0x15
#define TMF8829_CMD_LOAD_CONFIG      0x16
#define TMF8829_CMD_LOAD_8X8         0x40  // preset: 8x8 default   (64 zones)
#define TMF8829_CMD_LOAD_16X16       0x43  // preset: 16x16 default (256 zones)
#define TMF8829_CMD_LOAD_32X32       0x45  // preset: 32x32 default
#define TMF8829_CMD_LOAD_48X32       0x47  // preset: 48x32 default (1536 zones)
#define TMF8829_CMD_LOAD_48X32_HA    0x48  // preset: 48x32 high accuracy
#define TMF8829_CMD_STOP             0xFF

// ---------------------------------------------------------------------------
// Bootloader commands  (write to CMD_STAT 0x08 while in BL mode)
// ---------------------------------------------------------------------------
#define TMF8829_BL_START_RAM         0x16
#define TMF8829_BL_I2C_OFF           0x22  // disable I2C, lock to SPI
#define TMF8829_BL_SPI_OFF           0x20
#define TMF8829_BL_FIFO_BOTH         0x45  // setup FIFO write to both CPUs

// ---------------------------------------------------------------------------
// Status values  (read from CMD_STAT 0x08)
// ---------------------------------------------------------------------------
#define TMF8829_STAT_OK              0x00
#define TMF8829_STAT_ACCEPTED        0x01

// ---------------------------------------------------------------------------
// ENABLE register (0xF8) bits
// ---------------------------------------------------------------------------
#define TMF8829_ENABLE_CPU_READY     (1u << 7)
#define TMF8829_ENABLE_PON           (1u << 2)

// ---------------------------------------------------------------------------
// INT_STATUS / INT_ENAB (0xE1 / 0xE2) bits
// ---------------------------------------------------------------------------
#define TMF8829_INT_RESULT           (1u << 0)
#define TMF8829_INT_MOTION           (1u << 1)
#define TMF8829_INT_PROX             (1u << 2)
#define TMF8829_INT_HISTOGRAM        (1u << 3)

// ---------------------------------------------------------------------------
// FIFOSTATUS (0xFA) bits
// ---------------------------------------------------------------------------
#define TMF8829_FIFO_DIR             (1u << 0)  // 0=TX(dev→host), 1=RX
#define TMF8829_FIFO_DMA_BUSY        (1u << 1)  // device still filling FIFO
#define TMF8829_FIFO_EMPTY           (1u << 2)  // no data for host to read

// ---------------------------------------------------------------------------
// APP_ID values
// ---------------------------------------------------------------------------
#define TMF8829_APPID_BOOTLOADER     0x80
#define TMF8829_APPID_APP            0x01
#define TMF8829_CHIP_ID_VALUE        0x9E

// ---------------------------------------------------------------------------
// Result frame format (from driver tmf8829.h)
//   Read starts at FIFOSTATUS (0xFA): 5 bytes pre-header (status+systick)
//   span 0xFA..0xFE, then the read flows into FIFO (0xFF) for the 16-byte
//   frame header, then the payload.
// ---------------------------------------------------------------------------
#define TMF8829_PRE_HEADER_SIZE      5       // 0xFA + systick 0xFB..0xFE
#define TMF8829_FRAME_HEADER_SIZE    16      // frame header from FIFO
#define TMF8829_FRAME_HEADER_OFFSET  4       // header bytes not counted in payload
#define TMF8829_FRAME_FOOTER_SIZE    12
#define TMF8829_FRAME_EOF_SIZE       2
#define TMF8829_FRAME_EOF            0xE0F7  // end-of-frame marker
#define TMF8829_FID_RESULTS          0x10
#define TMF8829_FID_HISTOGRAMS       0x20
#define TMF8829_FID_MASK             0xF0

// ---------------------------------------------------------------------------
// SPI wire protocol  (datasheet section 7.11)
// ---------------------------------------------------------------------------
#define TMF8829_SPI_WR               0x02  // write command code
#define TMF8829_SPI_RD               0x03  // read command code
#define TMF8829_BL_CHUNK             128   // max firmware bytes per FIFO write

// ---------------------------------------------------------------------------
// Internal FIFO size  (device hardware)
// ---------------------------------------------------------------------------
#define TMF8829_FIFO_HW_SIZE         8192

// RESULT_FORMAT (0x2A): bits 2:0 = nr_peaks (1..4), bit3 signal, bit4 noise,
// bit5 xtalk.  We request 2 peaks per pixel so the firmware can skip the near
// cover-glass return and report the next (real) peak.  min_distance_uq was
// removed in datasheet v2-00, and proper crosstalk suppression needs ODG/
// factory calibration, so multi-peak selection is the practical approach.
#define TMF8829_RESULT_NR_PEAKS   2
// Peaks at or below this distance (0.25 mm units) are treated as cover-glass
// crosstalk and skipped.  ~10 mm cutoff for a cover glass ~1 mm away.
#define TMF8829_COVER_CUTOFF_UQ   40   // 40 * 0.25 mm = 10 mm

// FIFO must be read in bounded, CS-toggled transactions: a single long
// CS-held read stalls the sensor's result engine.  This forces many small
// (128-byte) reads per frame.  Per-chunk *DMA* on the STM32H5 GPDMA wedges
// irrecoverably after an initial burst (EOT lost, channel un-restartable even
// after full DeInit/Init), whereas chunked *blocking* reads stream forever.
// So the reliable path is interrupt-driven (EXTI) servicing with blocking
// chunked SPI reads — the CPU is idle except for the brief per-frame read.
// The DMA path is kept behind this switch for future experimentation.
#define TMF8829_FIFO_USE_DMA 0   // 1 = DMA per chunk (unstable), 0 = blocking
#define TMF8829_FIFO_CHUNK   128 // bytes per FIFO read transaction

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise: hard-reset, download firmware, verify app mode.
// Returns 0 on success, negative error code on failure.
int TMF8829_Init(void);

// Load 48x32 preset, set 15 fps period, write config and start measurement.
// Returns 0 on success, negative error code on failure.
int TMF8829_Configure_48x32_15fps(void);

// Poll for a completed measurement frame.
// On success sets *frame_out to an internal static buffer and *frame_size
// to the number of raw FIFO bytes read.  Buffer is valid until the next call.
// Returns  1  if a frame is ready
//          0  if no frame yet
//         -1  on communication error
int TMF8829_Poll(const uint8_t **frame_out, uint32_t *frame_size);

// ---------------------------------------------------------------------------
// Interrupt + DMA driven frame reception
// ---------------------------------------------------------------------------
// Set by the PB6 (INT) EXTI ISR when the sensor signals a result is ready.
// The main loop clears it and calls TMF8829_ReadFrame().
extern volatile uint8_t tmf8829_frame_pending;

// Configure GPDMA1 channels for SPI1 TX/RX and link them to hspi1.
// Call once after MX_SPI1_Init / before measuring.
void TMF8829_DMA_Init(void);

// Configure PB6 as a falling-edge EXTI (TMF8829 INT is active-low) and enable
// the EXTI NVIC line.  Call after the measurement has been started.
void TMF8829_EXTI_Init(void);

// Read + assemble the pending result frame (uses SPI DMA for the bulk read).
// Returns 1 if a result frame was assembled, 0 if none, <0 on error.
int TMF8829_ReadFrame(void);

extern volatile uint32_t tmf8829_loop_iters;
