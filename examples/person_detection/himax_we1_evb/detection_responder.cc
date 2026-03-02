/* HiMax WE-I Plus — Person Detection with JPEG Camera Streaming
 *
 * Drop-in replacement for the standard detection_responder.cc.
 * Adds JPEG frame output over UART so the dashboard can show live video.
 *
 * Frame protocol (binary, sent before the score text line):
 *   "IMGSTART"          (8 bytes, ASCII delimiter)
 *   <JPEG bytes>        (variable length)
 *   "IMGEND__"          (8 bytes, ASCII delimiter)
 *   "person score:X no person score Y\n"  (text, as usual)
 *
 * Build: replace examples/person_detection/himax_we1_evb/detection_responder.cc
 *        with this file, then `make person_detection_int8 && make flash example=person_detection_int8`
 */

#if defined(ARDUINO)
#define ARDUINO_EXCLUDE_CODE
#endif

#ifndef ARDUINO_EXCLUDE_CODE

#include "examples/person_detection/detection_responder.h"
#include "examples/person_detection/image_provider.h"
#include "examples/person_detection/model_settings.h"
#include "hx_drv_tflm.h"

// External camera config defined in image_provider.cc
extern hx_drv_sensor_image_config_t g_pimg_config;

// JPEG output buffer (640x480 YUV400 compresses to well under 40KB at q=50)
static uint8_t g_jpeg_buf[48 * 1024];
static uint32_t g_jpeg_size = 0;

// Frame delimiters (must match server.py)
static const char FRAME_START[] = "IMGSTART";
static const char FRAME_END[]   = "IMGEND__";

// Only stream camera every Nth inference to avoid saturating UART
// At 115200 baud, a 20KB JPEG takes ~1.7s to transmit.
// Set STREAM_EVERY_N = 1 to stream every frame (slower inference loop),
// or increase to skip frames (faster score updates, occasional image).
#ifndef STREAM_EVERY_N
#define STREAM_EVERY_N 3
#endif

static int g_frame_counter = 0;

void RespondToDetection(tflite::ErrorReporter* error_reporter,
                        int8_t person_score, int8_t no_person_score) {
  // LED indication
  if (person_score > no_person_score) {
    hx_drv_led_on(HX_DRV_LED_GREEN);
  } else {
    hx_drv_led_off(HX_DRV_LED_GREEN);
  }

  // Stream JPEG frame every STREAM_EVERY_N inferences
  if ((g_frame_counter++ % STREAM_EVERY_N) == 0) {
    // Configure JPEG encoder: quality 50, output to g_jpeg_buf
    hx_drv_jpeg_config_t jpeg_cfg;
    jpeg_cfg.jpeg_enctype   = HX_DRV_JPEG_ENC_TYPE_SW;
    jpeg_cfg.enc_width      = g_pimg_config.img_width;
    jpeg_cfg.enc_height     = g_pimg_config.img_height;
    jpeg_cfg.jpeg_quality   = 50;
    jpeg_cfg.raw_address    = g_pimg_config.raw_address;
    jpeg_cfg.jpeg_address   = (uint32_t)g_jpeg_buf;
    jpeg_cfg.jpeg_buf_size  = sizeof(g_jpeg_buf);

    hx_drv_jpeg_init(&jpeg_cfg);
    if (hx_drv_jpeg_encode(&jpeg_cfg) == HX_DRV_LIB_PASS) {
      g_jpeg_size = jpeg_cfg.jpeg_size;

      // Transmit: delimiter + JPEG + delimiter
      hx_drv_uart_print((char*)FRAME_START);
      hx_drv_uart_write(g_jpeg_buf, g_jpeg_size);
      hx_drv_uart_print((char*)FRAME_END);
    }
  }

  // Always print scores (parsed by server.py)
  TF_LITE_REPORT_ERROR(error_reporter,
                       "person score:%d no person score %d",
                       person_score, no_person_score);
}

#endif  // ARDUINO_EXCLUDE_CODE
