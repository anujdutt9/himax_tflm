/* HiMax WE-I Plus — Person Detection with Base64 JPEG Camera Streaming
 *
 * Uses the hardware JPEG encoder output already produced by hx_drv_sensor_capture()
 * (stored at g_pimg_config.jpeg_address / jpeg_size) and transmits it as
 * base64-encoded text over the hardware UART so the dashboard can show live video.
 *
 * Frame protocol (text lines, sent before the score line):
 *   "IMGB64START\n"
 *   <base64 JPEG data, 60-char lines>
 *   "IMGB64END\n"
 *   "person score:X no person score Y\n"   (parsed by server.py as usual)
 *
 * Build: replace examples/person_detection/himax_we1_evb/detection_responder.cc
 *        then `make person_detection_int8 && make flash example=person_detection_int8`
 */

#if defined(ARDUINO)
#define ARDUINO_EXCLUDE_CODE
#endif

#ifndef ARDUINO_EXCLUDE_CODE

#include "examples/person_detection/detection_responder.h"
#include "examples/person_detection/image_provider.h"
#include "hx_drv_tflm.h"

// Defined in image_provider.cc; populated by hx_drv_sensor_capture()
extern hx_drv_sensor_image_config_t g_pimg_config;

// Stream JPEG every N inferences to avoid saturating UART.
// At 115200 baud, a 15 KB JPEG encodes to ~20 KB base64 => ~1.7 s per frame.
// Increase STREAM_EVERY_N to get more frequent score updates.
#ifndef STREAM_EVERY_N
#define STREAM_EVERY_N 3
#endif

static int g_frame_counter = 0;

static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Encode 'len' bytes and transmit as base64 via hardware UART.
// Sends 60-char lines (45 source bytes each).
static void uart_send_base64(const uint8_t *data, uint32_t len) {
    char line[65];  // 60 chars + '\n' + '\0'
    uint32_t i = 0;
    while (i < len) {
        int lpos = 0;
        while (lpos < 60 && i < len) {
            uint32_t rem = len - i;
            uint8_t b0 = data[i];
            uint8_t b1 = (rem > 1) ? data[i + 1] : 0;
            uint8_t b2 = (rem > 2) ? data[i + 2] : 0;

            line[lpos++] = B64[b0 >> 2];
            line[lpos++] = B64[((b0 & 0x03) << 4) | (b1 >> 4)];
            line[lpos++] = (rem > 1) ? B64[((b1 & 0x0f) << 2) | (b2 >> 6)] : '=';
            line[lpos++] = (rem > 2) ? B64[b2 & 0x3f] : '=';

            i += (rem > 2) ? 3 : rem;
        }
        line[lpos++] = '\n';
        line[lpos]   = '\0';
        hx_drv_uart_print("%s", line);
    }
}

void RespondToDetection(tflite::ErrorReporter* error_reporter,
                        int8_t person_score, int8_t no_person_score) {
    // LED indication
    if (person_score > no_person_score) {
        hx_drv_led_on(HX_DRV_LED_GREEN);
    } else {
        hx_drv_led_off(HX_DRV_LED_GREEN);
    }

    // Stream hardware JPEG every STREAM_EVERY_N inferences
    if ((g_frame_counter++ % STREAM_EVERY_N) == 0 && g_pimg_config.jpeg_size > 0) {
        hx_drv_uart_print("IMGB64START\n");
        uart_send_base64(
            reinterpret_cast<const uint8_t*>(g_pimg_config.jpeg_address),
            g_pimg_config.jpeg_size);
        hx_drv_uart_print("IMGB64END\n");
    }

    // Always print scores (parsed by server.py)
    TF_LITE_REPORT_ERROR(error_reporter,
                         "person score:%d no person score %d",
                         person_score, no_person_score);
}

#endif  // ARDUINO_EXCLUDE_CODE
