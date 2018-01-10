/*
 *  Copyright (c) 2017 Medium One, Inc
 *  www.mediumone.com
 *
 *  Portions of this work may be based on third party contributions.
 *  Medium One, Inc reserves copyrights to this work whose
 *  license terms are defined under a separate Software License
 *  Agreement (SLA).  Re-distribution of any or all of this work,
 *  in source or binary form, is prohibited unless authorized by
 *  Medium One, Inc under SLA.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "net_thread.h"
#include "system_thread.h"
#include "nxd_dns.h"
#include "nxd_dhcp_client.h"
#include "fx_stdio.h"
#include "led.h"
#include "agg.h"
#include "app.h"
#include "file_transfer.h"
#include "helpers.h"

#include <m1_agent.h>
#ifdef USE_M1DIAG
#include <m1diagnostics_agent.h>
#endif

#define DATA_FLASH_BLOCK_SIZE 64
#define PROVISIONED_FLAG 1


extern TX_THREAD system_thread;
extern NX_IP g_http_ip;
extern NX_DNS g_dns_client;


/*
 * global sensor aggregates
 *
 * sensor threads sample their sensors and aggregate in local aggregate
 * structures.
 * when a transfer is requested, the sensor thread copies the local aggregate
 * data into the corresponding global structure below.
 */
volatile agg_t g_x = { .name = "x_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y = { .name = "y_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z = { .name = "z_accel", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_x_mag = { .name = "x_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_y_mag = { .name = "y_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_z_mag = { .name = "z_mag", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp1 = { .name = "temp1", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp2 = { .name = "temp2", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_temp3 = { .name = "temp3", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_humidity = { .name = "humidity", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_pressure = { .name = "pressure", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_mic = { .name = "mic", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile agg_t g_voice_fft = { .name = "voice_fft", .total = 0, .min = 0, .max = 0, .count = 0, .last_sent = 0, .value = 0, .threshold = 0, .absolute_threshold = 0, .last_sent_tick = 0 };
volatile uint32_t g_x_zero_crossings = 0, g_y_zero_crossings = 0, g_z_zero_crossings = 0;

app_config_t app_config;

#ifdef USE_M1DIAG
/*
 * the diagnostics agent asynchronously samples peripherals.
 * to ensure it sees real-time updated data, we expose the "local" aggregate
 * structures used by the sensor threads
 */
extern volatile agg_t x;
extern volatile agg_t y;
extern volatile agg_t z;
extern volatile agg_t temp1;
extern volatile agg_t x_mag;
extern volatile agg_t y_mag;
extern volatile agg_t z_mag;
extern volatile agg_t temp2;
extern volatile agg_t humidity;
extern volatile agg_t temp3;
extern volatile agg_t pressure;
extern volatile agg_t mic;
extern volatile agg_t voice_fft;

static periphery_access_t x_accel_access = {(char)1, NULL, {.address=&x.value}};
static periphery_access_t y_accel_access = {(char)1, NULL, {.address=&y.value}};
static periphery_access_t z_accel_access = {(char)1, NULL, {.address=&z.value}};
static periphery_access_t temp_access = {(char)1, NULL, {.address=&temp1.value}};
static periphery_access_t xmag_access = {(char)1, NULL, {.address=&x_mag.value}};
static periphery_access_t ymag_access = {(char)1, NULL, {.address=&y_mag.value}};
static periphery_access_t zmag_access = {(char)1, NULL, {.address=&z_mag.value}};
static periphery_access_t temp2_access = {(char)1, NULL, {.address=&temp2.value}};
static periphery_access_t humidity_access = {(char)1, NULL, {.address=&humidity.value}};
static periphery_access_t temp3_access = {(char)1, NULL, {.address=&temp3.value}};
static periphery_access_t pressure_access = {(char)1, NULL, {.address=&pressure.value}};
static periphery_access_t mic_access = {(char)1, NULL, {.address=&mic.value}};
static periphery_access_t voice_fft_access = {(char)1, NULL, {.address=&voice_fft.value}};

#ifdef USE_TLS
#define DIAG_SSL_MEM_SIZE (8 * 1024)
static char diag_ssl_mem[DIAG_SSL_MEM_SIZE];
#endif
#endif

#ifdef USE_TLS
#define SSL_MEM_SIZE (4 * 1024)
static char ssl_mem[SSL_MEM_SIZE];
#endif
#define MQTT_STACK_SIZE (8 * 1024)
static char mqtt_stack[MQTT_STACK_SIZE];
#define MQTT_MSG_MEM_SIZE (10 * 1024)
static char * mqtt_msg_mem[MQTT_MSG_MEM_SIZE];
#define CRYPTO_MEM_SIZE (4 * 1024)
char * crypto_mem[CRYPTO_MEM_SIZE];
#define MAX_CERTS 4
#define CERT_MEM_SIZE (MAX_CERTS * 2048)
char * cert_mem[CERT_MEM_SIZE];


#ifdef USE_TLS
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers_synergys7;
static const uint8_t uguu_root[] = {0x30, 0x82, 0x4, 0x92, 0x30, 0x82, 0x3, 0x7a, 0xa0, 0x3, 0x2, 0x1, 0x2, 0x2, 0x10, 0xa, 0x1, 0x41, 0x42, 0x0, 0x0, 0x1, 0x53, 0x85, 0x73, 0x6a, 0xb, 0x85, 0xec, 0xa7, 0x8, 0x30, 0xd, 0x6, 0x9, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0xd, 0x1, 0x1, 0xb, 0x5, 0x0, 0x30, 0x3f, 0x31, 0x24, 0x30, 0x22, 0x6, 0x3, 0x55, 0x4, 0xa, 0x13, 0x1b, 0x44, 0x69, 0x67, 0x69, 0x74, 0x61, 0x6c, 0x20, 0x53, 0x69, 0x67, 0x6e, 0x61, 0x74, 0x75, 0x72, 0x65, 0x20, 0x54, 0x72, 0x75, 0x73, 0x74, 0x20, 0x43, 0x6f, 0x2e, 0x31, 0x17, 0x30, 0x15, 0x6, 0x3, 0x55, 0x4, 0x3, 0x13, 0xe, 0x44, 0x53, 0x54, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41, 0x20, 0x58, 0x33, 0x30, 0x1e, 0x17, 0xd, 0x31, 0x36, 0x30, 0x33, 0x31, 0x37, 0x31, 0x36, 0x34, 0x30, 0x34, 0x36, 0x5a, 0x17, 0xd, 0x32, 0x31, 0x30, 0x33, 0x31, 0x37, 0x31, 0x36, 0x34, 0x30, 0x34, 0x36, 0x5a, 0x30, 0x4a, 0x31, 0xb, 0x30, 0x9, 0x6, 0x3, 0x55, 0x4, 0x6, 0x13, 0x2, 0x55, 0x53, 0x31, 0x16, 0x30, 0x14, 0x6, 0x3, 0x55, 0x4, 0xa, 0x13, 0xd, 0x4c, 0x65, 0x74, 0x27, 0x73, 0x20, 0x45, 0x6e, 0x63, 0x72, 0x79, 0x70, 0x74, 0x31, 0x23, 0x30, 0x21, 0x6, 0x3, 0x55, 0x4, 0x3, 0x13, 0x1a, 0x4c, 0x65, 0x74, 0x27, 0x73, 0x20, 0x45, 0x6e, 0x63, 0x72, 0x79, 0x70, 0x74, 0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 0x74, 0x79, 0x20, 0x58, 0x33, 0x30, 0x82, 0x1, 0x22, 0x30, 0xd, 0x6, 0x9, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0xd, 0x1, 0x1, 0x1, 0x5, 0x0, 0x3, 0x82, 0x1, 0xf, 0x0, 0x30, 0x82, 0x1, 0xa, 0x2, 0x82, 0x1, 0x1, 0x0, 0x9c, 0xd3, 0xc, 0xf0, 0x5a, 0xe5, 0x2e, 0x47, 0xb7, 0x72, 0x5d, 0x37, 0x83, 0xb3, 0x68, 0x63, 0x30, 0xea, 0xd7, 0x35, 0x26, 0x19, 0x25, 0xe1, 0xbd, 0xbe, 0x35, 0xf1, 0x70, 0x92, 0x2f, 0xb7, 0xb8, 0x4b, 0x41, 0x5, 0xab, 0xa9, 0x9e, 0x35, 0x8, 0x58, 0xec, 0xb1, 0x2a, 0xc4, 0x68, 0x87, 0xb, 0xa3, 0xe3, 0x75, 0xe4, 0xe6, 0xf3, 0xa7, 0x62, 0x71, 0xba, 0x79, 0x81, 0x60, 0x1f, 0xd7, 0x91, 0x9a, 0x9f, 0xf3, 0xd0, 0x78, 0x67, 0x71, 0xc8, 0x69, 0xe, 0x95, 0x91, 0xcf, 0xfe, 0xe6, 0x99, 0xe9, 0x60, 0x3c, 0x48, 0xcc, 0x7e, 0xca, 0x4d, 0x77, 0x12, 0x24, 0x9d, 0x47, 0x1b, 0x5a, 0xeb, 0xb9, 0xec, 0x1e, 0x37, 0x0, 0x1c, 0x9c, 0xac, 0x7b, 0xa7, 0x5, 0xea, 0xce, 0x4a, 0xeb, 0xbd, 0x41, 0xe5, 0x36, 0x98, 0xb9, 0xcb, 0xfd, 0x6d, 0x3c, 0x96, 0x68, 0xdf, 0x23, 0x2a, 0x42, 0x90, 0xc, 0x86, 0x74, 0x67, 0xc8, 0x7f, 0xa5, 0x9a, 0xb8, 0x52, 0x61, 0x14, 0x13, 0x3f, 0x65, 0xe9, 0x82, 0x87, 0xcb, 0xdb, 0xfa, 0xe, 0x56, 0xf6, 0x86, 0x89, 0xf3, 0x85, 0x3f, 0x97, 0x86, 0xaf, 0xb0, 0xdc, 0x1a, 0xef, 0x6b, 0xd, 0x95, 0x16, 0x7d, 0xc4, 0x2b, 0xa0, 0x65, 0xb2, 0x99, 0x4, 0x36, 0x75, 0x80, 0x6b, 0xac, 0x4a, 0xf3, 0x1b, 0x90, 0x49, 0x78, 0x2f, 0xa2, 0x96, 0x4f, 0x2a, 0x20, 0x25, 0x29, 0x4, 0xc6, 0x74, 0xc0, 0xd0, 0x31, 0xcd, 0x8f, 0x31, 0x38, 0x95, 0x16, 0xba, 0xa8, 0x33, 0xb8, 0x43, 0xf1, 0xb1, 0x1f, 0xc3, 0x30, 0x7f, 0xa2, 0x79, 0x31, 0x13, 0x3d, 0x2d, 0x36, 0xf8, 0xe3, 0xfc, 0xf2, 0x33, 0x6a, 0xb9, 0x39, 0x31, 0xc5, 0xaf, 0xc4, 0x8d, 0xd, 0x1d, 0x64, 0x16, 0x33, 0xaa, 0xfa, 0x84, 0x29, 0xb6, 0xd4, 0xb, 0xc0, 0xd8, 0x7d, 0xc3, 0x93, 0x2, 0x3, 0x1, 0x0, 0x1, 0xa3, 0x82, 0x1, 0x7d, 0x30, 0x82, 0x1, 0x79, 0x30, 0x12, 0x6, 0x3, 0x55, 0x1d, 0x13, 0x1, 0x1, 0xff, 0x4, 0x8, 0x30, 0x6, 0x1, 0x1, 0xff, 0x2, 0x1, 0x0, 0x30, 0xe, 0x6, 0x3, 0x55, 0x1d, 0xf, 0x1, 0x1, 0xff, 0x4, 0x4, 0x3, 0x2, 0x1, 0x86, 0x30, 0x7f, 0x6, 0x8, 0x2b, 0x6, 0x1, 0x5, 0x5, 0x7, 0x1, 0x1, 0x4, 0x73, 0x30, 0x71, 0x30, 0x32, 0x6, 0x8, 0x2b, 0x6, 0x1, 0x5, 0x5, 0x7, 0x30, 0x1, 0x86, 0x26, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x69, 0x73, 0x72, 0x67, 0x2e, 0x74, 0x72, 0x75, 0x73, 0x74, 0x69, 0x64, 0x2e, 0x6f, 0x63, 0x73, 0x70, 0x2e, 0x69, 0x64, 0x65, 0x6e, 0x74, 0x72, 0x75, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x30, 0x3b, 0x6, 0x8, 0x2b, 0x6, 0x1, 0x5, 0x5, 0x7, 0x30, 0x2, 0x86, 0x2f, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x61, 0x70, 0x70, 0x73, 0x2e, 0x69, 0x64, 0x65, 0x6e, 0x74, 0x72, 0x75, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x2f, 0x72, 0x6f, 0x6f, 0x74, 0x73, 0x2f, 0x64, 0x73, 0x74, 0x72, 0x6f, 0x6f, 0x74, 0x63, 0x61, 0x78, 0x33, 0x2e, 0x70, 0x37, 0x63, 0x30, 0x1f, 0x6, 0x3, 0x55, 0x1d, 0x23, 0x4, 0x18, 0x30, 0x16, 0x80, 0x14, 0xc4, 0xa7, 0xb1, 0xa4, 0x7b, 0x2c, 0x71, 0xfa, 0xdb, 0xe1, 0x4b, 0x90, 0x75, 0xff, 0xc4, 0x15, 0x60, 0x85, 0x89, 0x10, 0x30, 0x54, 0x6, 0x3, 0x55, 0x1d, 0x20, 0x4, 0x4d, 0x30, 0x4b, 0x30, 0x8, 0x6, 0x6, 0x67, 0x81, 0xc, 0x1, 0x2, 0x1, 0x30, 0x3f, 0x6, 0xb, 0x2b, 0x6, 0x1, 0x4, 0x1, 0x82, 0xdf, 0x13, 0x1, 0x1, 0x1, 0x30, 0x30, 0x30, 0x2e, 0x6, 0x8, 0x2b, 0x6, 0x1, 0x5, 0x5, 0x7, 0x2, 0x1, 0x16, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x63, 0x70, 0x73, 0x2e, 0x72, 0x6f, 0x6f, 0x74, 0x2d, 0x78, 0x31, 0x2e, 0x6c, 0x65, 0x74, 0x73, 0x65, 0x6e, 0x63, 0x72, 0x79, 0x70, 0x74, 0x2e, 0x6f, 0x72, 0x67, 0x30, 0x3c, 0x6, 0x3, 0x55, 0x1d, 0x1f, 0x4, 0x35, 0x30, 0x33, 0x30, 0x31, 0xa0, 0x2f, 0xa0, 0x2d, 0x86, 0x2b, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x63, 0x72, 0x6c, 0x2e, 0x69, 0x64, 0x65, 0x6e, 0x74, 0x72, 0x75, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x2f, 0x44, 0x53, 0x54, 0x52, 0x4f, 0x4f, 0x54, 0x43, 0x41, 0x58, 0x33, 0x43, 0x52, 0x4c, 0x2e, 0x63, 0x72, 0x6c, 0x30, 0x1d, 0x6, 0x3, 0x55, 0x1d, 0xe, 0x4, 0x16, 0x4, 0x14, 0xa8, 0x4a, 0x6a, 0x63, 0x4, 0x7d, 0xdd, 0xba, 0xe6, 0xd1, 0x39, 0xb7, 0xa6, 0x45, 0x65, 0xef, 0xf3, 0xa8, 0xec, 0xa1, 0x30, 0xd, 0x6, 0x9, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0xd, 0x1, 0x1, 0xb, 0x5, 0x0, 0x3, 0x82, 0x1, 0x1, 0x0, 0xdd, 0x33, 0xd7, 0x11, 0xf3, 0x63, 0x58, 0x38, 0xdd, 0x18, 0x15, 0xfb, 0x9, 0x55, 0xbe, 0x76, 0x56, 0xb9, 0x70, 0x48, 0xa5, 0x69, 0x47, 0x27, 0x7b, 0xc2, 0x24, 0x8, 0x92, 0xf1, 0x5a, 0x1f, 0x4a, 0x12, 0x29, 0x37, 0x24, 0x74, 0x51, 0x1c, 0x62, 0x68, 0xb8, 0xcd, 0x95, 0x70, 0x67, 0xe5, 0xf7, 0xa4, 0xbc, 0x4e, 0x28, 0x51, 0xcd, 0x9b, 0xe8, 0xae, 0x87, 0x9d, 0xea, 0xd8, 0xba, 0x5a, 0xa1, 0x1, 0x9a, 0xdc, 0xf0, 0xdd, 0x6a, 0x1d, 0x6a, 0xd8, 0x3e, 0x57, 0x23, 0x9e, 0xa6, 0x1e, 0x4, 0x62, 0x9a, 0xff, 0xd7, 0x5, 0xca, 0xb7, 0x1f, 0x3f, 0xc0, 0xa, 0x48, 0xbc, 0x94, 0xb0, 0xb6, 0x65, 0x62, 0xe0, 0xc1, 0x54, 0xe5, 0xa3, 0x2a, 0xad, 0x20, 0xc4, 0xe9, 0xe6, 0xbb, 0xdc, 0xc8, 0xf6, 0xb5, 0xc3, 0x32, 0xa3, 0x98, 0xcc, 0x77, 0xa8, 0xe6, 0x79, 0x65, 0x7, 0x2b, 0xcb, 0x28, 0xfe, 0x3a, 0x16, 0x52, 0x81, 0xce, 0x52, 0xc, 0x2e, 0x5f, 0x83, 0xe8, 0xd5, 0x6, 0x33, 0xfb, 0x77, 0x6c, 0xce, 0x40, 0xea, 0x32, 0x9e, 0x1f, 0x92, 0x5c, 0x41, 0xc1, 0x74, 0x6c, 0x5b, 0x5d, 0xa, 0x5f, 0x33, 0xcc, 0x4d, 0x9f, 0xac, 0x38, 0xf0, 0x2f, 0x7b, 0x2c, 0x62, 0x9d, 0xd9, 0xa3, 0x91, 0x6f, 0x25, 0x1b, 0x2f, 0x90, 0xb1, 0x19, 0x46, 0x3d, 0xf6, 0x7e, 0x1b, 0xa6, 0x7a, 0x87, 0xb9, 0xa3, 0x7a, 0x6d, 0x18, 0xfa, 0x25, 0xa5, 0x91, 0x87, 0x15, 0xe0, 0xf2, 0x16, 0x2f, 0x58, 0xb0, 0x6, 0x2f, 0x2c, 0x68, 0x26, 0xc6, 0x4b, 0x98, 0xcd, 0xda, 0x9f, 0xc, 0xf9, 0x7f, 0x90, 0xed, 0x43, 0x4a, 0x12, 0x44, 0x4e, 0x6f, 0x73, 0x7a, 0x28, 0xea, 0xa4, 0xaa, 0x6e, 0x7b, 0x4c, 0x7d, 0x87, 0xdd, 0xe0, 0xc9, 0x2, 0x44, 0xa7, 0x87, 0xaf, 0xc3, 0x34, 0x5b, 0xb4, 0x42};
#define OTA_CRYPTO_MEM_SIZE (9 * 1024)
char * ota_crypto_mem[OTA_CRYPTO_MEM_SIZE];
#define OTA_MAX_CERTS 2
#endif


// TODO: use message framework to update instead of volatile?
volatile ULONG sample_period = 15 * 60 * 100;

#define SENSORS 13
static volatile agg_t * const sensors[SENSORS] = {&g_x,
                                                  &g_y,
                                                  &g_z,
                                                  &g_x_mag,
                                                  &g_y_mag,
                                                  &g_z_mag,
                                                  &g_temp1,
                                                  &g_temp2,
                                                  &g_temp3,
                                                  &g_humidity,
                                                  &g_pressure,
                                                  &g_mic,
                                                  &g_voice_fft
};

// holds the bitmask of leds that should be blinked in the RTC callback
static volatile uint8_t leds_to_toggle = 0;
// holds the number of blinks that should currently be performed in the RTC callback
static volatile int n_blinks = 0;

extern char g_link_code[7];

extern TX_THREAD net_thread;
extern TX_THREAD adc_thread;
extern TX_THREAD bmc_thread;
extern TX_THREAD ens_thread;
extern TX_THREAD ms_thread;

// connection details for diagnostics agent
static const char * rna_broker_url = "mqtt-renesas-na-sandbox.mediumone.com";
static const uint16_t rna_port = 61619;


// extern GPIO pin setting function
void set_pin(ioport_port_pin_t pin, ioport_level_t level);

// prototype for m1_callback below
void m1_callback(int type, char * topic, char * msg, int length);

// OTA parameters
static char g_update_url[640];
static uint32_t g_update_hash[8];
static volatile uint8_t g_update_firmware = 0;
extern FX_MEDIA g_qspi_media;


/*
 * handles messages published to one of the topics subscribed to by the m1 agent.
 * we use the convention of the first character of the msg indicating a command.
 * multiple commands can be sent in one message, separated by ';'. ';' must not
 * be used within a command. Supported commands are:
 *      - S: set new sampling period
 *      - T: update sensor threshold
 *      - L: set, clear, or toggle LED(s)
 *      - B: blink LED(s)
 *      - G: set GPIO pin output level
 */
void m1_callback(int type, char * topic, char * msg, int length) {
    SSP_PARAMETER_NOT_USED(type);
    SSP_PARAMETER_NOT_USED(topic);
    SSP_PARAMETER_NOT_USED(length);
    char * p = msg, * temp_str;
    int temp;
    int msglen;

    // TODO: hack...
    msg[length] = '\0';
    while (p != NULL) {
        temp_str = strchr(p, ';');
        if (temp_str != NULL)
            msglen = temp_str - p;
        else
            msglen = length - (p - msg);
        if (type == 1) {
            switch (p[0]) {
                case 'S': {
                    ULONG new_sample_period;
                    sscanf(&p[1], "%lu", &new_sample_period);
                    sample_period = new_sample_period;
                    tx_thread_wait_abort(&net_thread);
                    break;
                }
                case 'T': {
                    char name[30];
                    float threshold;
                    int threshold_type;
                    if (sscanf(&p[1], "%29[^:]:%d:%f", name, &threshold_type, &threshold) == 3) {
                        for (int i = 0; i < SENSORS; i++) {
                            if (!strcmp(sensors[i]->name, name)) {
                                sensors[i]->absolute_threshold = threshold_type ? 1 : 0;
                                sensors[i]->threshold = threshold;
                                break;
                            }
                        }
                        tx_event_flags_set(&g_sensor_event_flags, BMC_THRESHOLD_UPDATE | ADC_THRESHOLD_UPDATE | ENS_THRESHOLD_UPDATE | MS_THRESHOLD_UPDATE, TX_OR);
                    }
                    break;
                }
                case 'L': {
                    int val;
                    sscanf(&p[1], "%d", &temp);
                    temp_str = strchr(&p[1], ':');
                    // no strnchr...
                    if ((temp_str != NULL) && ((temp_str - p) < msglen)) {
                        if (sscanf(temp_str + 1, "%d", &val) == 1)
                            set_led((uint8_t)temp, (uint8_t)val);
                    } else
                        toggle_leds((temp >> 2) & 0x01, (temp >> 1) & 0x01, temp & 0x01);
                    break;
                }
                case 'B':
                    n_blinks = 0;
                    if (sscanf(&p[1], "%d:%d", &temp, &n_blinks) < 1)
                        break;
                    if (!temp)
                        break;
                    if (g_rtc0.p_api->open(g_rtc0.p_ctrl, g_rtc0.p_cfg) != SSP_SUCCESS)
                        break;
                    leds_to_toggle = (uint8_t)temp;
                    g_rtc0.p_api->periodicIrqRateSet(g_rtc0.p_ctrl, RTC_PERIODIC_IRQ_SELECT_1_SECOND);
                    g_rtc0.p_api->calendarCounterStart(g_rtc0.p_ctrl);
                    g_rtc0.p_api->irqEnable(g_rtc0.p_ctrl, RTC_EVENT_PERIODIC_IRQ);
                    break;
                case 'G': {
                    int pin;
                    int level;
                    if (sscanf(&p[1], "%d:%d:%d", &temp, &pin, &level) == 3)
                        set_pin(pin + (temp << 8), level == 0 ? 0 : 1);
                    break;
                }
                case 'U': {
                    int i = 0;
                    int minor;
                    int ret = sscanf(&p[1], "%d:%d:%*[^:]:%n",
                                     &temp,
                                     &minor,
                                     &i);
                    if ((ret == 2) && (i < msglen)) {
                        strncpy(g_update_url, &p[i + 1], (size_t)(msglen - i));
                        for (int j = 0; j < 8; j++) {
                            p[i - (j * 8)] = '\0';
                            g_update_hash[7 - j] = (uint32_t)strtoull(&p[i - ((j + 1) * 8)], NULL, 16);
                        }
                        if ((temp != VERSION_MAJOR) || (minor != VERSION_MINOR)) {
                            g_update_firmware = 1;
                            tx_thread_wait_abort(&net_thread);
                        }
                    }
                    break;
		}
                default:
                    break;
            }
        } else if (type == 2) {
            switch (p[0]) {
                case 'U': {
                    int i = 0;
                    int minor;
                    int ret = sscanf(&p[1], "%d:%d:%*[^:]:%n",
                                     &temp,
                                     &minor,
                                     &i);
                    if ((ret == 2) && (i < msglen)) {
                        strncpy(g_update_url, &p[i + 1], (size_t)(msglen - i));
                        for (int j = 0; j < 8; j++) {
                            p[i - (j * 8)] = '\0';
                            g_update_hash[7 - j] = (uint32_t)strtoull(&p[i - ((j + 1) * 8)], NULL, 16);
                        }
                        if ((temp != VERSION_MAJOR) || (minor != VERSION_MINOR))
                            g_update_firmware = 1;
                    }
                    break;
                }
            }
        }
        p = strchr(p, ';');
        if (p != NULL)
            p++;
    }
}

/*
 * rtc callback. blinks LEDs if required
 */
void rtc_callback(rtc_callback_args_t * p_args) {
    static int count = 0;
    if (p_args->event == RTC_EVENT_PERIODIC_IRQ) {
        if (count++ < (n_blinks * 2)) {
            toggle_leds((leds_to_toggle >> 2) & 0x01, (leds_to_toggle >> 1) & 0x01, leds_to_toggle & 0x01);
        } else {
            count = 0;
            g_rtc0.p_api->close(g_rtc0.p_ctrl);
        }
    }
}


/*
 * helper function to determine how many dataflash blocks are spanned
 */
static uint32_t bytes_to_df_blocks(uint32_t bytes) {
    if (!bytes)
        return 0;
    return bytes / DATA_FLASH_BLOCK_SIZE + 1;
}

static int write_struct_to_flash(void * obj, size_t size) {
    uint32_t blocks = bytes_to_df_blocks(size);
    ssp_err_t status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    if (status)
        return -1;
    status = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, blocks);
    if (status)
        return -2;
    status = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)obj, 0x40100000, blocks * DATA_FLASH_BLOCK_SIZE);
    if (status)
        return -3;
    status = g_flash0.p_api->close(g_flash0.p_ctrl);
    if (status)
        return -4;
    return 0;
}

static int write_app_config_to_flash(app_config_t * p_app_config) {
    char buf[sizeof(app_config_t) + 1];
    buf[0] = 'P';
    memcpy(&buf[1], p_app_config, sizeof(app_config_t));
    return write_struct_to_flash(buf, sizeof(buf));
}

static void toggle_green_led(int disk, int network) {
    if (disk)
        set_led(2, 1);
    else if (network)
        set_led(1, 1);
    else
        set_led(3, 0);
}

static int read_certs_and_connect(const char * p_mqtt_broker_host,
                                  int mqtt_broker_port,
                                  project_credentials_t * project,
                                  user_credentials_t * registration,
                                  user_credentials_t * device) {
#ifdef CERT_BASED_AUTH
    char cert_buf[MAX_CERT_LENGTH];
    char key_buf[MAX_KEY_LENGTH];
#endif

    m1_connect_params params = {
                                .mqtt_url = p_mqtt_broker_host,
                                .mqtt_port = mqtt_broker_port,
                                .project = project,
                                .registration = registration,
                                .device = device,
                                .device_id = "s5d9",
                                .retry_delay = 5,
                                .retry_limit = 5,
                                .mqtt_heart_beat = 600,
                                .stack_mem = mqtt_stack,
                                .stack_mem_size = MQTT_STACK_SIZE,
                                .mqtt_msg_mem = mqtt_msg_mem,
                                .mqtt_msg_mem_size = MQTT_MSG_MEM_SIZE,
#ifdef USE_TLS
                                .tls_enabled = 1,
                                .ssl_mem = ssl_mem,
                                .ssl_mem_size = SSL_MEM_SIZE,
                                .crypto_mem = crypto_mem,
                                .crypto_mem_size = CRYPTO_MEM_SIZE,
                                .cert_mem = cert_mem,
                                .cert_mem_size = CERT_MEM_SIZE,
#else
                                .tls_enabled = 0,
#endif
                                .p_ppool = &g_http_packet_pool,
                                .p_ip = &g_http_ip,
                                .p_dns = &g_dns_client
    };
#ifdef CERT_BASED_AUTH
#if 0
    FILE * device_cert = fopen("device.crt", "r");
    FILE * device_key = fopen("device.key", "r");
    if (device_cert && device_key) {
        memset(cert_buf, 0, sizeof(cert_buf));
        memset(key_buf, 0, sizeof(key_buf));
        fread(cert_buf, MAX_CERT_LENGTH, 1, device_cert);
        fread(key_buf, MAX_KEY_LENGTH, 1, device_key);
        fclose(device_cert);
        fclose(device_key);
        params.device_cert = cert_buf;
        params.device_key = key_buf;
    }
#endif
#endif

    return m1_auto_enroll_connect_2(&params);
}

static int device_compare(user_credentials_t * config1, user_credentials_t * config2) {
    if (strncmp(config1->password, config2->password, sizeof(config1->password)))
        return -1;
    if (strncmp(config1->user_id, config2->user_id, sizeof(config1->user_id)))
        return -1;
    return 0;
}

static int project_compare(project_credentials_t * config1, project_credentials_t * config2) {
    if (strncmp(config1->apikey, config2->apikey, sizeof(config1->apikey)))
        return -1;
    if (strncmp(config1->proj_id, config2->proj_id, sizeof(config1->proj_id)))
        return -1;
    return 0;
}

static int config_compare(app_config_t * config1, app_config_t * config2) {
    if (device_compare(&config1->device, &config2->device))
        return -1;
    if (device_compare(&config1->diagnostics, &config2->diagnostics))
        return -2;
    if (device_compare(&config1->registration, &config2->registration))
        return -3;
    if (project_compare(&config1->project, &config2->project))
        return -4;
    return 0;
}

/*
 * extracts credentials from m1config.txt, and optionally m1user.txt and
 * m1broker.txt.
 *
 * if m1 diagnostics agent is enabled, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * if m1user.txt provided user credentials, attempts connection
 * with those credentials. if connection fails, checks dataflash for
 * auto-enrolled credentials. if credentials are present in dataflash, attempts
 * connection with those credentials. if connection fails, attempts connection
 * with registration user credentials. if registration user credentials are not
 * available, board goes into error state. if registration user connection
 * succeeds, attempts auto-enrollment. if auto-enrollment is successful, stores
 * new user credentials in dataflash and attempts connection. if auto-enrollment
 * fails, board goes into error state.
 *
 * after successful connections, sends "on-connect" event and resumes sensor
 * threads. if diagnostics agent is enabled, configures sensor peripherals.
 * finally, enters infinite loop requesting sensor aggregates every sample_period.
 * if sensor aggregates are not received within 1 second of request, we assume
 * a sensor thread is dead-locked and reset the board. after receiving sensor
 * aggregates, publishes events for all sensor aggregates to the cloud.
 */
void net_thread_entry(void)
{
    ssp_err_t status = SSP_SUCCESS;
    int ret = 0;
    char buf[400];
    fmi_product_info_t * effmi;
    app_config_t provisioned_config;
    user_credentials_t original_device;
    original_device.user_id[0] = '\0';
    original_device.password[0] = '\0';
    memset(&provisioned_config, 0, sizeof(provisioned_config));
#ifdef USE_M1DIAG
    const project_credentials_t diag_project = {
#if 1
        .proj_id="kxwuKiOTycI",
        .apikey="2TXT7PJ4BGC4UA24XANNORBQGU2DIYTDG42DOMTDGQZDAMBQ"
#else
        .proj_id="Ohafs8Q31jU",
        .apikey="VRWM4JXNCKZEHIDP4T7TQRZQGU4TSOJQGI4WMZBQGQ3DAMBQ"
#endif
    };
    user_credentials_t original_diag_device;
    original_diag_device.user_id[0] = '\0';
    original_diag_device.password[0] = '\0';
    app_config.diagnostics.user_id[0] = '\0';
    app_config.diagnostics.password[0] = '\0';
    user_credentials_t diag_registration = {
#if 0
        .user_id="f_acx06uqDA",
#else
        .user_id="1tyKNEkeZpQ",
        .password="Medium-1!"
#endif
    };
#endif
    const char * p_mqtt_broker_host = rna_broker_url;
    uint16_t mqtt_broker_port = rna_port;
    ULONG start, actual_flags;
    char event[1536];


    g_fmi.p_api->productInfoGet(&effmi);

    status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    if (status)
        goto err;
    status = g_flash0.p_api->read(g_flash0.p_ctrl, (UCHAR *)buf, 0x40100000, sizeof(buf) - 1);
    if (status)
        goto err;
    status = g_flash0.p_api->close(g_flash0.p_ctrl);
    if (status)
        goto err;
    buf[sizeof(buf) - 1] = '\0';
    if (buf[0] == 'P') {
        // board may have been auto-provisioned; try provisioned credentials
        app_config_t * cfg_from_flash = (app_config_t *)&buf[1];
        memcpy(&provisioned_config, cfg_from_flash, sizeof(provisioned_config));
        if (strlen(app_config.project.apikey))
            memcpy(&cfg_from_flash->project, &app_config.project, sizeof(cfg_from_flash->project));
        if (strlen(app_config.device.password))
            memcpy(&cfg_from_flash->device, &app_config.device, sizeof(cfg_from_flash->device));
        if (strlen(app_config.registration.password))
            memcpy(&cfg_from_flash->registration, &app_config.registration, sizeof(cfg_from_flash->registration));
        memcpy(&app_config, cfg_from_flash, sizeof(app_config));
#ifdef USE_M1DIAG
        memcpy(&original_diag_device, &app_config.diagnostics, sizeof(original_diag_device));
#endif
        memcpy(&original_device, &app_config.device, sizeof(original_device));
    }

    set_leds(0, 1, 1);

    int idlen = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        idlen += sprintf(&event[idlen], "%02X", effmi->unique_id[k]);

#if 0  // enable, with UART on SCI driver instance, for MQTT debug logging
    logger_init((void *) &g_uart0, 0);
#endif

#ifdef USE_M1DIAG
    m1_diag_initdata_t initdata;
    strcpy(initdata.email_address, "your@email.add");
    int sn_len = 0;
    for (unsigned int k = 0; k < sizeof(effmi->unique_id); k++)
        sn_len += sprintf(&initdata.serial_number[sn_len], "%02X", effmi->unique_id[k]);
    strcpy(initdata.model_number, "s5d9-revB");
    snprintf(initdata.fw_version, sizeof(initdata.fw_version), "%c%c%c%c%c%c%c%c-%d.%d",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR);
#if 0
    ret = m1_diag_start("mqtt.mediumone.com",
                            61618,
#else
    ret = m1_diag_start("ladder.mediumone.com",
                            61613,
#endif
                            &diag_project,
                            &diag_registration,
                            &app_config.diagnostics,
                            event,
                            5,
                            5,
                            1000,
#ifdef USE_TLS
                            1,
                            diag_ssl_mem,
                            DIAG_SSL_MEM_SIZE,
#else
                            0,
                            NULL,
                            0,
#endif
                            &g_http_packet_pool,
                            &g_http_ip,
                            &g_dns_client,
                            900,
                            &initdata,
                            debug,
                            NULL);
    if ((ret == M1_SUCCESS) && strncmp(original_diag_device.password, app_config.diagnostics.password, sizeof(original_diag_device.password))) {
        ret = write_app_config_to_flash(&app_config);
        if (ret)
            goto err;
    }
#endif

    ret = m1_register_subscription_callback(m1_callback);
    ret = read_certs_and_connect(p_mqtt_broker_host, mqtt_broker_port, &app_config.project, &app_config.registration, &app_config.device);
    if (ret != M1_SUCCESS)
        goto err;

    set_leds(0, 0, 1);

    if (config_compare(&provisioned_config, &app_config)) {
        ret = write_app_config_to_flash(&app_config);
        if (ret)
            goto err;
    }


    unsigned long mac_address[2];
    nx_ip_interface_info_get(&g_http_ip, 0, NULL, NULL, NULL, NULL, &mac_address[0], &mac_address[1]);
    sprintf(event,
            "{\"firmware_version\":\"%c%c%c%c%c%c%c%c-%d.%d\",\"mac_address\":\"%02lX:%02lX:%02lX:%02lX:%02lX:%02lX\"}",
            BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
            BUILD_MONTH_CH0, BUILD_MONTH_CH1,
            BUILD_DAY_CH0, BUILD_DAY_CH1,
            VERSION_MAJOR, VERSION_MINOR,
            (mac_address[0] >> 8) & 0xff,
            mac_address[0] & 0xff,
            (mac_address[1] >> 24) & 0xff,
            (mac_address[1] >> 16) & 0xff,
            (mac_address[1] >> 8) & 0xff,
            mac_address[1] & 0xff);
    m1_publish_event(event, NULL);
    sprintf(event, "{\"link_code\":\"%s\"}", g_link_code);
    m1_publish_event(event, NULL);
#ifdef USE_M1DIAG
    M1_LOG(info, "M1 VSA connected", 0);
    sprintf((char *)buf, "{\"mqtt_user_id\":\"%s\",\"mqtt_project_id\":\"%s\",\"m1_api_key\":\"%s\"}",
            app_config.device.user_id,
            app_config.project.proj_id,
            app_config.project.apikey);
    m1_diag_update_configuration((char *)buf);
#endif

    status = g_i2c0.p_api->open(g_i2c0.p_ctrl, g_i2c0.p_cfg);
    if (status)
        goto err;

    tx_thread_resume(&adc_thread);
    tx_thread_resume(&bmc_thread);
    tx_thread_resume(&ens_thread);
    tx_thread_resume(&ms_thread);
    g_timer1.p_api->open(g_timer1.p_ctrl, g_timer1.p_cfg);
    g_timer1.p_api->start(g_timer1.p_ctrl);

#ifdef USE_M1DIAG
    m1_diag_register_acceleration(&x_accel_access, &y_accel_access, &z_accel_access);
    m1_diag_register_humidity(&humidity_access);
    m1_diag_register_periphery("temp_ambient_c_1", M1_DIAG_FLOAT, &temp_access);
    m1_diag_register_periphery("x_magnetic_t", M1_DIAG_FLOAT, &xmag_access);
    m1_diag_register_periphery("y_magnetic_t", M1_DIAG_FLOAT, &ymag_access);
    m1_diag_register_periphery("z_magnetic_t", M1_DIAG_FLOAT, &zmag_access);
    m1_diag_register_periphery("temp_ambient_c_2", M1_DIAG_FLOAT, &temp2_access);
    m1_diag_register_periphery("temp_ambient_c_3", M1_DIAG_FLOAT, &temp3_access);
    m1_diag_register_periphery("pressure_mbar", M1_DIAG_FLOAT, &pressure_access);
    m1_diag_register_periphery("mic", M1_DIAG_FLOAT, &mic_access);
    m1_diag_register_periphery("voice_fft", M1_DIAG_FLOAT, &voice_fft_access);
#endif

    start = tx_time_get();

    while (1) {
        if (g_update_firmware) {
            char filename[40];
            g_ioport.p_api->pinCfg(IOPORT_PORT_04_PIN_07, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_NMOS_ENABLE);
            // suspend system thread to prevent FX media concurrent interaction
            tx_thread_suspend(&system_thread);
            NX_SECURE_TLS_SESSION tls_session;
            NX_SECURE_X509_CERT ota_root_cert;
            memset(&tls_session, 0, sizeof(tls_session));
            nx_secure_x509_certificate_initialize(&ota_root_cert, (uint8_t *)uguu_root, sizeof(uguu_root), NULL, 0, NULL, 0, NX_SECURE_X509_KEY_TYPE_NONE);
            nx_secure_tls_session_create(&tls_session, &nx_crypto_tls_ciphers_synergys7, ota_crypto_mem, sizeof(ota_crypto_mem));
            ret = update_firmware(g_update_url, g_update_hash, &g_qspi_media, &g_http_ip, &g_http_packet_pool, &g_dns_client, &g_sce_hash_0, &g_sce_0, filename, NULL, toggle_green_led, &tls_session, &ota_root_cert);
            if (!ret) {
                remove("loaded.txt");
                remove("update.txt");
                FILE * config_file = fopen("update.txt", "w");
                if (config_file && (fwrite(filename, strlen(filename) + 1, 1, config_file) == 1)) {
                    fclose(config_file);
#ifdef USE_M1DIAG
                    M1_LOG(info, "OTA update downloaded", 0);
#endif
                    // sleep to let file system updates flush
                    tx_thread_sleep(500);
                    // reset
                    SCB->AIRCR = 0x05FA0004;
                }
            }
            // if we get here there was an error
            g_update_firmware = 0;
            tx_thread_resume(&system_thread);
            // sleep to allow FX media flush
            tx_thread_sleep(200);
            g_ioport.p_api->pinCfg(IOPORT_PORT_04_PIN_07, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS);
        }
        actual_flags = tx_time_get() - start;
        tx_thread_sleep(actual_flags <= sample_period  ? sample_period - actual_flags : 0);
        start = tx_time_get();
        tx_event_flags_set(&g_sensor_event_flags, 0x0000000F, TX_OR);
        status = tx_event_flags_get(&g_sensor_event_flags, 0x000000F0, TX_AND_CLEAR, &actual_flags, 100);
        if (status == TX_NO_EVENTS) {
#ifdef USE_M1DIAG
            M1_LOG(error, "Sensor threads did not respond in time", status);
            tx_thread_sleep(500);
#endif
            // most likely means deadlock (i2c callback not called)
            SCB->AIRCR = 0x05FA0004;
        }
        int index = 1;
        event[0] = '{';
        event[1] = '\0';
        int i;
        for (i = 0; i < SENSORS; i++) {
            if (sensors[i]->count) {
                index += sprintf(&event[index],
                                "\"%s\":{\"value\":%f,\"avg\":%f,\"min\":%f,\"max\":%f,\"samples\":%d},",
                                sensors[i]->name, sensors[i]->value, sensors[i]->total / (float)sensors[i]->count,
                                sensors[i]->min, sensors[i]->max, sensors[i]->count);
            }
        }
        sprintf(&event[index],
                "\"x_zero_crossings\":%lu,\"y_zero_crossings\":%lu,\"z_zero_crossings\":%lu}",
                g_x_zero_crossings, g_y_zero_crossings, g_z_zero_crossings);
        m1_publish_event(event, NULL);
    }

err:
#ifdef USE_M1DIAG
    M1_LOG(error, "Critical error (flash, i2c, or initial connection)", ret + (int)status);
#endif
    while (1) {
        set_leds(1, 1, 1);
        tx_thread_sleep(100);
        set_leds(0, 0, 0);
        tx_thread_sleep(100);
    }
}

void sensor_scheduler(timer_callback_args_t * p_args) {
    static unsigned int ticks = 0;
    ULONG flags_to_set = 0;

    if (p_args->event == TIMER_EVENT_EXPIRED) {
        ticks++;
        if (!(ticks % 3))  // 2 kHz (accelerometer) provided by sensor, but we only need 333.33 Hz
            flags_to_set |= BMC_SAMPLE_REQUEST;
        if (!(ticks % 131))  // 130 ms conversion time for relative humidity & temp
            flags_to_set |= ENS_SAMPLE_REQUEST;
        if (!(ticks % 35))  // 17 ms conversion-time at OSR = 8192, once for pressure and once for temperature
            flags_to_set |= MS_SAMPLE_REQUEST;
        if (flags_to_set)
            tx_event_flags_set(&g_sensor_event_flags, flags_to_set, TX_OR);
    }
}
