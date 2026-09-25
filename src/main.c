#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define I2C_SDA_GPIO GPIO_NUM_0
#define I2C_SCL_GPIO GPIO_NUM_1
#define LIS2DU12_INT1_GPIO GPIO_NUM_15
#define LIS2DU12_INT2_GPIO GPIO_NUM_14
#define STATUS_LED_GPIO GPIO_NUM_8
#define LIS2DU12_WHO_AM_I_VALUE 0x45
#define LIS2DU12_I2C_TIMEOUT_MS 100

#define LIS2DU12_REG_IF_CTRL 0x0E
#define LIS2DU12_REG_CTRL1 0x10
#define LIS2DU12_REG_CTRL2 0x11
#define LIS2DU12_REG_CTRL3 0x12
#define LIS2DU12_REG_CTRL4 0x13
#define LIS2DU12_REG_CTRL5 0x14
#define LIS2DU12_REG_FIFO_CTRL 0x15
#define LIS2DU12_REG_FIFO_WTM 0x16
#define LIS2DU12_REG_INTERRUPT_CFG 0x17
#define LIS2DU12_REG_TAP_THS_X 0x18
#define LIS2DU12_REG_TAP_THS_Y 0x19
#define LIS2DU12_REG_TAP_THS_Z 0x1A
#define LIS2DU12_REG_INT_DUR 0x1B
#define LIS2DU12_REG_WAKE_UP_THS 0x1C
#define LIS2DU12_REG_WAKE_UP_DUR 0x1D
#define LIS2DU12_REG_FREE_FALL 0x1E
#define LIS2DU12_REG_MD1_CFG 0x1F
#define LIS2DU12_REG_MD2_CFG 0x20
#define LIS2DU12_REG_WAKE_UP_SRC 0x21
#define LIS2DU12_REG_TAP_SRC 0x22
#define LIS2DU12_REG_ALL_INT_SRC 0x24
#define LIS2DU12_REG_FIFO_STATUS2 0x27
#define LIS2DU12_REG_OUT_X_L 0x28
#define LIS2DU12_REG_WHO_AM_I 0x43

#define LIS2DU12_TAP_SRC_SINGLE_TAP (1U << 5)
#define LIS2DU12_TAP_SRC_DOUBLE_TAP (1U << 4)
#define LIS2DU12_WAKE_UP_SRC_FF_IA (1U << 5)

#define WIFI_AP_SSID "LIS2DU12-Test"
#define WIFI_AP_PASSWORD "lis2du12test"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONNECTIONS 4

static const char *TAG = "lis2du12_test";

typedef struct {
	bool sensor_found;
	bool i2c_connected;
	bool sda_level;
	bool scl_level;
	uint8_t who_am_i;
	uint8_t i2c_address;
	esp_err_t i2c_error;
	float x_g;
	float y_g;
	float z_g;
	uint8_t fifo_samples;
	uint8_t tap_source;
	uint8_t wake_up_source;
	uint8_t all_int_source;
	bool int2_level;
	uint8_t ctrl1_reg;
	uint32_t single_taps;
	uint32_t double_taps;
	uint32_t freefalls;
	uint32_t interrupts;
} sensor_state_t;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t lis2du12;
static uint8_t lis2du12_address;
static rmt_channel_handle_t led_channel;
static rmt_encoder_handle_t led_encoder;
static TaskHandle_t sensor_task_handle;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static sensor_state_t sensor_state;
static volatile uint32_t isr_edge_count = 0;
static volatile uint32_t int2_edge_count = 0;
static bool tap_event_active = false;
static bool freefall_event_active = false;
static uint32_t cpu_freq_mhz = 0;
static wifi_ps_type_t wifi_ps_mode = WIFI_PS_NONE;

static const char index_html[] =
	"<!doctype html><html lang=\"de\"><head><meta charset=\"utf-8\">"
	"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>LIS2DU12 Test</title><style>"
	"body{margin:0;background:#edf2f3;color:#17232b;font:16px Georgia,serif}"
	"main{max-width:760px;margin:0 auto;padding:26px 18px}h1{margin:0;color:#0d5860}"
	"p{color:#4a5960}.ok{color:#167247}.error{color:#b14e27}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}"
	"section{margin-top:18px;background:#fff;border:1px solid #b7c8ca;padding:14px;border-radius:6px}"
	".value{font:700 28px ui-monospace,monospace;color:#b14e27}.label{font:13px Arial,sans-serif;color:#53666b}"
	"#status{font-weight:bold}@media(max-width:460px){.grid{grid-template-columns:1fr}}</style></head><body>"
	"<main><h1>LIS2DU12 Test</h1><p id=\"status\">Verbinde...</p><section><div class=\"label\">I2C-Verbindung</div><p id=\"i2c\">Pruefe Bus...</p><p>SDA: <span id=\"sda\">-</span> | SCL: <span id=\"scl\">-</span></p></section><section><div class=\"grid\">"
	"<div><div class=\"label\">X-Achse</div><div class=\"value\" id=\"x\">-</div></div>"
	"<div><div class=\"label\">Y-Achse</div><div class=\"value\" id=\"y\">-</div></div>"
	"<div><div class=\"label\">Z-Achse</div><div class=\"value\" id=\"z\">-</div></div>"
	"</div></section><section><div class=\"grid\">"
	"<div><div class=\"label\">FIFO-Samples</div><div class=\"value\" id=\"fifo\">-</div></div>"
	"<div><div class=\"label\">Single Taps</div><div class=\"value\" id=\"single\">-</div></div>"
	"<div><div class=\"label\">Double Taps</div><div class=\"value\" id=\"double\">-</div></div>"
	"<div><div class=\"label\">Freefall</div><div class=\"value\" id=\"freefalls\">-</div></div>"
	"</div><p>Interrupts: <span id=\"interrupts\">-</span> | TAP_SRC: <span id=\"tap_source\">-</span> | WAKE_UP_SRC: <span id=\"wake_up_source\">-</span></p><p>INT2 (GPIO14): <span id=\"int2\">-</span> | INT2-Flanken: <span id=\"int2_edges\">-</span></p></section>"
	"<section><div class=\"label\">Energiemodus (keine echte Strommessung)</div>"
	"<p>LIS2DU12: <span id=\"lis_mode\">-</span></p>"
	"<p>Richtwert lt. Datenblatt: <span id=\"lis_power_note\">-</span></p>"
	"<p>ESP32 CPU-Takt: <span id=\"cpu_freq\">-</span> MHz | Wi-Fi Power-Save: <span id=\"wifi_ps\">-</span></p>"
	"</section></main>"
	"<script>async function update(){let controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2000);try{let response=await fetch('/api/state?ts='+Date.now(),{cache:'no-store',signal:controller.signal});let d=await response.json();"
	"let i=document.querySelector('#i2c');i.textContent=d.i2c_connected?'Verbunden mit Adresse 0x'+d.i2c_address.toString(16).padStart(2,'0'):'Keine Antwort an 0x19 oder 0x18 (I2C-Fehler '+d.i2c_error+')';i.className=d.i2c_connected?'ok':'error';"
	"document.querySelector('#sda').textContent=d.sda_level?'HIGH (3.3 V)':'LOW (0 V)';document.querySelector('#scl').textContent=d.scl_level?'HIGH (3.3 V)':'LOW (0 V)';"
	"document.querySelector('#status').textContent=d.sensor_found?'Sensor OK, WHO_AM_I: 0x'+d.who_am_i.toString(16).padStart(2,'0'):'Sensor nicht gefunden';"
	"for(let k of ['x','y','z'])document.querySelector('#'+k).textContent=d[k+'_g'].toFixed(3)+' g';"
	"for(let k of ['fifo','single','double','freefalls','interrupts'])document.querySelector('#'+k).textContent=d[k+(k==='fifo'?'_samples':k==='single'?'_taps':k==='double'?'_taps':k==='freefalls'?'':'')];"
	"document.querySelector('#tap_source').textContent='0x'+d.tap_source.toString(16).padStart(2,'0');document.querySelector('#wake_up_source').textContent='0x'+d.wake_up_source.toString(16).padStart(2,'0');"
	"document.querySelector('#int2').textContent=d.int2_level?'HIGH (3.3 V)':'LOW (0 V)';document.querySelector('#int2_edges').textContent=d.int2_edges;"
	"document.querySelector('#lis_mode').textContent=d.lis2du12_mode+' @ '+d.lis2du12_odr_hz+' Hz'+(d.lis2du12_lp_submode?' ('+d.lis2du12_lp_submode+')':'');"
	"document.querySelector('#lis_power_note').textContent=d.lis2du12_power_note;"
	"document.querySelector('#cpu_freq').textContent=d.cpu_freq_mhz;document.querySelector('#wifi_ps').textContent=d.wifi_ps_mode;"
	"}catch(e){document.querySelector('#status').textContent='API nicht erreichbar';document.querySelector('#i2c').textContent='I2C-Status nicht abrufbar';document.querySelector('#i2c').className='error'}finally{clearTimeout(timer);setTimeout(update,500)}}update()</script>"
	"</body></html>";

static void set_status_led(uint8_t red, uint8_t green, uint8_t blue)
{
	if (led_channel == NULL || led_encoder == NULL) {
		return;
	}
	uint8_t grb[] = {green, red, blue};
	rmt_transmit_config_t transmit_config = {.loop_count = 0};
	rmt_transmit(led_channel, led_encoder, grb, sizeof(grb), &transmit_config);
	rmt_tx_wait_all_done(led_channel, -1);
}

static esp_err_t lis2du12_write(uint8_t reg, uint8_t value)
{
	uint8_t data[] = {reg, value};
	return i2c_master_transmit(lis2du12, data, sizeof(data), LIS2DU12_I2C_TIMEOUT_MS);
}

static esp_err_t lis2du12_read(uint8_t reg, uint8_t *data, size_t length)
{
	if (length > 1) {
		reg |= 0x80;
	}
	return i2c_master_transmit_receive(lis2du12, &reg, 1, data, length, LIS2DU12_I2C_TIMEOUT_MS);
}

static bool lis2du12_connect(void)
{
	portENTER_CRITICAL(&state_lock);
	sensor_state.sda_level = gpio_get_level(I2C_SDA_GPIO) != 0;
	sensor_state.scl_level = gpio_get_level(I2C_SCL_GPIO) != 0;
	portEXIT_CRITICAL(&state_lock);
	if (lis2du12 != NULL) {
		return true;
	}

	const uint8_t addresses[] = {0x19, 0x18};
	for (size_t index = 0; index < sizeof(addresses); index++) {
		uint8_t address = addresses[index];
		if (i2c_master_probe(i2c_bus, address, LIS2DU12_I2C_TIMEOUT_MS) != ESP_OK) {
			continue;
		}

		i2c_device_config_t device_config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = address,
			.scl_speed_hz = 100000,
		};
		ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &device_config, &lis2du12));
		lis2du12_address = address;
		portENTER_CRITICAL(&state_lock);
		sensor_state.i2c_connected = true;
		sensor_state.i2c_address = address;
		sensor_state.i2c_error = ESP_OK;
		portEXIT_CRITICAL(&state_lock);
		ESP_LOGI(TAG, "I2C device found at 0x%02X", address);
		return true;
	}

	portENTER_CRITICAL(&state_lock);
	sensor_state.i2c_connected = false;
	sensor_state.i2c_address = 0;
	sensor_state.i2c_error = ESP_ERR_TIMEOUT;
	portEXIT_CRITICAL(&state_lock);
	ESP_LOGE(TAG, "No I2C device responded at 0x19 or 0x18");
	ESP_LOGE(TAG, "I2C idle levels: SDA=%s, SCL=%s",
		sensor_state.sda_level ? "HIGH" : "LOW", sensor_state.scl_level ? "HIGH" : "LOW");
	return false;
}

static bool lis2du12_configure(void)
{
	uint8_t who_am_i = 0;
	esp_err_t result = lis2du12_read(LIS2DU12_REG_WHO_AM_I, &who_am_i, 1);
	portENTER_CRITICAL(&state_lock);
	sensor_state.who_am_i = who_am_i;
	sensor_state.sensor_found = result == ESP_OK && who_am_i == LIS2DU12_WHO_AM_I_VALUE;
	portEXIT_CRITICAL(&state_lock);

	if (result != ESP_OK || who_am_i != LIS2DU12_WHO_AM_I_VALUE) {
		ESP_LOGE(TAG, "Unexpected device at 0x%02X: %s, WHO_AM_I=0x%02X",
			lis2du12_address, esp_err_to_name(result), who_am_i);
		return false;
	}

	// This mounting needs a much more sensitive tap threshold than ST's reference (verified working)
	const struct { uint8_t reg; uint8_t value; } config[] = {
		{LIS2DU12_REG_IF_CTRL, 0x10},           // Enable register address auto-increment
		{LIS2DU12_REG_CTRL1, 0x1C},              // Enable wake-up engines and address increment
		{LIS2DU12_REG_CTRL2, 0x00},
		{LIS2DU12_REG_CTRL3, 0x00},              // Push-pull, active-high interrupt outputs
		{LIS2DU12_REG_CTRL4, 0x20},              // Block-data update
		{LIS2DU12_REG_CTRL5, 0x70},              // 400 Hz, +/-2 g full scale
		{LIS2DU12_REG_FIFO_CTRL, 0x86},          // Continuous FIFO mode, XYZ rounding
		{LIS2DU12_REG_FIFO_WTM, 0x0A},           // FIFO watermark 10 samples
		{LIS2DU12_REG_INTERRUPT_CFG, 0x03},      // Enable and latch interrupt sources
		{LIS2DU12_REG_TAP_THS_X, 0x02},          // Tap threshold X = 2
		{LIS2DU12_REG_TAP_THS_Y, 0x02},          // Tap threshold Y = 2
		{LIS2DU12_REG_TAP_THS_Z, 0xE2},          // Enable X/Y/Z tap axes, tap threshold Z = 2
		{LIS2DU12_REG_INT_DUR, 0x7F},            // Tap shock, quiet and latency windows
		{LIS2DU12_REG_WAKE_UP_THS, 0x80},        // Enable single/double-tap threshold format
		{LIS2DU12_REG_WAKE_UP_DUR, 0x00},        // Freefall duration MSB and wake duration
		{LIS2DU12_REG_FREE_FALL, 0x33},          // FF_THS=3, FF_DUR=6 samples
		{LIS2DU12_REG_MD1_CFG, 0x58},            // Route freefall, single- and double-tap to INT1
		{LIS2DU12_REG_MD2_CFG, 0x00},            // INT2 unused
	};

	for (size_t index = 0; index < sizeof(config) / sizeof(config[0]); index++) {
		result = lis2du12_write(config[index].reg, config[index].value);
		if (result != ESP_OK) {
			ESP_LOGE(TAG, "Register 0x%02X write failed: %s", config[index].reg, esp_err_to_name(result));
			return false;
		}
	}

	uint8_t readback[10] = {0};
	lis2du12_read(LIS2DU12_REG_CTRL1, &readback[0], 1);
	lis2du12_read(LIS2DU12_REG_CTRL3, &readback[1], 1);
	lis2du12_read(LIS2DU12_REG_CTRL4, &readback[2], 1);
	lis2du12_read(LIS2DU12_REG_CTRL5, &readback[3], 1);
	lis2du12_read(LIS2DU12_REG_TAP_THS_X, &readback[4], 1);
	lis2du12_read(LIS2DU12_REG_TAP_THS_Y, &readback[5], 1);
	lis2du12_read(LIS2DU12_REG_TAP_THS_Z, &readback[6], 1);
	lis2du12_read(LIS2DU12_REG_INT_DUR, &readback[7], 1);
	lis2du12_read(LIS2DU12_REG_WAKE_UP_THS, &readback[8], 1);
	lis2du12_read(LIS2DU12_REG_INTERRUPT_CFG, &readback[9], 1);
	uint8_t free_fall_reg = 0;
	lis2du12_read(LIS2DU12_REG_FREE_FALL, &free_fall_reg, 1);
	ESP_LOGI(TAG, "Interrupt regs: CTRL3=0x%02X CTRL4=0x%02X CTRL5=0x%02X INT_CFG=0x%02X WAKE_UP_THS=0x%02X FREE_FALL=0x%02X",
		readback[1], readback[2], readback[3], readback[9], readback[8], free_fall_reg);

	ESP_LOGI(TAG, "LIS2DU12 detected and configured: 400 Hz, +/-2 g, FIFO, tap and freefall enabled");
	portENTER_CRITICAL(&state_lock);
	sensor_state.ctrl1_reg = readback[3];
	portEXIT_CRITICAL(&state_lock);
	return true;
}

// Decodes CTRL1 (ODR[7:4], MODE[3:2], LP_MODE[1:0]) into a human-readable power mode.
static float lis2du12_odr_hz(uint8_t ctrl5)
{
	static const float odr_table[] = {0.0f, 1.6f, 12.5f, 25.0f, 50.0f, 100.0f, 200.0f, 400.0f, 800.0f, 1600.0f};
	uint8_t odr = (ctrl5 >> 4) & 0x0F;
	return odr < (sizeof(odr_table) / sizeof(odr_table[0])) ? odr_table[odr] : 0.0f;
}

static const char *lis2du12_mode_name(uint8_t ctrl1)
{
	(void)ctrl1;
	return "Low-Power";
}

static const char *lis2du12_lp_submode_name(uint8_t ctrl1)
{
	(void)ctrl1;
	return "12-bit";
}

// Only the ultra-low-power headline figures from ST's datasheet are quoted here;
// High-Performance draw depends heavily on ODR and needs a real current-sensor measurement.
static const char *lis2du12_power_note(uint8_t ctrl1)
{
	if (((ctrl1 >> 4) & 0x0F) == 0) {
		return "~50 nA typ. (power-down, ST datasheet)";
	}
	(void)ctrl1;
	return "see LIS2DU12 datasheet; measure externally for an exact value";
}

static void read_sensor_sample(void)
{
	uint8_t raw[6];
	uint8_t fifo_samples = 0;
	if (lis2du12_read(LIS2DU12_REG_OUT_X_L, raw, sizeof(raw)) != ESP_OK) {
		return;
	}
	lis2du12_read(LIS2DU12_REG_FIFO_STATUS2, &fifo_samples, 1);

	int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
	int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
	int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
	portENTER_CRITICAL(&state_lock);
	sensor_state.x_g = (x >> 4) * 0.0009765625f; // 12-bit, +/-2 g full scale
	sensor_state.y_g = (y >> 4) * 0.0009765625f;
	sensor_state.z_g = (z >> 4) * 0.0009765625f;
	sensor_state.fifo_samples = fifo_samples;
	portEXIT_CRITICAL(&state_lock);
}

static void process_interrupt(void)
{
	uint8_t tap_source = 0;
	uint8_t wake_up_source = 0;
	uint8_t all_int_source = 0;
	if (lis2du12_read(LIS2DU12_REG_TAP_SRC, &tap_source, 1) != ESP_OK) {
		portENTER_CRITICAL(&state_lock);
		sensor_state.tap_source = tap_source;
		portEXIT_CRITICAL(&state_lock);
		return;
	}
	lis2du12_read(LIS2DU12_REG_WAKE_UP_SRC, &wake_up_source, 1);
	lis2du12_read(LIS2DU12_REG_ALL_INT_SRC, &all_int_source, 1);
	bool freefall_active = (wake_up_source & LIS2DU12_WAKE_UP_SRC_FF_IA) != 0;
	bool tap_active = (tap_source & (LIS2DU12_TAP_SRC_SINGLE_TAP | LIS2DU12_TAP_SRC_DOUBLE_TAP)) != 0;
	bool new_freefall = freefall_active && !freefall_event_active;
	bool new_tap = tap_active && !tap_event_active;
	freefall_event_active = freefall_active;
	tap_event_active = tap_active;

	portENTER_CRITICAL(&state_lock);
	sensor_state.tap_source = tap_source;
	sensor_state.wake_up_source = wake_up_source;
	sensor_state.all_int_source = all_int_source;
	if (new_freefall) {
		sensor_state.freefalls++;
		sensor_state.interrupts++;
	}
	if (new_tap) {
		sensor_state.interrupts++;
		if ((tap_source & LIS2DU12_TAP_SRC_DOUBLE_TAP) != 0) {
			sensor_state.double_taps++;
		} else if ((tap_source & LIS2DU12_TAP_SRC_SINGLE_TAP) != 0) {
			sensor_state.single_taps++;
		}
	}
	portEXIT_CRITICAL(&state_lock);
	if (new_freefall) {
		set_status_led(80, 30, 0);
		ESP_LOGI(TAG, "Freefall detected: WAKE_UP_SRC=0x%02X", wake_up_source);
	}
	if (new_tap) {
		set_status_led(20, 0, 80);
		ESP_LOGI(TAG, "Tap source: 0x%02X (single=%d double=%d)",
			tap_source,
			(bool)((tap_source & LIS2DU12_TAP_SRC_SINGLE_TAP) != 0),
			(bool)((tap_source & LIS2DU12_TAP_SRC_DOUBLE_TAP) != 0));
	}
}

static void sensor_task(void *argument)
{
	TickType_t next_probe = 0;
	TickType_t next_report = 0;
	TickType_t next_led = 0;
	while (true) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
		sensor_state_t state;
		portENTER_CRITICAL(&state_lock);
		state = sensor_state;
		portEXIT_CRITICAL(&state_lock);
		if (state.sensor_found) {
			read_sensor_sample();
			process_interrupt();
			portENTER_CRITICAL(&state_lock);
			sensor_state.int2_level = gpio_get_level(LIS2DU12_INT2_GPIO) != 0;
			state = sensor_state;
			portEXIT_CRITICAL(&state_lock);
		} else if (xTaskGetTickCount() >= next_probe) {
			next_probe = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
			if (lis2du12_connect()) {
				lis2du12_configure();
				portENTER_CRITICAL(&state_lock);
				state = sensor_state;
				portEXIT_CRITICAL(&state_lock);
			}
		}
		if (xTaskGetTickCount() >= next_led) {
			next_led = xTaskGetTickCount() + pdMS_TO_TICKS(500);
			set_status_led(state.sensor_found ? 0 : 80, state.sensor_found ? 35 : 0, 0);
		}
		if (xTaskGetTickCount() >= next_report) {
			next_report = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
			ESP_LOGI(TAG,
				"DATA i2c=%s addr=0x%02X error=%d sensor=%s WHO_AM_I=0x%02X "
				"SDA=%s SCL=%s XYZ=(%.3f, %.3f, %.3f)g FIFO=%u TAP_SRC=0x%02X WAKE_UP_SRC=0x%02X ALL_INT_SRC=0x%02X taps=%lu/%lu freefalls=%lu irq=%lu INT1_EDGES=%lu INT2=%s INT2_EDGES=%lu",
				state.i2c_connected ? "OK" : "FAIL", state.i2c_address, state.i2c_error,
				state.sensor_found ? "OK" : "FAIL", state.who_am_i,
				state.sda_level ? "HIGH" : "LOW", state.scl_level ? "HIGH" : "LOW",
				state.x_g, state.y_g, state.z_g, state.fifo_samples, state.tap_source, state.wake_up_source, state.all_int_source,
				(unsigned long)state.single_taps, (unsigned long)state.double_taps, (unsigned long)state.freefalls,
				(unsigned long)state.interrupts, (unsigned long)isr_edge_count,
				state.int2_level ? "HIGH" : "LOW", (unsigned long)int2_edge_count);
		}
	}
}

static void IRAM_ATTR int1_isr_handler(void *argument)
{
	isr_edge_count++;
	BaseType_t higher_priority_task_woken = pdFALSE;
	vTaskNotifyGiveFromISR(sensor_task_handle, &higher_priority_task_woken);
	if (higher_priority_task_woken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

static void IRAM_ATTR int2_isr_handler(void *argument)
{
	int2_edge_count++;
}

static esp_err_t index_handler(httpd_req_t *request)
{
	httpd_resp_set_type(request, "text/html");
	httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
	httpd_resp_set_hdr(request, "Pragma", "no-cache");
	return httpd_resp_send(request, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t state_handler(httpd_req_t *request)
{
	sensor_state_t state;
	char response[640];
	portENTER_CRITICAL(&state_lock);
	state = sensor_state;
	portEXIT_CRITICAL(&state_lock);
	httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
	const char *wifi_ps_name = wifi_ps_mode == WIFI_PS_NONE ? "None (radio always on)"
		: wifi_ps_mode == WIFI_PS_MIN_MODEM ? "Minimum modem sleep" : "Maximum modem sleep";
	int length = snprintf(response, sizeof(response),
		"{\"sensor_found\":%s,\"i2c_connected\":%s,\"sda_level\":%s,\"scl_level\":%s,\"i2c_address\":%u,\"i2c_error\":%d,\"who_am_i\":%u,\"x_g\":%.4f,\"y_g\":%.4f,\"z_g\":%.4f,"
		"\"fifo_samples\":%u,\"single_taps\":%lu,\"double_taps\":%lu,\"freefalls\":%lu,\"interrupts\":%lu,\"tap_source\":%u,\"wake_up_source\":%u,\"int2_level\":%s,\"int2_edges\":%lu,"
		"\"lis2du12_mode\":\"%s\",\"lis2du12_odr_hz\":%.1f,\"lis2du12_lp_submode\":\"%s\",\"lis2du12_power_note\":\"%s\",\"cpu_freq_mhz\":%lu,\"wifi_ps_mode\":\"%s\"}",
		state.sensor_found ? "true" : "false", state.i2c_connected ? "true" : "false",
		state.sda_level ? "true" : "false", state.scl_level ? "true" : "false",
		state.i2c_address, state.i2c_error, state.who_am_i, state.x_g, state.y_g, state.z_g,
		state.fifo_samples, (unsigned long)state.single_taps, (unsigned long)state.double_taps,
		(unsigned long)state.freefalls, (unsigned long)state.interrupts, state.tap_source, state.wake_up_source,
		state.int2_level ? "true" : "false", (unsigned long)int2_edge_count,
		lis2du12_mode_name(state.ctrl1_reg), lis2du12_odr_hz(state.ctrl1_reg), lis2du12_lp_submode_name(state.ctrl1_reg),
		lis2du12_power_note(state.ctrl1_reg), (unsigned long)cpu_freq_mhz, wifi_ps_name);
	httpd_resp_set_type(request, "application/json");
	if (length <= 0 || length >= sizeof(response)) {
		return httpd_resp_send(request, "{}", 2);
	}
	return httpd_resp_send(request, response, length);
}

static void start_web_server(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	if (httpd_start(&server, &config) != ESP_OK) {
		ESP_LOGE(TAG, "Could not start HTTP server");
		return;
	}
	httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
	httpd_uri_t state_uri = {.uri = "/api/state", .method = HTTP_GET, .handler = state_handler};
	httpd_register_uri_handler(server, &index_uri);
	httpd_register_uri_handler(server, &state_uri);
}

static void start_access_point(void)
{
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&init_config));
	wifi_config_t ap_config = {
		.ap = {
			.channel = WIFI_AP_CHANNEL,
			.max_connection = WIFI_AP_MAX_CONNECTIONS,
			.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};
	strcpy((char *)ap_config.ap.ssid, WIFI_AP_SSID);
	strcpy((char *)ap_config.ap.password, WIFI_AP_PASSWORD);
	ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	esp_wifi_get_ps(&wifi_ps_mode);
	cpu_freq_mhz = esp_rom_get_cpu_ticks_per_us();
	ESP_LOGI(TAG, "Access point started: %s, open http://192.168.4.1", WIFI_AP_SSID);
}

void app_main(void)
{
	esp_err_t result = nvs_flash_init();
	if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		result = nvs_flash_init();
	}
	ESP_ERROR_CHECK(result);

	rmt_tx_channel_config_t led_channel_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = STATUS_LED_GPIO,
		.mem_block_symbols = 64,
		.resolution_hz = 10 * 1000 * 1000,
		.trans_queue_depth = 1,
	};
	rmt_bytes_encoder_config_t led_encoder_config = {
		.bit0 = {.level0 = 1, .duration0 = 4, .level1 = 0, .duration1 = 9},
		.bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 4},
		.flags.msb_first = 1,
	};
	ESP_ERROR_CHECK(rmt_new_tx_channel(&led_channel_config, &led_channel));
	ESP_ERROR_CHECK(rmt_new_bytes_encoder(&led_encoder_config, &led_encoder));
	ESP_ERROR_CHECK(rmt_enable(led_channel));
	set_status_led(60, 25, 0);

	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = I2C_SDA_GPIO,
		.scl_io_num = I2C_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
	gpio_config_t int_config = {
		.pin_bit_mask = 1ULL << LIS2DU12_INT1_GPIO,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&int_config));
	gpio_config_t int2_config = {
		.pin_bit_mask = 1ULL << LIS2DU12_INT2_GPIO,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_POSEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&int2_config));
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

	if (lis2du12_connect()) {
		lis2du12_configure();
	}
	xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle);
	ESP_ERROR_CHECK(gpio_isr_handler_add(LIS2DU12_INT1_GPIO, int1_isr_handler, NULL));
	ESP_ERROR_CHECK(gpio_isr_handler_add(LIS2DU12_INT2_GPIO, int2_isr_handler, NULL));
	start_access_point();
	start_web_server();
}