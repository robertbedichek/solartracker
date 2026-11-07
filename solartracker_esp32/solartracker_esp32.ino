
/*
   Solar tracker: tilts four LG Neon-2 420 W panels with a hydraulic motor, to get more out of them than a fixed
   mount would (and to look better than a reverse-racked fixed mount).

   ESP32-S3 port of solartracker.ino (SparkFun RedBoard), for the Waveshare ESP32-S3-POE-ETH-8DI-8DO (8 isolated
   Darlington outputs, 500 mA sink each, behind a TCA9554 I2C expander; 8 isolated inputs; W5500 Ethernet with PoE;
   PCF85063 RTC; isolated RS485) plus a Waveshare Modbus RTU Analog Input 8CH and a WitMotion HWT905-RS485
   inclinometer, both on the RS485 pair.  The control logic is the Arduino version's; the I/O, the clock and the
   telemetry path are new, as in the pool controller's port:

   - DO1 sinks the motor power supply SSR's input, DO2 the check-valve solenoid supply SSR's.  The Arduino drove
     each SSR's + input high with its - input grounded; here the SSR's + goes to the supply and its - to the DO
     terminal.  A manual override switch from the SSR's - to ground still works alongside.
   - DI1 is the "lower panels" switch (dry contact between DI1 and DGND, not the terminal printed COM).
   - The inclinometer replaces the draw-wire sensor: the panel position is the tilt in degrees above the
     lowered (rest) position, on whichever of its axes the panels tilt about.  "cal down" (panels resting at the
     bottom) and "cal up" (panels at the top stop) measure it on the frame, and the Arduino's limits, which were
     in draw-wire counts (rest about 50, top stop about 363), become the same fractions of that travel.  Until
     both are done the panels never move.  If the inclinometer stops answering while they move, they carry on:
     the top limit switch cuts the motor supply SSR, and the 5 s lift limit and current checks still apply.
   - Analog CH1 is the motor current Hall sensor (was A1), CH2 the PV current Hall sensor (was A2), read in
     millivolts.
   - Time comes from NTP and is kept in the RTC; the 'd' and 't' console commands are gone.
   - Telemetry lines go to the USB serial port, to a ring buffer served at /log, and (when uploads are on) over
     HTTPS to bedichek.org.  trackctl replaces the serial console.
   - New firmware is uploaded over the network (trackctl update).  A new image must run for a minute with the
     output expander answering and /status fetched over the network, or the bootloader goes back to the old one.

   THIS FIRMWARE IS FOR THE 8DO BOARD ONLY.  On the 8RO the expander's latch polarity is the opposite, and
   writing "all off" here would turn every relay on.  trackctl update refuses a board that doesn't report 8DO.

   Creative Commons Licence
   Robert Bedichek
*/

#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_sntp.h>

#include "secrets.h"  // TRACKER_LAN_TOKEN and TRACKER_INGEST_TOKEN, written by trackctl; not in git

#define FIRMWARE_VERSION __DATE__ " " __TIME__

// Defined before the prototypes the Arduino builder puts after the #includes.
// Inclinometer values (WitMotion HWT905-RS485, Modbus registers 0x34-0x42)
struct Incl {
  bool ok;
  float acc_g[3], gyro_dps[3], roll, pitch, yaw, temp_c;
  int16_t mag[3];
};

// 'P' power-on, 'B' brown-out, 'W' watchdog, 'S' software (e.g. after an update), 'C' crash, 'X' other
char reset_cause = 'P';
bool time_valid = false;

/*****************************************************************************************************/
// Board pins (Waveshare ESP32-S3-POE-ETH-8DI-8DO)

#define I2C_SDA_PIN      (42)
#define I2C_SCL_PIN      (41)
#define TCA9554_ADDR     (0x20)   // Output expander: EXIO1..8 = DO1..8, bit clear = output on (sinking)
#define PCF85063_ADDR    (0x51)   // RTC

#define ETH_CS_PIN       (16)
#define ETH_INT_PIN      (12)
#define ETH_SCLK_PIN     (15)
#define ETH_MOSI_PIN     (13)
#define ETH_MISO_PIN     (14)

#define RS485_TX_PIN     (17)
#define RS485_RX_PIN     (18)
#define RS485_DE_PIN     (21)     // Transceiver direction, driven as the UART's RTS in RS485 half-duplex mode

#define DI1_PIN          (4)      // DI1..DI8 are GPIO4..GPIO11, low when the input is active

#define LOWER_PANEL_REQUEST_INPUT_PIN (DI1_PIN)  // When closed, we turn on the solenoid and let the panels fall

#define MOTOR_PS_OUTPUT     (1)   // DO1: SSR that powers the 9 V / 100 A supply wired straight to the hydraulic motor
#define SOLENOID_PS_OUTPUT  (2)   // DO2: SSR that powers the check-valve solenoid, which lets the panels down

// Analog module channels (0-based)
#define MOTOR_CURRENT_CHANNEL   (0)   // CH1: Hall sensor on the motor lead: 2.5 V at 0 A, 20 mV per amp
#define PV_CURRENT_CHANNEL      (1)   // CH2: Hall sensor clamped on one panel's DC cable: 2.5 V at 0 A, 200 mV per amp

/*****************************************************************************************************/
// Outputs (DO1-8 behind the TCA9554).  All off at every boot: nothing may move until the control code asks.

class Onboard_Outputs {
public:
  bool begin(void);
  void on(uint8_t n)        { set(n, true); }
  void off(uint8_t n)       { set(n, false); }
  bool state(uint8_t n)     { return bits & (1 << (n - 1)); }
  uint8_t all(void)         { return bits; }
  bool verify(void);
private:
  uint8_t bits = 0;
  void set(uint8_t n, bool on);
};

Onboard_Outputs *outputs;

/*****************************************************************************************************/
#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

Scheduler ts;

#define SERIAL_BAUD (115200)

// This string variable is used by multiple functions below, but not at the same time
char cbuf[100];

bool lower_panel_request;  // True when the "lower panel request" switch is closed, indicating that the user wants the panels to go down

const unsigned long max_solenoid_on_time = 1800 * 1000UL;
const unsigned long max_solenoid_off_time = 2700 * 1000UL;
unsigned long solenoid_power_supply_on_off_time;  // Value of millis() when we last turned on the solenoid on or off

// Value of millis() the last time we drove the panels up
unsigned long last_drive_panels_up_time;


const unsigned motor_current_threshold_low = 10;
const unsigned motor_current_threshold_high = 90;

//   No-movement mode stays alive and reports but does not move the panels; position mode tracks.
enum mode_e { no_panel_movement_mode,
              position_mode,
              last_mode };

const char *mode_names[] = {"off", "track"};

struct calvals_s {
  float position_upper_limit;     // Degrees above rest; the highest we drive to
  float position_lower_limit;     // Below this the panels are down
  enum mode_e operation_mode;     // Saved in flash ("mode off|track")
} calvals;

// Tilt calibration, saved in flash.  Position = sign x (angle on 'axis' - 'down'), in degrees, 0 at rest.
struct Tilt_Cal {
  bool valid;
  int axis;              // 0 roll, 1 pitch
  float down;            // That axis's angle with the panels resting at the bottom
  float travel;          // Degrees from rest to the top stop, signed by the axis's direction
  bool have_down;        // "cal down" done since boot (needed before "cal up")
  float down_angles[2];  // Both axes' angles at "cal down"
} tcal;

const char *axis_names[] = {"roll", "pitch"};

// The Arduino's thresholds were in draw-wire counts with the rest position about 50 and the top stop about 363.
// This turns a distance in those counts into the same fraction of the measured travel, in degrees.
static float from_counts(float counts) { return counts / 313.0 * fabsf(tcal.travel); }

//   Calculated from 'position_sensor_val' and the position limit calibration values.
bool at_upper_position_limit = false, at_lower_position_limit = false;

float position_sensor_val;                // Degrees above the rest position (inclinometer)
float pv_current(void);                   // Amps flowing past Hall Effect current sensor clamped to a solar panel DC cable
float peak_pv_current;                    // Highest recorded PV current during last panel raise
float minimum_pv_current_measured = 100.0;

bool position_fresh;                      // The inclinometer answered at the last read; else position_sensor_val is the last known
float last_position_sensor_val;           // Position value at last call to status print
float last_position_sensor_val_stall;     // Position sensor value at last sample in monitor_motor_stall_callback()

unsigned daily_stalls;                    // Zeroed at dawn, incremented on each motor stall
unsigned long stall_start_time;
unsigned long under_current_start_time;

bool failed = false;                      // Set by fail(): nothing moves until a reboot
const char *fail_reason = "";

bool panels_going_up = false;
bool panels_going_down = false;
bool motor_power_supply_is_on = false;
bool solenoid_power_supply_is_on = false;

void read_time_and_sensor_inputs_callback();
Task read_time_and_sensor_inputs(TASK_SECOND, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
void emit_telemetry_callback();
Task emit_telemetry(TASK_SECOND, TASK_FOREVER, &emit_telemetry_callback, &ts, true);
void control_hydraulics_callback();
Task control_hydraulics(TASK_SECOND * 10, TASK_FOREVER, &control_hydraulics_callback, &ts, true);
void monitor_position_limits_callback();
Task monitor_position_limits(50, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
void monitor_stall_and_motor_current_callback();
Task monitor_stall_and_motor_current(200, TASK_FOREVER, &monitor_stall_and_motor_current_callback, &ts, false);
void monitor_serial_console_callback(void);
Task monitor_serial_console(100, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);
void check_outputs_callback(void);
Task check_outputs(TASK_SECOND * 10, TASK_FOREVER, &check_outputs_callback, &ts, true);
void sync_rtc_callback(void);
Task sync_rtc(TASK_SECOND, TASK_FOREVER, &sync_rtc_callback, &ts, true);
void solenoid_test_callback(void);
Task solenoid_test(TASK_SECOND, TASK_FOREVER, &solenoid_test_callback, &ts, false);
unsigned long solenoid_test_until;
void motor_test_callback(void);
Task motor_test(100, TASK_FOREVER, &motor_test_callback, &ts, false);
unsigned long motor_test_until;        // "motor on" from trackctl: DO1 on for at most 5 s (a lift's limit), for wiring checks     // "solenoid on" from trackctl: DO2 on for a limited time, for wiring checks
void mark_firmware_good_callback(void);
Task mark_firmware_good(TASK_SECOND * 60, TASK_ONCE, &mark_firmware_good_callback, &ts, false);

/*****************************************************************************************************/
// Output.  Everything the controller says goes through 'out': to the USB serial port, to a ring of
// recent lines (served at /log), and to a queue that the upload task sends to bedichek.org.

class Line_Buffer {
public:
  void init(size_t capacity, bool use_psram);
  void append(const char *line, size_t n);      // n includes the newline
  size_t length(void) { return len; }
  size_t peek(char *dst, size_t max);           // Copies up to max bytes of whole lines from the front
  void consume(size_t n);
  void copy_tail(String &dst, size_t max);
private:
  char *buf = nullptr;
  size_t cap = 0;
  size_t len = 0;
};

void Line_Buffer::init(size_t capacity, bool use_psram)
{
  buf = (char *)(use_psram && psramFound() ? ps_malloc(capacity) : malloc(capacity));
  cap = buf ? capacity : 0;
}

void Line_Buffer::append(const char *line, size_t n)
{
  if (n > cap) {
    return;
  }
  if (len + n > cap) {    // Drop the oldest whole lines to make room
    size_t drop = len + n - cap;
    while (drop < len && buf[drop - 1] != '\n') {
      drop++;
    }
    consume(drop);
  }
  memcpy(buf + len, line, n);
  len += n;
}

size_t Line_Buffer::peek(char *dst, size_t max)
{
  size_t n = len < max ? len : max;
  while (n > 0 && buf[n - 1] != '\n') {
    n--;
  }
  memcpy(dst, buf, n);
  return n;
}

void Line_Buffer::consume(size_t n)
{
  if (n >= len) {
    len = 0;
  } else {
    memmove(buf, buf + n, len - n);
    len -= n;
  }
}

void Line_Buffer::copy_tail(String &dst, size_t max)
{
  size_t start = len > max ? len - max : 0;
  while (start > 0 && start < len && buf[start - 1] != '\n') {
    start++;
  }
  dst.concat(buf + start, len - start);
}

Line_Buffer recent_lines;     // The last 64 KB, for /log
Line_Buffer upload_queue;     // Not yet accepted by bedichek.org
bool upload_enabled = false;  // Saved in flash; off on the bench ("upload on|off")
SemaphoreHandle_t lines_mutex;

class Tee_Output : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *b, size_t n) override { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
private:
  char line[400];
  size_t n = 0;
};

size_t Tee_Output::write(uint8_t c)
{
  Serial.write(c);
  if (c == '\r') {
    return 1;
  }
  if (n < sizeof(line) - 1) {
    line[n++] = c;
  }
  if (c == '\n') {
    if (line[n - 1] != '\n') {
      line[n++] = '\n';
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    recent_lines.append(line, n);
    if (upload_enabled) {
      upload_queue.append(line, n);
    }
    xSemaphoreGive(lines_mutex);
    n = 0;
  }
  return 1;
}

Tee_Output out;

/*****************************************************************************************************/
// Time.  These have the names and meanings of the TimeLib functions the Arduino version used.

time_t now(void)          { return time(nullptr); }
static struct tm tm_of(time_t t) { struct tm tm; localtime_r(&t, &tm); return tm; }
int hour(time_t t)        { return tm_of(t).tm_hour; }
int minute(time_t t)      { return tm_of(t).tm_min; }
int second(time_t t)      { return tm_of(t).tm_sec; }
int day(time_t t)         { return tm_of(t).tm_mday; }
int month(time_t t)       { return tm_of(t).tm_mon + 1; }
int year(time_t t)        { return tm_of(t).tm_year + 1900; }

#define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"

/*****************************************************************************************************/
// RS485 bus: the analog module (address 1) and the inclinometer (0x50) share it, 9600 8N1.  A FreeRTOS task
// owns the bus: every 100 ms it reads the analog module (about 30 ms) and the inclinometer (about 60 ms), and
// leaves the readings here for the control tasks.

#define ANALOG_MODBUS_ADDR   (1)
#define INCL_MODBUS_ADDR     (0x50)
#define BUS_BAUD             (9600)
#define ANALOG_CHANNELS_READ (2)
#define RING                 (8)        // Readings kept per channel (800 ms)
#define ANGLE_RING           (4)
#define INCL_REGS            (15)       // 0x34..0x42: acceleration, rotation rate, magnetometer, angles

HardwareSerial RS485(1);

struct Bus_Data {
  uint16_t ring[ANALOG_CHANNELS_READ][RING];  // Millivolts, newest at index (count - 1) % RING
  uint32_t count;                             // Analog readings so far
  uint32_t analog_failures;                   // Consecutive failed analog reads
  int16_t incl_regs[INCL_REGS];
  float angles[ANGLE_RING][2];                // Roll and pitch, newest at (incl_reads - 1) % ANGLE_RING
  bool incl_ok;
  uint32_t incl_failures;                     // Consecutive failed inclinometer reads
  uint32_t incl_reads;
} bus;
portMUX_TYPE bus_lock = portMUX_INITIALIZER_UNLOCKED;

// Why the last transaction failed: 0 ok, 1 no reply, 2 short reply, 3 wrong address/function, 4 bad CRC
volatile int modbus_last_error;
volatile size_t modbus_last_got;
uint32_t incl_errors[5];          // Inclinometer reads by modbus_last_error
uint8_t incl_last_bad[40];        // First bytes of the last failed inclinometer reply
size_t incl_last_bad_len;

static uint16_t modbus_crc(const uint8_t *b, size_t n)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= b[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// Sends a request (the CRC is appended here) and reads a reply of reply_len bytes.  Returns true
// if the reply has the right length, address, function and CRC.
static bool modbus_transaction(uint8_t *req, size_t req_len, uint8_t *reply, size_t reply_len)
{
  uint16_t crc = modbus_crc(req, req_len);
  req[req_len] = crc & 0xFF;
  req[req_len + 1] = crc >> 8;
  // Modbus needs at least 3.5 characters (4 ms at 9600) of silence between frames.  Without a gap after the
  // analog module's reply, the inclinometer took the two frames as one and ignored most requests (2026-11-05).
  vTaskDelay(pdMS_TO_TICKS(10));
  while (RS485.available()) {
    RS485.read();
  }
  RS485.write(req, req_len + 2);
  RS485.flush();
  size_t got = 0;
  unsigned long start = millis();
  while (got < reply_len && millis() - start < 100) {
    if (RS485.available()) {
      reply[got++] = RS485.read();
    } else {
      vTaskDelay(1);
    }
  }
  modbus_last_got = got;
  if (got != reply_len || reply[0] != req[0] || reply[1] != req[1]) {
    modbus_last_error = got == 0 ? 1 : (got != reply_len ? 2 : 3);
    return false;
  }
  if (modbus_crc(reply, reply_len - 2) != (reply[reply_len - 2] | (reply[reply_len - 1] << 8))) {
    modbus_last_error = 4;
    return false;
  }
  modbus_last_error = 0;
  return true;
}

static bool analog_read_millivolts(uint16_t mv[ANALOG_CHANNELS_READ])
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x04, 0x00, 0x00, 0x00, ANALOG_CHANNELS_READ};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_READ];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_READ; i++) {
    mv[i] = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
  }
  return true;
}

// Puts the channels we read in 0-5 V mode (data type 0) if they are not already.  The module keeps the
// setting, so this normally writes nothing.  Each channel's jumper must be removed for voltage mode.
static bool analog_set_voltage_mode(void)
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x03, 0x10, 0x00, 0x00, ANALOG_CHANNELS_READ};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_READ];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_READ; i++) {
    uint16_t type = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
    if (type != 0) {
      uint8_t wreq[8] = {ANALOG_MODBUS_ADDR, 0x06, 0x10, (uint8_t)i, 0x00, 0x00};
      uint8_t wreply[8];
      if (!modbus_transaction(wreq, 6, wreply, sizeof(wreply))) {
        return false;
      }
      out.printf("# analog channel %d set to 0-5V mode\n", i + 1);
    }
  }
  return true;
}

static bool incl_read(int16_t regs[INCL_REGS])
{
  uint8_t req[8] = {INCL_MODBUS_ADDR, 0x03, 0x00, 0x34, 0x00, INCL_REGS};
  uint8_t reply[5 + 2 * INCL_REGS];
  bool ok = modbus_transaction(req, 6, reply, sizeof(reply));
  incl_errors[modbus_last_error]++;
  if (!ok) {
    incl_last_bad_len = modbus_last_got < sizeof(incl_last_bad) ? modbus_last_got : sizeof(incl_last_bad);
    memcpy(incl_last_bad, reply, incl_last_bad_len);
    return false;
  }
  for (int i = 0; i < INCL_REGS; i++) {
    regs[i] = (int16_t)((reply[3 + 2 * i] << 8) | reply[4 + 2 * i]);
  }
  return true;
}

void bus_task(void *arg)
{
  bool mode_set = false;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    if (!mode_set) {
      mode_set = analog_set_voltage_mode();
    }
    uint16_t mv[ANALOG_CHANNELS_READ];
    bool ok = mode_set && analog_read_millivolts(mv);
    portENTER_CRITICAL(&bus_lock);
    if (ok) {
      int slot = bus.count % RING;
      for (int c = 0; c < ANALOG_CHANNELS_READ; c++) {
        bus.ring[c][slot] = mv[c];
      }
      bus.count++;
      bus.analog_failures = 0;
    } else {
      bus.analog_failures++;
    }
    portEXIT_CRITICAL(&bus_lock);

    int16_t regs[INCL_REGS];
    bool iok = incl_read(regs);
    portENTER_CRITICAL(&bus_lock);
    if (iok) {
      memcpy(bus.incl_regs, regs, sizeof(regs));
      int slot = bus.incl_reads % ANGLE_RING;
      bus.angles[slot][0] = regs[9] / 32768.0 * 180.0;     // Roll (see inclinometer())
      bus.angles[slot][1] = regs[10] / 32768.0 * 180.0;    // Pitch
      bus.incl_ok = true;
      bus.incl_failures = 0;
      bus.incl_reads++;
    } else if (++bus.incl_failures >= 5) {   // Half a second without an answer
      bus.incl_ok = false;
    }
    portEXIT_CRITICAL(&bus_lock);
    if ((int32_t)(xTaskGetTickCount() - wake) > 0) {
      wake = xTaskGetTickCount();       // Overran; don't try to catch up
    }
  }
}

// The last n readings of a channel (newest last); returns how many there were
static int channel_readings(int channel, int n, int dst[])
{
  portENTER_CRITICAL(&bus_lock);
  int have = bus.count < (uint32_t)n ? bus.count : n;
  for (int i = 0; i < have; i++) {
    dst[i] = bus.ring[channel][(bus.count - have + i) % RING];
  }
  portEXIT_CRITICAL(&bus_lock);
  return have;
}

bool analog_ok(void)
{
  return bus.count > 0 && bus.analog_failures < 10;   // 1 s
}

static int mv_to_counts(int mv) { return (mv * 1023L + 2500) / 5000; }

Incl inclinometer(void)
{
  Incl v = {};
  int16_t r[INCL_REGS];
  portENTER_CRITICAL(&bus_lock);
  v.ok = bus.incl_ok;
  memcpy(r, bus.incl_regs, sizeof(r));
  portEXIT_CRITICAL(&bus_lock);
  for (int i = 0; i < 3; i++) {
    v.acc_g[i] = r[i] / 32768.0 * 16.0;
    v.gyro_dps[i] = r[3 + i] / 32768.0 * 2000.0;
    v.mag[i] = r[6 + i];
  }
  // Angles are 16-bit, value / 32768 x 180 degrees (0x3D roll, 0x3E pitch, 0x3F yaw), checked against the
  // accelerometer 2026-11-05; 0x40 is the temperature in hundredths of a degree C
  v.roll  = r[9]  / 32768.0 * 180.0;
  v.pitch = r[10] / 32768.0 * 180.0;
  v.yaw   = r[11] / 32768.0 * 180.0;
  v.temp_c = r[12] / 100.0;
  return v;
}

static float wrap180(float a)
{
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

// Mean of the last two readings of both axes; false if the inclinometer isn't answering
static bool incl_angles(float a[2])
{
  portENTER_CRITICAL(&bus_lock);
  bool ok = bus.incl_ok && bus.incl_reads >= 2;
  if (ok) {
    float *x = bus.angles[(bus.incl_reads - 1) % ANGLE_RING], *y = bus.angles[(bus.incl_reads - 2) % ANGLE_RING];
    for (int i = 0; i < 2; i++) {
      a[i] = x[i] + wrap180(y[i] - x[i]) / 2;
    }
  }
  portEXIT_CRITICAL(&bus_lock);
  return ok;
}

// The panels' tilt above rest, in degrees; false if it isn't known (not calibrated, or no inclinometer)
bool panel_tilt(float *deg)
{
  float a[2];
  if (!tcal.valid || !incl_angles(a)) {
    return false;
  }
  *deg = wrap180(a[tcal.axis] - tcal.down) * (tcal.travel < 0 ? -1 : 1);
  return true;
}

bool position_known(void)
{
  float d;
  return panel_tilt(&d);
}

// The Arduino's limits (lower 70, upper 330, rest 50, in counts) as degrees of the measured travel
void apply_calibration(void)
{
  calvals.position_lower_limit = from_counts(70 - 50);
  calvals.position_upper_limit = from_counts(330 - 50);
}

void save_calibration(void)
{
  Preferences prefs;
  prefs.begin("tracker", false);
  prefs.putBool("cal_ok", tcal.valid);
  prefs.putInt("cal_axis", tcal.axis);
  prefs.putFloat("cal_down", tcal.down);
  prefs.putFloat("cal_travel", tcal.travel);
  prefs.end();
}

void load_calibration(void)
{
  Preferences prefs;
  prefs.begin("tracker", true);
  tcal.valid = prefs.getBool("cal_ok", false);
  tcal.axis = prefs.getInt("cal_axis", 0) ? 1 : 0;
  tcal.down = prefs.getFloat("cal_down", 0);
  tcal.travel = prefs.getFloat("cal_travel", 0);
  prefs.end();
  if (fabsf(tcal.travel) < 10) {
    tcal.valid = false;
  }
  apply_calibration();
}

// "cal down": the panels are resting at the bottom.  "cal up": they are at the top stop; the axis that moved
// most is the tilt axis.  "cal clear" forgets it (nothing moves until it is done again).  "cal" reports it.
void calibrate(const char *what)
{
  float a[2];
  if (strcmp(what, "") == 0) {
    if (tcal.valid) {
      out.printf("# calibrated: axis %s, rest at %.2f, travel %.2f deg; limits %.2f..%.2f deg\n", axis_names[tcal.axis],
                 tcal.down, tcal.travel, calvals.position_lower_limit, calvals.position_upper_limit);
    } else {
      out.println(F("# not calibrated: 'cal down' with the panels resting at the bottom, then 'cal up' at the top stop"));
    }
  } else if (strcmp(what, "clear") == 0) {
    stop_driving_panels("cal clear");
    tcal.valid = false;
    tcal.have_down = false;
    save_calibration();
    out.println(F("# calibration cleared; the panels won't move until 'cal down' and 'cal up'"));
  } else if (!incl_angles(a)) {
    out.println(F("# inclinometer not answering"));
  } else if (strcmp(what, "down") == 0) {
    tcal.down_angles[0] = a[0];
    tcal.down_angles[1] = a[1];
    tcal.have_down = true;
    out.printf("# rest position: roll %.2f, pitch %.2f; now 'cal up' with the panels at the top stop\n", a[0], a[1]);
  } else if (strcmp(what, "up") == 0) {
    if (!tcal.have_down) {
      out.println(F("# 'cal down' first"));
      return;
    }
    float d[2] = {wrap180(a[0] - tcal.down_angles[0]), wrap180(a[1] - tcal.down_angles[1])};
    int axis = fabsf(d[1]) > fabsf(d[0]) ? 1 : 0;
    if (fabsf(d[axis]) < 10) {
      out.printf("# only %.2f deg roll and %.2f deg pitch from rest; are the panels at the top stop?\n", d[0], d[1]);
      return;
    }
    stop_driving_panels("calibration");
    tcal.axis = axis;
    tcal.down = tcal.down_angles[axis];
    tcal.travel = d[axis];
    tcal.valid = true;
    save_calibration();
    apply_calibration();
    out.printf("# alert calibrated: axis %s, travel %.2f deg (the other axis moved %.2f); limits %.2f..%.2f deg\n",
               axis_names[axis], d[axis], d[1 - axis], calvals.position_lower_limit, calvals.position_upper_limit);
  } else {
    out.println(F("# cal, cal down, cal up, cal clear"));
  }
}

/*****************************************************************************************************/
// Flag a warning if there is less than this much free heap.
#define LOW_MEMORY_LIMIT (20000)
uint32_t lowest_memory = UINT32_MAX;

void check_free_memory(void)
{
  static bool warned = false;
  uint32_t fm = ESP.getFreeHeap();
  if (fm < LOW_MEMORY_LIMIT && !warned) {
    out.printf("# alert low memory: %u\n", (unsigned)fm);
    warned = true;
  }
  if (fm < lowest_memory) {
    lowest_memory = fm;
  }
}

/*****************************************************************************************************/

void set_calvals_to_defaults() {
  calvals.operation_mode = position_mode;  // Mode in which we should start operation
  load_calibration();                      // Sets the limits
}

void turn_on_motor_power_supply(void)
{
  if (outputs) outputs->on(MOTOR_PS_OUTPUT);
  motor_power_supply_is_on = true;
}

void turn_off_motor_power_supply(void)
{
  if (outputs) outputs->off(MOTOR_PS_OUTPUT);
  motor_power_supply_is_on = false;
}

void turn_on_solenoid_power_supply(void)
{
  if (outputs) outputs->on(SOLENOID_PS_OUTPUT);
  solenoid_power_supply_is_on = true;
  solenoid_power_supply_on_off_time = millis();
}

void turn_off_solenoid_power_supply(void)
{
  if (outputs) outputs->off(SOLENOID_PS_OUTPUT);
  solenoid_power_supply_is_on = false;
  solenoid_power_supply_on_off_time = millis();
}

/*
    Turn off both outputs.  Since we aren't driving the panels, we also disable the tasks that monitor position
    and motor current.
*/
void stop_driving_panels(const char *who_called)
{
  turn_off_motor_power_supply();
  turn_off_solenoid_power_supply();
  read_time_and_sensor_inputs.setInterval(TASK_SECOND);
  emit_telemetry.setInterval(TASK_SECOND);

  if (who_called != nullptr) {
    out.printf("# stop_driving_panels() secs=: %u, why=%s\n",
               (unsigned)((millis() - last_drive_panels_up_time) / 1000UL), who_called);
  }
  panels_going_down = false;
  panels_going_up = false;

  monitor_position_limits.disable();
  monitor_stall_and_motor_current.disable();
}

/*
   Called by failure paths that should never happen.  The Arduino stopped for good (abort); here the panels are
   stopped and nothing moves again until a reboot, but the controller still answers on the network.
*/
void fail(const char *fail_message)
{
  stop_driving_panels("fail");
  failed = true;
  fail_reason = fail_message;
  out.printf("# alert fail: %s; panels stopped until a reboot\n", fail_message);
}

/*
   Start the panels moving up and enable the tasks that monitor position and current.  It is a fatal error if the
   panels were going down when this was called.
*/
void drive_panels_up(void)
{
  if (failed) {
    return;
  }
  if (panels_going_down) {
    fail("drive_panels_up()");
  } else {
    out.println(F("# lift "));
    turn_on_motor_power_supply();
    last_drive_panels_up_time = millis();
    peak_pv_current = pv_current();     // Start new peak-finding interval
    panels_going_up = true;
    stall_start_time = 0;
    under_current_start_time = 0;
    last_position_sensor_val_stall = position_sensor_val;
    read_time_and_sensor_inputs.setInterval(100); // Sample quickly when panels are in motion
    emit_telemetry.setInterval(200);
    monitor_position_limits.enable();
    monitor_stall_and_motor_current.enable();
  }
}

/*
   Start the panels moving down and enable the task that monitors position.  It is a fatal error if the panels
   were going up when this was called.
*/
void drive_panels_down(const char *why)
{
  if (failed || at_lower_position_limit) {
    return;
  } else if (panels_going_up) {
    fail("drive_panels_down");
  } else {
    turn_on_solenoid_power_supply(); // Release check valve so that panels can go down
    out.printf("# retract: %s\n", why);
    panels_going_down = true;
    stall_start_time = 0;
    under_current_start_time = 0;
    monitor_position_limits.enable();
  }
}

/*
   Reads the sensors and deposits the results in the globals above.  Every second, or every 100 ms while the panels
   move.  The position is the inclinometer's tilt (mean of its last two readings, 100 ms apart).  The analog
   module is still required while moving: without the motor current there's no over-current stop.
*/
void read_time_and_sensor_inputs_callback()
{
  check_free_memory();

  // If the inclinometer stops answering while the panels move, they keep moving on the last position (Rob,
  // 2026-11-05): up is the only direction we drive, and the top limit switch cuts the motor supply SSR by itself.
  // The 5 s lift limit and the motor current checks still apply; the stall check, which needs the position to
  // rise, is skipped.  A move doesn't start without it.
  float tilt;
  position_fresh = panel_tilt(&tilt);
  if (position_fresh) {
    position_sensor_val = tilt;
  }
  if ((panels_going_up || panels_going_down) && !analog_ok()) {
    stop_driving_panels("analog module not answering");
  }

  at_upper_position_limit = tcal.valid && position_sensor_val >= (calvals.position_upper_limit - from_counts(20));
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;

  lower_panel_request = digitalRead(LOWER_PANEL_REQUEST_INPUT_PIN) == LOW;

  static bool reported_analog = true;
  if (analog_ok() != reported_analog) {
    reported_analog = analog_ok();
    out.println(reported_analog ? F("# alert analog module answering") : F("# alert analog module not answering"));
  }
  static bool reported_incl = true;
  bool incl_ok = inclinometer().ok;
  if (incl_ok != reported_incl) {
    reported_incl = incl_ok;
    out.println(incl_ok ? F("# inclinometer answering") : F("# alert inclinometer not answering"));
  }
}

int motor_amps(void)
{
  int r[3];
  int n = channel_readings(MOTOR_CURRENT_CHANNEL, 3, r);
  if (n == 0 || !analog_ok()) {
    return 0;      // "analog module not answering" has been reported
  }
  int motor_current_sense_millivolts = 0;
  int samples = 0;
  for (int i = 0; i < n; i++) {
    if (r[i] > 0 && r[i] < 5000) {
      motor_current_sense_millivolts += r[i];
      samples++;
    }
  }
  if (samples == 0) {
    static int warnings;
    if (warnings++ < 10) {
      out.println(F("# alert all motor current samples out of range"));
    }
    return 0;
  }
  motor_current_sense_millivolts /= samples;

  // The Hall Effect current sensor outputs a signal that is nominally 2.5 +/- 2V, 20 mV per amp.  The clamp
  // could be on either way round, so take the absolute value of the difference from 2.5 V.
  float motor_current_val = (2500.0 - motor_current_sense_millivolts) / 20.0;
  if (motor_current_val < 0) {
    motor_current_val = -motor_current_val;
  }
  float motor_current_offset = 0.0;
  if (motor_power_supply_is_on == false) {
    motor_current_offset = motor_current_val;
  }
  return motor_current_val - motor_current_offset;
}

// Return the number of amps that one of the panels is producing.  We use this to help determine
// the optimal panel position.
float pv_current(void)
{
  int r[RING];
  int n = channel_readings(PV_CURRENT_CHANNEL, RING, r);
  if (n == 0) {
    return 0;
  }
  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += r[i];
  }
  // Rounded to the Arduino's 10-bit steps (4.9 mV), so the 0.1 A peak test sees the same resolution
  float pv_current_sensor_millivolts = mv_to_counts(sum / n) * 5000.0 / 1023.0;

  // 2.5 V nominal at 0 A, 200 mV per amp, either direction (see motor_amps)
  float pv_current_val = (2476.4 - pv_current_sensor_millivolts) / 200.0;
  if (pv_current_val < 0) {
    pv_current_val = -pv_current_val;
  }
  if (pv_current_val < minimum_pv_current_measured) {
    minimum_pv_current_measured = pv_current_val;
  }
  return pv_current_val - minimum_pv_current_measured;
}

/*
   The telemetry line, in the Arduino's format (Pos now in degrees above rest; "PV Amps" is now one column, PVAmps,
   so the header names line up), then every inclinometer value: acceleration (g), rotation rate (deg/s),
   magnetometer (raw), roll, pitch, yaw (deg) and its temperature (C).  "-" when it isn't answering.  Arduino-era line:

  2025-04-06 01:01:07 1   58    0    0    0  328  11.534   0   0   1   0   1   0   0   0   0  0.0
*/

int force_telemetry;               // Incrementing this forces a line of telemetry to be emitted

void emit_telemetry_callback(void)
{
  static unsigned char line_counter = 0;
  static enum mode_e last_operation_mode = last_mode;  // Force a difference the first time
  static bool last_panels_going_up;
  static bool last_panels_going_down;
  static bool last_solenoid_is_on;
  static bool last_motor_is_on;
  static bool last_lower_panel_request;
  static int last_motor_amps;
  static float last_pv_current;
  static int last_force_telemetry;
  static int skipped_record_counter = 0;

  if (!time_valid) {
    return;   // Lines without a real date would land in the wrong place on the graphs
  }

  int cur_motor_amps = motor_amps();
  if (last_position_sensor_val == 0) {
    last_position_sensor_val = position_sensor_val;
  }
  float position_difference = position_sensor_val - last_position_sensor_val;

  if (force_telemetry != last_force_telemetry ||
      fabsf(position_difference) > from_counts(10) ||
      last_operation_mode != calvals.operation_mode ||
      last_panels_going_up != panels_going_up ||
      last_panels_going_down != panels_going_down ||
      last_solenoid_is_on != solenoid_power_supply_is_on ||
      last_motor_is_on != motor_power_supply_is_on ||
      last_lower_panel_request != lower_panel_request ||
      last_motor_amps != cur_motor_amps ||
      (panels_going_up || skipped_record_counter-- <= 0)) {

    last_force_telemetry = force_telemetry;
    last_position_sensor_val = position_sensor_val;
    last_operation_mode = calvals.operation_mode;
    last_panels_going_up = panels_going_up;
    last_panels_going_down = panels_going_down;
    last_solenoid_is_on = solenoid_power_supply_is_on;
    last_motor_is_on = motor_power_supply_is_on;
    last_lower_panel_request = lower_panel_request;
    last_motor_amps = cur_motor_amps;
    last_pv_current = pv_current();
    skipped_record_counter = at_lower_position_limit ? 900 : 180;
    if (line_counter == 0) {
      out.println(F("# Date     Time     Md   Pos Amps UpL Dnl GUp GDn Sol Mot Low PVAmps  AccX   AccY   AccZ    GyrX    GyrY    GyrZ   MagX  MagY  MagZ    Roll   Pitch     Yaw  ITmp"));
      line_counter = 20;
    } else {
      line_counter--;
    }

    time_t t = now();
    char pv[12];
    dtostrf(last_pv_current, 4, 1, pv);
    Incl incl = inclinometer();
    char iv[200] = "";
    if (incl.ok) {
      snprintf(iv, sizeof(iv), "%6.3f %6.3f %6.3f %7.2f %7.2f %7.2f %5d %5d %5d %7.2f %7.2f %7.2f %5.2f",
               incl.acc_g[0], incl.acc_g[1], incl.acc_g[2], incl.gyro_dps[0], incl.gyro_dps[1], incl.gyro_dps[2],
               incl.mag[0], incl.mag[1], incl.mag[2], incl.roll, incl.pitch, incl.yaw, incl.temp_c);
    } else {
      strcpy(iv, "     -      -      -       -       -       -     -     -     -       -       -       -     -");
    }
    out.printf("%4u-%02u-%02u %02u:%02u:%02u %d %5.1f %4d %3d %3d %3d %3d %3d %3d %3d %6s %s\n",
               year(t), month(t), day(t), hour(t), minute(t), second(t),
               calvals.operation_mode, position_sensor_val, cur_motor_amps,
               at_upper_position_limit, at_lower_position_limit, panels_going_up, panels_going_down,
               solenoid_power_supply_is_on, motor_power_supply_is_on, lower_panel_request,
               pv, iv);
  }
}

/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/
void monitor_position_limits_callback()
{
  if (panels_going_up && (millis() - last_drive_panels_up_time) > 5000UL) {
    stop_driving_panels("5s raise limit");
  }
  if (panels_going_up && position_sensor_val > (calvals.position_upper_limit + from_counts(5))) {
    stop_driving_panels("upper limit reached");
  }
  if (panels_going_down) {
    if (position_sensor_val <= (calvals.position_lower_limit - from_counts(5))) {
      stop_driving_panels("lower limit reached");
    } else {
      if (hour(now()) == 8) {
        // By 8AM, give up letting the panels fall so that we can reset global flags and be ready to start raising the panels
        stop_driving_panels("letting panels fall");
      } else {
        // Turn the solenoid on and off until the panels reach the lower limit or 8AM rolls around
        if (solenoid_power_supply_is_on && (millis() - solenoid_power_supply_on_off_time) > max_solenoid_on_time) {
          turn_off_solenoid_power_supply();
        } else if (solenoid_power_supply_is_on == false && (millis() - solenoid_power_supply_on_off_time) > max_solenoid_off_time) {
          turn_on_solenoid_power_supply();
        }
      }
    }
  }
}

/*
 * When motor is engaged, ensure that the panels are moving and that
 * the current it is drawing is in range.
 */
void monitor_stall_and_motor_current_callback()
{
  int amps = motor_amps();
  if (panels_going_up) {
    if (position_fresh && amps > 1 && position_sensor_val <= last_position_sensor_val_stall + from_counts(0.5)) {
      if (stall_start_time == 0) {
        stall_start_time = millis();
      } else if ((millis() - stall_start_time) > 500) {
        stop_driving_panels("motor stall going up");
        daily_stalls++;
        if (daily_stalls > 2) {
          calvals.position_upper_limit -= from_counts(5);
          out.println(F("# alert decreasing upper limit due to stall going up"));
        }
      }
    } else {
      stall_start_time = 0; // Panels are moving up, reset the time of last stall start
    }
  } else {
    fail("stall yet panels not in motion");
    return;
  }

  // Now check to see if the hydraulic motor is taking too little or too much current
  if (amps < (int)motor_current_threshold_low) {
    if (under_current_start_time == 0) {
      under_current_start_time = millis();
    } else if ((millis() - under_current_start_time) > 2000) {
      if (position_sensor_val < from_counts(150 - 50)) {
        stop_driving_panels("low hydraulic motor current at low panel angle");
        daily_stalls++;
      } else {
        stop_driving_panels("low hydaulic motor current, probably at upper stop");
        out.printf("# setting upper position limit to current position: %.2f\n", position_sensor_val);
        calvals.position_upper_limit = position_sensor_val;
      }
    }
  } else {
    under_current_start_time = 0; // Not taking too little, reset the start time of undercurrent

    // If the motor is taking too much current, stop it immediately
    if (amps > (int)motor_current_threshold_high) {
      stop_driving_panels("high hydraulic motor current");
      out.printf("# amps=%d\n", amps);
    }
  }
  if (!panels_going_up) {
    return;
  }
  float pv = pv_current();
  if (peak_pv_current < pv) {
    peak_pv_current = pv;

    // If we have been driving the panels for at least one second and the pv current is decreasing from
    // the peak, then stop driving the panels up
  } else if ((millis() - last_drive_panels_up_time) > 1000UL && pv < (peak_pv_current - 0.1)) {
    stop_driving_panels("decreasing panel current");
    out.printf("# pv amps=%.2f, peak=%.2f\n", pv, peak_pv_current);
  }
  last_position_sensor_val_stall = position_sensor_val;
}

void drive_panels_to_desired_position(void)
{
  static unsigned long low_pv_start_time = 0;
  static bool was_pv_low = true;  // start true so dawn detection fires on first sunrise

  float pv = pv_current();
  bool pv_is_low = (pv < 0.2);
  bool pv_is_high = pv > 3.0;
  static bool new_day = true;

  // Dawn: when sun rises after a dark period and panels are down, reset daily stall count
  if (was_pv_low && !pv_is_low && at_lower_position_limit) {
    daily_stalls = 0;
  }
  was_pv_low = pv_is_low;

  if (pv_is_low) {
    if (low_pv_start_time == 0) {
      low_pv_start_time = millis();
    } else if ((millis() - low_pv_start_time) > 45UL * 60 * 1000) {
      drive_panels_down("pv low 45 min");
      new_day = true;
      low_pv_start_time = 0;
    }
  } else {
    low_pv_start_time = 0;
    if (pv_is_high) {
      static unsigned long first_high_pv_time;
      if (new_day) {
        new_day = false;
        first_high_pv_time = millis();
      } else {
        // Ensure that it has been at least 30 minutes since the last time we attempted to raise the panels and that
        // it has been at least 60 minutes since the start of generating more than 3 amps of panel current in the new day.
        bool sufficient_delay = ((millis() - last_drive_panels_up_time) >  30UL * 60 * 1000) && ((millis() - first_high_pv_time) > 60UL * 60 * 1000);
        if (daily_stalls < 5 && sufficient_delay && !at_upper_position_limit) {
          drive_panels_up();
        }
      }
    }
  }
}

void control_hydraulics_callback()
{
  if (failed || !analog_ok() || !position_known()) {
    return;
  }
  if (!panels_going_up && !panels_going_down) {
    if (lower_panel_request) {
      drive_panels_down("switch request");
    } else {
      switch (calvals.operation_mode) {
        case no_panel_movement_mode:
          break;

        case position_mode:
          drive_panels_to_desired_position();
          break;

        default:
          fail("?mode");
          break;
      }
    }
  }
}

/*****************************************************************************************************/
// Commands, from the USB serial console or from trackctl (POST /cmd).  One per line:
//   mode off|track       off: report but never move by itself; track: the normal mode (saved in flash)
//   up                   one lift, with all the usual limits (5 s at most)
//   down                 let the panels down (solenoid), until the lower limit or 8 AM
//   stop                 both outputs off now
//   motor on|off         DO1 on for at most 5 s, for wiring checks (not while moving or with the solenoid on)
//   solenoid on|off      DO2 on for at most 60 s, for wiring checks (not while the panels are moving)
//   cal [down|up|clear]  tilt calibration: rest position, then the top stop (saved in flash)
//   telemetry            emit a telemetry line now
//   upload on|off        send telemetry to bedichek.org or not (saved in flash)
//   reboot               restart (both outputs go off at boot)

void save_pref_bool(const char *key, bool v)
{
  Preferences prefs;
  prefs.begin("tracker", false);
  prefs.putBool(key, v);
  prefs.end();
}

void run_command(const char *command)
{
  out.printf("# command: %s\n", command);
  force_telemetry++;

  if (strncmp(command, "mode ", 5) == 0) {
    for (int m = 0; m < last_mode; m++) {
      if (strcmp(command + 5, mode_names[m]) == 0) {
        calvals.operation_mode = (mode_e)m;
        save_pref_bool("track", m == position_mode);
        out.printf("# mode now %s\n", mode_names[m]);
        return;
      }
    }
    out.println(F("# modes are off, track"));
  } else if (strcmp(command, "up") == 0) {
    if (failed) {
      out.printf("# failed (%s); reboot first\n", fail_reason);
    } else if (!tcal.valid) {
      out.println(F("# not calibrated (cal down, cal up); not moving"));
    } else if (!position_known() || !analog_ok()) {
      out.println(F("# no inclinometer or current readings; not moving"));
    } else if (panels_going_up || panels_going_down) {
      out.println(F("# already moving; stop first"));
    } else if (at_upper_position_limit) {
      out.println(F("# at the upper limit"));
    } else {
      drive_panels_up();
    }
  } else if (strcmp(command, "down") == 0) {
    if (failed) {
      out.printf("# failed (%s); reboot first\n", fail_reason);
    } else if (!tcal.valid) {
      out.println(F("# not calibrated (cal down, cal up); not moving"));
    } else if (!position_known() || !analog_ok()) {
      out.println(F("# no inclinometer or current readings; not moving"));
    } else if (panels_going_up || panels_going_down) {
      out.println(F("# already moving; stop first"));
    } else if (at_lower_position_limit) {
      out.println(F("# at the lower limit"));
    } else {
      drive_panels_down("command");
    }
  } else if (strcmp(command, "cal") == 0 || strncmp(command, "cal ", 4) == 0) {
    calibrate(command[3] ? command + 4 : "");
  } else if (strcmp(command, "solenoid on") == 0) {
    if (panels_going_up || panels_going_down) {
      out.println(F("# panels moving; stop first"));
    } else {
      turn_on_solenoid_power_supply();
      solenoid_test_until = millis() + 60 * 1000UL;
      solenoid_test.enable();
      out.println(F("# solenoid (DO2) on for 60 s, test"));
    }
  } else if (strcmp(command, "motor on") == 0) {
    if (panels_going_up || panels_going_down || solenoid_power_supply_is_on) {
      out.println(F("# panels moving or solenoid on; stop first"));
    } else {
      turn_on_motor_power_supply();
      motor_test_until = millis() + 5 * 1000UL;
      motor_test.enable();
      out.println(F("# motor supply (DO1) on for 5 s, test"));
    }
  } else if (strcmp(command, "motor off") == 0) {
    motor_test.disable();
    turn_off_motor_power_supply();
    out.println(F("# motor supply (DO1) off"));
  } else if (strcmp(command, "solenoid off") == 0) {
    solenoid_test.disable();
    turn_off_solenoid_power_supply();
    out.println(F("# solenoid (DO2) off"));
  } else if (strcmp(command, "stop") == 0) {
    stop_driving_panels("command");
  } else if (strcmp(command, "telemetry") == 0) {
    // force_telemetry was incremented above
  } else if (strcmp(command, "upload on") == 0 || strcmp(command, "upload off") == 0) {
    upload_enabled = strcmp(command, "upload on") == 0;
    save_pref_bool("upload", upload_enabled);
    out.println(upload_enabled ? F("# uploads to bedichek.org on") : F("# uploads to bedichek.org off"));
  } else if (strcmp(command, "reboot") == 0) {
    stop_driving_panels("reboot");
    out.println(F("# rebooting"));
    delay(500);
    ESP.restart();
  } else {
    out.printf("# Unknown command: %s\n", command);
    out.println(F("# choices are: 'mode off|track', up, down, stop, 'motor on|off', 'solenoid on|off', 'cal [down|up|clear]', telemetry, 'upload on|off', reboot"));
  }
}

void monitor_serial_console_callback(void)
{
  static char command_buf[40];

  while (Serial.available() > 0) {
    char received_char = Serial.read();
    size_t l = strlen(command_buf);
    if (received_char == '\r') {
      continue;
    }
    if (l >= sizeof(command_buf) - 1) {
      out.println(F("# command buffer overflow"));
      command_buf[0] = '\0';
      continue;
    }
    if (received_char == '\n') {
      run_command(command_buf);
      command_buf[0] = '\0';
    } else {
      command_buf[l] = received_char;
      command_buf[l + 1] = '\0';
    }
  }
}

/*****************************************************************************************************/
// Output expander

static bool tca_write(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static int tca_read(uint8_t reg)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(TCA9554_ADDR, 1) != 1) {
    return -1;
  }
  return Wire.read();
}

// Unlike the pool controller, the outputs are never kept through a reset: a motor left on with no monitoring
// would run to the stops.
bool Onboard_Outputs::begin(void)
{
  bits = 0;
  return tca_write(1, 0xFF) && tca_write(3, 0x00);   // Latch all off first, then make the pins outputs
}

void Onboard_Outputs::set(uint8_t n, bool on)
{
  uint8_t b = on ? (bits | (1 << (n - 1))) : (bits & ~(1 << (n - 1)));
  if (tca_write(1, ~b & 0xFF)) {
    bits = b;
  } else {
    out.printf("# alert output expander did not answer setting DO%d\n", n);
  }
}

// Reads the latch back and rewrites it if it differs (e.g. the expander lost power).
bool Onboard_Outputs::verify(void)
{
  int latch = tca_read(1);
  int direction = tca_read(3);
  if (latch < 0 || direction < 0) {
    return false;
  }
  if (latch != (~bits & 0xFF) || direction != 0x00) {
    out.printf("# alert output latch was 0x%02X, rewriting 0x%02X\n", latch, ~bits & 0xFF);
    tca_write(1, ~bits & 0xFF);
    tca_write(3, 0x00);
  }
  return true;
}

void check_outputs_callback(void)
{
  static int failures;
  if (outputs == nullptr) {
    outputs = new Onboard_Outputs();
    if (!outputs->begin()) {
      delete outputs;
      outputs = nullptr;
      if (failures++ % 60 == 0) {
        out.println(F("# alert output expander not found"));
      }
    }
    return;
  }
  if (!outputs->verify()) {
    if (failures++ % 60 == 0) {
      out.println(F("# alert output expander did not answer"));
    }
  } else {
    failures = 0;
  }
}

/*****************************************************************************************************/
// Clock.  The RTC holds UTC.  At boot the system clock is set from it; NTP then keeps the system clock
// and once an hour after each NTP sync we write the RTC.

static uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtc_read(time_t *t)
{
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(PCF85063_ADDR, 7) != 7) {
    return false;
  }
  uint8_t r[7];
  for (int i = 0; i < 7; i++) {
    r[i] = Wire.read();
  }
  if (r[0] & 0x80) {          // Oscillator stopped: the time is not valid
    return false;
  }
  struct tm tm = {};
  tm.tm_sec = bcd2bin(r[0] & 0x7F);
  tm.tm_min = bcd2bin(r[1] & 0x7F);
  tm.tm_hour = bcd2bin(r[2] & 0x3F);
  tm.tm_mday = bcd2bin(r[3] & 0x3F);
  tm.tm_mon = bcd2bin(r[5] & 0x1F) - 1;
  tm.tm_year = bcd2bin(r[6]) + 100;
  if (tm.tm_year < 125 || tm.tm_year > 140) {   // Before 2025: never set; after 2040: garbage
    return false;
  }
  setenv("TZ", "UTC0", 1);
  tzset();
  *t = mktime(&tm);
  setenv("TZ", TIMEZONE, 1);
  tzset();
  return true;
}

bool rtc_write(time_t t)
{
  struct tm tm;
  gmtime_r(&t, &tm);
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  Wire.write(bin2bcd(tm.tm_sec));       // Also clears the oscillator-stopped flag
  Wire.write(bin2bcd(tm.tm_min));
  Wire.write(bin2bcd(tm.tm_hour));
  Wire.write(bin2bcd(tm.tm_mday));
  Wire.write(tm.tm_wday);
  Wire.write(bin2bcd(tm.tm_mon + 1));
  Wire.write(bin2bcd(tm.tm_year - 100));
  return Wire.endTransmission() == 0;
}

volatile bool ntp_synced = false;

void time_sync_notification(struct timeval *tv)
{
  ntp_synced = true;
}

void sync_rtc_callback(void)
{
  static unsigned long last_rtc_write;
  if (ntp_synced) {
    ntp_synced = false;
    if (!time_valid) {
      time_valid = true;
      out.println(F("# clock set from NTP"));
    }
    if (last_rtc_write == 0 || millis() - last_rtc_write > 3600 * 1000UL) {
      if (rtc_write(time(nullptr))) {
        last_rtc_write = millis();
      } else {
        out.println(F("# alert could not write the RTC"));
      }
    }
  }
}

void setup_time(void)
{
  setenv("TZ", TIMEZONE, 1);
  tzset();
  time_t t;
  if (rtc_read(&t)) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
    time_valid = true;
  } else {
    out.println(F("# alert RTC has no valid time; waiting for NTP"));
  }
  sntp_set_time_sync_notification_cb(time_sync_notification);
  configTzTime(TIMEZONE, "pool.ntp.org", "time.apple.com");
}

/*****************************************************************************************************/
// Network: Ethernet (W5500, powered over PoE), static 192.168.1.8 (solartracker in /etc/hosts), mDNS
// solartracker.local, a small HTTP server for trackctl, and an upload task for bedichek.org.

#define HOSTNAME "solartracker"
#define STATIC_IP IPAddress(192, 168, 1, 8)
#define GATEWAY_IP IPAddress(192, 168, 1, 1)      // The AT&T gateway, also the DNS server
#define SUBNET_MASK IPAddress(255, 255, 255, 0)
#define DNS2_IP IPAddress(1, 1, 1, 1)
#define INGEST_URL "https://bedichek.org/home/api/ingest/solartracker"

volatile bool eth_connected = false;
bool status_served = false;              // Someone fetched /status since boot: the network works

void network_event(arduino_event_id_t event, arduino_event_info_t info)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:   // The address is static, so link up means ready
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      eth_connected = false;
      break;
    default:
      break;
  }
}

WebServer server(80);

static bool authorized(void)
{
  if (server.header("X-Token") == TRACKER_LAN_TOKEN) {
    return true;
  }
  server.send(403, "text/plain", "forbidden\n");
  return false;
}

static String f(float v, int places) { return String(v, places); }

void handle_status(void)
{
  status_served = true;
  time_t t = now();
  struct tm tm = tm_of(t);
  char when[32];
  strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S %Z", &tm);
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(running, &state);

  uint16_t mv[ANALOG_CHANNELS_READ] = {};
  uint32_t incl_reads;
  int16_t regs[INCL_REGS];
  portENTER_CRITICAL(&bus_lock);
  if (bus.count > 0) {
    for (int c = 0; c < ANALOG_CHANNELS_READ; c++) {
      mv[c] = bus.ring[c][(bus.count - 1) % RING];
    }
  }
  incl_reads = bus.incl_reads;
  memcpy(regs, bus.incl_regs, sizeof(regs));
  portEXIT_CRITICAL(&bus_lock);
  Incl incl = inclinometer();

  String s;
  s.reserve(1600);
  s += "{\n";
  s += "  \"board\": \"8DO\", \"name\": \"" HOSTNAME "\",\n";
  s += "  \"time\": \"" + String(when) + "\", \"time_valid\": " + String(time_valid ? "true" : "false") + ",\n";
  s += "  \"mode\": \"" + String(mode_names[calvals.operation_mode]) + "\", \"failed\": " + String(failed ? "true" : "false") +
       ", \"fail_reason\": \"" + String(fail_reason) + "\",\n";
  s += "  \"calibrated\": " + String(tcal.valid ? "true" : "false") + ", \"cal_axis\": \"" + String(axis_names[tcal.axis]) +
       "\", \"cal_rest_deg\": " + f(tcal.down, 2) + ", \"cal_travel_deg\": " + f(tcal.travel, 2) + ",\n";
  s += "  \"position_deg\": " + f(position_sensor_val, 2) + ", \"lower_limit\": " + f(calvals.position_lower_limit, 2) +
       ", \"upper_limit\": " + f(calvals.position_upper_limit, 2) + ", \"at_lower\": " + String(at_lower_position_limit) +
       ", \"at_upper\": " + String(at_upper_position_limit) + ",\n";
  s += "  \"going_up\": " + String(panels_going_up) + ", \"going_down\": " + String(panels_going_down) +
       ", \"motor\": " + String(motor_power_supply_is_on) + ", \"solenoid\": " + String(solenoid_power_supply_is_on) +
       ", \"outputs\": \"0x" + String(outputs ? outputs->all() : 0, HEX) + "\",\n";
  s += "  \"lower_request\": " + String(lower_panel_request) + ", \"daily_stalls\": " + String(daily_stalls) + ",\n";
  s += "  \"motor_amps\": " + String(motor_amps()) + ", \"pv_amps\": " + f(pv_current(), 2) +
       ", \"pv_min_amps\": " + f(minimum_pv_current_measured, 2) + ",\n";
  s += "  \"analog_module\": " + String(analog_ok() ? "true" : "false") + ", \"analog_mv\": [" + String(mv[0]) + ", " +
       String(mv[1]) + "],\n";
  s += "  \"inclinometer\": " + String(incl.ok ? "true" : "false") + ", \"incl_reads\": " + String(incl_reads) +
       ", \"roll\": " + f(incl.roll, 3) + ", \"pitch\": " + f(incl.pitch, 3) + ", \"yaw\": " + f(incl.yaw, 3) + ", \"incl_temp_c\": " + f(incl.temp_c, 2) + ",\n";
  s += "  \"acc_g\": [" + f(incl.acc_g[0], 3) + ", " + f(incl.acc_g[1], 3) + ", " + f(incl.acc_g[2], 3) + "], \"gyro_dps\": [" +
       f(incl.gyro_dps[0], 2) + ", " + f(incl.gyro_dps[1], 2) + ", " + f(incl.gyro_dps[2], 2) + "], \"mag\": [" +
       String(incl.mag[0]) + ", " + String(incl.mag[1]) + ", " + String(incl.mag[2]) + "],\n";
  s += "  \"incl_results\": {\"ok\": " + String(incl_errors[0]) + ", \"no_reply\": " + String(incl_errors[1]) +
       ", \"short\": " + String(incl_errors[2]) + ", \"wrong_header\": " + String(incl_errors[3]) +
       ", \"bad_crc\": " + String(incl_errors[4]) + "}, \"incl_last_bad\": \"";
  for (size_t i = 0; i < incl_last_bad_len; i++) {
    char h[3];
    snprintf(h, sizeof(h), "%02x", incl_last_bad[i]);
    s += h;
  }
  s += "\",\n";
  s += "  \"incl_regs\": [";
  for (int i = 0; i < INCL_REGS; i++) {
    s += String(regs[i]) + (i < INCL_REGS - 1 ? ", " : "");
  }
  s += "],\n";
  s += "  \"relay_board\": " + String(outputs != nullptr ? "true" : "false") + ",\n";
  s += "  \"upload\": " + String(upload_enabled ? "true" : "false") + ", \"upload_queue_bytes\": " + String(upload_queue.length()) + ",\n";
  s += "  \"uptime_s\": " + String(millis() / 1000) + ", \"reset_cause\": \"" + String(reset_cause) + "\",\n";
  s += "  \"firmware\": \"" FIRMWARE_VERSION "\", \"partition\": \"" + String(running->label) +
       "\", \"firmware_verified\": " + String(state == ESP_OTA_IMG_PENDING_VERIFY ? "false" : "true") + ",\n";
  s += "  \"free_heap\": " + String(ESP.getFreeHeap()) + ", \"lowest_heap\": " + String(lowest_memory) + "\n";
  s += "}\n";
  server.send(200, "application/json", s);
}

// The page at http://solartracker/: everything in /status, refreshed every second.
const char status_page[] PROGMEM = R"PAGE(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Solar Tracker</title>
<style>
:root { --bg:#fbfbf9; --fg:#1d1d1b; --muted:#6b6b66; --line:#e3e2dc; --card:#ffffff; --ok:#1f7a3d; --bad:#b3261e; }
@media (prefers-color-scheme: dark) { :root { --bg:#171716; --fg:#ecebe6; --muted:#9a9993; --line:#33332f; --card:#1f1f1d; --ok:#5cc983; --bad:#ff8a80; } }
body { margin:0; padding:16px; background:var(--bg); color:var(--fg); font:15px/1.4 -apple-system, system-ui, sans-serif; }
h1 { font-size:20px; margin:0 0 4px; } #when { color:var(--muted); font-size:13px; margin-bottom:16px; }
.grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(300px, 1fr)); gap:12px; }
section { background:var(--card); border:1px solid var(--line); border-radius:8px; padding:12px 14px; }
h2 { font-size:13px; text-transform:uppercase; letter-spacing:.05em; color:var(--muted); margin:0 0 8px; }
table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
td { padding:3px 0; border-bottom:1px solid var(--line); } tr:last-child td { border-bottom:0; }
td:last-child { text-align:right; font-family:ui-monospace, Menlo, monospace; }
th { text-align:right; font-weight:500; color:var(--muted); font-size:12px; } th:first-child { text-align:left; }
#axes td { text-align:right; font-family:ui-monospace, Menlo, monospace; } #axes td:first-child { text-align:left; font-family:inherit; }
.ok { color:var(--ok); } .bad { color:var(--bad); }
.big { font-size:28px; font-family:ui-monospace, Menlo, monospace; }
</style></head><body>
<h1>Solar Tracker</h1><div id="when">loading...</div>
<div class="grid">
<section><h2>Panels</h2><div class="big" id="pos">-</div><table id="panels"></table></section>
<section><h2>Inclinometer</h2><table id="incl"></table></section>
<section><h2>Inclinometer axes</h2><table id="axes"></table></section>
<section><h2>Currents and inputs</h2><table id="io"></table></section>
<section><h2>Controller</h2><table id="ctl"></table></section>
</div>
<script>
const $ = id => document.getElementById(id);
const row = (k, v, cls) => `<tr><td>${k}</td><td${cls ? ` class="${cls}"` : ""}>${v}</td></tr>`;
const yes = (b, good = true) => `<span class="${!!b === good ? "ok" : "bad"}">${b ? "yes" : "no"}</span>`;
const n = (v, d) => (typeof v === "number" ? v.toFixed(d) : v);
async function tick() {
  try {
    const s = await (await fetch("/status", {cache: "no-store"})).json();
    $("when").textContent = `${s.time}  ·  mode ${s.mode}${s.failed ? "  ·  FAILED: " + s.fail_reason : ""}`;
    $("pos").textContent = s.calibrated ? `${n(s.position_deg, 2)}°` : "not calibrated";
    $("panels").innerHTML =
      row("Calibrated", yes(s.calibrated)) +
      row("Tilt axis", s.cal_axis) + row("Rest angle", n(s.cal_rest_deg, 2) + "°") + row("Travel", n(s.cal_travel_deg, 2) + "°") +
      row("Limits", `${n(s.lower_limit, 2)}° to ${n(s.upper_limit, 2)}°`) +
      row("At lower / upper", `${s.at_lower ? "yes" : "no"} / ${s.at_upper ? "yes" : "no"}`) +
      row("Going up / down", `${s.going_up ? "yes" : "no"} / ${s.going_down ? "yes" : "no"}`) +
      row("Motor supply (DO1)", s.motor ? "ON" : "off", s.motor ? "bad" : "") +
      row("Solenoid supply (DO2)", s.solenoid ? "ON" : "off", s.solenoid ? "bad" : "") +
      row("Daily stalls", s.daily_stalls);
    $("incl").innerHTML =
      row("Answering", yes(s.inclinometer)) + row("Roll", n(s.roll, 3) + "°") + row("Pitch", n(s.pitch, 3) + "°") +
      row("Yaw", n(s.yaw, 3) + "°") + row("Temperature", n(s.incl_temp_c, 2) + " °C") + row("Reads", s.incl_reads) +
      row("Results", Object.entries(s.incl_results).map(([k, v]) => `${k} ${v}`).join(", "));
    const ax = ["X", "Y", "Z"];
    $("axes").innerHTML = `<tr><th>Axis</th><th>Accel (g)</th><th>Rate (°/s)</th><th>Mag</th></tr>` +
      ax.map((a, i) => `<tr><td>${a}</td><td>${n(s.acc_g[i], 3)}</td><td>${n(s.gyro_dps[i], 2)}</td><td>${s.mag[i]}</td></tr>`).join("") +
      `<tr><td>|g|</td><td>${n(Math.hypot(...s.acc_g), 3)}</td><td></td><td></td></tr>`;
    $("io").innerHTML =
      row("Analog module", yes(s.analog_module)) + row("Motor current (CH1)", `${s.motor_amps} A  (${s.analog_mv[0]} mV)`) +
      row("PV current (CH2)", `${n(s.pv_amps, 2)} A  (${s.analog_mv[1]} mV)`) + row("PV minimum seen", n(s.pv_min_amps, 2) + " A") +
      row("Lower-panels switch (DI1)", s.lower_request ? "closed" : "open");
    $("ctl").innerHTML =
      row("Output expander", yes(s.relay_board)) + row("Outputs", s.outputs) + row("Clock set", yes(s.time_valid)) +
      row("Uploads", s.upload ? "on" : "off") + row("Uptime", Math.floor(s.uptime_s / 3600) + " h " + Math.floor(s.uptime_s % 3600 / 60) + " min") +
      row("Reset cause", s.reset_cause) + row("Firmware", s.firmware + (s.firmware_verified ? "" : " (unverified)")) +
      row("Free heap", s.free_heap);
  } catch (e) {
    $("when").textContent = "no answer from the controller";
  }
}
tick(); setInterval(tick, 1000);
</script></body></html>
)PAGE";

void handle_page(void)
{
  server.send_P(200, "text/html", status_page);
}

void handle_log(void)
{
  size_t max = server.hasArg("bytes") ? server.arg("bytes").toInt() : 8000;
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, max);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

void handle_cmd(void)
{
  if (!authorized()) {
    return;
  }
  String c = server.arg("plain");
  c.trim();
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  size_t before = recent_lines.length();
  xSemaphoreGive(lines_mutex);
  run_command(c.c_str());
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, recent_lines.length() > before ? recent_lines.length() - before : 0);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

// Firmware update: trackctl posts the .bin as multipart form data.  The panels are stopped first.

bool update_authorized;

void handle_update_done(void)
{
  if (!update_authorized) {
    server.send(403, "text/plain", "forbidden\n");
    return;
  }
  if (Update.hasError()) {
    server.send(500, "text/plain", String("update failed: ") + Update.errorString() + "\n");
    out.printf("# alert firmware update failed: %s\n", Update.errorString());
    return;
  }
  server.send(200, "text/plain", "update written, restarting\n");
  out.println(F("# firmware update written, restarting"));
  delay(500);
  ESP.restart();
}

void handle_update_upload(void)
{
  HTTPUpload &upload = server.upload();
  esp_task_wdt_reset();
  if (upload.status == UPLOAD_FILE_START) {
    update_authorized = server.header("X-Token") == TRACKER_LAN_TOKEN;
    if (update_authorized) {
      stop_driving_panels("firmware update");
      out.printf("# firmware update starting: %s\n", upload.filename.c_str());
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
  } else if (!update_authorized) {
    return;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void setup_network(void)
{
  Network.onEvent(network_event);
  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, -1, SPI);
  ETH.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, GATEWAY_IP, DNS2_IP);

  const char *headers[] = {"X-Token"};
  server.collectHeaders(headers, 1);
  server.on("/status", HTTP_GET, handle_status);
  server.on("/", HTTP_GET, handle_page);
  server.on("/log", HTTP_GET, handle_log);
  server.on("/cmd", HTTP_POST, handle_cmd);
  server.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
  server.begin();
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

// Runs on its own FreeRTOS task so that a slow upload never delays the control tasks.  Lines are dropped from
// the queue only when bedichek.org accepts them.

void upload_task(void *arg)
{
  static char chunk[32768];
  unsigned long backoff_ms = 10000;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    if (!eth_connected) {
      continue;
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    size_t n = upload_queue.peek(chunk, sizeof(chunk));
    xSemaphoreGive(lines_mutex);
    if (n == 0) {
      continue;
    }
    NetworkClientSecure client;
    client.useBuiltinCACertBundle();
    HTTPClient http;
    http.setTimeout(15000);
    int code = -1;
    if (http.begin(client, INGEST_URL)) {
      http.addHeader("Authorization", "Bearer " TRACKER_INGEST_TOKEN);
      http.addHeader("Content-Type", "text/plain");
      code = http.POST((uint8_t *)chunk, n);
      http.end();
    }
    if (code == 200) {
      xSemaphoreTake(lines_mutex, portMAX_DELAY);
      upload_queue.consume(n);
      xSemaphoreGive(lines_mutex);
      backoff_ms = 10000;
    } else {
      Serial.printf("# upload to bedichek.org failed: %d\n", code);
      backoff_ms = backoff_ms < 300000 ? backoff_ms * 2 : 300000;
    }
  }
}

/*****************************************************************************************************/
// Firmware rollback.  Returning true here tells the core not to mark a new image good at boot; we do it after a
// minute of running with the output expander answering and /status fetched over the network.

extern "C" bool verifyRollbackLater()   // C linkage, or the core keeps its own version, which marks every image good
{
  return true;
}

void motor_test_callback(void)
{
  if ((long)(millis() - motor_test_until) >= 0) {
    motor_test.disable();
    if (!panels_going_up) {
      turn_off_motor_power_supply();
      out.println(F("# motor test over, DO1 off"));
    }
  }
}

void solenoid_test_callback(void)
{
  if ((long)(millis() - solenoid_test_until) >= 0) {
    solenoid_test.disable();
    if (!panels_going_down) {
      turn_off_solenoid_power_supply();
      out.println(F("# solenoid test over, DO2 off"));
    }
  }
}

void mark_firmware_good_callback(void)
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    if (outputs != nullptr && status_served) {
      esp_ota_mark_app_valid_cancel_rollback();
      out.println(F("# new firmware marked good"));
    } else {
      stop_driving_panels("rollback");
      out.println(outputs == nullptr ? F("# alert new firmware cannot reach the output expander; going back to the previous firmware")
                                     : F("# alert new firmware was not reached over the network; going back to the previous firmware"));
      delay(500);
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }
}

/*****************************************************************************************************/

void setup_reset_cause(void)
{
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  reset_cause = 'P'; break;
    case ESP_RST_BROWNOUT: reset_cause = 'B'; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      reset_cause = 'W'; break;
    case ESP_RST_SW:       reset_cause = 'S'; break;
    case ESP_RST_PANIC:    reset_cause = 'C'; break;
    default:               reset_cause = 'X'; break;
  }
  if (reset_cause == 'W' || reset_cause == 'C' || reset_cause == 'B') {
    Preferences prefs;
    prefs.begin("tracker", false);
    uint32_t count = prefs.getUInt("bad_resets", 0) + 1;
    prefs.putUInt("bad_resets", count);
    prefs.end();
    out.printf("# alert Reset cause: %c, total count=%u\n", reset_cause, (unsigned)count);
  }
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  lines_mutex = xSemaphoreCreateMutex();
  recent_lines.init(64 * 1024, true);
  upload_queue.init(1024 * 1024, true);

  // Outputs off before anything else
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(25);
  check_outputs_callback();     // This sets "outputs" and turns them all off

  set_calvals_to_defaults();
  Preferences prefs;
  prefs.begin("tracker", true);
  upload_enabled = prefs.getBool("upload", false);
  calvals.operation_mode = prefs.getBool("track", true) ? position_mode : no_panel_movement_mode;
  prefs.end();

  setup_reset_cause();
  for (int pin = DI1_PIN; pin <= DI1_PIN + 7; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }
  lower_panel_request = digitalRead(LOWER_PANEL_REQUEST_INPUT_PIN) == LOW;

  RS485.begin(BUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  RS485.setPins(-1, -1, -1, RS485_DE_PIN);
  RS485.setMode(UART_MODE_RS485_HALF_DUPLEX);
  xTaskCreatePinnedToCore(bus_task, "rs485", 4096, nullptr, 2, nullptr, 0);

  setup_time();

  out.printf("# alert reboot SolarTracker (ESP32) " FIRMWARE_VERSION " reset cause %c, mode %s\n",
             reset_cause, mode_names[calvals.operation_mode]);

  turn_off_motor_power_supply();
  turn_off_solenoid_power_supply();

  setup_network();
  xTaskCreatePinnedToCore(upload_task, "upload", 8192, nullptr, 1, nullptr, 0);

  mark_firmware_good.enableDelayed(TASK_SECOND * 60);

  esp_task_wdt_config_t wdt_config = {.timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(nullptr);
  out.println(F("# Init done"));
}

void loop(void)
{
  esp_task_wdt_reset();  // Pet the watchdog
  ts.execute();
  server.handleClient();
}
