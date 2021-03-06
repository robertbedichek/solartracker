
/*
   This controls three relays that control a hydraulic motor, a hydraulic valve, and a power supply.  The goal is
   to get more power out of four LG Neon-2 420 Watt panels than if they were fixed-mounted.  A secondary goal is to
   have them better looking.  If they were fixed-mounted, they'd need to be reversed-racked, which is not a great look.

    The core of the controls system is an Arduino:
   KTA-223v3 by Ocean Controls www.oceancontrols.com.au

   Creative Commons Licence
   Robert Bedichek
*/

#include <string.h> //Use the string Library
#include <ctype.h>
#include <EEPROM.h>
#include <assert.h>

#ifndef RS485
#define enable_rs485_output()
#define disable_rs485_output()
#define txenabled (0)
#endif

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <TimeLib.h>     // for update/display of time

/*
    This is for the LCD and button interface we added for local control and display.
*/
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

/*
   This is for the Sparkfun 4-relay board
*/
#include "SparkFun_Qwiic_Relay.h"
#define RELAY_ADDR (0x6D)

Qwiic_Relay quad_relay(RELAY_ADDR);

/*
   We needed more analog inputs than the KTA-223v3 had, so we added an ADS1015, a 4-input, 12bit
   A/D
*/
#include <Adafruit_ADS1X15.h>
//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */


/*
    This is for the 1307 RTC we installed on the Ocean Controls main board.
*/
#include <DS1307RTC.h>


/*
   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
*/
time_t arduino_time;

/*
   If we are able to read the DS1307 RTC over I2C, then we set this true and we can depend on
   the time of day being valid.
*/
bool time_of_day_valid = false;


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define BACKLIGHT_OFF   (0)
#define BACKLIGHT_WHITE (7)

/*
   Set this to set the time to the build time.  This is currently the only way to set time (let this be defined, compile, and
   run right away so that the compile-time time is pretty close to the actual time).
*/
// #define SETTIME

/*
   Speed at which we run the serial connection (both via USB and RS-485)
*/
#define SERIAL_BAUD (9600)

/*
   Default number of seconds the backlight will stay on.
*/
#define DEFAULT_BACKLIGHT_ON_TIME (60 * 10)

/*
    This controls how much output we will emit on the RS-485 line.
      0: only events are reported
      1: status messages are emitted every three seconds
      2: all messages are enabled
*/
#define VERBOSE_NONE   (0)
#define VERBOSE_MEDIUM (1)
#define VERBOSE_ALL    (2)

int verbose = VERBOSE_MEDIUM;

//void fail(string);

/*
   Number of milliseconds to wait after boot and build display before starting operational part.
*/
#define INITIAL_DELAY (2000)

/*
   Default maximum time the motor can be on continuously.  This is just the default value for when
   the user resets to "factory defaults".  Units are milliseconds.
*/
#define MOTOR_MAX_ON_TIME_DEFAULT (90000)

/*
   On-time hysteresis in milliseconds.  Once the motor has been running too long, past the max-motor-on-time limit,
   we turn off until the on-time falls below the limit minus this vale.  This avoids the system turning on and off
   rapidly when the limit is reached.
*/
#define ON_TIME_HYSTERSIS (20000)


/*
   We can operate in one of three modes.  The first is a kind of "off" mode where we stay alive, keep the battery charged, talk on the
   serial port, update the LCD, but do not move the panels.   The two "on" modes are Sun-sensor mode and time-of-day/day-of-year mode.
*/
enum mode_e { no_panel_movement_mode, time_mode, sensor_mode, last_mode};


/*
   Allocation of ATMEGA328P Analog lines -- all on "Port C"
    PC0 (KTA-223v3 Analog 3)      - Draw-string (position) sensor
    PC1 (KTA-223v3 opto-input A1) - Supply voltage (removed R16 pull-up resistor, wired directly to main board K2.2)
    PC2 (KTA-223v3 opto-input A2) - Battery temperature (removed R17 pull-up resistor, wired TMP36 output directly to main board K2.3)
    PC3 (KTA-223v3 opto-input A3) - Battery current (removed R18 pull resistor, wired Atopilot I output directly to main board K2.4)
    PC4 (KTA-223v3 opto-input A4) - I2C SDA (data) line
    PC5 (KTA-TXEN)                - I2C SCL (clock) line (required removing D7 to disconnect this from driving RS-485 transmit-enable)
    PC6 (KTA-223v3 Analog 1)      - Sun sensor lower cell
    PC7 (KTA-223v3 Analog 2)      - Sun sensor upper cell
*/
/*
    The inputs and outputs are thus:
    KTA Analog 1, which is Arduino Analog In 6, measures the voltage on the lower cell in the sun angle senseor
    KTA Analog 2, which is Arudino Analog In 7, measures the voltage on the upper cell in the sun angle sensor
    KTA Analog 3, which is Arudino Analog In 0, measures the voltage from the draw-string position sensor
*/

#define ANIN1 6   // Analog 1 is connected to Arduino Analog In 6
#define ANIN2 7   // Analog 2 is connected to Arduino Analog In 7
#define ANIN3 0   // Analog 3 is connected to Arduino Analog In 0

/*
     These are the only analog inputs that the KTA-223 has on the outside of its shell.  However, I needed more,
    so I have repurposed the opto-isolator inputs by removing the pull-up resistors that connect the outputs
    of the KTA-223 opto-isolators to the Arudino inputs.  I have wired directly to the Arudino board thusly:

*/

/*
    To monitor the supply voltage I used Arduino Analog 1. We have a resistor divider with a 68.284k Ohm resistor
    in series with a 32.823k Ohm resistor. The the center tap is connected to A1 (analog input 1), the other
   leads are connected to the supply voltage and the ground.  This input had been driven by an opto isolator with a
   4.7 k Ohm pull up.  We removed the pull up.
*/
#define ANIN4 1

/*
     We needed another analog input to monitor battery temperature, so we used Arduino Analog 2.
   It is connected to a TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
*/
#define ANIN5 2

// And for the output of a current sensor on the 4V line we use Arduino analog 3
#define ANIN6 3

/*
    We use three relay outputs.  The first passes 12VDC when active to the go-up input of the contactor
   that controls the power to the hydraulic motor.  The second passes 12VDC when active to the go-down
   input of the contactor and to a solenoid in the check valve.  This check valve normally only allows
   hydraulic pressure to pass to the cycliner to make it go up.  When the hydraulic motor stops, we want
   the panels to retain their position.  Before we had this check valve, they would gradually drift back to
    the down position.  When we want the panels to lower, at the end of the day, we have to both release the
   check vavle by activating its solenoid and by power the hydralic motor in reverse, which is accomplished
   by driving 12VDC to the "go-down" input of the contactor. The third relay we use is relay 8, which enables
   120VAC or 220VAC to pass to a "wall-wart" style power supply that generates 12.3VDC @ 2A.  When our battery
   voltage falls below 12.1VDC, we turn it on.  When it rises above 12.2VDC, we turn it off.
*/

// REL1 drives the go-up side of the contactor
#define REL1 2  // Relay 1 is connected to Arduino Digital 2

// REL2 drives the go-down side of the contactor
#define REL2 3  // Relay 2 is connected to Arduino Digital 3 PWM


// REL3 through REL6 are unused, but are available, in this design
#define REL3 4  // Relay 3 is connected to Arduino Digital 4
#define REL4 5  // Relay 4 is connected to Arduino Digital 5 PWM
#define REL5 6  // Relay 5 is connected to Arduino Digital 6 PWM
#define REL6 7  // Relay 6 is connected to Arduino Digital 7

// Power supply input control
#define REL7 8  // Relay 7 is connected to Arduino Digital 8


void setup_time(void);

const char *operation_modestring(void);

/*
    Central (and only) task scheduler data structure.
*/
Scheduler ts;


// REL8 controls the AC input of the power supply that charges the battery
#define REL8 9  // Relay 8 is connected to Arduino Digital 9 PWM


/*
   We moved the transmit-enable to PB5/SCK as we are not using the SD interface, but we are using the built-in I2C interface.
   We did this by removing D7 and replacing it with a new diode whose anode is on the SCK through-hole and who's cathode
   is on the pad for D7's cathode.  The SPI interface must be off, (SPCR is 0), or else the SPI clock will be driven on this pin.
*/
#define TXEN 13

// #define TXEN 19 // RS-485 Transmit Enable is connected to Arduino Analog 5 which is Digital 19


/*
   We allocated one more byte than the length of these compiler-defined strings, to make space for the null-terminator.
*/
char build_date[12] = __DATE__;
char build_time[9] = __TIME__;

/*
   These are values that may need to be tweaked.  They should be stored in EEPROM and changable through the RS-485
   interface and possibly by a future keypad or other direct, at the device, interface.
*/
struct calvals_s {
  unsigned position_upper_limit;                         // Max position value (i.e., fully tilted up)
  unsigned position_lower_limit;
  unsigned time_tilted_up_limit_in_minutes;                  // In minutes, Lower the panels after this many minutes, maximum.
  unsigned darkness_threshold;

  int battery_charging_temperature_limit;          // Maximum temperature to be able to start charging
  unsigned supply_voltage_lower_limit;             // Below 11 volts, don't try to move panels other than lowering them.
  unsigned supply_voltage_charge_limit_low;        // Turn on charger if voltage drops below this value
  unsigned supply_voltage_charge_limit_high;       // Turn off charge when voltage goes over this value
  unsigned supply_voltage_bms_open_threshold;      // If we see a voltage this low, we assume the BMS has opened due to potential cell overcharge
  unsigned motor_amps_limit;                         // If we see a current above this many amps, we will go into no-movement mode.

  unsigned wind_speed_limit;                       // Above this speed, we will lower the panels
  unsigned long accumulated_motor_on_time_limit;        // This is on-time in milliseconds, aged at 1/20th time
  unsigned accumulated_motor_on_time_aging_rate;   // Sets ratio of maximum on time to off time (1:<this variable>)
  unsigned backlight_on_time;                           // Number of seconds the LCD backlight stays on
  enum mode_e operation_mode;                             // Mode to start in
  int subversion;                                       // Changing this forces an calvals EEPROM reload
} calvals;

/*
   The following are global variables that are refreshed every 200 msec and availble for all tasks to
   make decisions with.
*/
unsigned long time_now;
unsigned long seconds;
unsigned long milliseconds;

/*
   Currently this is just used by the status tracing function.  It is true when we are calling for charging power, false when we are not.
*/
bool power_supply_on = false;

/*
   These come from Arudino Analog input 1, which is driven by the center tap of two resistors that connect to the 12V line and
   ground.  We need this resistor divider network to move the 12V line to the range of the ADC, which is 0..5V.
*/
unsigned supply_tap_voltage_raw;  // Raw ADC value: 0..1023 for 0..5V
unsigned supply_tap_voltage;      // Raw value converted to millivolts (of the tap)
unsigned supply_voltage;          // Tap voltage converted to supply voltage

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is in the middle of the pack.
*/
unsigned battery_temperature_volts_raw;     // raw ADC value: 0..1023 for 0..5V
unsigned battery_temperature_millivolts;    // raw value converted to millivolts
int battery_temperature_C_x10;         // millivolts converted to degrees Celsiuis
int battery_temperature_F;             // Degrees C converter to Farenheight

/*
   We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
   is unused.  Its current output goes to Arduino Analog input 3.
*/
unsigned current_sense_4V_volts_raw;    // What is read from input 3, 0..1023 for 0..5V
unsigned current_sense_4V_millivolts;   // Raw ADC value converted to millivolts
unsigned current_sense_4V_amps;    // Millivolts from the Attopilot device converter to milliamps it is sensing
unsigned max_4V_current_amps = 0;  // Max recorded current since we started

/*
   These two are from the sun angle sensor and are the raw values from the two tiny solar cells in that sensor
*/
unsigned lower_solar_val;  // Raw ADC values 0..1023 for 0..5V
unsigned upper_solar_val;  // Raw ADC values 0..1023 for 0..5V

/*
   This is from the 1000mm pull-string sensor that tells us where the panels are.
*/
unsigned position_sensor_val;// Raw ADC values 0..1023 for 0..5V
bool position_sensor_failed = false;  // Set true if the position sensor value is ever out of range

/*
   Record the time today when we first raised the panels.  This is used to decide, later in the day, that it must be
   time to lower them (in case for some reason the sun angle sensor doesn't tell us it is night time).
*/
unsigned long time_of_first_raise = 0ULL;

/*
   These are calculated from the values read from sun angle sensor.  'sun_high' means the sun is in a position that puts
   its rays at an angle lower than 90 degrees and naturally happens as the earth rotates during the day.  To remedy this,
   we move the panels up.  'sun_low' means the opposite and naturally happens when we rotate the panels past the point
   where the sun is at a 90 degree angle to the panels.  That's our trigger to stop moving the panels.  'dark' is when
   both sensors indicate the light level is so low that it must be nighttime.  That is our trigger to lower the panels to
   their nighttime, resting position.
*/
bool sun_high, sun_low, dark;

/*
   Calculated from 'position_sensor_val' and the position limit calibration values.
*/
bool at_upper_position_limit, at_lower_position_limit;

/*
   The number of milliseconds the motor has been on in total, but aged (so it goes down with time that the motor is off).  This is used to
   decide how much we've been running the contactor and motor.
*/
unsigned long accumulated_motor_on_time = 0;

/*
   This is set when the accumulated motor on time is greater than its limit and is reset when the on-time falls below the hystersis value.
*/
bool motor_cooling_off = false;

bool motor_overcurrent = false;

/*
   Counts down when the backlight is on.  When it is zero, we turn the backlight off.  Units are seconds.
*/
unsigned backlight_timer;

/*
   Wind speed variables
*/
int16_t wind_speed_raw;        // Value read from ADC
float wind_speed_volts;        // Wind_spee_raw converted to volts
unsigned wind_speed_knots;      // In knots
unsigned max_wind_speed_knots = 0;  // Maximum recorded value since we started

// Forward definitions of the call-back functions that we pass to the task scheduler.

void read_time_and_sensor_inputs_callback();
void print_status_to_rs485_callback();


/*
    The next five callbacks and tasks interact with I2C devices.
*/
void display_status_on_lcd_callback();
void monitor_buttons_callback();
void monitor_lcd_backlight_callback();
void monitor_cron_callback();
void monitor_upward_moving_panels_and_stop_when_time_elapsed_callback();

Task display_status_on_lcd(TASK_SECOND / 2, TASK_FOREVER, &display_status_on_lcd_callback, &ts, true);
Task monitor_buttons(100, TASK_FOREVER, &monitor_buttons_callback, &ts, true);
Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);
Task monitor_cron(TASK_SECOND * 3600 * 4, TASK_FOREVER, &monitor_cron_callback, &ts, true);
Task monitor_upward_moving_panels_and_stop_when_time_elapsed(TASK_SECOND, TASK_FOREVER, &monitor_upward_moving_panels_and_stop_when_time_elapsed_callback, &ts, false);

void control_battery_charger_callback();
void control_hydraulics_callback();
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback();

void monitor_position_limits_callback();
void monitor_motor_on_time_callback();
void monitor_rs485_input_callback();


/*
   These are the tasks that are the heart of the logic that controls this system.  Some run periodically and are always enabled.  Some run only
   when the panels are in motion.
*/

Task read_time_and_sensor_inputs(200, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task print_status_to_rs485(TASK_SECOND * 10, TASK_FOREVER, &print_status_to_rs485_callback, &ts, true);
Task control_battery_charger(TASK_SECOND * 10, TASK_FOREVER, &control_battery_charger_callback, &ts, true);
Task control_hydraulics(TASK_SECOND * 60, TASK_FOREVER, &control_hydraulics_callback, &ts, true);
Task monitor_upward_moving_panels_and_stop_when_sun_angle_correct(300, TASK_FOREVER, &monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback, &ts, false);

Task monitor_position_limits(300, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
Task monitor_motor_on_time(TASK_SECOND * 2, TASK_FOREVER, &monitor_motor_on_time_callback, &ts, true);
Task monitor_rs485_input(100, TASK_FOREVER, &monitor_rs485_input_callback, &ts, true);


/*
   Keep track of whether we are driving the panels up or down.  These two should never be true at the same time.
*/
bool panels_going_up = false;
bool panels_going_down = false;

/*
   Set the calibration values to their "factory default".
*/
void set_calvals_to_defaults()
{
  calvals.position_upper_limit = 400;                          // Max position value (i.e., fully tilted up)
  calvals.position_lower_limit = 112;
  calvals.time_tilted_up_limit_in_minutes = 10 * 60; // Lower the panels after 10 hours, maximum.
  calvals.darkness_threshold = 50;

  calvals.battery_charging_temperature_limit = 90;        // Do not start charging when battery temperature above 85F
  calvals.supply_voltage_lower_limit = 11000;             // Below 11 volts, don't try to move panels other than lowering them.
  calvals.supply_voltage_charge_limit_low = 12100;        // Turn on charger if voltage drops below this value
  calvals.supply_voltage_charge_limit_high = 12300;       // Turn off charge when voltage goes over this value
  calvals.supply_voltage_bms_open_threshold = 11000;      // If we see a voltage this low, we assume the BMS has opened due to potential cell overcharge
  calvals.motor_amps_limit = 90;                          // If we see a current this high, we'll go into no-movement mode
  calvals.wind_speed_limit = 15;                          // Above this wind speed, in knots, we will lower the panels
  calvals.accumulated_motor_on_time_limit = MOTOR_MAX_ON_TIME_DEFAULT; // This is on-time in milliseconds, aged at 1/20th time
  calvals.accumulated_motor_on_time_aging_rate = 20;                 // Sets ratio of maximum on time to off time (1:<this variable>)
  calvals.backlight_on_time = DEFAULT_BACKLIGHT_ON_TIME;             // Number of seconds the LCD backlight stays on
  calvals.operation_mode = time_mode;                                // Mode in which we should start operation
  calvals.subversion = 12;
}

/*
   When we are in time_mode, the following table gives us the program that we run.  There is one string per month.  The string
   has time-command pairs.  The times are in 24 hour time.  The commands are either "U", for go-up, or "D" for go-down.  Following
   "U" there is optionaly the number of seconds to drive the panels up.  The default is one second.  We put this table in Flash,
   as we have plenty of program space (Flash), but are constrained by RAM.
*/
#define MAX_CRON_STRING (60)


const char January[]   PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:00D";
const char February[]  PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:30D";
const char March[]     PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:45D";
const char April[]     PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 18:00D";
const char May[]       PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 14:00U5 19:45D";
const char June[]      PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 14:00U5 20:00D";
const char July[]      PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 14:00U5 19:45D";
const char August[]    PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 14:00U5 19:30D";
const char September[] PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:50D";
const char October[]   PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:30D";
const char November[]  PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:20D";
const char December[]  PROGMEM = "9:00U2 10:00U2 12:00U5 13:00U5 17:00D";

const char *const cron_table[] PROGMEM = {
  January, February, March, April, May, June, July, August, September, October, November, December
};

char todays_cron_string[MAX_CRON_STRING];
char *current_cron_p = NULL;

/*
   When this is non-zero, do not let the normal update happen to the LCD so that the value the user
   wanted to see stays on the display for some number of seconds (like 10).
*/
char vtd_timeout = 0;

/*
   Called every second the backlight is on and turns off the backlight, and disables itself, when the backlight timer
   has counted down to zero.
*/
void monitor_lcd_backlight_callback(void)
{
  if (backlight_timer > 0) {
    backlight_timer--;
    if (backlight_timer == 0) {
      monitor_lcd_backlight.disable();
      lcd.setBacklight(BACKLIGHT_OFF);
    }
  }
  if (vtd_timeout > 0) {
    vtd_timeout--;
  }
}

enum val_to_display_e { vtd_none = 0, vtd_max_4V_current, vtd_temperature, vtd_voltage, vtd_build_date, vtd_build_time, vtd_time, vtd_mode, vtd_position, vtd_sunlow, vtd_sunhigh,
                        vtd_motor_on_time, vtd_max_knots, vtd_position_upper_limit,
                        vtd_position_lower_limit, vtd_time_tilted_up_limit_in_minutes, vtd_darkness_threshold, vtd_battery_charging_temperature_limit, vtd_motor_amps_limit,
                        vtd_wind_speed_limit, vtd_accumulated_motor_on_time_limit, vtd_accumulated_motor_on_time_aging_rate, vtd_backlight_on_time, vtd_last
                      } vtd_current = vtd_none;


void vtd_display_current(void)
{
  vtd_timeout = 10;
  char bytes = 0;
  lcd.setCursor(0, 0);
  switch (vtd_current) {
    case vtd_none:
      vtd_timeout = 0;
      bytes = 16;
      break;

    case vtd_build_date:
      lcd.print(F("Build date      "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(build_date);
      break;

    case vtd_build_time:
      lcd.print(F("Build time      "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(build_time);
      break;

    case vtd_time:
      lcd.print(F("Date   Time   "));
      lcd.setCursor(0, 1);
      lcd.print(monthShortStr(month(arduino_time)));
      lcd.write(' ');
      lcd.print(day(arduino_time));
      lcd.write(' ');
      lcd.print(hour(arduino_time));
      lcd.write(':');
      lcd.print(minute(arduino_time));
      lcd.write(':');
      lcd.print(second(arduino_time));

      bytes = 15;
      break;

    case vtd_mode:
      lcd.print(F("Operating Mode  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(operation_mode_string());
      break;

    case vtd_position:
      lcd.print(F("Panel Position   "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(position_sensor_val);
      break;

    case vtd_sunlow:
      lcd.print(F("Lower sun sensor"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(lower_solar_val);
      break;

    case vtd_sunhigh:
      lcd.print(F("Upper sun sensor"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(upper_solar_val);
      break;

    case vtd_voltage:
      lcd.print(F("Supply Voltage  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(supply_voltage / 1000);
      bytes += lcd.print(F("."));
      bytes += lcd.print(supply_voltage % 1000);
      break;

    case vtd_temperature:
      lcd.print(F("Battery Temperat"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(battery_temperature_F) + 1;
      lcd.write('F');
      break;

    case vtd_max_4V_current:
      lcd.print(F("Max Motor Amps  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(max_4V_current_amps);
      break;

    case vtd_motor_on_time:
      lcd.print(F("Motor-on time   "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(accumulated_motor_on_time);
      break;

    case vtd_max_knots:
      lcd.print(F("Max windspeed KT"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(max_wind_speed_knots);
      break;

    case vtd_position_upper_limit:
      lcd.print(F("Pos upper limit "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.position_upper_limit);
      break;

    case vtd_position_lower_limit:
      lcd.print(F("Pos lower limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.position_lower_limit);
      break;

    case vtd_time_tilted_up_limit_in_minutes:
      lcd.print(F("Time tilted up limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.time_tilted_up_limit_in_minutes);
      lcd.print(F(" min"));
      break;

    case vtd_darkness_threshold:
      lcd.print(F("Darkness thresh"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.darkness_threshold);
      break;

    case vtd_battery_charging_temperature_limit:
      lcd.print(F("Charging temp limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.battery_charging_temperature_limit);
      break;

    case vtd_motor_amps_limit:
      lcd.print(F("Motor amps limit  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.motor_amps_limit);
      break;

    case vtd_wind_speed_limit:
      lcd.print(F("Wind speed limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.wind_speed_limit);
      break;

    case vtd_accumulated_motor_on_time_limit:
      lcd.print(F("Accum motor limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.accumulated_motor_on_time_limit / 1000);
      break;

    case vtd_accumulated_motor_on_time_aging_rate:
      lcd.print(F("Accum aging rate "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.accumulated_motor_on_time_aging_rate);
      break;

    case vtd_backlight_on_time:
      lcd.print(F("Backlight time   "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.backlight_on_time);
      break;

    default:
      fail("vtd");
      break;
  }
  while (bytes < 16) {
    bytes += lcd.write(' ');
  }

}
/*
   Called every 100msec to monitor the buttons that are below the LCD.
*/
void monitor_buttons_callback()
{
  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    monitor_lcd_backlight.enable();
    backlight_timer = calvals.backlight_on_time;
    lcd.setBacklight(BACKLIGHT_WHITE);

    if (buttons & (BUTTON_UP | BUTTON_DOWN)) {
      int delta = (buttons & BUTTON_UP) ? 1 : -1;
      switch (vtd_current) {
        case vtd_mode:
          if (buttons & BUTTON_UP) {
            if (calvals.operation_mode < sensor_mode) {
              calvals.operation_mode = (enum mode_e)((int)calvals.operation_mode + 1);
            }
          } else {
            if (calvals.operation_mode > no_panel_movement_mode) {
              calvals.operation_mode = (enum mode_e)((int)calvals.operation_mode - 1);
            }
          }
          break;

        case vtd_position_upper_limit:
          calvals.position_upper_limit += delta;
          break;

        case vtd_position_lower_limit:
          calvals.position_lower_limit += delta;
          break;

        case vtd_time_tilted_up_limit_in_minutes:
          calvals.time_tilted_up_limit_in_minutes += delta;
          break;

        case vtd_darkness_threshold:
          calvals.darkness_threshold += delta;
          break;

        case vtd_battery_charging_temperature_limit:
          calvals.battery_charging_temperature_limit += delta;
          break;

        case vtd_motor_amps_limit:
          calvals.motor_amps_limit += delta;
          break;

        case vtd_wind_speed_limit:
          calvals.wind_speed_limit += delta;
          break;

        case vtd_accumulated_motor_on_time_limit:
          calvals.accumulated_motor_on_time_limit += delta * 1000;
          break;

        case vtd_accumulated_motor_on_time_aging_rate:
          calvals.accumulated_motor_on_time_aging_rate += delta;
          break;

        case vtd_backlight_on_time:
          calvals.backlight_on_time += delta;
          break;

      }
      vtd_display_current();
    }

    if (buttons & BUTTON_LEFT) {
      if (vtd_current > vtd_none) {
        vtd_current = (enum val_to_display_e)((int)vtd_current - 1);
        vtd_display_current();
      }

    }
    if (buttons & BUTTON_RIGHT) {
      vtd_current = (enum val_to_display_e)((int)vtd_current + 1);
      if (vtd_current == vtd_last) {
        vtd_current = vtd_none;
      }
      vtd_display_current();
    }

    if (buttons & BUTTON_SELECT) {

    }
  }
}

/*
    Turn off both relays that drive 12VDC to the contactor inputs.  Since we aren't driving
    the panels, we also disable the task that monitors position and estimated temperature limits
*/
void stop_driving_panels(void)
{
  quad_relay.turnRelayOff(1);
  quad_relay.turnRelayOff(2);
  //digitalWrite(REL1, LOW); // Turn off go-up relay
  //digitalWrite(REL2, LOW); // Turn off go-down relay
  panels_going_down = false;
  panels_going_up = false;
  monitor_position_limits.disable();
  monitor_upward_moving_panels_and_stop_when_sun_angle_correct.disable();

  monitor_upward_moving_panels_and_stop_when_time_elapsed.disable();
  lcd.setCursor(0, 1);
  lcd.print(F("Stopped "));
  lcd.print(position_sensor_val);
  vtd_timeout = 10;
}

/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const char *fail_message)
{
  stop_driving_panels();
  enable_rs485_output();
  Serial.print(F("todays_cron_string:"));
  Serial.println(todays_cron_string);
  Serial.print(F("current_cron_p: "));
  Serial.println(current_cron_p);
  Serial.println(fail_message);

  lcd.setCursor(0, 0);
  lcd.print(fail_message);

  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}

#ifdef RS485
/*
   Set when we have enabled the Arduino serial output (from its UART) to drive the RS-485 line.  Most of the time, this is not
   set.  This allows us to listen for characters from the other side.  We only set this when we need to send characters.
*/
bool txenabled;

/*
   Our RS-485 output is only active when we are transmitting.  Otherwise, we have this half-duplex channel
   in a listening mode so that we can receive commands.  See
*/
void enable_rs485_output()
{
  if (UCSR0A | (1 << TXC0)) {
    //  fail("enable485");
  }
  digitalWrite(TXEN, HIGH);   //Enable Transmit
  delay(1);                   //Let 485 chip go into Transmit Mode
  txenabled = true;
}

/*
   Disable the connection from the output of our UART to the RS-485 lines.  See
   https://exploreembedded.com/wiki/UART_Programming_with_Atmega128
   For more information on the UART programming.
*/
void disable_rs485_output()
{
  while (!(UCSR0A & (1 << TXC0)));      // Wait for the transmit buffer to be empty
  digitalWrite(TXEN, LOW);              //Turn off transmit enable
  txenabled = false;
}
#endif

/*
   Start the panels moving up and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going down when this was called.
*/
void drive_panels_up(void)
{
  if (panels_going_down) {
    fail("drive_panels_up()");
  } else {
    quad_relay.turnRelayOn(1);
    //digitalWrite(REL1, HIGH); // Turn on go-up relay
    lcd.setCursor(0, 1);
    lcd.print(F("Going up "));
    vtd_timeout = 10;
    panels_going_up = true;
    monitor_position_limits.enable();
  }
}

/*
   Start the panels moving down and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going up when this was called.
*/
void drive_panels_down(void)
{
  if (panels_going_up) {
    fail("drive_panels_down");
  } else {
    lcd.setCursor(0, 1);
    lcd.print(F("Going down "));
    vtd_timeout = 10;
    quad_relay.turnRelayOn(2);
    //digitalWrite(REL2, HIGH); // Turn on go-down relay
    panels_going_down = true;
    monitor_position_limits.enable();
  }
}


/*
   Turn on the AC input of the power supply, so that we may charge our lithium battery pack.
*/
void turn_on_power_supply(void)
{
  //digitalWrite(REL8, HIGH);
  quad_relay.turnRelayOn(4);
  power_supply_on = true;
}

/*
   Turn off the AC input of the power supply.  We do this when the battery voltage is high enough
   that we do not need to charge further.
*/
void turn_off_power_supply(void)
{
  //digitalWrite(REL8, LOW);
  quad_relay.turnRelayOff(3);
  power_supply_on = false;
}


/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback()
{
  arduino_time = now();

  supply_tap_voltage_raw = analogRead(ANIN4);

  /*
     The value by which we multiple the raw value, and the value we add or subtract after that are determined
     first by calculation based on the measured values of the resistors and then fine tuned with measurements
     with 5.5 digit multimeter (Rigol DM3058).
  */
  supply_tap_voltage = (((unsigned long)supply_tap_voltage_raw * 5036UL) / 1023UL) + 2;
  supply_voltage = (supply_tap_voltage * 10000UL) / 3245UL;

  battery_temperature_volts_raw = analogRead(ANIN5);
  battery_temperature_millivolts = (((unsigned long)battery_temperature_volts_raw * 5000UL) / 1023) + 20 /* Calibration value */;
  battery_temperature_C_x10 = battery_temperature_millivolts - 500;
  battery_temperature_F = ((((long)battery_temperature_C_x10 * 90L) / 50L) + 320L) / 10L;

  current_sense_4V_volts_raw = analogRead(ANIN6);
  current_sense_4V_millivolts = ((unsigned long)current_sense_4V_volts_raw * 5000UL) / 1023UL;
  current_sense_4V_amps = ((unsigned long)current_sense_4V_millivolts * 1000UL) / 36600UL;
  if (max_4V_current_amps < current_sense_4V_amps) {
    max_4V_current_amps = current_sense_4V_amps;
  }
  /*
      The values we read for the sun sensor and position sensors jump around, I guess due to noise.  To compensate and
      have more stable values average the last reading with this reading (and the 'last reading' is a running average)
  */
  lower_solar_val = (analogRead(ANIN1) + lower_solar_val) / 2;
  upper_solar_val = (analogRead(ANIN2) + upper_solar_val) / 2;
  position_sensor_val = (analogRead(ANIN3) + position_sensor_val) / 2;

  // Read the position sensor up to ten times with .5 second break in between readings to find a value
  // That is is in range.  Normally the first reading is in range and there is no .5 delay.  If after
  // ten tries, none are in range, set 'position_sensor_failed' to true

  for (int i = 0 ; i < 10 ; i++) {
    position_sensor_val = (analogRead(ANIN3) + position_sensor_val) / 2;
    if (position_sensor_val >= 10 && position_sensor_val <= 500) {
      break;
    } else {
      delay(500);
    }
    position_sensor_failed = true;
  }

  dark = (lower_solar_val + upper_solar_val) / 2 <= calvals.darkness_threshold;
  sun_high = (dark == false) && ((lower_solar_val + 10) < upper_solar_val);
  sun_low = dark || (lower_solar_val > (upper_solar_val + 10));
  at_upper_position_limit = position_sensor_val >= calvals.position_upper_limit;
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;

  /*
     Empirical correction from measurement at zero windspeed, means 'wind_speed_volts is zero at zero windspeed.
  */
  wind_speed_raw = ads.readADC_SingleEnded(0);
  wind_speed_volts = ads.computeVolts(wind_speed_raw);

  /*
     I measured 0.41556V at zero windspeed.  From the Adafruit web site, the anenometer is supposed to generate
     a 2V signal at 32.2 meter/sec (or 62.9 knots) and 0.4V at 0 meters/sec.  62.9/(2.0 - 0.4155) = 39.69
  */
  float knots = (wind_speed_volts - 0.4155) * 39.69;
  if (knots < 0) {
    wind_speed_knots = 0;
  } else {
    wind_speed_knots = knots;
    if (wind_speed_knots > max_wind_speed_knots) {
      max_wind_speed_knots = wind_speed_knots;
    }
  }
}

void print2digits(int number)
{
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void display_status_on_lcd_callback()
{
  if (vtd_timeout == 0) {
    lcd.setCursor(0, 0);
    int bytes = lcd.print(position_sensor_val);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(supply_voltage / 1000);
    bytes += lcd.print(F("."));
    bytes += lcd.print(supply_voltage % 1000);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(max_4V_current_amps);
    while (bytes < 16) {
      bytes += lcd.print(F(" "));
    }
    lcd.setCursor(0, 1);
    char c1 = ' ';
    if (panels_going_up) {
      c1 = 'U';
    } else if (panels_going_down) {
      c1 = 'D';
    } else if (motor_cooling_off) {
      c1 = 'C';
    }
    bytes = lcd.print(c1);

    char c2 = ' ';
    if (position_sensor_failed) {
      c2 = 'F';
    } else if (at_upper_position_limit) {
      c2 = 'u';
    } else if (at_lower_position_limit) {
      c2 = 'l';
    }
    bytes += lcd.print(c2);

    char c3 = ' ';
    if (power_supply_on) {
      c3 = 'P';
    }
    bytes += lcd.print(c3);

    bytes += lcd.print(F(" "));
    bytes += lcd.print(lower_solar_val);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(upper_solar_val);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(wind_speed_knots);

    while (bytes < 16) {
      bytes += lcd.print(F(" "));
    }
  }
}

void print_in_four_columns(int val)
{
  int spaces = 0;
  int original_val = val;
  if (val < 0) {
    val *= -1;
    spaces = -1;  // The minus sign takes one position
  }
  if (val < 10) {
    spaces += 3;
  } else if (val < 100) {
    spaces += 2;
  } else if (val < 1000) {
    spaces += 1;
  }
  for (int i = 0 ; i < spaces ; i++) {
    Serial.write(' ');
  }
  Serial.print(original_val);
}

void print_in_three_columns_with_leading_zeroes(int val)
{
  if (val < 100) {
    if (val < 10) {
      Serial.print(F("00"));
    } else {
      Serial.print(F("0"));
    }
  }
  Serial.print(val);
}
/*
   This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
   We display this information in a form that gnuplot(1) can readily absorb.

   # Date Time   Mode Pos Lwr Uppr Volts Temp Amps MotOT Drk SnH SnL UpL LwL GUp GDn PoS CoL OvC KTS
   Nov 17 17:15:54 2  137  11  10  12.80   77    0   0   1   0   1   0   1   0   0   0   0   0   0
   Nov 17 17:16:01 2  183  14  13  12.80   77    0   0   1   0   1   0   1   0   0   0   0   0   0

   Here is a gnuplot file that works to parse some of these lines where the text has been put in a file 'tracker.dat'.
   Still to do is to have gnuplot understand fields 13 through 22 (booleans from "Dark" to "Overcurrent").


  set term png size 1500, 300

  set output 'tracker.png'
  set xdata time
  set timefmt "%b %d %H:%M:%S %Y"
  set ylabel "Position/Volts/Temperature/Knots"
  set yrange [-10:400]
  set xrange [*:*]
  set xlabel " "
  set grid

  plot 'tracker.data' using 1:(($5*10))  t "Mode" with lines lc rgb "black", \
     'tracker.data' using 1:6          t "Position" with lines lc rgb "blue", \
     'tracker.data' using 1:7          t "Lower sensor" with lines lc rgb "red", \
     'tracker.data' using 1:8          t "Upper sensor" with lines lc rgb "green", \
     'tracker.data' using 1:9          t "Battery Volts" with lines lc rgb "pink", \
     'tracker.data' using 1:10         t "Battery Temperature" with lines lc rgb "gray", \
     'tracker.data' using 1:11         t "Battery Amps" with lines lc rgb "purple", \
     'tracker.data' using 1:12         t "Motor On Time" with lines lc rgb "red", \
     'tracker.data' using 1:(($23*10)) t "Windspeed (Knots/10)" with lines lc rgb "blue"

*/


void print_status_to_rs485_callback(void)
{
  static char line_counter = 0;

  if (verbose > VERBOSE_NONE) {
    enable_rs485_output();
    if (line_counter == 0) {
      Serial.println(F("# Date Time   Mode Pos Lwr Uppr Volts  Temp Amps MotOT Drk SnH SnL UpL LwL GUp GDn PoS CoL OvC KTS"));
      line_counter = 20;
    } else {
      line_counter--;
    }
    Serial.print(monthShortStr(month(arduino_time)));
    Serial.write(' ');
    Serial.print(day(arduino_time));
    Serial.write(' ');
    print2digits(hour(arduino_time));
    Serial.write(':');
    print2digits(minute(arduino_time));
    Serial.write(':');
    print2digits(second(arduino_time));
    Serial.write(' ');

    Serial.print(calvals.operation_mode);

    Serial.write(' ');
    print_in_four_columns(position_sensor_val);

    print_in_four_columns(lower_solar_val);
    print_in_four_columns(upper_solar_val);

    print_in_four_columns(supply_voltage / 1000);
    Serial.write('.');
    print_in_three_columns_with_leading_zeroes(supply_voltage % 1000);
    Serial.write(' ');

    print_in_four_columns(battery_temperature_F);
    Serial.write(' ');
#if 0
    Serial.print(F("bat temp raw="));
    Serial.print(battery_temperature_volts_raw);
    Serial.print(F(" bat milli="));
    Serial.print(battery_temperature_millivolts);    // raw value converted to millivolts
    Serial.print(F(" C="));
    Serial.print(battery_temperature_C_x10);
#endif


    print_in_four_columns(current_sense_4V_amps);
    print_in_four_columns(accumulated_motor_on_time / 1000);
    print_in_four_columns(dark);
    print_in_four_columns(sun_high);
    print_in_four_columns(sun_low);
    print_in_four_columns(at_upper_position_limit);
    print_in_four_columns(at_lower_position_limit);
    print_in_four_columns(panels_going_up);
    print_in_four_columns(panels_going_down);
    print_in_four_columns(power_supply_on);
    print_in_four_columns(motor_cooling_off);
    print_in_four_columns(motor_overcurrent);
    Serial.write(' ');
    Serial.println(wind_speed_knots);
#if 0
    Serial.write(' ');
    Serial.print(wind_speed_volts);
#endif


    disable_rs485_output();
  }
}

/*
  Check to see if the battery charger needs to be turned on or off.
*/
void control_battery_charger_callback()
{
  if (supply_voltage < calvals.supply_voltage_charge_limit_low && battery_temperature_F <= calvals.battery_charging_temperature_limit) {

    /*
       If the power supply was already on and the voltage is really low, then
       we conclude that the BMS opened because our charge limit range was too high.
       In this case, we lower the the charge limit range unless the upper limit is
       already below 12 volts.
    */
    if (power_supply_on && supply_voltage < calvals.supply_voltage_bms_open_threshold) {
      if (calvals.supply_voltage_charge_limit_high > 12000) {
        enable_rs485_output();
        Serial.print(F("BMS open?  supply_voltage: "));
        Serial.println(supply_voltage);

        /*
           Move the charge range down 10 millivolts
        */
        calvals.supply_voltage_charge_limit_high -= 10;
        calvals.supply_voltage_charge_limit_low -= 10;
        Serial.print(F(" reducing supply_voltage_charge_limit_high: "));
        Serial.print(calvals.supply_voltage_charge_limit_high);
        Serial.print(F(" and supply_voltage_charge_limit_low: "));
        Serial.print(calvals.supply_voltage_charge_limit_low);
        disable_rs485_output();
        turn_off_power_supply();
      }
    } else {   // Turn on power supply to charge our batteries
      turn_on_power_supply();
    }
  } else if (supply_voltage > calvals.supply_voltage_charge_limit_high) {
    turn_off_power_supply();
  }
}

#define POSITION_HYSTERSIS (20)

/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/
void monitor_position_limits_callback()
{
  if (position_sensor_failed) {
    stop_driving_panels();
  } else {
    if (panels_going_up && at_upper_position_limit && position_sensor_val > (calvals.position_upper_limit + POSITION_HYSTERSIS)) {
      stop_driving_panels();
    }
    if (panels_going_down && at_lower_position_limit && position_sensor_val <= (calvals.position_lower_limit - POSITION_HYSTERSIS)) {
      stop_driving_panels();
    }
  }
}

/*
   Model the temperature of the hydraulic motor and the contactor and the check valve solenoid by keeping track
   of how long they are on vs. how long they are off. This runs all the time doing this modeling.  This will stop
   the motor if it has run too long without time to cool down.  Other functions use the 'motor_cooling_off' as
   well to decide to not start running the motor when they otherwise would.
*/
void monitor_motor_on_time_callback()
{
  unsigned long time_now = millis();
  static unsigned long last_time_now = 0;                 // Values from millis() start at zero when system starts, so this is correct initialization vale
  unsigned long time_elapsed = time_now - last_time_now;  // Works even if we had a roll-over

  last_time_now = time_now;

  if (panels_going_up || panels_going_down) {
    accumulated_motor_on_time += time_elapsed;
    if (calvals.motor_amps_limit < current_sense_4V_amps) {
      stop_driving_panels();
      calvals.operation_mode = no_panel_movement_mode;
      motor_overcurrent = true;
    }
  } else {
    // For every second the motor is on, let it cool for a number of (configurable) seconds.
    unsigned long delta = time_elapsed / calvals.accumulated_motor_on_time_aging_rate;
    if (accumulated_motor_on_time < delta) {
      accumulated_motor_on_time = 0;
    } else {
      accumulated_motor_on_time -= delta;
    }
  }
  if (motor_cooling_off == false) {
    bool motor_might_be_hot = accumulated_motor_on_time > calvals.accumulated_motor_on_time_limit;
    if (motor_might_be_hot && (panels_going_up || panels_going_down)) {
      motor_cooling_off = true;
      stop_driving_panels();
    }
  } else if (accumulated_motor_on_time < (calvals.accumulated_motor_on_time_limit - ON_TIME_HYSTERSIS)) {
    motor_cooling_off = false;
  }
}

/*
   This is called back periodically when the panels are moving up.  Stop their upwards movement if the sun angle sensor
   tell us that the sun is now normal to the plane of the panels.  If that is the case, disable this task.
*/
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback()
{
  if (panels_going_up == false) {
    fail("monitor_up..");
  }
  if (sun_low == true) {
    stop_driving_panels();
  }
}

/*
   Number of seconds we are driving the panels up, only used in time_mode.
*/
char seconds_left_driving_panel_up;

/*
   This is called back periodically when the panels are moving up.  Stop their upwards movement if the sun angle sensor
   tell us that the sun is now normal to the plane of the panels.  If that is the case, disable this task.
*/
void monitor_upward_moving_panels_and_stop_when_time_elapsed_callback()
{
  if (panels_going_up == false) {
    fail("MON");
  }
  if (seconds_left_driving_panel_up < 0) {
    fail("PUT");
  }
  if (seconds_left_driving_panel_up == 1) {
    stop_driving_panels();
    seconds_left_driving_panel_up = 0;
  } else {
    seconds_left_driving_panel_up--;
  }
}

/*
   Check to see if the sun angle sensor is showing that the sun has move to be higher in the sky and so the suns rays are no
   longer normal to the panels (and sensor).  In this case, start raising the panels.

   Also check to see if it is now night time and we should lower the panels to their resting (down) position and start them moving
   down if that is so.
*/
void control_hydraulics_callback()
{
  bool supply_voltage_ok = supply_voltage > calvals.supply_voltage_lower_limit;

  if (panels_going_up || panels_going_down || !supply_voltage_ok || motor_cooling_off || position_sensor_failed || millis() < 10000) {
    return;
  }

  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      break;

    case time_mode:
      //  Parse a cron string, example: "9:00U2 10:00U3 12:00U9 17:00D"
      if (current_cron_p != NULL && *current_cron_p != '\0') {
        int command_hour = atoi(current_cron_p);
        if (command_hour < 0 || command_hour > 24) {
          fail("h"); // Command hour of range
        }
        char *minute_p = strchr(current_cron_p, ':');
        if (minute_p == NULL) {
          fail("c"); // Missing minute
        }
        minute_p++;
        int command_minute = atoi(minute_p);
        if (command_minute < 0 || command_minute > 59) {
          fail("m"); // Command minute out of range
        }
        if (hour(arduino_time) >= command_hour && minute(arduino_time) >= command_minute) {
          Serial.print(F("Matching: "));
          Serial.println(current_cron_p);

          char *p = minute_p;
          while (isdigit(*++p));    // Skip over digits in minutes field
          if (*p == '\0') {         // p should now point to the command character (U or D for now)
            fail("missing cmd");
          }
          switch (*p) {
            case 'U':
              {
                seconds_left_driving_panel_up = atoi(p + 1);
                if (seconds_left_driving_panel_up < 0 || seconds_left_driving_panel_up > 30) {
                  fail("seconds-to-run");
                }
                if (!at_upper_position_limit) {
                  drive_panels_up();
                  monitor_upward_moving_panels_and_stop_when_time_elapsed.enable();
                }
              }
              while (*p != '\0' && !isspace(*p)) {
                p++;
              }
              if (*p == '\0') {
                fail("D"); // Missing 'D'
              }
              p++;
              if (!isdigit(*p)) {
                fail("missing D");
              }
              current_cron_p = p;
              if (verbose > VERBOSE_NONE) {
                Serial.print(F("Executing Up command for "));
                Serial.print((unsigned)seconds_left_driving_panel_up);
                Serial.println(F(" seconds"));
              }
              break;

            case 'D':
              // D should end the line
              if (*(p + 1) != '\0') {
                fail("garbage");
              }
              if (verbose > VERBOSE_NONE) {
                Serial.print(F("Executing D command"));
              }
              if (!at_lower_position_limit) {
                drive_panels_down();
              }
              current_cron_p = p + 1;
              break;

            default: // Lots of debugging printfs that can be removed later.
              Serial.println(command_hour);
              Serial.println(command_minute);
              Serial.println(minute_p - current_cron_p);
              Serial.println(p - minute_p);
              Serial.println(minute_p);
              Serial.println(p);
              fail("unknown command");
              break;
          }
        }
      }
      break;

    case sensor_mode:
      {
        // If we need to raise the panel and the panel is not at is position limit, turn on the relay to raise it.
        if (sun_high && at_upper_position_limit == false) {
          if (time_of_first_raise == 0ULL) {
            time_of_first_raise = time_now;
          }
          drive_panels_up();
          monitor_upward_moving_panels_and_stop_when_sun_angle_correct.enable();
        }
        unsigned time_since_first_raise_in_minutes = ((time_now - time_of_first_raise) / 1000) / 60;
        bool bedtime = (time_since_first_raise_in_minutes > calvals.time_tilted_up_limit_in_minutes) || dark;
        if (panels_going_up == false && panels_going_down == false && bedtime) {
          if (at_lower_position_limit == false) {
            drive_panels_down();
          }
        }
      }
      break;

    default:
      fail("?mode");
      break;
  }
}

/*
   Set the global string pointer that keeps track of what the next time-command is that we should run.
*/
void set_todays_cron_string()
{
  strcpy_P(todays_cron_string, (char *)pgm_read_word(&(cron_table[month(arduino_time)])));  // Necessary casts and dereferencing, just copy.
  current_cron_p = todays_cron_string;
  Serial.print(F("set_todays_cron_string(): "));
  Serial.println(current_cron_p);
}

/*
   This is called every four hours.  It will do something (maybe more than once) in the early hours.
*/
void monitor_cron_callback(void)
{
  /*
     In the wee hours, set up the cron string for the next day
  */
  if (hour(arduino_time) < 6) {
    setup_time();
    set_todays_cron_string();
  }
}

/*
   Function copied from Arduino tutorial, modified to use arbitrary pointer, calculates the CRC
*/
unsigned long crc(unsigned char *base_p, unsigned len)
{
  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < len  ; ++index) {
    crc = crc_table[(crc ^ base_p[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (base_p[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

/*
   Write the structure 'calvals' to EEPROM starting at EEPROM location zero.  Calculate its
   checksum and write that after 'calvals'.  Read back 'calvals' and verify it was correct.
   This assumes that the RS-485 transmitter is enabled.
*/
void write_calvals_to_eeprom()
{
  unsigned eeprom_crc_value = 0;
  unsigned calvals_crc_value = 0;
  struct calvals_s temp_calvals;
#ifdef RS485
  if (txenabled == false) {
    fail("write_calvals..");
  }
#endif
  int i;
  for (i = 0; i < sizeof(calvals) ; i++) {
    EEPROM.update(i, ((unsigned char *)&calvals)[i]);
  }
  // Now write the CRC bytes to EEPROM
  calvals_crc_value = crc((unsigned char *)&calvals, sizeof(calvals));
  for (int j = 0 ; j < sizeof(unsigned long) ; j++) {
    EEPROM.update(i + j, ((unsigned char *)&calvals_crc_value)[j]);
  }
  // Fetch the EEPROM contents back and verify that they match what we wroge
  EEPROM.get(0 /* EEPROM address */, temp_calvals);

  for (int i = 0; i < sizeof(calvals) ; i++) {
    if (((unsigned char *)&temp_calvals)[i] != ((unsigned char *)&calvals)[i]) {

      Serial.print(F("Error writing calvals to EEPROM starting at byte "));
      Serial.println(i);
      for (int i = 0; i < sizeof(calvals) ; i++) {
        Serial.write(' ');
        Serial.print(((unsigned char *)&temp_calvals)[i], HEX);
      }
      Serial.println("");

      break;
    }
  }
  EEPROM.get(sizeof(calvals), eeprom_crc_value);
  if (eeprom_crc_value != calvals_crc_value) {
    Serial.print(F("Error reading CRC to EEPROM"));
  }
}

/*
   Print the calibration values to RS-485 output.  This assumes that the RS-485 transmitter is already enabled.
*/
void print_calvals()
{
#ifdef RS485
  if (txenabled == false) {
    fail("print_calvals");
  }
#endif

  Serial.print(F("Build date: "));
  Serial.println(build_date);

  Serial.print(F("Build time: "));
  Serial.println(build_time);

  Serial.print(F("unsigned position_upper_limit: "));
  Serial.println(calvals.position_upper_limit);

  Serial.print(F("position_lower_limit: "));
  Serial.println(calvals.position_lower_limit);

  Serial.print(F("time_tilted_up_limit: "));
  Serial.println(calvals.time_tilted_up_limit_in_minutes);

  Serial.print(F("darkness_threshold: "));
  Serial.print(calvals.darkness_threshold);

  Serial.print(F("supply_voltage_lower_limit: "));
  Serial.println(calvals.supply_voltage_lower_limit);

  Serial.print(F("supply_voltage_charge_limit_low: "));
  Serial.println(calvals.supply_voltage_charge_limit_low);

  Serial.print(F("supply_voltage_charge_limit_high: "));
  Serial.println(calvals.supply_voltage_charge_limit_high);

  Serial.print(F("supply_voltage_bms_open_threshold: "));
  Serial.println(calvals.supply_voltage_bms_open_threshold);

  Serial.print(F("accumulated_motor_on_time_limit: "));
  Serial.println(calvals.accumulated_motor_on_time_limit);

  Serial.print(F("accumulated_motor_on_time_aging_rate: "));
  Serial.println(calvals.accumulated_motor_on_time_aging_rate);
}

/*
   Read the calibration values from the EEPROM.  If their checksum fails, then do not update the calibration values
   in 'calvals'.  If the passed flag is set and the checksum doesn't match, then write the existing (default)
   calibration values to EEPROM instead.
   This function assumes that the rs-485 is enabled.
*/
void read_calvals_from_eeprom(bool fix_if_checksum_fails)
{
  unsigned eeprom_crc_value = 0;
  unsigned calvals_crc_value = 0;
  unsigned stored_eeprom_crc_value = 0;
  struct calvals_s temp_calvals;
#ifdef RS485
  if (txenabled == false) {
    fail("read_calvals");
  }
#endif
  /*
     Fetch the calvals structure from EEPROM and the stored CRC that follows it.
  */
  EEPROM.get(0 /* EEPROM address */, temp_calvals);
  EEPROM.get(sizeof(calvals), stored_eeprom_crc_value);

  /*
     Calculate the CRC of what we just fetched from EEPROM
  */
  eeprom_crc_value = crc((unsigned char *)&temp_calvals, sizeof(temp_calvals));

  bool crc_mismatch = eeprom_crc_value != stored_eeprom_crc_value;
  bool eeprom_version_out_of_date = temp_calvals.subversion < calvals.subversion;
  if (crc_mismatch || eeprom_version_out_of_date) {
    if (crc_mismatch) {
      Serial.println(F("CRCs don't match or "));
    }
    if (eeprom_version_out_of_date) {
      Serial.println(F("EEPROM calvals out of date"));
    }

    if (fix_if_checksum_fails) {
      Serial.println(F("updating EERPOM with defaults"));
      write_calvals_to_eeprom();
    }
  } else {
    calvals = temp_calvals;
  }
}

/*
   Read the RTC chip and set the 'Arduino' time based on it.  We do this on system start and once per day.
   The Arduino clock is not as accurate as the RTC.
*/
void setup_time(void)
{
  tmElements_t tm;

  if (RTC.read(tm)) {
    int dst_correction = 0;

    if (tm.Month > 3 && tm.Month < 11) {
      dst_correction = 1;
    } else if (tm.Month == 3 && tm.Day >= 11) {
      dst_correction = 1;
    } else if (tm.Month == 11 && tm.Day < 6) {
      dst_correction = 1;
    }
    setTime(tm.Hour + dst_correction, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
    time_of_day_valid = true;
  } else {
    if (RTC.chipPresent()) {
      Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
      Serial.println(F("example to initialize the time and begin running."));
      Serial.println();
    } else {
      Serial.println(F("DS1307 read error!  Please check the circuitry."));
      Serial.println();
    }
  }
}
/*
   This is the function that the Arudino run time system calls once, just after start up.  We have to set the
   pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
   our operation and emit a startup message.
*/
void setup()
{
  analogReference(DEFAULT);

  //pinMode(REL1, OUTPUT);  // declare the relay pin as an output
  //pinMode(REL2, OUTPUT);  // declare the relay pin as an output
  //pinMode(REL8, OUTPUT);  // declare the relay pin as an output

  //pinMode(TXEN, OUTPUT);

  // Set the working calibration values to the defaults that are in this file
  set_calvals_to_defaults();

  Wire.begin();
  lcd.begin(16, 2);
  lcd.print(F("Suntracker 1.0"));
  lcd.setCursor(0, 1);
  lcd.print(build_date);

  if (!quad_relay.begin()) {
    fail("REL");
  }
#ifdef SETTIME
  void setup_rtc();
  setup_rtc();
#endif

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  enable_rs485_output();
  Serial.println("");
  Serial.print(F("Suntracker "));
  Serial.print(build_date);
  Serial.write(' ');
  Serial.println(build_time);

  // Try reading calibration values from EEPROM.  If that fails, write our default calibration values to EERPOM
  read_calvals_from_eeprom(true);
  backlight_timer = calvals.backlight_on_time;

  if (!ads.begin()) {
    fail("ADS");
  } else {
    ads.setGain(GAIN_TWO);
  }

  disable_rs485_output();         //Turn off transmit enable
  delay(INITIAL_DELAY);

  /*
     Set a valid initial value for lots of globals.  We need this in order to get the tm structure set,
     s that we can set today's cron string.  We need to do that before running in time_mode.
  */
  read_time_and_sensor_inputs_callback();

  setup_time();
  set_todays_cron_string();
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop()
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}

/*
   This serial code _was_ working, then it seemed to throw away most of the input characters and
   when it did receive some, they were often values like 0xff or other bytes with
   most bits set.  Converting to using serialEvent() does not seem to have changed that.
*/
void monitor_rs485_input_callback()
{
#ifdef RS485
  if (txenabled == true) {
    fail("monitor_rs485...");
  }
#endif
}

const char *operation_mode_string(void)
{
  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      return ("no-panel-movement");
      break;

    case time_mode:
      return ("time");
      break;

    case sensor_mode:
      return ("sensor");
      break;

    default:
      fail("??mode");
      break;
  }
}

void serialEvent()
{
  if (Serial.available() == 0) {
    fail("serialEvent()");
  }

  // This may fail, but I haven't see it fail yet.  I think it is just a matter of time, but in the meantime, I want
  // to ensure that the issues I am seeing are not due to entering this function with txenabled.
#ifdef RS485
  if (txenabled == true) {
    fail("txenabled");
  }
#endif
  char c = Serial.read();      // Get the waiting character
  if (c == 0) {
    return;
  }

  switch (c) {
    case 'v':
      enable_rs485_output();
      verbose = (verbose + 1) % (VERBOSE_ALL + 1); // Cycle between 0, 1, 2
      Serial.print(F("Verbosity level now: "));
      Serial.println(verbose);
      break;

    case '*':
      while (Serial.available() == 0);
      c = Serial.read();
      enable_rs485_output();
      Serial.println("");
      switch (c) {
        case 'f':
          set_calvals_to_defaults();
          Serial.println(F("restored factory defaults to working calibration values"));
          Serial.println(F("use the s command to save them to EEPROM"));
          break;

        case 'u':  // Set upper position limit to current position
          calvals.position_upper_limit = position_sensor_val;
          Serial.print(F("Setting upper position limit to "));
          Serial.println(calvals.position_upper_limit);
          break;

        case 'l':  // Set lower position limit to current position
          calvals.position_lower_limit = position_sensor_val;
          Serial.print(F("Setting lower position limit to "));
          Serial.println(calvals.position_lower_limit);
          break;

        case 'd':   // Set darkness threshold.  Should be done at twighlight
          calvals.darkness_threshold = (lower_solar_val + upper_solar_val) / 2;
          Serial.print(F("Setting darkness threshold to: "));
          Serial.println(calvals.darkness_threshold);
          break;

        case 'h':
          Serial.println(F("*f - set factory defaults"));
          Serial.println(F("*u - set upper position limit to current position"));
          Serial.println(F("*l - set lower position limit to current position"));
          Serial.println(F("*d - set darkness threshold to current light level"));
          Serial.println(F("*h - print this help message"));
          Serial.println(F("*p - display the values of the calibration values"));
          Serial.println(F("*r - restore the calibration values from the EEPROM"));
          Serial.println(F("*s - save calibration values to EEPROM"));
          Serial.println(F("*t - toggle operational mode"));
          break;

        case 'p': // Print the calvals
          print_calvals();
          break;

        case 'r': // Restore calibration values from EEPROM
          read_calvals_from_eeprom(false);
          break;

        case 's': // Save calibration values to EEPROM
          Serial.println(F("Saving calibration values to EEPROM"));
          write_calvals_to_eeprom();
          break;

        case 't': // Toggle operational mode on and off
          {
            enum mode_e new_operation_mode = (enum mode_e)(((int)calvals.operation_mode + 1) % last_mode);
            if (new_operation_mode == time_mode && time_of_day_valid == false) {
              Serial.println(F("Cannot enter time mode when RTC inop"));
            } else {
              calvals.operation_mode = new_operation_mode;
            }
            Serial.print(F("Operational mode "));
            Serial.print(operation_mode_string());
            Serial.println("");
          }
          break;

        default:
          Serial.println(F("Unrecognized command character (expected 'u', 'l', 'd', or 'h'): "));
          break;
      }
      break;

    default:
      enable_rs485_output();
      Serial.print(F("Unrecog: "));
      Serial.println(c, HEX);
      break;
  }
  disable_rs485_output();         //Turn off transmit enable
}

/*
   If SETTIME is defined, we compile-in the code below to set the device's time to the build time.
*/
#ifdef SETTIME


tmElements_t tm;

void setup_rtc()
{
  bool parse = false;
  bool config = false;

  // get the date and time the compiler was run
  if (getDate(build_date) && getTime(build_time)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  if (parse && config) {
    Serial.print(F("DS1307 configured Time"));
  } else if (parse) {
    fail("DS1307");

  } else {
    fail("time string");
  }
}


bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;
  const char *monthName[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
#endif
