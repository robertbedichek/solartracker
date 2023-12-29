
/*
   This controls three relays that control a hydraulic motor, a hydraulic valve, and a power supply.  The goal is
   to get more power out of four LG Neon-2 420 Watt panels than if they were fixed-mounted.  A secondary goal is to
   have them better looking.  If they were fixed-mounted, they'd need to be reversed-racked, which is not a great look.

    The core of the controls system is an Arduino: Sparkfun Redboard Qwiic 
    with the Sparkfun 4-relay Qwiic, a 1307-based RTC, a 2x16 line LCD from Adafruit, and a 4-input I2C ADC.

   Creative Commons Licence
   Robert Bedichek
*/

#include <string.h> //Use the string Library
#include <ctype.h>
#include <EEPROM.h>
#include <assert.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <TimeLib.h>     // for update/display of time

/*
    This is for the LCD and button interface we added for local control and display.
*/
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#include <SoftwareSerial.h>

const unsigned long rs232_baud = 9600 ;

const byte rxPin = 2; // Wire this to Tx Pin of RS-232 level shifter
const byte txPin = 3; // Wire this to Rx Pin of RS-232 level shifter
 
SoftwareSerial rs232 (rxPin, txPin);
bool send_to_rs232 = true;

// The following is for the Sparkfun 4-relay board.  Relay numbers are 1..4.  Relay 1 is for "up", 3 is for "down".  
// 2 and 4 are unused.
// Relay 1, when activated, passes 12VDC when active to the go-up input of the contactor
// that controls the power to the hydraulic motor.  The second (relay 3) passes 12VDC when active to the go-down
// input of the contactor and to a solenoid in the check valve.  This check valve normally only allows
// hydraulic pressure to pass to the cycliner to make it go up.  When the hydraulic motor stops, we want
// the panels to retain their position.  Before we had this check valve, they would gradually drift back to
// the down position.  When we want the panels to lower, at the end of the day, we have to both release the
// check vavle by activating its solenoid and by power the hydralic motor in reverse, which is accomplished
// by driving 12VDC to the "go-down" input of the contactor.

#include "SparkFun_Qwiic_Relay.h"
#define RELAY_ADDR (0x6D)           // Default I2C address of the 4-relay board
Qwiic_Relay quad_relay(RELAY_ADDR);

// Relay numbers for moving the panels up and down.
#define RELAY_UP (1)
#define RELAY_DOWN (3)

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//    This is for the 1307 RTC we installed on the Ocean Controls main board.
#include <DS1307RTC.h>

//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
time_t arduino_time;

//   If we are able to read the DS1307 RTC over I2C, then we set this true and we can depend on
//  the time of day being valid.
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
 * Force 'Calvals' EEPROM value to be reloaded if non-zero, not if zero (normal case).  Needed when moving 'subversion' field.
 */
#define FORCE_CALVALS_RELOAD (1)

/*
   Speed at which we run the serial connection (both via USB and RS-485)
*/
#define SERIAL_BAUD (38400)


//   Number of milliseconds to wait after boot and build display before starting operational part.
#define INITIAL_DELAY (2000)

//   Default maximum time the motor can be on continuously.  This is just the default value for when
//   the user resets to "factory defaults".  Units are milliseconds.
#define MOTOR_MAX_ON_TIME_DEFAULT (90000)

//  On-time hysteresis in milliseconds.  Once the motor has been running too long, past the max-motor-on-time limit,
//  we turn off until the on-time falls below the limit minus this vale.  This avoids the system turning on and off
//  rapidly when the limit is reached.
#define ON_TIME_HYSTERSIS (20000)


//   We can operate in one of three modes.  The first is a kind of "off" mode where we stay alive, talk on the
//   serial port, update the LCD, but do not move the panels.   The two "on" modes are Sun-sensor mode and time-of-day/day-of-year mode.
enum mode_e { no_panel_movement_mode, position_mode, rain_stow_mode, wind_stow_mode, last_mode};

#define SSR_ENABLE_OUT (7) // Arduino output pin 7 on J4, writing '1' turns on 8V/100A power supply

//    Arudino Analog In 0, measures the voltage from the draw-string position sensor
#define DRAW_STRING_IN (0)

//   To monitor the supply voltage we use Arduino Analog 1. We have a resistor divider with a 68.284k Ohm resistor
//   in series with a 32.823k Ohm resistor. The the center tap is connected to A1 (analog input 1), the other
//   leads are connected to the supply voltage and the ground. 
#define SUPPLY_VOLTAGE_IN (1)

//   analog input to monitor contactor temperature, so we used Arduino Analog 2.
//   It is connected to a TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
#define TMP36_IN (2)

// And for the output of a current sensor on the 4V line we use Arduino analog 3
#define CURRENT_SENSE_IN (3)

void setup_time(void);

const char *operation_mode_string(void);

/*
    Central (and only) task scheduler data structure.
*/
Scheduler ts;

/*
   These are values that may need to be tweaked.  They should be stored in EEPROM and changable through the RS-485
   interface and possibly by a future keypad or other direct, at the device, interface.  The first field, 'subversion', must
   be the first element.  This is so that incrementing subversion will always cause a reload of the EEPROM version of the
   calvals, even when fields are changed.
*/
  
struct calvals_s {
  int subversion;                                  // Changing this forces an calvals EEPROM reload
  unsigned position_upper_limit;                   // Max position value (i.e., fully tilted up)
  unsigned position_lower_limit;
  float darkness_threshold;
  enum mode_e operation_mode;                      // Mode to start in
} calvals;

#define MOTOR_AMPS_DOWN_LIMIT (30)
#define MOTOR_AMPS_UP_LIMIT (70)

#define WIND_SPEED_LIMIT (5)          // Above this many knots, we will consider stowing
#define HIGH_WIND_THRESHOLD (10)       // Must be above 'WIND_SPEED_LIMIT' for this many times before we stow

#define ACCUMULATED_MOTOR_ON_TIME_LIMIT (60000)
#define ACCUMULATED_MOTOR_ON_TIME_AGING_RATE (1)
#define BACKLIGHT_ON_TIME (3600)
#define INITIAL_OPERATION_MODE (mode_position)


/*
   These come from Arudino Analog input 1, which is driven by the center tap of two resistors that connect to the 12V line and
   ground.  We need this resistor divider network to move the 12V line to the range of the ADC, which is 0..5V.
*/
unsigned supply_volts;          // Tap voltage converted to supply voltage

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is in the middle of the pack.
*/
int contactor_temperature_F;                // Degrees C converter to Farenheight

/*
   We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
   is unused.  Its current output goes to Arduino Analog input 3.
*/
unsigned current_sense_8V_amps;       // Millivolts from the Attopilot device converter to milliamps it is sensing
unsigned max_8V_current_amps = 0;     // Max recorded current since we started

float solar_volts, last_solar_volts;  

float rain_sensor_volts;

/*
   This is from the 1000mm pull-string sensor that tells us where the panels are.
*/
unsigned position_sensor_val;         // Raw ADC values 0..1023 for 0..5V
bool position_sensor_failed = false;  // Set true if the position sensor value is ever out of range
unsigned daily_stalls;                // Zeroed at midnight, incremented on each motor stall
unsigned stall_count;                 // Incremented when no change in position while motor on
unsigned last_position_sensor_val;    // Position value at last call to status print
unsigned last_position_sensor_val_stall; // Position sensor value at last sample in monitor_motor_stall_callback()

/*
    'dark' is when
   both sensors indicate the light level is so low that it must be nighttime.  That is our trigger to lower the panels to
   their nighttime, resting position.
*/
bool dark;

/*
   Calculated from 'position_sensor_val' and the position limit calibration values.
*/
bool at_upper_position_limit = false, at_lower_position_limit = false;

/*
   The number of milliseconds the motor has been on in total, but aged (so it goes down with time that the motor is off).  This is used to
   decide how much we've been running the contactor and motor.
*/
unsigned long accumulated_motor_on_time = 0;

/*
   This is set when the accumulated motor on time is greater than its limit and is reset when the on-time falls below the hystersis value.
*/
bool motor_cooling_off = false;

bool motor_up_overcurrent = false;
bool motor_down_overcurrent = false;

/*
   Counts down when the backlight is on.  When it is zero, we turn the backlight off.  Units are seconds.
*/
unsigned backlight_timer;

/*
   Wind speed variables
*/
float wind_speed_knots = 0.0   ;         // In knots
float recent_max_wind_speed_knots = 0.0; // Maximum recorded value since we started

int16_t rain_sensor_raw;

int16_t solar_raw;

// Forward definitions of the call-back functions that we pass to the task scheduler.

void read_time_and_sensor_inputs_callback();

/*
    The next five callbacks and tasks interact with I2C devices.
*/
void display_status_on_lcd_callback();
void monitor_buttons_callback();
void monitor_lcd_backlight_callback();
void print_status_to_serial_callback();
void monitor_cron_callback();
void monitor_upward_moving_panels_and_stop_when_target_position_reached_callback();
void monitor_rain_sensor_callback();
void monitor_wind_sensor_callback();

Task display_status_on_lcd(TASK_SECOND / 2, TASK_FOREVER, &display_status_on_lcd_callback, &ts, true);
Task monitor_buttons(100, TASK_FOREVER, &monitor_buttons_callback, &ts, true);
Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);
Task monitor_cron(TASK_SECOND * 3600 * 4, TASK_FOREVER, &monitor_cron_callback, &ts, true);
Task monitor_upward_moving_panels_and_stop_when_target_position_reached(TASK_SECOND/10, TASK_FOREVER, 
                                              &monitor_upward_moving_panels_and_stop_when_target_position_reached_callback, &ts, false);
Task monitor_rain_sensor(TASK_SECOND * 30, TASK_FOREVER, &monitor_rain_sensor_callback, &ts, true);
Task monitor_wind_sensor(TASK_SECOND / 2, TASK_FOREVER, &monitor_wind_sensor_callback, &ts, true);

void control_hydraulics_callback();
void monitor_position_limits_callback();
void monitor_motor_on_time_callback();
void monitor_motor_stall_callback();

/*
   These are the tasks that are the heart of the logic that controls this system.  Some run periodically and are always enabled.  Some run only
   when the panels are in motion.
*/

Task read_time_and_sensor_inputs(200, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task print_status_to_serial(TASK_SECOND * 5, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
Task control_hydraulics(TASK_SECOND * 10, TASK_FOREVER, &control_hydraulics_callback, &ts, true);

Task monitor_position_limits(50, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
Task monitor_motor_on_time(TASK_SECOND * 2, TASK_FOREVER, &monitor_motor_on_time_callback, &ts, true);
Task monitor_motor_stall(500, TASK_FOREVER, &monitor_motor_stall_callback, &ts, false);


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
  calvals.position_upper_limit = 350;                          // Max position value (i.e., fully tilted up)
  calvals.position_lower_limit = 60;
  calvals.darkness_threshold = .5;
  calvals.operation_mode = position_mode;                              // Mode in which we should start operation
  calvals.subversion = 22;
}


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

enum val_to_display_e { vtd_none = 0, vtd_max_8V_current, vtd_temperature, vtd_volts, vtd_build_date, vtd_build_time, vtd_time, vtd_mode, vtd_position, 
                       vtd_sun, vtd_motor_on_time, vtd_max_knots, vtd_position_upper_limit,
                        vtd_position_lower_limit, /* vtd_time_tilted_up_limit_in_minutes, */ vtd_darkness_threshold, vtd_motor_amps_limit,
                        vtd_wind_speed_limit, vtd_accumulated_motor_on_time_limit, vtd_accumulated_motor_on_time_aging_rate, vtd_last
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
      bytes = lcd.print(F(__DATE__));
      break;

    case vtd_build_time:
      lcd.print(F("Build time      "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(F(__TIME__));
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

    case vtd_sun:
      lcd.print(F("sun sensor"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(solar_volts);
      break;
      
    case vtd_volts:
      lcd.print(F("Supply Voltage  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(supply_volts / 1000);
      bytes += lcd.print(F("."));
      bytes += lcd.print(supply_volts % 1000);
      bytes += lcd.print(F(" "));
      break;

    case vtd_temperature:
      lcd.print(F("Relay Temperat"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(contactor_temperature_F) + 1;
      lcd.write('F');
      break;

    case vtd_max_8V_current:
      lcd.print(F("Max Motor Amps  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(max_8V_current_amps);
      break;

    case vtd_motor_on_time:
      lcd.print(F("Motor-on time   "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(accumulated_motor_on_time);
      break;

    case vtd_max_knots:
      lcd.print(F("Max windspeed KT"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(recent_max_wind_speed_knots);
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

    case vtd_darkness_threshold:
      lcd.print(F("Darkness thresh"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(calvals.darkness_threshold);
      break;

    case vtd_motor_amps_limit:
      lcd.print(F("Motor amps limit  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(MOTOR_AMPS_DOWN_LIMIT);
      bytes += lcd.print(F(" "));
      bytes += lcd.print(MOTOR_AMPS_UP_LIMIT);
      break;

    case vtd_wind_speed_limit:
      lcd.print(F("Wind speed limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(WIND_SPEED_LIMIT);
      break;

    case vtd_accumulated_motor_on_time_limit:
      lcd.print(F("Accum motor limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(ACCUMULATED_MOTOR_ON_TIME_LIMIT / 1000);
      break;

    case vtd_accumulated_motor_on_time_aging_rate:
      lcd.print(F("Accum aging rate "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(ACCUMULATED_MOTOR_ON_TIME_AGING_RATE);
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
    if (buttons > 16) {
      Serial.print(F("# Ignoring keypad: "));
      Serial.println(buttons);
      monitor_buttons.disable();
    }
   
    monitor_lcd_backlight.enable();
    backlight_timer = BACKLIGHT_ON_TIME;
    lcd.setBacklight(BACKLIGHT_WHITE);

    if (buttons & (BUTTON_UP | BUTTON_DOWN)) {
      int delta = (buttons & BUTTON_UP) ? 1 : -1;
      switch (vtd_current) {
        case vtd_mode:
          if (buttons & BUTTON_UP) {
            if (calvals.operation_mode < (enum mode_e)((int)last_mode - 1)) {
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

        case vtd_darkness_threshold:
          calvals.darkness_threshold += delta;
          break;

        default:
          Serial.print(F("# VTD "));
          Serial.println(vtd_current);
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
void stop_driving_panels(const char *who_called)
{
  Serial.print(F("# stop_driving_panels(), called by "));
  Serial.println(who_called);

  // Turn off 8V supply and wait 500 milliseconds
  // digitalWrite(SSR_ENABLE_OUT, LOW);

  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an 
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(SSR_ENABLE_OUT, INPUT);

  delay(500);
  // Now turn off the relays, less wear on them to not be switching power
  quad_relay.turnRelayOff(RELAY_UP);
  quad_relay.turnRelayOff(RELAY_DOWN);
  panels_going_down = false;
  panels_going_up = false;

  /*
   * We were called by one of these.  Ensure they are all disabled so we don't get called twice.
   */
  monitor_position_limits.disable();
  monitor_motor_stall.disable();
  monitor_upward_moving_panels_and_stop_when_target_position_reached.disable();

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
  stop_driving_panels("fail");
  Serial.println(fail_message);

  lcd.setCursor(0, 0);
  lcd.print(fail_message);

  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}



/*
   Start the panels moving up and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going down when this was called.
*/
void drive_panels_up(void)
{
  if (panels_going_down) {
    fail("drive_panels_up()");
  } else {
    Serial.println(F("# lift "));
    // Turn on "up" relay, wait 100 milliseconds, then turn on 8V supply, less wear on relay
    quad_relay.turnRelayOn(RELAY_UP);
    delay(100);
    pinMode(SSR_ENABLE_OUT, OUTPUT);
    digitalWrite(SSR_ENABLE_OUT, HIGH);
    delay(3000);  // It takes time for the 100A power supply to develop power
  
    lcd.setCursor(0, 1);
    lcd.print(F("Going up "));
    vtd_timeout = 10;
    panels_going_up = true;
    stall_count = 0;
    monitor_position_limits.enable();
    monitor_motor_stall.enable();
  }
}

/*
   Start the panels moving down and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going up when this was called.
*/
void drive_panels_down(void)
{
  if (at_lower_position_limit) {
    return;
  } else if (panels_going_up) {
    fail("drive_panels_down");
  } else {
    Serial.println(F("# retract"));
    lcd.setCursor(0, 1);
    lcd.print(F("Going down "));
    vtd_timeout = 10;

    // Turn on "down" relay, wait 100 milliseconds, then turn on SSR to power 8V supply, less wear on mechanical relay
    quad_relay.turnRelayOn(RELAY_DOWN);
    delay(100);
    monitor_position_limits.enable();
    pinMode(SSR_ENABLE_OUT, OUTPUT);
    digitalWrite(SSR_ENABLE_OUT, HIGH);
    panels_going_down = true;
    stall_count = 0;
    monitor_position_limits.enable();
    monitor_motor_stall.enable();
  }
}

/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback()
{
  arduino_time = now();

  unsigned supply_tap_volts_raw;  // Raw ADC value: 0..1023 for 0..5V
  unsigned supply_tap_volts;      // Raw value converted to millivolts (of the tap)

  supply_tap_volts_raw = analogRead(/* Arduino analog input 1 */ SUPPLY_VOLTAGE_IN);

  /*
     The value by which we multiple the raw value, and the value we add or subtract after that are determined
     first by calculation based on the measured values of the resistors and then fine tuned with measurements
     with 5.5 digit multimeter (Rigol DM3058).
  */
  supply_tap_volts = (((unsigned long)supply_tap_volts_raw * 5036UL) / 1023UL) + 2;
  supply_volts = (supply_tap_volts * 10000UL) / 3061UL;
  
  unsigned current_sense_8V_volts_raw;   // What is read from input 3, 0..1023 for 0..5V
  unsigned current_sense_8V_millivolts;  // Raw ADC value converted to millivolts

  current_sense_8V_volts_raw = analogRead(/* Arduino analog input 3 */ CURRENT_SENSE_IN);

  /* Empirical calibration: the raw value seems to never be less than four, even with no current */
  if (current_sense_8V_volts_raw >= 4) {
    current_sense_8V_volts_raw -= 4;
  }
  current_sense_8V_millivolts = ((unsigned long)current_sense_8V_volts_raw * 5000UL) / 1023UL;
  current_sense_8V_amps = ((unsigned long)current_sense_8V_millivolts * 1000UL) / 36600UL;
  if (max_8V_current_amps < current_sense_8V_amps) {
    max_8V_current_amps = current_sense_8V_amps;
  }
#undef DEBUG_CURRENT
#ifdef DEBUG_CURRENT
  Serial.print(F("current_sense_8V_volts_raw: "));
  Serial.print(current_sense_8V_volts_raw);
  Serial.print(F(" millivolts: "));
  Serial.print(current_sense_8V_millivolts);
  Serial.print(F(" amps: "));
  Serial.println(current_sense_8V_amps);
#endif

  unsigned contactor_temperature_volts_raw;    // raw ADC value: 0..1023 for 0..5V
  unsigned contactor_temperature_millivolts;  // raw value converted to millivolts
  int contactor_temperature_C_x10;            // millivolts converted to degrees Celsiuis

  contactor_temperature_volts_raw = analogRead(/* Arduino analong input */ TMP36_IN);
  contactor_temperature_millivolts = (((unsigned long)contactor_temperature_volts_raw * 5000UL) / 1023) + 20 /* Calibration value */;
  contactor_temperature_C_x10 = contactor_temperature_millivolts - 500;
  contactor_temperature_F = ((((long)contactor_temperature_C_x10 * 90L) / 50L) + 320L) / 10L;


  // Empirical correction from measurement at zero windspeed, means 'wind_speed_volts is zero at zero windspeed.
 int16_t wind_speed_raw;             // Value read from ADC
 float wind_speed_volts;            // Wind_spee_raw converted to volts

  wind_speed_raw = ads.readADC_SingleEnded(0);
  wind_speed_volts = ads.computeVolts(wind_speed_raw);

  /*
     I measured 0.41556V at zero windspeed.  From the Adafruit web site, the anenometer is supposed to generate
     a 2V signal at 32.2 meter/sec (or 62.9 knots) and 0.4V at 0 meters/sec.  62.9/(2.0 - 0.4155) = 39.69
  */
  float knots = (wind_speed_volts - 0.4155) * 39.69;
  if (knots < 0) {
    wind_speed_knots = 0.0;
  } else {
    wind_speed_knots = knots;
    if (wind_speed_knots > recent_max_wind_speed_knots) {
      recent_max_wind_speed_knots = wind_speed_knots;    
    } else {
      if (recent_max_wind_speed_knots > 0.0) {
        // This function is called five times a second.  We reduce the recent maximum wind speed variable
        // so that it goes down by one knot every thousand calls, or 200 seconds, about 3 minutes.
        recent_max_wind_speed_knots -= 0.001;
        if (recent_max_wind_speed_knots < 0.0) {
          recent_max_wind_speed_knots = 0.0;
        }
      }
    }
  }
  
  rain_sensor_raw = ads.readADC_SingleEnded(/* ADS1115 input */1);
  rain_sensor_volts = ads.computeVolts(rain_sensor_raw);

// Input 2 (the four inputs are numbered 0, 1, 2, 3) is unused 
//  _sensor_raw = ads.readADC_SingleEnded(/* ADS1115 input */2);
//  _sensor_volts = ads.computeVolts(_sensor_raw);

  /*
      The values we read for the sun sensor and position sensors jump around, I guess due to noise.  To compensate and
      have more stable values average the last reading with this reading (and the 'last reading' is a running average)
  */
  solar_raw = ads.readADC_SingleEnded(/* ADS1115 input */3);
  last_solar_volts = solar_volts;
  solar_volts = ads.computeVolts(solar_raw);
  
  position_sensor_val = (analogRead(DRAW_STRING_IN) + position_sensor_val) / 2;

  // Read the position sensor up to ten times with .5 second break in between readings to find a value
  // That is is in range.  Normally the first reading is in range and there is no .5 delay.  If after
  // ten tries, none are in range, set 'position_sensor_failed' to true
  bool previous_position_sensor_failed = position_sensor_failed;
  for (int i = 0 ; i < 10 ; i++) {
    position_sensor_val = (analogRead(/* Arduino analog input */ DRAW_STRING_IN) + position_sensor_val) / 2;
    if (position_sensor_val >= 10 && position_sensor_val <= 500) {
      if (previous_position_sensor_failed) {
        Serial.println(F("#Position sensor working"));
        position_sensor_failed = false;
      } 
      break;
    } else {
      delay(500);
    }
    position_sensor_failed = true;
  }
  if (previous_position_sensor_failed == false && position_sensor_failed) {
    Serial.println(F("#Position sensor failed"));
  }

  dark = solar_volts <= calvals.darkness_threshold;
  
  at_upper_position_limit = position_sensor_val >= calvals.position_upper_limit;
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;   // Panels are at a good lower position when position_sensor_val is 50

/*
 * If the panels at either position limit and not moving, only send output once every ten minutes.
 * If the panels are not at either limit and not in moving, send output every ten seconds
 * If the panels are moving, send output every 200 milliseconds
 */
  long current_interval_in_milliseconds = print_status_to_serial.getInterval();
  const long ten_minutes_in_milliseconds = 10 * 60 * TASK_SECOND;
  if (panels_going_up || panels_going_down || wind_speed_knots >= WIND_SPEED_LIMIT) {
    const long three_hundred_milliseconds = 300;
    if (current_interval_in_milliseconds != three_hundred_milliseconds) {
      Serial.print(F("# print-status interval set to 300ms, was="));
      Serial.println(current_interval_in_milliseconds);
      print_status_to_serial.setInterval(three_hundred_milliseconds);
    }
    // If at either position limit or if we are stuck in no-operation mode, we'd like to go into "slow print" mode
  } else if (at_upper_position_limit || at_lower_position_limit || calvals.operation_mode == no_panel_movement_mode) {
    // However, do not go into "slow print mode" until at least 30 seconds after start up.
    if (millis() > 30000 && current_interval_in_milliseconds != ten_minutes_in_milliseconds) {
      Serial.print(F("# print-status interval set to ten minutes, pos="));
      Serial.print(position_sensor_val);
      Serial.print(F(" interval was="));
      Serial.println(current_interval_in_milliseconds);
      print_status_to_serial.setInterval(ten_minutes_in_milliseconds);
    }
  } else {
    long ten_seconds_in_milliseconds = 10 * TASK_SECOND; // default case, panels not moving and not at either limit
    if (current_interval_in_milliseconds != ten_seconds_in_milliseconds) {
      Serial.print(F("# print-status interval set to ten seconds, was="));
      Serial.println(current_interval_in_milliseconds);
      print_status_to_serial.setInterval(ten_seconds_in_milliseconds);
    }
  }
}

void display_status_on_lcd_callback()
{
  if (vtd_timeout == 0) {
    lcd.setCursor(0, 0);
    int bytes = lcd.print(position_sensor_val);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(supply_volts / 1000);
    bytes += lcd.print(F("."));
    bytes += lcd.print(supply_volts % 1000);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(max_8V_current_amps);
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
   
    bytes += lcd.print(c3);

    bytes += lcd.print(F(" "));
    bytes += lcd.print((int)(solar_volts * 10.0));
    bytes += lcd.print(F(" "));
   
    bytes += lcd.print((int)(rain_sensor_volts * 10.0));
    bytes += lcd.print(F(" "));
    
    bytes += lcd.print((int)wind_speed_knots);

    while (bytes < 16) {
      bytes += lcd.print(F(" "));
    }
  }
}

/*
   This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
   We display this information in a form that gnuplot(1) can readily absorb.


# Date    Time Year Md  Pos  Dif  Sol Delt Rain  Volts Tmp Amp Mot Drk UpL LwL GUp GDn CoL OvC  Knots
Apr 15 09:47:46 2023 1   56  -56   31    0  330 11.600  59   0   0   0   0   1   0   0   0   0  0.0
Apr 15 09:47:56 2023 1   56    0   31    0  330 11.568  59   0   0   0   0   1   0   0   0   0  0.1
Apr 15 09:48:06 2023 1   56    0   31    0  330 11.584  59   0   0   0   0   1   0   0   0   0  0.2

  
   Here is a gnuplot file that works to parse some of these lines where the text has been put in a file 'tracker.dat'.
   Still to do is to have gnuplot understand fields 13 through 22 (booleans from "Dark" to "Overcurrent").

set term png size 1500, 300
set output 'tracker.png'
set xdata time
set xrange [time(0) - 3*24*60*60:]
set timefmt "%b %d %H:%M:%S %Y"
set ylabel "Position/Volts/Temperature/Knots"
set yrange [-10:1200]
set xlabel " "
set grid
# set size 1.2 ,0.5
set key top left
set datafile separator whitespace

# Typical data

# Date Time      Year Mode Pos Diff  Sol Rain  Volts  Temp Amps MotT Drk UpL LwL GUp GDn CoL OvC KTS
# Dec 25 19:08:18 2022 2   51  -51   19  328  11.728   63    3   0   0   0   1   0   0   0   0   0

plot 'tracker.data' using 1:(($5 * 250.0))    t "Mode" with points lc rgb "black", \
     'tracker.data' using 1:(($6 * 3.0))      t "Position" with lines lc rgb "blue", \
     'tracker.data' using 1:(($8 * 10.0))     t "light sensor" with lines lc rgb "red", \
     'tracker.data' using 1:(($9 * 10.0))     t "light sensor" with lines lc rgb "red", \
     'tracker.data' using 1:((1000 - ($10 * 3))) t "Rain sensor" with lines lc rgb "green", \
     'tracker.data' using 1:(($11 * 100.0))    t "System Voltage (x100)" with lines lc rgb "black", \
     'tracker.data' using 1:(($12 * 10.0))    t "Contactor Temperature (x10 F)" with lines lc rgb "gray", \
     'tracker.data' using 1:(($13 * 10.0))    t "Motor Amps (x10)" with lines lc rgb "purple", \
     'tracker.data' using 1:(($14 * 10.0))   t "Motor On Time" with lines lc rgb "cyan", \
     'tracker.data' using 1:(($22 * 185.0))  t "Windspeed" with lines lc rgb "orange"

*/

// Send the passed string to one or both of our serial channels
void print_buf(char *b)
{
  Serial.print(b);  
  if (send_to_rs232) {
    rs232.print(b);
  }
}

// Called periodically.  Sends relevant telemetry back over one or both of the serial channels
void print_status_to_serial_callback(void)
{
  static char line_counter = 0;

  int position_difference;              // Amount position changed since last call to status print
  position_difference = (int)last_position_sensor_val - (int)position_sensor_val;
  last_position_sensor_val = position_sensor_val;

  if (line_counter == 0) {
    const char *m PROGMEM = "# Date    Time Year Md  Pos  Dif  Sol Delt Rain  Volts Tmp Amp Mot Drk UpL LwL GUp GDn CoL OvC  Knots\n\r";
    Serial.print(m);
    if (send_to_rs232) {
      rs232.print(m);
    }
    line_counter = 20;
  } else {
    line_counter--;
  }

    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.
    
  char buf[45];
  snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d %d %4d %4d %4d %4d", 
      monthShortStr(month(arduino_time)), 
      day(arduino_time), 
      hour(arduino_time),
      minute(arduino_time), 
      second(arduino_time), 
      year(arduino_time) - 30,
      calvals.operation_mode,
      position_sensor_val,
      position_difference,
      (int)(solar_volts * 100.0),
      (int)(solar_volts - last_solar_volts) * 1000.0);
  
  print_buf(buf);
    
  snprintf(buf, sizeof(buf), " %4d %2d.%03d %3d %3d %3d ",
       (int)(rain_sensor_volts * 100.0),
       (int)supply_volts / 1000,
       supply_volts % 1000,
       contactor_temperature_F,
       current_sense_8V_amps,
       accumulated_motor_on_time / 1000);
  print_buf(buf);
    
  snprintf(buf, sizeof(buf), "%3d %3d %3d %3d %3d %3d %3d %2d.%1d\n\r",
       dark,
       at_upper_position_limit,
       at_lower_position_limit,
       panels_going_up,
       panels_going_down,
       motor_cooling_off,
       motor_down_overcurrent | motor_up_overcurrent,
       (int)recent_max_wind_speed_knots,
       (int)(recent_max_wind_speed_knots * 10.0) % 10);
  print_buf(buf);
}


#define POSITION_HYSTERSIS (1)

/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/

void monitor_position_limits_callback()
{
  if (position_sensor_failed) {
    stop_driving_panels("failed pos sen");
  } else {
    if (panels_going_up && position_sensor_val > (calvals.position_upper_limit + POSITION_HYSTERSIS)) {
      stop_driving_panels("upper L");
    }
    if (panels_going_down && position_sensor_val <= (calvals.position_lower_limit - POSITION_HYSTERSIS)) {
      stop_driving_panels("lower L");
    }
  }
}

/*
 * When motor is engaged, ensure that the panels are moving.
 */
void monitor_motor_stall_callback()
{
  if (panels_going_up) {
    if (position_sensor_val <= last_position_sensor_val_stall) {
      stall_count++;
    } else {
      stall_count = 0;
    }
    
  } else if (panels_going_down) {
    if (position_sensor_val >= last_position_sensor_val_stall) {
      stall_count++;
    } else {
      stall_count = 0;
    }
  } else {
    fail("stall");
  }
  if (stall_count > 4) {
    stop_driving_panels("stall");
    daily_stalls++;
  }
  last_position_sensor_val_stall = position_sensor_val;
}

/*
 * This is called periodically and is never disabled.  It measures the voltage on the output
 * of an op-amp whose input is connected to a circuit board that is exposed to the sky.  The circuit board
 * has two sets of traces that are interleaved.  When rain falls on these traces, there is a tiny conduction
 * from the op-amp input circuit and the voltage on the op-amp output falls.  When the board is dry, the op-amp
 * output is about 3.3V.  When it is wet, it falls to as low as .5V.  
 * 
 * This monitor function measures the voltage and considers anything below 1.6V to be wet.  If the mode is not
 * in a stow mode and not in no-operation mode, then we drive the panels down.  If the voltage is above 1.6 V and we 
 * are in rain-stow mode, then we count 200 "beats" or calls to this function.  At 30 seconds per call, it will take 100
 * minutes.  After this, if the sensor is still dry, we exit rain-stow mode for either time-mode or position-mode.
 */

void monitor_rain_sensor_callback()
{
  static unsigned rain_stow_mode_delay;
  const float rain_threshold = 3.0;

  if (rain_sensor_volts < rain_threshold && !panels_going_up && !panels_going_down) {
    rain_stow_mode_delay = 0;
    if (calvals.operation_mode != no_panel_movement_mode && 
        calvals.operation_mode != rain_stow_mode && 
        calvals.operation_mode != wind_stow_mode) {
      
      Serial.println(F("# rain stow"));
      calvals.operation_mode = rain_stow_mode;
      if (!at_lower_position_limit) {  
        drive_panels_down();
      }
    }
    // The sensor is now dry, if we are in rain-stow mode, then start incrementing a counter
    // If that counter gets to some value, which indicates the sensor has been dry for a while, then
    // leave rain-stow mode and go into position mode if the position sensor seems to work or time mode if it does not.
  } else if (calvals.operation_mode == rain_stow_mode && rain_sensor_volts > rain_threshold) {
    rain_stow_mode_delay++;
    if (rain_stow_mode_delay >= 200) {
      calvals.operation_mode = position_mode;
    } 
  }
}

/*
 * This is called periodically to check the anenometer to see if it is measuring a wind speed above the limit
 * we have set.  If it finds such a wind, it will retract the panels and go into wind-stow-mode, much like
 * the logic for rain-stow-mode.  It will then wait for 720 calls with the wind being below the limit before it
 * goes back to time or position mode.  As this is called every 10 seconds, this means it will wait for two hours
 * of wind below the limit before allowing the panels to move out of the stow position.
 */
void monitor_wind_sensor_callback()
{
  static unsigned wind_stow_mode_delay;
  static unsigned high_wind_count;

   if (wind_speed_knots < 1.0) {  // If the wind is calm, start counting down
    high_wind_count = 0;
    if (calvals.operation_mode == wind_stow_mode) {
      wind_stow_mode_delay++;
      if (wind_stow_mode_delay > 3600 * 2 * 2) { // Wait two hours
        calvals.operation_mode = position_mode;
      }
    }
  } else if (wind_speed_knots >= WIND_SPEED_LIMIT) {
    if (high_wind_count < HIGH_WIND_THRESHOLD) {
      high_wind_count++;
    } else {
      if (!panels_going_up && !panels_going_down) {// Avoid panel movement when the panels are in motion
        wind_stow_mode_delay = 0;
        if (calvals.operation_mode != no_panel_movement_mode && 
            calvals.operation_mode != rain_stow_mode && 
            calvals.operation_mode != wind_stow_mode) {
          
          Serial.println(F("# wind stow"));
          calvals.operation_mode = wind_stow_mode;
          if (!at_lower_position_limit) {  
            drive_panels_down();
          }
        }
      } 
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
    if (panels_going_up && MOTOR_AMPS_UP_LIMIT < current_sense_8V_amps) {
      // Require two consecutive readings of overcurrent before tripping the overcurrent mechanism
      if (motor_up_overcurrent) {
        stop_driving_panels("A limit up");
        Serial.println(current_sense_8V_amps);
        calvals.operation_mode = no_panel_movement_mode;
      }
      motor_up_overcurrent = true;
    } else {
      motor_up_overcurrent = false;
    }
    if (panels_going_down && MOTOR_AMPS_DOWN_LIMIT < current_sense_8V_amps) {
      if (motor_down_overcurrent) {
        stop_driving_panels("A limit down");
        calvals.operation_mode = no_panel_movement_mode;
      }
      motor_down_overcurrent = true;
    } else {
      motor_down_overcurrent = false;
    }
  } else {
    // For every second the motor is on, let it cool for a number of (configurable) seconds.
    unsigned long delta = time_elapsed / ACCUMULATED_MOTOR_ON_TIME_AGING_RATE;
    if (accumulated_motor_on_time < delta) {
      accumulated_motor_on_time = 0;
    } else {
      accumulated_motor_on_time -= delta;
    }
  }
  if (motor_cooling_off == false) {
    bool motor_might_be_hot = accumulated_motor_on_time > ACCUMULATED_MOTOR_ON_TIME_LIMIT;
    if (motor_might_be_hot && (panels_going_up || panels_going_down)) {
      motor_cooling_off = true;
      stop_driving_panels("m limit");
    }
  } else if (accumulated_motor_on_time < (ACCUMULATED_MOTOR_ON_TIME_LIMIT - ON_TIME_HYSTERSIS)) {
    motor_cooling_off = false;
  }
}

/*
 * This global variable is set to the position value to which we are trying to raise the panels.
 */
int desired_position = 0;

void monitor_upward_moving_panels_and_stop_when_target_position_reached_callback()
{
  if (panels_going_up == false) {
    fail("MP");
  }
  if (position_sensor_val >= desired_position) {
    if (solar_volts < last_solar_volts) {
      stop_driving_panels("tpos1");
    } else if ((position_sensor_val >= (desired_position + 100))) {
      stop_driving_panels("tpos2");
    }
  } 
}

/*
   Check to see if the sun angle sensor is showing that the sun has move to be higher in the sky and so the suns rays are no
   longer normal to the panels (and sensor).  In this case, start raising the panels.

   Also check to see if it is now night time and we should lower the panels to their resting (down) position and start them moving
   down if that is so.
*/

void drive_panels_to_desired_position(void)
{ 
  int h = hour(arduino_time);
  int minutes_of_hour = minute(arduino_time);
  int raise_hour = 8;            // Hour to start raising panels (default for winter)
  int top_position_hour = 10;    // Hour at which panels should be in top position
  int lower_hour = 17;           // Hour at which considering lowering panels, wait for dark to lower them

  // Normally, I'd use data structures to define the times to raise and lower the panels.  However, our little Arduino processors has
  // 7,000 bytes of instruction memory free, but only 310 bytes of data free.  I don't know how much of that free memory is used
  // by the stack, but I don't think I can afford to make it any smaller.  So I am using a code-intensive approach in this function.

  switch (month(arduino_time)) {
    case 11: case 12: case 1:
      // Use defaults
      break;
      
    
    case 2: case 10:
      raise_hour = 9;
      top_position_hour = 12;
      lower_hour = 18;
      break;

    default:
      raise_hour = 10;
      top_position_hour = 13;
      lower_hour = 19;
      break;
  }
  if (raise_hour <= h && h < lower_hour && daily_stalls < 20) {
    
    int position_range = calvals.position_upper_limit - calvals.position_lower_limit;
    int minute_range = (top_position_hour - raise_hour) * 60;
    int minutes_from_start = (h - raise_hour) * 60 + minutes_of_hour;
    float percentage_of_position = (float)minutes_from_start / (float)minute_range;
    if (percentage_of_position > 1.0) {
      percentage_of_position = 1.0;
    }
    int desired_position_offset = (float)position_range * percentage_of_position;
    desired_position = calvals.position_lower_limit + desired_position_offset;
    if (desired_position > calvals.position_upper_limit) {
      desired_position = calvals.position_upper_limit;       // This should be redundant with check above
    }
    if (desired_position > (position_sensor_val + 10 /* hysterisis */)) {
      drive_panels_up();
      monitor_upward_moving_panels_and_stop_when_target_position_reached.enable();
    }
  } else if (lower_hour <= h && dark) {
    drive_panels_down();
  }
  // At midnight, reset the number of daily stalls
  if (h == 0 && daily_stalls > 0) {
    char buf[5];
    snprintf(buf, sizeof(buf), "#%3d", daily_stalls);
    print_buf(buf);
    daily_stalls = 0;
  }
}

void control_hydraulics_callback()
{

  if (panels_going_up || panels_going_down || motor_cooling_off || position_sensor_failed || millis() < 10000) {
    return;
  }

  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      break;

    case position_mode:
      if (time_of_day_valid) {
        if (position_sensor_failed) {
          fail("PS");
        }
        drive_panels_to_desired_position();
      }
      break;

    case rain_stow_mode:
    case wind_stow_mode:
      break;

    default:
      fail("?mode");
      break;
  }
}

/*
   This is called every four hours.  It will do something (maybe more than once) in the early hours.
*/
void monitor_cron_callback(void)
{
  /*
   * When the controller restarts, arduino_time will be just a few seconds and we will execute the body  
   * of this "if".  After that, in the wee hours, it will also executes.
   * The body copies the RTC to 'arduino_time' and it set up the cron string for the day
  */
  if (hour(arduino_time) < 6) {
    setup_time();
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
    fail("wc");
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

      Serial.print(F("#Error writing calvals to EEPROM starting at byte "));
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
    Serial.print(F("#Error reading CRC to EEPROM"));
  }
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
  if (crc_mismatch || eeprom_version_out_of_date || FORCE_CALVALS_RELOAD) {
    if (crc_mismatch) {
      Serial.println(F("#CRCs don't match or "));
    }
    if (eeprom_version_out_of_date) {
      Serial.println(F("#EEPROM calvals out of date"));
    }

    if (fix_if_checksum_fails) {
      Serial.println(F("#updating EERPOM with defaults"));
      write_calvals_to_eeprom();
    }
  } else {
    calvals = temp_calvals;
  }
}

int dst_correction(tmElements_t *tm)
{
  if (tm->Month > 3 && tm->Month < 11) {
     return 1;
  }
  if (tm->Month == 3 && tm->Day >= 11) {
    return 1;
  }
  if (tm->Month == 11 && tm->Day < 6) {
    return 1;
  }
  return 0;
}

/*
   Read the RTC chip and set the 'Arduino' time based on it.  We do this on system start and once per day.
   The Arduino clock is not as accurate as the RTC.
*/
void setup_time(void)
{
  tmElements_t tm;

  if (RTC.read(tm)) {
    setTime(tm.Hour + dst_correction(&tm), tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
    time_of_day_valid = true;
  } else {
    if (RTC.chipPresent()) {
      Serial.println(F("#The DS1307 is stopped.  Please run the SetTime"));
      Serial.println(F("#example to initialize the time and begin running."));
      Serial.println();
    } else {
      Serial.println(F("#DS1307 read error!  Please check the circuitry."));
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
  delay(1000); // In case something further along crashes and we restart quickly, this will give a one-second pause
  analogReference(DEFAULT);
  
  rs232.begin(rs232_baud);
  
  // Set the working calibration values to the defaults that are in this file
  set_calvals_to_defaults();

  Wire.begin();

  lcd.begin(16, 2);
  lcd.print(F("Suntracker 1.0"));
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);

  if (!quad_relay.begin()) {
    fail("REL");
  }
  quad_relay.turnAllRelaysOff();

 
#ifdef SETTIME
  setup_rtc();
#endif

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  Serial.print(F("\n\r#Suntracker "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));

  rs232.println(F("\n\r#Suntracker "));
  
// #define RELAY_TEST // For bench testing only
#ifdef RELAY_TEST
  digitalWrite(SSR_ENABLE_OUT, HIGH); 
  delay(1000);
  for (int relay = 1 ; relay <= 4 ; relay++) {
    
    if (quad_relay.getState(relay) != 0) {
      lcd.setCursor(0, 1);
      lcd.print(F("relay mismatch (not 0)"));
      Serial.println(F("relay mismatch (not 0)"));
    }
    Serial.println(F("turning on relay"));
    quad_relay.turnRelayOn(relay);
    if (quad_relay.getState(relay) != 1) {
      lcd.setCursor(0, 1);
      lcd.print(F("relay mismatch (not 1)"));
      Serial.println("relay mismatch (not 1)");
    }
    delay(10000);
    if (quad_relay.getState(relay) != 1) {
      lcd.setCursor(0, 1);
      lcd.print(F("relay mismatch (not 1)"));
      Serial.println(F("relay mismatch (not 1)"));
    }
    Serial.println(F("turning off relay"));
    quad_relay.turnRelayOff(relay);
    if (quad_relay.getState(relay) != 0) {
      lcd.setCursor(0, 1);
      lcd.print(F("relay mismatch (not 0)"));
      Serial.println("relay mismatch (not 0)");
    }
    delay(1000);
  }
  digitalWrite(SSR_ENABLE_OUT, LOW);
  
#endif
  // Try reading calibration values from EEPROM.  If that fails, write our default calibration values to EERPOM
  read_calvals_from_eeprom(true);
  backlight_timer = BACKLIGHT_ON_TIME;

  if (!ads.begin()) {
    fail("ADS");
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
  }

  /*
     Set a valid initial value for lots of globals.  We need this in order to get the tm structure set,
     s that we can set today's cron string.  We need to do that before running in time_mode.
  */
  read_time_and_sensor_inputs_callback();
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

    case position_mode:
      return("position");
      break;

    case rain_stow_mode:
      return("rain");
      break;

    case wind_stow_mode:
      return("wind");
      break;
      
    default:
      fail("??mode");
      break;
  }
}

const char *operation_mode_string(void);

//   If SETTIME is defined, we compile-in the code below to set the device's time to the build time.
#ifdef SETTIME
bool getTime(const char *str, tmElements_t *tm)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm->Hour = Hour;
  tm->Minute = Min;
  tm->Second = Sec;
  return true;
}

bool getDate(const char *str, tmElements_t *tm)
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
  tm->Day = Day;
  tm->Month = monthIndex + 1;
  tm->Year = CalendarYrToTm(Year);
  return true;
}
void setup_rtc()
{

  tmElements_t tm;

  bool parse = false;
  bool config = false;

  // get the date and time the compiler was run
  if (getDate(build_date, &tm) && getTime(build_time, &tm)) {
    parse = true;

    // The value we store into the RTC is the non-daylight-savings-time value
    if (dst_correction(&tm)) {
      if (tm.Hour == 0) {
        fail("T");
      }
      tm.Hour--;   
      tm.Second += 10; // Empirical: difference between build time and time to run this code
      if (tm.Second > 59) {
        tm.Minute++;
        if (tm.Minute > 59) {
          tm.Hour++;
          if (tm.Hour > 23) {
            tm.Day++;
          }
        }
      }
    }
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
#endif
