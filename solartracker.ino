
/*
   This controls three relays that control a hydraulic motor, a hydraulic valve, and a power supply.  The goal is
   to get more power out of four LG Neon-2 420 Watt panels than if they were fixed-mounted.  A secondary goal is to
   have them better looking.  If they were fixed-mounted, they'd need to be reversed-racked, which is not a great look.

    The core of the controls system is an Arduino: Sparkfun Redboard Qwiic 
    with the Sparkfun 4-relay Qwiic, a 1307-based RTC, a 2x16 line LCD from Adafruit, and a 4-input I2C ADC.

   Creative Commons Licence
   Robert Bedichek
*/

#include <string.h>  //Use the string Library
#include <ctype.h>
#include <EEPROM.h>
#include <assert.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <TimeLib.h>  // for update/display of time

/*
    This is for the LCD and button interface we added for local control and display.
*/
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#include <SoftwareSerial.h>
#include <Adafruit_RGBLCDShield.h>

// This string variable is used by multiple functions below, but not at the same time
char cbuf[55];

const bool let_panels_fall_without_power_global = true;
const unsigned long max_solenoid_on_time = 1800 * 1000UL;
const unsigned long max_solenoid_off_time = 2700 * 1000UL;
unsigned long solenoid_power_supply_on_time;  // Value of millis() when we last turned on the solenoid, 0 when off
unsigned long solenoid_power_supply_off_time; // Value of millis() when we last turned off thesolenoid, 0 when on


// The following is for the Sparkfun 4-relay board.  Relay numbers are 1..4.  Relay 1 is for "up", 3 is for "down".
// 2 is unused.  Relay 4 controls the connection to the rain sensor.  We leave it off most of the time, especially
// when it is wet, to avoid contamination from electrolysis.
// Relay 1, when activated, passes 12VDC when active to the go-up input of the contactor
// that controls the power to the hydraulic motor.  The second (relay 3) passes 12VDC when active to the go-down
// input of the contactor and to a solenoid in the check valve.  This check valve normally only allows
// hydraulic pressure to pass to the cycliner to make it go up.  When the hydraulic motor stops, we want
// the panels to retain their position.  Before we had this check valve, they would gradually drift back to
// the down position.  When we want the panels to lower, at the end of the day, we have to both release the
// check vavle by activating its solenoid and by power the hydralic motor in reverse, which is accomplished
// by driving 12VDC to the "go-down" input of the contactor.

#include "SparkFun_Qwiic_Relay.h"
#define RELAY_ADDR (0x6D)  // Default I2C address of the 4-relay board
Qwiic_Relay quad_relay(RELAY_ADDR);

// Relay numbers for moving the panels up and down.
#define RELAY_UP          (1)
#define RELAY_DOWN        (3)
#define RELAY_RAIN_SENSOR (4)

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

//    This is for the 1307 RTC
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

#define BACKLIGHT_OFF (0)
#define BACKLIGHT_WHITE (7)

/*
   Set this to set the time to the build time.  This is currently the only way to set time (let this be defined, compile, and
   run right away so that the compile-time time is pretty close to the actual time).
*/
const bool set_rtc_time_from_build_time = false;

/*
   Speed at which we run the USB serial connection 
*/
const unsigned long serial_baud = 115200;


//   Number of milliseconds to wait after boot and build display before starting operational part.
const int boot_delay_milliseconds = 2000;

//   Default maximum time the motor can be on continuously.  This is just the default value for when
//   the user resets to "factory defaults".  Units are milliseconds.
const unsigned long motor_max_on_time_default = 90000;

//  On-time hysteresis in milliseconds.  Once the motor has been running too long, past the max-motor-on-time limit,
//  we turn off until the on-time falls below the limit minus this vale.  This avoids the system turning on and off
//  rapidly when the limit is reached.
const int motor_on_time_hysterisis = 20000;

const unsigned long accumulated_motor_on_time_limit = 60000;
const int accumulated_motor_on_time_ageing_rate = 1;

const int position_hysteresis = 1;
unsigned low_current_count = 0;

const unsigned motor_current_threshold_low = 10;
const unsigned motor_current_threshold_high = 95;
bool panels_retracted;

//   We can operate in one of three modes.  The first is a kind of "off" mode where we stay alive, talk on the
//   serial port, update the LCD, but do not move the panels.   The two "on" modes are Sun-sensor mode and time-of-day/day-of-year mode.
enum mode_e { no_panel_movement_mode,
              position_mode,
              rain_stow_mode,
              wind_stow_mode,
              last_mode };

#define MOTOR_PS_SSR_ENABLE_PIN (7)  // Arduino output pin 7 on J4, writing '1' turns on 9V/100A power supply

#define SOLENOID_PS_SSR_ENABLE_PIN (8)

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

void set_arduino_time_from_rtc(void);

const char *operation_mode_string(void);

/*
    Central (and only) task scheduler data structure.
*/
Scheduler ts;

/*
   These are values that may need to be tweaked.  They used to be stored in EEPROM and back then,
   it was convenient to store them in a structure.
*/

struct calvals_s {
  unsigned position_upper_limit;  // Max position value (i.e., fully tilted up)
  unsigned position_lower_limit;
  float darkness_threshold;
  enum mode_e operation_mode;  // Mode to start in
} calvals;

const int motor_amps_down_limit = 40;
const int motor_amps_up_limit = 99;

const float low_wind_threshold = 6;
const float high_wind_threshold = 12;

#define BACKLIGHT_ON_TIME (3600)
#define INITIAL_OPERATION_MODE (mode_position)

/*
   These come from Arudino Analog input 1, which is driven by the center tap of two resistors that connect to the 12V line and
   ground.  We need this resistor divider network to move the 12V line to the range of the ADC, which is 0..5V.
*/
float supply_volts;  // Tap voltage converted to supply voltage

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is in the middle of the pack.
*/
float contactor_temperature_F;  // Degrees in Farenheight of temperature sensor in controller box

/*
   We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
   is unused.  Its current output goes to Arduino Analog input 3.
*/

float max_motor_amps = 0;  // Max recorded current since we started

float solar_volts;

float rain_sensor_volts;

/*
   This is from the 1000mm pull-string sensor that tells us where the panels are.
*/
float position_sensor_val;                // Raw ADC values 0..1023 for 0..5V
bool position_sensor_failed = false;      // Set true if the position sensor value is ever out of range
unsigned daily_stalls;                    // Zeroed at midnight, incremented on each motor stall
unsigned long stall_start_time;           // Incremented when no change in position while motor on
unsigned long under_current_start_time;
unsigned last_position_sensor_val;        // Position value at last call to status print
unsigned last_position_sensor_val_stall;  // Position sensor value at last sample in monitor_motor_stall_callback()

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
float wind_speed_knots = 0.0;             // In knots
float recent_max_wind_speed_knots = 0.0;  // Maximum recorded value since we started

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

Task display_status_on_lcd(TASK_SECOND, TASK_FOREVER, &display_status_on_lcd_callback, &ts, true);
Task monitor_buttons(100, TASK_FOREVER, &monitor_buttons_callback, &ts, true);
Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);
Task monitor_cron(TASK_SECOND * 3600 * 4, TASK_FOREVER, &monitor_cron_callback, &ts, true);
Task monitor_upward_moving_panels_and_stop_when_target_position_reached(TASK_SECOND / 10, TASK_FOREVER,
                                                                        &monitor_upward_moving_panels_and_stop_when_target_position_reached_callback, &ts, false);
Task monitor_rain_sensor(TASK_SECOND * 60, TASK_FOREVER, &monitor_rain_sensor_callback, &ts, true);
Task monitor_wind_sensor(TASK_SECOND * 5, TASK_FOREVER, &monitor_wind_sensor_callback, &ts, true);

void control_hydraulics_callback();
void monitor_position_limits_callback();
void monitor_stall_and_motor_current_callback();

/*
   These are the tasks that are the heart of the logic that controls this system.  Some run periodically and are always enabled.  Some run only
   when the panels are in motion.
*/

Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
Task control_hydraulics(TASK_SECOND * 10, TASK_FOREVER, &control_hydraulics_callback, &ts, true);

Task monitor_position_limits(50, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
Task monitor_stall_and_motor_current(200, TASK_FOREVER, &monitor_stall_and_motor_current_callback, &ts, false);

/*
   Keep track of whether we are driving the panels up or down.  These two should never be true at the same time.
*/
bool panels_going_up = false;
bool panels_going_down = false;

/*
   Set the calibration values to their "factory default".
*/
void set_calvals_to_defaults() {
  calvals.position_upper_limit = 340;  // Max position value (i.e., fully tilted up) minus overshoot (real max is around 363)
  calvals.position_lower_limit = 70;
  calvals.darkness_threshold = .5;
  calvals.operation_mode = position_mode;  // Mode in which we should start operation
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
void monitor_lcd_backlight_callback(void) {
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

enum val_to_display_e { vtd_none = 0,
                        vtd_max_9V_current,
                        vtd_temperature,
                        vtd_volts,
                        vtd_build_date,
                        vtd_build_time,
                        vtd_time,
                        vtd_mode,
                        vtd_position,
                        vtd_sun,
                        vtd_motor_on_time,
                        vtd_max_knots,
                        vtd_position_upper_limit,
                        vtd_position_lower_limit,
                        /* vtd_time_tilted_up_limit_in_minutes, */ vtd_darkness_threshold,
                        vtd_motor_amps_limit,
                        vtd_wind_speed_limit,
                        vtd_accumulated_motor_on_time_limit,
                        vtd_accumulated_motor_on_time_aging_rate,
                        vtd_last
} vtd_current = vtd_none;


void vtd_display_current(void) {
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
      bytes = lcd.print((int)position_sensor_val);
      break;

    case vtd_sun:
      lcd.print(F("sun sensor"));
      lcd.setCursor(0, 1);
      bytes = lcd.print((int)solar_volts);
      break;

    case vtd_volts:
      lcd.print(F("Supply Voltage  "));
      lcd.setCursor(0, 1);
      dtostrf(supply_volts, 3, 6, cbuf);
      bytes += lcd.print(cbuf);
      break;

    case vtd_temperature:
      lcd.print(F("Relay Temperat"));
      lcd.setCursor(0, 1);
      bytes = lcd.print((int)contactor_temperature_F) + 1;
      lcd.write('F');
      break;

    case vtd_max_9V_current:
      lcd.print(F("Max Motor Amps  "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(max_motor_amps);
      break;

    case vtd_motor_on_time:
      lcd.print(F("Motor-on time   "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(accumulated_motor_on_time);
      break;

    case vtd_max_knots:
      lcd.print(F("Max windspeed KT"));
      lcd.setCursor(0, 1);
      bytes = lcd.print((int)recent_max_wind_speed_knots);
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
      bytes = lcd.print(motor_amps_down_limit);
      bytes += lcd.print(F(" "));
      bytes += lcd.print(motor_amps_up_limit);
      break;

    case vtd_wind_speed_limit:
      lcd.print(F("Wind speed limit"));
      lcd.setCursor(0, 1);
    //  bytes = lcd.print(wind_speed_limit);
      break;

    case vtd_accumulated_motor_on_time_limit:
      lcd.print(F("Accum motor limit"));
      lcd.setCursor(0, 1);
      bytes = lcd.print(accumulated_motor_on_time_limit / 1000);
      break;

    case vtd_accumulated_motor_on_time_aging_rate:
      lcd.print(F("Accum aging rate "));
      lcd.setCursor(0, 1);
      bytes = lcd.print(accumulated_motor_on_time_ageing_rate);
      break;

    default:
      fail(F("vtd"));
      break;
  }
  while (bytes < 16) {
    bytes += lcd.write(' ');
  }
}
/*
   Called every 100msec to monitor the buttons that are below the LCD.
*/
void monitor_buttons_callback() {
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

unsigned long time_of_last_use_of_motor_power_supply;

void turn_on_motor_power_supply(void) 
{
  unsigned long current_second = millis() / 1000;

  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(MOTOR_PS_SSR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, HIGH);
  time_of_last_use_of_motor_power_supply = current_second;
}

void turn_off_motor_power_supply_if_idle(void) 
{
  unsigned long current_second = millis() / 1000;

  if ((current_second - time_of_last_use_of_motor_power_supply) > 600) {
    // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
    // input.  That allows us to drive it high with a power-supply over ride switch without having that
    // conflict with the ATMega output driver.
    pinMode(MOTOR_PS_SSR_ENABLE_PIN, INPUT);
    digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, LOW);  // We write the bit with a zero ust so we can query later to see the state 
  }
}


void turn_on_solenoid_power_supply(void) 
{
  if (digitalRead(SOLENOID_PS_SSR_ENABLE_PIN) == LOW) {
    pinMode(SOLENOID_PS_SSR_ENABLE_PIN, OUTPUT);
    digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, HIGH);
    delay(2000);  // Wait for supply to develop power
    Serial.println(F("# solenoid power on"));
  }
  solenoid_power_supply_on_time = millis();
  solenoid_power_supply_off_time = 0;
}

void turn_off_solenoid_power_supply(void) 
{
  if (digitalRead(SOLENOID_PS_SSR_ENABLE_PIN) == HIGH) {
    // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
    // input.  That allows us to drive it high with a power-supply over ride switch without having that
    // conflict with the ATMega output driver.
    pinMode(SOLENOID_PS_SSR_ENABLE_PIN, INPUT);
    digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, LOW);  // Just so we can query later
    solenoid_power_supply_on_time = 0;
    solenoid_power_supply_off_time = millis();
    Serial.println(F("# solenoid power off"));
  }
}

/*
    Turn off both relays that drive 12VDC to the contactor inputs.  Since we aren't driving
    the panels, we also disable the task that monitors position and estimated temperature limits
*/
void stop_driving_panels(const __FlashStringHelper *who_called) 
{
  Serial.print(F("# stop_driving_panels(), called by "));
  Serial.println(who_called);

  quad_relay.turnRelayOff(RELAY_UP);
  quad_relay.turnRelayOff(RELAY_DOWN);
  panels_going_down = false;
  panels_going_up = false;

  /*
   * We were called by one of these.  Ensure they are all disabled so we don't get called twice.
   */
  monitor_position_limits.disable();
  monitor_stall_and_motor_current.disable();
  monitor_upward_moving_panels_and_stop_when_target_position_reached.disable();
  low_current_count = 0;
  lcd.setCursor(0, 1);
  lcd.print(F("Stopped "));
  lcd.print((int)position_sensor_val);
  vtd_timeout = 10;
  turn_off_solenoid_power_supply();
}

/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const __FlashStringHelper *fail_message) {
  stop_driving_panels(F("fail"));
  Serial.println(fail_message);

  lcd.setCursor(0, 0);
  lcd.print(fail_message);

  delay(500);  // Give the serial link time to propogate the error message before execution ends
  abort();
}

/*
   Start the panels moving up and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going down when this was called.
*/
void drive_panels_up(void) 
{
  if (panels_going_down) {
    fail(F("drive_panels_up()"));
  } else {
    Serial.println(F("# lift "));
    quad_relay.turnRelayOn(RELAY_UP);
    turn_on_motor_power_supply();
    lcd.setCursor(0, 1);
    lcd.print(F("Going up "));
    vtd_timeout = 10;
    panels_going_up = true;
    stall_start_time = 0;
    under_current_start_time = 0;
    panels_retracted = false;
    monitor_position_limits.enable();
    monitor_stall_and_motor_current.enable();
  }
}

/*
   Start the panels moving down and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going up when this was called.
*/
void drive_panels_down(const __FlashStringHelper *why, bool let_panels_fall_without_power) 
{
  if (at_lower_position_limit || panels_retracted) {
    return;
  } else if (panels_going_up) {
    fail(F("drive_panels_down"));
  } else {
    turn_on_solenoid_power_supply(); // Release check valve so that panels can go down
    Serial.print(F("# retract: "));
    Serial.println(why);
    lcd.setCursor(0, 1);
    lcd.print(F("Going down "));
    vtd_timeout = 10;
    if (let_panels_fall_without_power == false) {
      quad_relay.turnRelayOn(RELAY_DOWN);
      turn_on_motor_power_supply();
      monitor_stall_and_motor_current.enable();
    }
    
    panels_going_down = true;
    stall_start_time = 0;
    under_current_start_time = 0;
    panels_retracted = true;
    monitor_position_limits.enable();
  }
}

/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback() 
{
  arduino_time = now();
  const float ema_alpha = 0.5;
  float alpha;
  const int samples = 10;  // # of samples in arithmetic average
  float supply_volts_temp = 0;
  float position_sensor_val_temp = 0;
  float contactor_temperature_F_temp = 0;
  float wind_speed_knots_temp = 0;
  float rain_sensor_volts_temp = 0;
  float solar_volts_temp = 0;
  static bool first_time = true;

  if (first_time) {
    alpha = 1.0;
    first_time = false;
  } else {
    alpha = ema_alpha;
  }

  for (int sample = 0; sample < samples; sample++) {
    unsigned supply_tap_volts_raw = analogRead(/* Arduino analog input 1 */ SUPPLY_VOLTAGE_IN);

    /*
      The value by which we multiple the raw value, and the value we add or subtract after that are determined
      first by calculation based on the measured values of the resistors and then fine tuned with measurements
      with 5.5 digit multimeter (Rigol DM3058).
    */
    float supply_tap_volts = supply_tap_volts_raw * 5.0 / 1023.0;
    supply_volts_temp += supply_tap_volts * 3.266;

    unsigned contactor_temperature_volts_raw = analogRead(/* Arduino analong input */ TMP36_IN);
    float contactor_temperature_millivolts = contactor_temperature_volts_raw * 5000.0 / 1023.0 + 20 /* Calibration value */;
    float contactor_temperature_C = (contactor_temperature_millivolts - 500.0) / 10.0;
    contactor_temperature_F_temp += contactor_temperature_C * 9.0 / 5.0 + 32.0;

    int wind_speed_raw = ads.readADC_SingleEnded(0);
    float wind_speed_volts = ads.computeVolts(wind_speed_raw);

    /*
      I measured 0.41556V at zero windspeed.  From the Adafruit web site, the anenometer is supposed to generate
      a 2V signal at 32.2 meter/sec (or 62.9 knots) and 0.4V at 0 meters/sec.  62.9/(2.0 - 0.4155) = 39.69
    */
    float knots = (wind_speed_volts - 0.4155) * 39.69;
    if (knots > 0) {
      wind_speed_knots_temp += knots;
    }

        // Input 2 (the four inputs are numbered 0, 1, 2, 3) is unused
    //  _sensor_raw = ads.readADC_SingleEnded(/* ADS1115 input */2);
    //  _sensor_volts = ads.computeVolts(_sensor_raw);

    /*
        The values we read for the sun sensor and position sensors jump around, I guess due to noise.  To compensate and
        have more stable values average the last reading with this reading (and the 'last reading' is a running average)
    */
    int solar_raw = ads.readADC_SingleEnded(/* ADS1115 input */ 3);
    solar_volts_temp += ads.computeVolts(solar_raw);

    unsigned position_sensor_raw = (unsigned)analogRead(DRAW_STRING_IN);
    position_sensor_val_temp += position_sensor_raw;
  }

  position_sensor_val_temp /= samples;
  position_sensor_val = alpha * position_sensor_val_temp + (1 - alpha) * position_sensor_val;

  wind_speed_knots_temp /= samples;
  wind_speed_knots = alpha * wind_speed_knots_temp + (1 - alpha) * wind_speed_knots;

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

  supply_volts_temp /= samples;
  supply_volts = supply_volts_temp;  // Don't do EMA on this, we want to see spikes

  contactor_temperature_F_temp /= samples;
  contactor_temperature_F = alpha * contactor_temperature_F_temp + (1 - alpha) * contactor_temperature_F;

  solar_volts_temp /= samples;
  solar_volts = alpha * solar_volts_temp + (1 - alpha) * solar_volts;

  dark = solar_volts <= calvals.darkness_threshold;
  at_upper_position_limit = position_sensor_val >= calvals.position_upper_limit;
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;  // Panels are at a good lower position when position_sensor_val is 50
}

int motor_amps(void) 
{
  unsigned motor_current_sense_volts_raw = analogRead(/* Arduino analog input 3 */ CURRENT_SENSE_IN);
  const bool verbose = false;

  if (verbose) {
    Serial.print(F("# current sense raw="));
    Serial.println(motor_current_sense_volts_raw);
  }
  /* Empirical calibration: the raw value seems to never be less than four, even with no current */
  if (motor_current_sense_volts_raw >= 4) {
    motor_current_sense_volts_raw -= 4;
  }
  float motor_current_sense_millivolts = motor_current_sense_volts_raw * 5000.0 / 1023.0;
  if (verbose) {
    Serial.print(F("# current sense millivolts="));
    Serial.println((int)motor_current_sense_millivolts);
  }
  return (int)(motor_current_sense_millivolts / 50.0); // 50 mV/A is what I measured
}

void display_status_on_lcd_callback() 
{
  if (vtd_timeout == 0) {
    lcd.setCursor(0, 0);
    int bytes = lcd.print((int)position_sensor_val);
    dtostrf(supply_volts, 7, 3, cbuf);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(cbuf);
    bytes += lcd.print(F(" "));
    bytes += lcd.print(max_motor_amps);
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
   This is the main system tracing function.  It emits a line of ASCII to USB serial line with lots of information.
   We display this information in a form that plotyJS can readily absorb.

  2025-04-06 01:01:07 1   58    0    0    0  328  11.534   0   0   1   0   1   0   0   0   0  0.0

*/

// Called periodically.  Sends relevant telemetry back over one or both of the serial channels
void print_status_to_serial_callback(void) 
{
  static char line_counter = 0;
  static enum mode_e last_operation_mode = last_mode;  // Force a difference the first time
  static float last_solar_volts;
  static bool last_at_upper_position_limit;
  static bool last_at_lower_position_limit;
  static bool last_panels_going_up;
  static bool last_panels_going_down;
  static unsigned skipped_record_counter;
  static float last_recent_max_wind_speed_knots;

  int position_difference;  // Amount position changed since last call to status print
  if (last_position_sensor_val == 0) {
    last_position_sensor_val = position_sensor_val;
  }
  position_difference = last_position_sensor_val - (int)position_sensor_val;

  if (abs(position_difference) > 5 || 
     last_operation_mode != calvals.operation_mode || 
     abs(last_solar_volts - solar_volts) > 0.1 || 
     last_at_upper_position_limit != at_upper_position_limit || 
     last_at_lower_position_limit != at_lower_position_limit || 
     last_panels_going_up != panels_going_up || 
     last_panels_going_down != panels_going_down || 
     abs(recent_max_wind_speed_knots - last_recent_max_wind_speed_knots) > 1 || 
     skipped_record_counter++ > 1200) {

    last_position_sensor_val = position_sensor_val;
    last_operation_mode = calvals.operation_mode;
    last_solar_volts = solar_volts;
    last_at_upper_position_limit = at_upper_position_limit;
    last_at_lower_position_limit = at_lower_position_limit;
    last_panels_going_up = panels_going_up;
    last_panels_going_down = panels_going_down;
    last_recent_max_wind_speed_knots = recent_max_wind_speed_knots;
    skipped_record_counter = 0;

    if (line_counter == 0) {
      Serial.println(F("# Date     Time     Md  Pos  Dif  Sol Delt Rain  Volts Tmp Mot Drk UpL LwL GUp GDn CoL OvC  Knots"));
      line_counter = 20;
    } else {
      line_counter--;
    }

    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
             year(arduino_time),
             month(arduino_time),
             day(arduino_time),
             hour(arduino_time),
             minute(arduino_time),
             second(arduino_time));
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), "%d %4d %4d %4d %4d",
             calvals.operation_mode,
             (int)position_sensor_val,
             position_difference,
             (int)(solar_volts * 100.0),
             (int)(solar_volts - last_solar_volts) * 1000.0);

    Serial.print(cbuf);
    {
      char supply_volts_str[8];
      dtostrf(supply_volts, 7, 3, supply_volts_str);
      snprintf(cbuf, sizeof(cbuf), " %4d %s %3d %3d ",
               (int)(rain_sensor_volts * 100.0),
               supply_volts_str,
               (int)contactor_temperature_F,
               accumulated_motor_on_time / 1000);
    }
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), "%3d %3d %3d %3d %3d %3d %3d %2d.%1d",
             dark,
             at_upper_position_limit,
             at_lower_position_limit,
             panels_going_up,
             panels_going_down,
             motor_cooling_off,
             motor_down_overcurrent | motor_up_overcurrent,
             (int)recent_max_wind_speed_knots,
             (int)(recent_max_wind_speed_knots * 10.0) % 10);
    Serial.println(cbuf);
  }
}


/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/
void monitor_position_limits_callback() 
{
  if (position_sensor_failed) {
    stop_driving_panels(F("failed position sensor"));
  } else {
    if (panels_going_up && position_sensor_val > (calvals.position_upper_limit + position_hysteresis)) {
      stop_driving_panels(F("upper limit reached"));
    }
    if (panels_going_down) {
      if (position_sensor_val <= (calvals.position_lower_limit - position_hysteresis)) {
        stop_driving_panels(F("lower limit reached"));
      }
      if (let_panels_fall_without_power_global) {
        if (hour(arduino_time) == 8) {
          // By 8AM, give up letting the panels fall so that we can reset global flags and be ready to start raising the panels
          stop_driving_panels(F("letting panels fall"));
        }
        // Turn the solenoid on and off until the panels reach the lower limit or 8AM rolls around
        if (solenoid_power_supply_on_time != 0 && (millis() - solenoid_power_supply_on_time) > max_solenoid_on_time) {
          turn_off_solenoid_power_supply();
        }
        if (solenoid_power_supply_off_time != 0 && (millis() - solenoid_power_supply_off_time) > max_solenoid_off_time) {
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
  // First check to see that the panels are moving (and thus not stalled)
  unsigned long now = millis();
  if (panels_going_up) {
    if (position_sensor_val <= last_position_sensor_val_stall) {
      if (stall_start_time == 0) {
        stall_start_time = now;
      } else if ((now - stall_start_time) > 500) {
        stop_driving_panels(F("motor stall going up"));
        daily_stalls++;
        if (daily_stalls > 2) {
          calvals.position_upper_limit -= 5;
          Serial.println(F("# alert decreasing upper limit due to stall going up"));
        }
      }
    } else {
      stall_start_time = 0; // Panels are moving up, reset the time of last stall start
    }
  } else if (panels_going_down) {
    if (position_sensor_val >= last_position_sensor_val_stall) {
      if (stall_start_time == 0) {
        stall_start_time = now;
      } else if ((now - stall_start_time) > 500) {
        stop_driving_panels(F("motor stall going down"));
        daily_stalls++;
        if (daily_stalls > 2) {
          calvals.position_lower_limit += 5;
          Serial.println(F("# alert increasing lower limit due to stall going down more than 2 times"));
        }
      }
    } else {
      stall_start_time = 0; // Panels are moving down, reset the time of last stall start
    }
  } else {
    fail(F("stall yet panels not in motion"));
  }
  // Now check to see if the hydraulic motor is taking too little or too much current
  int amps = motor_amps();
  char vbuf[10];
  dtostrf(supply_volts, 6, 3, vbuf);

  snprintf(cbuf, sizeof(cbuf), "# %02u:%02u:%02u position=%u amps=%d supply_volts=%s ",
             hour(arduino_time),
             minute(arduino_time),
             second(arduino_time),
             (unsigned)position_sensor_val,
             amps,
            vbuf);
  
  Serial.println(cbuf);
  if (amps < motor_current_threshold_low) {
    unsigned long now = millis();
    if (under_current_start_time == 0) {
      under_current_start_time = now;
    } else if ((now - under_current_start_time) > 9000) {
      stop_driving_panels(F("low current for 9 seconds"));
    }
  } else {
    under_current_start_time = 0; // Not taking too little, reset the start time of undercurrent

    // If the motor is taking too much current, stop it immediately
    if (amps > motor_current_threshold_high) {
      stop_driving_panels(F("# alert high current"));
    }
  }
  last_position_sensor_val_stall = position_sensor_val;
}

const float rain_threshold = 3.0;  // Below this voltage, we say it is raining

// Returns true if the rain sensor is conducting enough.  This will turn on the
// rain sensor if it is off.  We seek to only turn it on if we are going to make
// a decision to raise the panels or if we are waiting to see if it has
// stopped raining.  There is a separate function to explicitly turn off
// the rain sensor.

bool is_raining(void)
{
  if (quad_relay.getState(RELAY_RAIN_SENSOR) == false) {
    quad_relay.turnRelayOn(RELAY_RAIN_SENSOR);
    delay(100); // Hopefully the relay will close in less than 100 milliseconds
    Serial.println(F("# rain sensor on"));
  }

  // Take ar number of samples to get a better estimate of actual voltage.
  unsigned long rain_sensor_raw = 0;
  const int samples = 10;
  for (int i = 0 ; i < samples ; i++) {
   rain_sensor_raw += ads.readADC_SingleEnded(/* ADS1115 input */ 1);
  }
  rain_sensor_volts = ads.computeVolts(rain_sensor_raw / samples);

  return rain_sensor_volts < rain_threshold;
}

void turn_off_rain_sensor(void)
{
  if (quad_relay.getState(RELAY_RAIN_SENSOR)) {
     quad_relay.turnRelayOff(RELAY_RAIN_SENSOR);
    Serial.println(F("# rain sensor off"));
  }
}

int panel_movement_start_hour(void);
int panel_movement_end_hour(void);

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
  int h = hour(arduino_time);
  bool panel_movement_time = panel_movement_start_hour() <= h && h <= panel_movement_end_hour();
  if (panel_movement_time &&
      !panels_going_up && 
      !panels_going_down && 
      calvals.operation_mode != no_panel_movement_mode && 
      calvals.operation_mode != rain_stow_mode && 
      calvals.operation_mode != wind_stow_mode &&
      is_raining()) {

    turn_off_rain_sensor(); // Now that we know it is raining, preserve the contacts by depowering it for an hour
    Serial.println(F("# alert rain stow"));
    calvals.operation_mode = rain_stow_mode;
    if (!at_lower_position_limit) {
      drive_panels_down(F("rain-stow"), false);
    }
    monitor_rain_sensor.setInterval(60UL * 60UL * 1000UL); // 60 minutes so that we don't turn the rain sensor on too often

    // The sensor is now dry, if we are in rain-stow mode, then start incrementing a counter
    // If that counter gets to some value, which indicates the sensor has been dry for a while, then
    // leave rain-stow mode and go into position mode if the position sensor seems to work or time mode if it does not.
  } else if (calvals.operation_mode == rain_stow_mode) {
    static unsigned rain_stopped_time = 0;
    bool raining = is_raining();
    turn_off_rain_sensor();
    if (raining) {
      rain_stopped_time = 0;     // It is still raining, keep this set to zero.
    } else {
      unsigned long current_second = millis() / 1000UL;
      
      if (rain_stopped_time == 0) {
        rain_stopped_time = current_second;
      } else {
        if ((current_second - rain_stopped_time) > 7200UL) {
          Serial.println(F("# alert leaving rain-stow mode, resuming normal operation"));
          // The rain stopped two hours ago, leave rain-stow mode.
          calvals.operation_mode = position_mode;
          rain_stopped_time = 0;
          monitor_rain_sensor.setInterval(30 * 1000UL); // Back to normal checking for rain every 30 seconds
          if (!panel_movement_time) {
            monitor_rain_sensor.disable();
          }
        }
      }
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

  if (wind_speed_knots < low_wind_threshold) {  // If the wind is calm, start counting down
    high_wind_count = 0;
    if (calvals.operation_mode == wind_stow_mode) {
      wind_stow_mode_delay++;
      if (wind_stow_mode_delay > 3600 * 2 / 5) {  // Wait two hours
        Serial.println(F("# alert leaving wind-stow mode"));
        calvals.operation_mode = position_mode;
      }
    }
  } else if (wind_speed_knots >= high_wind_threshold) {
    if (high_wind_count < 7) { // Must see this high wind this many times before we go into wind-stow mode
      high_wind_count++;
    } else {
      if (!panels_going_up && !panels_going_down) {  // Avoid panel movement when the panels are in motion
        wind_stow_mode_delay = 0;
        if (calvals.operation_mode != no_panel_movement_mode && 
            calvals.operation_mode != rain_stow_mode && 
            calvals.operation_mode != wind_stow_mode) {

          Serial.print(F("# alert wind-stow knots="));
          Serial.print((int)wind_speed_knots);
          calvals.operation_mode = wind_stow_mode;
          if (!at_lower_position_limit) {
            drive_panels_down(F("wind-stow"), false);
          }
        }
      }
    }
  }
}

/*
 * This global variable is set to the position value to which we are trying to raise the panels.
 */
int desired_position = 0;

void monitor_upward_moving_panels_and_stop_when_target_position_reached_callback() 
{
  if (panels_going_up == false) {
    fail(F("MP"));
  }
  if (position_sensor_val >= desired_position) {
    stop_driving_panels(F("target position reached"));
  }
}

/*
   Check to see if the sun angle sensor is showing that the sun has move to be higher in the sky and so the suns rays are no
   longer normal to the panels (and sensor).  In this case, start raising the panels.

   Also check to see if it is now night time and we should lower the panels to their resting (down) position and start them moving
   down if that is so.
*/

int panel_movement_start_hour(void)
{
  int h = hour(arduino_time);
  int minutes_of_hour = minute(arduino_time);
  int raise_hour = 8;          // Hour to start raising panels (default for winter)

  switch (month(arduino_time)) {
    case 11:
    case 12:
    case 1:
      // Use defaults
      break;

    case 2:
    case 10:
      raise_hour = 9;
      break;

    default:
      raise_hour = 10;
      break;
  }
  return raise_hour;
}

int panel_movement_end_hour(void)
{
  int lower_hour = 17;         // Hour at which considering lowering panels, wait for dark to lower them

  // Normally, I'd use data structures to define the times to raise and lower the panels.  However, our little Arduino processors has
  // 7,000 bytes of instruction memory free, but only 310 bytes of data free.  I don't know how much of that free memory is used
  // by the stack, but I don't think I can afford to make it any smaller.  So I am using a code-intensive approach in this function.

  switch (month(arduino_time)) {
    case 11:
    case 12:
    case 1:
      // Use defaults
      break;

    case 2:
    case 10:
      lower_hour = 18;
      break;

    default:
      lower_hour = 19;
      break;
  }
  return lower_hour;
}

void drive_panels_to_desired_position(void) 
{
  int h = hour(arduino_time);
  int minutes_of_hour = minute(arduino_time);
  int raise_hour = panel_movement_start_hour();          // Hour to start raising panels (default for winter)
  int top_position_hour = 10;  // Hour at which panels should be in top position
  int lower_hour = panel_movement_end_hour();         // Hour at which considering lowering panels, wait for dark to lower them

  // Normally, I'd use data structures to define the times to raise and lower the panels.  However, our little Arduino processors has
  // 7,000 bytes of instruction memory free, but only 310 bytes of data free.  I don't know how much of that free memory is used
  // by the stack, but I don't think I can afford to make it any smaller.  So I am using a code-intensive approach in this function.

  switch (month(arduino_time)) {
    case 11:
    case 12:
    case 1:
      // Use defaults
      break;

    case 2:
    case 10:
      top_position_hour = 12;
      break;

    default:
      top_position_hour = 13;
      break;
  }
  if (raise_hour <= h && h < lower_hour && daily_stalls < 5) {
    monitor_rain_sensor.enable();
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
      desired_position = calvals.position_upper_limit;  // This should be redundant with check above
    }
    if (desired_position > (position_sensor_val + 10 /* hysterisis */)) {
      monitor_rain_sensor.enable();
      if (is_raining() == false) {
        drive_panels_up();
        monitor_upward_moving_panels_and_stop_when_target_position_reached.enable();
      }
    }
  } else if (lower_hour <= h && dark) {
    turn_off_rain_sensor();  // With panels all the way down, no need to monitor for rain
    monitor_rain_sensor.disable();
    drive_panels_down(F("stowing for night"), let_panels_fall_without_power_global);
  }
  // At midnight, reset the number of daily stalls
  if (h == 0 && daily_stalls > 0) {
    snprintf(cbuf, sizeof(cbuf), "#%3d", daily_stalls);
    Serial.print(cbuf);
    daily_stalls = 0;
  }
}

void control_hydraulics_callback() 
{
  turn_off_motor_power_supply_if_idle();

  if (panels_going_up || panels_going_down || motor_cooling_off || position_sensor_failed || millis() < 10000) {
    return;
  }

  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      break;

    case position_mode:
      if (time_of_day_valid) {
        if (position_sensor_failed) {
          fail(F("PS"));
        }
        drive_panels_to_desired_position();
      }
      break;

    case rain_stow_mode:
    case wind_stow_mode:
      break;

    default:
      fail(F("?mode"));
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
    set_arduino_time_from_rtc();
  }
}

int dst_correction(tmElements_t *tm) {
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
void set_arduino_time_from_rtc(void) 
{
  tmElements_t tm;

  if (RTC.read(tm)) {
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
             tmYearToCalendar(tm.Year),
             tm.Month,
             tm.Day,
             tm.Hour,
             tm.Minute,
             tm.Second);

    setTime(tm.Hour + dst_correction(&tm), tm.Minute, tm.Second, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
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
  delay(1000);  // In case something further along crashes and we restart quickly, this will give a one-second pause
  analogReference(DEFAULT);

  // Set the working calibration values to the defaults that are in this file
  set_calvals_to_defaults();

  Wire.begin();

  lcd.begin(16, 2);
  lcd.print(F("reboot SolarTracker"));
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);

  if (!quad_relay.begin()) {
    fail(F("REL"));
  }
  quad_relay.turnAllRelaysOff();

  Serial.begin(serial_baud);
  UCSR0A = UCSR0A | (1 << TXC0);  //Clear Transmit Complete Flag

  if (set_rtc_time_from_build_time) {
    set_rtc_from_build_date_and_time();
  }
  set_arduino_time_from_rtc();

  Serial.print(F("\n\r# reboot SolarTracker "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));

  backlight_timer = BACKLIGHT_ON_TIME;

  if (!ads.begin()) {
    fail(F("ADS"));
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

const char *operation_mode_string(void) 
{
  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      return ("no-panel-movement");
      break;

    case position_mode:
      return ("position");
      break;

    case rain_stow_mode:
      return ("rain");
      break;

    case wind_stow_mode:
      return ("wind");
      break;

    default:
      fail(F("??mode"));
      break;
  }
}

const char *operation_mode_string(void);

//  These functions are only called if set_rtc_from_build_time is defined, but we always compile it to reduce the chance that
// it suffers bit rot.
bool getTime(const char *str, tmElements_t *tm) 
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) {
    return false;
  }
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

void set_rtc_from_build_date_and_time() 
{
  tmElements_t tm;

  bool parse = false;
  bool config = false;

  // get the date and time the compiler was run
  if (getDate(__DATE__, &tm) && getTime(__TIME__, &tm)) {
    parse = true;

    // The value we store into the RTC is the non-daylight-savings-time value
    if (dst_correction(&tm)) {
      if (tm.Hour == 0) {
        fail(F("set_rtc_from_build_date_and_time()"));
      }
      tm.Hour--;
      tm.Second += 10;  // Empirical: difference between build time and time to run this code
      if (tm.Second > 59) {
        tm.Second -= 60;
        tm.Minute++;
        if (tm.Minute > 59) {
          tm.Minute -= 60;
          tm.Hour++;
          if (tm.Hour > 23) {
            tm.Hour -= 24;
            tm.Day++;
          }
        }
      }
    }

    // and configure the RTC with this info
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
             tmYearToCalendar(tm.Year),
             tm.Month,
             tm.Day,
             tm.Hour,
             tm.Minute,
             tm.Second);


    if (RTC.write(tm)) {
      if (RTC.read(tm)) {
        snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
                 tmYearToCalendar(tm.Year),
                 tm.Month,
                 tm.Day,
                 tm.Hour,
                 tm.Minute,
                 tm.Second);
        config = true;
      }
    }
  }

  if (parse && config) {
      // Normal path
  } else if (parse) {
    fail(F("# alert DS1307"));

  } else {
    fail(F("# alert time string"));
  }
}
