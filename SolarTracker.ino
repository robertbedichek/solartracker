

/*
 * This controls three relays that control a hydraulic motor, a hydraulic valve, and a power supply.  The goal is 
 * to get more power out of four LG Neon-2 420 Watt panels than if they were fixed-mounted.  A secondary goal is to
 * have them better looking.  If they were fixed-mounted, they'd need to be reversed-racked, which is not a great look.
 * 
 * The core of the controls system is an Arduino:
 * KTA-223v3 by Ocean Controls www.oceancontrols.com.au
 */

#include <string.h> //Use the string Library
#include <ctype.h>
#include <EEPROM.h>
#include <assert.h>

/*
 * This is for the LCD and button interface we added for local control and display.
 */
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

/*
 * This is for the 1307 RTC we installed on the Ocean Controls main board.
 */
#include <TimeLib.h>
#include <DS1307RTC.h>

/*
 * Set this to set the time to the build time.  This is currently the only way to set time (let this be defined, compile, and
 * run right away so that the compile-time time is pretty close to the actual time).
 */
// #define SETTIME

/*
 * Speed at which we run the serial connection (both via USB and RS-485)
 */
#define SERIAL_BAUD (38400)

/* 
 *  This controls how much output we will emit on the RS-485 line.  
 *    0: only events are reported
 *    1: status messages are emitted every three seconds
 *    2: all messages are enabled
 */
#define VERBOSE_NONE   (0)
#define VERBOSE_MEDIUM (1)
#define VERBOSE_ALL    (2)

int verbose = VERBOSE_MEDIUM;

/*
 * Number of milliseconds to wait after boot and build display before starting operational part.
 */
#define INITIAL_DELAY (2000)

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <TimeLib.h>     // for update/display of time

/*
 * The inputs and outputs are thus:
 *  KTA Analog 1, which is Arduino Analog In 6, measures the voltage on the lower cell in the sun angle senseor
 *  KTA Analog 2, which is Arudino Analog In 7, measures the voltage on the upper cell in the sun angle sensor
 *  KTA Analog 3, which is Arudino Analog In 0, measures the voltage from the draw-string position sensor  
 */

#define ANIN1 6   // Analog 1 is connected to Arduino Analog In 6
#define ANIN2 7   // Analog 2 is connected to Arduino Analog In 7
#define ANIN3 0   // Analog 3 is connected to Arduino Analog In 0

/*
 *  These are the only analog inputs that the KTA-223 has on the outside of its shell.  However, I needed more,
 *  so I have repurposed the opto-isolator inputs by removing the pull-up resistors that connect the outputs
 *  of the KTA-223 opto-isolators to the Arudino inputs.  I have wired directly to the Arudino board thusly:
 *  */

/* To monitor the supply voltage I used Arduino Analog 1. We have a resistor divider with a 68.284k Ohm resistor 
 * in series with a 32.823k Ohm resistor. The the center tap is connected to A1 (analog input 1), the other 
 * leads are connected to the supply voltage and the ground.  This input had been driven by an opto isolator with a 
 * 4.7 k Ohm pull up.  We removed the pull up.
 */
#define ANIN4 1   

/* 
 *  We needed another analog input to monitor battery temperature, so we used Arduino Analog 2.
 * It is connected to a TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
 */
#define ANIN5 2

// And for the output of a current sensor on the 4V line we use Arduino analog 3
#define ANIN6 3

/*
 * We use three relay outputs.  The first passes 12VDC when active to the go-up input of the contactor
 * that controls the power to the hydraulic motor.  The second passes 12VDC when active to the go-down
 * input of the contactor and to a solenoid in the check valve.  This check valve normally only allows
 * hydraulic pressure to pass to the cycliner to make it go up.  When the hydraulic motor stops, we want
 * the panels to retain their position.  Before we had this check valve, they would gradually drift back to
 * the down position.  When we want the panels to lower, at the end of the day, we have to both release the
 * check vavle by activating its solenoid and by power the hydralic motor in reverse, which is accomplished
 * by driving 12VDC to the "go-down" input of the contactor. The third relay we use is relay 8, which enables
 * 120VAC or 220VAC to pass to a "wall-wart" style power supply that generates 12.3VDC @ 2A.  When our battery
 * voltage falls below 12.1VDC, we turn it on.  When it rises above 12.2VDC, we turn it off.
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

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

/*
 * Central (and only) task scheduler data structure.
 */
Scheduler ts;

// The next three forward declarations are hold-overs from the example code from OC.
void writedops(void);
void printaddr(char x);
void setbaud(char Mybaud);

// REL8 controls the AC input of the power supply that charges the battery
#define REL8 9  // Relay 8 is connected to Arduino Digital 9 PWM

#define TXEN 19 // RS-485 Transmit Enable is connected to Arduino Analog 5 which is Digital 19


/*
 * We allocated one more byte than the length of these compiler-defined strings, to make space for the null-terminator.
 */
char build_date[12] = __DATE__;
char build_time[9] = __TIME__;

/*
 * These are values that may need to be tweaked.  They should be stored in EEPROM and changable through the RS-485
 * interface and possibly by a future keypad or other direct, at the device, interface.
 */
struct calvals_s {
  unsigned position_upper_limit;                          // Max position value (i.e., fully tilted up)
  unsigned position_lower_limit;
  unsigned long max_time_tilted_up; // Lower the panels after 10 hours, maximum.
  unsigned darkness_threshold;
  
  unsigned long supply_voltage_lower_limit;             // Below 11 volts, don't try to move panels other than lowering them.
  unsigned long supply_voltage_charge_limit_low;        // Turn on charger if voltage drops below this value
  unsigned long supply_voltage_charge_limit_high;       // Turn off charge when voltage goes over this value
  unsigned long supply_voltage_bms_open_threshold;       // If we see a voltage this low, we assume the BMS has opened due to potential cell overcharge
  
  unsigned long accumulated_motor_on_time_limit;        // This is on-time in milliseconds, aged at 1/20th time
  unsigned long accumulated_motor_on_time_aging_rate;      // Sets ratio of maximum on time to off time (1:<this variable>)
  unsigned backlight_on_time; // Number of seconds the LCD backlight stays on
} calvals;

/*
 * The following are global variables that are refreshed every 200 msec and availble for all tasks to 
 * make decisions with.
 */
unsigned long time_now;
unsigned long seconds;
unsigned long milliseconds;

/*
 * These come from Arudino Analog input 1, which is driven by the center tap of two resistors that connect to the 12V line and 
 * ground.  We need this resistor divider network to move the 12V line to the range of the ADC, which is 0..5V.
 */
unsigned supply_tap_voltage_raw;  // Raw ADC value: 0..1023 for 0..5V
unsigned supply_tap_voltage; // Raw value converted to millivolts (of the tap)
unsigned supply_voltage;     // Tap voltage converted to supply voltage

/*
 * These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is in the middle of the pack.
 */
unsigned battery_temperature_volts_raw;          // raw ADC value: 0..1023 for 0..5V
unsigned battery_temperature_millivolts;    // raw value converted to millivolts
long battery_temperature_C_x10;                  // millivolts converted to degrees Celsiuis
unsigned long battery_temperature_F;             // Degrees C converter to Farenheight

/*
 * We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
 * is unused.  Its current output goes to Arduino Analog input 3.
 */
unsigned current_sense_4V_volts_raw;         // What is read from input 3, 0..1023 for 0..5V
unsigned current_sense_4V_millivolts;   // Raw ADC value converted to millivolts
unsigned current_sense_4V_milliamps;    // Millivolts from the Attopilot device converter to milliamps it is sensing

/*
 * These two are from the sun angle sensor and are the raw values from the two tiny solar cells in that sensor
 */
unsigned lower_solar_val;  // Raw ADC values 0..1023 for 0..5V
unsigned upper_solar_val;  // Raw ADC values 0..1023 for 0..5V

/*
 * This is from the 1000mm pull-string sensor that tells us where the panels are.
 */
unsigned position_sensor_val;// Raw ADC values 0..1023 for 0..5V

/*
 * Record the time today when we first raised the panels.  This is used to decide, later in the day, that it must be
 * time to lower them (in case for some reason the sun angle sensor doesn't tell us it is night time).
 */
unsigned long time_of_first_raise = 0ULL;

/*
 * These are calculated from the values read from sun angle sensor.  'sun_high' means the sun is in a position that puts
 * its rays at an angle lower than 90 degrees and naturally happens as the earth rotates during the day.  To remedy this,
 * we move the panels up.  'sun_low' means the opposite and naturally happens when we rotate the panels past the point
 * where the sun is at a 90 degree angle to the panels.  That's our trigger to stop moving the panels.  'dark' is when
 * both sensors indicate the light level is so low that it must be nighttime.  That is our trigger to lower the panels to
 * their nighttime, resting position.
 */
bool sun_high, sun_low, dark;

/* 
 * Calculated from 'position_sensor_val' and the position limit calibration values.
 */
bool at_upper_position_limit, at_lower_position_limit;

/*
 * The number of milliseconds the motor has been on in total, but aged (so it goes down with time that the motor is off).  This is used to 
 * decide how much we've been running the contactor and motor.
 */
unsigned long accumulated_motor_on_time = 0;

/*
 * This is set when the accumulated motor on time is greater than its limit
 */
bool motor_might_be_hot = false;


/*
 * When true, do the normal control of the motor relays.  When false, stay alive, but do not control anything other than
 * the serial port and the battery charger.
 */
bool operational_mode = true;

/*
 * Counts down when the backlight is on.  When it is zero, we turn the backlight off.  Units are seconds.
 */
unsigned backlight_timer;

// Forward definitions of the call-back functions that we pass to the task scheduler.

void read_inputs_callback();
void print_status_callback();
void control_battery_charger_callback();
void control_hydraulics_callback();
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback();
void monitor_position_limits_callback();
void monitor_motor_on_time_callback();
void monitor_rs485_input_callback();
void monitor_buttons_callback();
void monitor_lcd_backlight_callback();

/* 
 * These are the tasks that are the heart of the logic that controls this system.  Some run periodically and are always enabled.  Some run only
 * when the panels are in motion.
 */

Task read_inputs(200, TASK_FOREVER, &read_inputs_callback, &ts, true);
Task print_status(TASK_SECOND * 3, TASK_FOREVER, &print_status_callback, &ts, true);
Task control_battery_charger(TASK_SECOND * 10, TASK_FOREVER, &control_battery_charger_callback, &ts, true);
Task control_hydraulics(TASK_SECOND * 3, TASK_FOREVER, &control_hydraulics_callback, &ts, true);
Task monitor_upward_moving_panels_and_stop_when_sun_angle_correct(300, TASK_FOREVER, &monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback, &ts, false);
Task monitor_position_limits(300, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
Task monitor_motor_on_time(TASK_SECOND * 2, TASK_FOREVER, &monitor_motor_on_time_callback, &ts, true);
Task monitor_rs485_input(100, TASK_FOREVER, &monitor_rs485_input_callback, &ts, true);
Task monitor_buttons(100, TASK_FOREVER, &monitor_buttons_callback, &ts, true);
Task monitor_lcd_backlight(TASK_SECOND, TASK_FOREVER, &monitor_lcd_backlight_callback, &ts, true);

/*
 * Keep track of whether we are driving the panels up or down.  These two should never be true at the same time.
 */
bool panels_going_up = false;
bool panels_going_down = false;

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7 

/*
 * Set the calibration values to their "factory default".
 */
void set_calvals_to_defaults()
{
  calvals.position_upper_limit = 350;                          // Max position value (i.e., fully tilted up)
  calvals.position_lower_limit = 200;
  calvals.max_time_tilted_up = 10ULL * 3600ULL * 1000ULL; // Lower the panels after 10 hours, maximum.
  calvals.darkness_threshold = 50;
  
  calvals.supply_voltage_lower_limit = 11000;             // Below 11 volts, don't try to move panels other than lowering them.
  calvals.supply_voltage_charge_limit_low = 12000;        // Turn on charger if voltage drops below this value
  calvals.supply_voltage_charge_limit_high = 12300;       // Turn off charge when voltage goes over this value
  calvals.supply_voltage_bms_open_threshold = 11000;       // If we see a voltage this low, we assume the BMS has opened due to potential cell overcharge
  
  calvals.accumulated_motor_on_time_limit = 60000;        // This is on-time in milliseconds, aged at 1/20th time
  calvals.accumulated_motor_on_time_aging_rate = 20;      // Sets ratio of maximum on time to off time (1:<this variable>)
  calvals.backlight_on_time = 60; // Number of seconds the LCD backlight stays on
}
/*
 * Called every second the backlight is on and turns off the backlight, and disables itself, when the backlight timer
 * has counted down to zero.
 */
void monitor_lcd_backlight_callback()
{
  if (backlight_timer > 0) {
    backlight_timer--;
    if (backlight_timer == 0) {
      monitor_lcd_backlight.disable();
      lcd.setBacklight(OFF);
    }
  }
}

/*
 * Called every 100msec to monitor the buttons that are below the LCD.
 */
void monitor_buttons_callback()
{
  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    monitor_lcd_backlight.enable();
    backlight_timer = calvals.backlight_on_time;
    lcd.setBacklight(YELLOW);
    
    if (buttons & BUTTON_UP) {
      
    }
    if (buttons & BUTTON_DOWN) {
      
    }
    if (buttons & BUTTON_LEFT) {
      
    }
    if (buttons & BUTTON_RIGHT) {
      
    }
    if (buttons & BUTTON_SELECT) {
      
    }
  }
}

/*
 * Turn off both relays that drive 12VDC to the contactor inputs.  Since we aren't driving
 * the panels, we also disable the task that monitors position and estimated temperature limits
 */
void stop_driving_panels(void)
{
  digitalWrite(REL1, LOW); // Turn off go-up relay
  digitalWrite(REL2,LOW);  // Turn off go-down relay
  panels_going_down = false;
  panels_going_up = false;
  monitor_position_limits.disable();
  monitor_upward_moving_panels_and_stop_when_sun_angle_correct.disable();
  lcd.setCursor(0,1);
  lcd.print("Stopped ");
  lcd.print(position_sensor_val);
}

/*
 * Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
 * to do things in this case and perhaps resume operation.
 */
void fail(char *fail_message)
{
  stop_driving_panels();
  enable_rs485_output();
  Serial.println(fail_message);
  lcd.setCursor(0,0);
  lcd.print(fail_message);
  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}

bool txenabled;

/*
 * Our RS-485 output is only active when we are transmitting.  Otherwise, we have this half-duplex channel
 * in a listening mode so that we can receive commands.  See
 */
void enable_rs485_output()
{
  digitalWrite(TXEN, HIGH);   //Enable Transmit
  delay(1);                   //Let 485 chip go into Transmit Mode
  txenabled = true;
}

/*
 * Disable the connection from the output of our UART to the RS-485 lines.  See
 * https://exploreembedded.com/wiki/UART_Programming_with_Atmega128
 * For more information on the UART programming.
 */
void disable_rs485_output()
{
  Serial.flush();
  while (!(UCSR0A & (1 << TXC0)));      // Wait for the transmit buffer to be empty
  digitalWrite(TXEN,LOW);               //Turn off transmit enable
  txenabled = false;
  delay(100); // Desparate attempt
}

/*
 * Start the panels moving up and enable the task that monitors position and estimated temperature.  It is a fatal
 * error if the panels were going down when this was called.
 */
void drive_panels_up(void)
{
  if (panels_going_down) {
    fail("drive_panels_up() called while panels_going_down is true");
  } else {
    digitalWrite(REL1,HIGH); // Turn on go-up relay
    lcd.setCursor(0,1);
    lcd.print("Going up ");
    panels_going_up = true;
    monitor_position_limits.enable();
  }
}

/*
 * Start the panels moving down and enable the task that monitors position and estimated temperature.  It is a fatal
 * error if the panels were going up when this was called.
 */
void drive_panels_down(void)
{
  if (panels_going_up) {
    enable_rs485_output();
    fail("drive_panels_down() called while panels_going_up is true");
    
  } else {
    lcd.setCursor(0,1);
    lcd.print("Going down ");
    digitalWrite(REL2,HIGH);  // Turn on go-down relay
    panels_going_down = true;
    monitor_position_limits.enable();
  }
}

/*
 * Currently this is just used by the status tracing function.  It is true when we are calling for charging power, false when we are not.
 */
bool power_supply_on = false;

/*
 * Turn on the AC input of the power supply, so that we may charge our lithium battery pack.
 */
void turn_on_power_supply(void)
{
  digitalWrite(REL8, HIGH);
  power_supply_on = true;
}

/*
 * Turn off the AC input of the power supply.  We do this when the battery voltage is high enough
 * that we do not need to charge further.
 */
void turn_off_power_supply(void)
{
  digitalWrite(REL8, LOW);
  power_supply_on = false;
}

/*
 * This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
 */
void read_inputs_callback()
{  
  time_now = millis();
  seconds = time_now / 1000;
  milliseconds = time_now % 1000;

  supply_tap_voltage_raw = analogRead(ANIN4);

  /*
   * The value by which we multiple the raw value, and the value we add or subtract after that are determined
   * first by calculation based on the measured values of the resistors and then fine tuned with measurements
   * with 5.5 digit multimeter (Rigol DM3058).
   */
  supply_tap_voltage = (((unsigned long)supply_tap_voltage_raw * 5036UL) / 1023UL) + 2;
  supply_voltage = (supply_tap_voltage * 10000UL) / 3245UL;
  
  battery_temperature_volts_raw = analogRead(ANIN5);
  battery_temperature_millivolts = (((unsigned long)battery_temperature_volts_raw * 5000UL) / 1023) + 20 /* Calibration value */;
  battery_temperature_C_x10 = battery_temperature_millivolts - 500;
  battery_temperature_F = ((((long)battery_temperature_C_x10 * 90L) / 50L) + 320L) / 10L;
  
  current_sense_4V_volts_raw = analogRead(ANIN6);
  current_sense_4V_millivolts = ((unsigned long)current_sense_4V_volts_raw * 5000UL) / 1023UL;
  current_sense_4V_milliamps = ((unsigned long)current_sense_4V_millivolts * 1000UL) / 36600UL;

  /* 
   *  The values we read for the sun sensor and position sensors jump around, I guess due to noise.  To compensate and
   *  have more stable values average the last reading with this reading (and the 'last reading' is a running average)
   */
  lower_solar_val = (analogRead(ANIN1) + lower_solar_val) / 2;
  upper_solar_val = (analogRead(ANIN2) + upper_solar_val) / 2;
  position_sensor_val = (analogRead(ANIN3) + position_sensor_val) / 2;

  dark = (lower_solar_val + upper_solar_val) / 2 <= calvals.darkness_threshold;
  sun_high = (dark == false) && ((lower_solar_val + 10) < upper_solar_val);
  sun_low = dark || (lower_solar_val > (upper_solar_val + 10));
  at_upper_position_limit = position_sensor_val >= calvals.position_upper_limit;
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

/*
 * This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
 * Typical:
 * 17:39:46.312 -> [0:45] <12.135V><76F><0mA><0ST>Position: 95  Lower Solar: 45  Upper Solar: 47<Night><Sun-low><Lower-position-limit>
 * 17:39:49.268 -> [3:0] <12.135V><75F><0mA><0ST>Position: 187  Lower Solar: 81  Upper Solar: 73<Night><Sun-low><Lower-position-limit>
 * 17:39:52.270 -> [6:0] <12.135V><75F><0mA><0ST>Position: 178  Lower Solar: 72  Upper Solar: 71<Night><Sun-low><Lower-position-limit>
 */
void print_status_callback()
{
  static bool try_rtc = true;

  enable_rs485_output();
  if (verbose > VERBOSE_NONE) {
    if (try_rtc) {
      tmElements_t tm;
  
      if (RTC.read(tm)) {
        Serial.write('[');
        Serial.print(tm.Day);
        Serial.write('-');
        Serial.print(tm.Month);
        Serial.write('-');
        Serial.print(tmYearToCalendar(tm.Year));
        Serial.print(' ');
        print2digits(tm.Hour);
        Serial.write(':');
        print2digits(tm.Minute);
        Serial.write(':');
        print2digits(tm.Second);
        Serial.print(F("] "));
        
       } else {
         if (RTC.chipPresent()) {
           Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
           Serial.println(F("example to initialize the time and begin running."));
           Serial.println();
         } else {
           Serial.println(F("DS1307 read error!  Please check the circuitry."));
           try_rtc = false;
           Serial.println();
         }
      }
    } else {
      // No RTC, use the counts-from-boot default time.
      Serial.write('[');
      Serial.print(seconds);
      Serial.write(':');
      Serial.print(milliseconds);
      Serial.print(F("] "));
      Serial.write('<');
    }
    Serial.write('<');
    if (VERBOSE_MEDIUM > 1) {
      Serial.print(supply_tap_voltage_raw);
      Serial.write(',');
      Serial.print(supply_tap_voltage);
      Serial.write(',');
    }
    Serial.print(supply_voltage / 1000);
    Serial.write('.');
    Serial.print(supply_voltage % 1000);
    Serial.print(F("V><"));
   
    if (verbose > VERBOSE_MEDIUM) { 
      Serial.print(battery_temperature_millivolts);
      Serial.print(F("VDC,"));
    }
    Serial.print(battery_temperature_F);
    Serial.print(F("F>"));
   
    Serial.write('<');
    if (verbose > VERBOSE_MEDIUM) {
      Serial.print(current_sense_4V_millivolts);
      Serial.print(F("mV,"));
    }
    Serial.print(current_sense_4V_milliamps);
    Serial.print(F("mA><"));
    Serial.print(accumulated_motor_on_time);
    Serial.print(F("msec>Position: "));
    
    Serial.print(position_sensor_val);
    
    Serial.print(F("  Lower Solar: "));
    Serial.print(lower_solar_val);

    Serial.print(F("  Upper Solar: "));
    Serial.print(upper_solar_val);

    if (dark) {
      Serial.print(F("<Night>"));
    } else {
      Serial.print(F("<Day>"));
    }
    if (sun_high) {
      Serial.print(F("<Sun-high>"));
    }
    if (sun_low) {
      Serial.print(F("<Sun-low>"));
    }
    if (at_upper_position_limit) {
      Serial.print(F("<Upper-position-limit>"));
    }
    if (at_lower_position_limit) {
      Serial.print(F("<Lower-position-limit>"));
    }
    if (panels_going_up) {
      Serial.print(F("<Up>"));
    }
    if (panels_going_down) {
      Serial.print(F("<Down>"));
    }
    if (power_supply_on) {
      Serial.print(F("<power supply on>"));
    } 
    if (motor_might_be_hot) {
      Serial.print(F("<motor might be hot>"));
    }
    Serial.println("");
    disable_rs485_output();
  }
  
  
  lcd.setCursor(0,0);
  int bytes = lcd.print(position_sensor_val);
  bytes += lcd.print(" ");
  bytes += lcd.print(supply_voltage / 1000);
  bytes += lcd.print(".");
  bytes += lcd.print(supply_voltage % 1000);
  bytes += lcd.print(" ");
  bytes += lcd.print(battery_temperature_F);
  while (bytes < 16) {
    bytes += lcd.print(" ");
  }
  lcd.setCursor(0,1);
  char c1 = ' ';
  if (panels_going_up) {
    c1 = 'U';
  } else if (panels_going_down) {
    c1 = 'D';
  } else if (motor_might_be_hot) {
    c1 = 'H';
  }
  bytes = lcd.print(c1);

  char c2 = ' ';
  if (at_upper_position_limit) {
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

  bytes += lcd.print(" ");
  bytes += lcd.print(lower_solar_val);
  bytes += lcd.print(" ");
  bytes += lcd.print(upper_solar_val);
  while (bytes < 16) {
    bytes += lcd.print(" ");
  }
}

/*
 * Check to see if the battery charger needs to be turned on or off.
 */
void control_battery_charger_callback()
{
  if (supply_voltage < calvals.supply_voltage_charge_limit_low) {

    /*
     * If the power supply was already on and the voltage is really low, then
     * we conclude that the BMS opened because our charge limit range was too high. 
     * In this case, we lower the the charge limit range unless the upper limit is 
     * already below 12 volts.
     */
    if (power_supply_on && supply_voltage < calvals.supply_voltage_bms_open_threshold) {
      if (calvals.supply_voltage_charge_limit_high > 12000) {
        enable_rs485_output();
        Serial.print(F("BMS open?  supply_voltage: "));
        Serial.println(supply_voltage);

        /*
         * Move the charge range down 10 millivolts
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

/*
 * Check to see if the system has reached its high or low limits and stop the panels from moving if so.
 */
void monitor_position_limits_callback()
{
  if (panels_going_up && at_upper_position_limit && position_sensor_val > (calvals.position_upper_limit + 20)) {
    stop_driving_panels();
  }
  if (panels_going_down && at_lower_position_limit && position_sensor_val <= (calvals.position_lower_limit - 20)) {
    stop_driving_panels();
  }
}

/*
 * Model the temperature of the hydraulic motor and the contactor and the check valve solenoid by keeping track
 * of how long they are on vs. how long they are off. This runs all the time doing this modeling.  This will stop
 * the motor if it has run too long without time to cool down.  Other functions use the 'motor_might_be_hot' as
 * well to decide to not start running the motor when they otherwise would.
 */
void monitor_motor_on_time_callback()
{
  unsigned long time_now = millis();
  static unsigned long last_time_now = 0;                 // Values from millis() start at zero when system starts, so this is correct initialization vale
  unsigned long time_elapsed = time_now - last_time_now;  // Works even if we had a roll-over

  last_time_now = time_now;

  if (panels_going_up || panels_going_down) {
    accumulated_motor_on_time += time_elapsed;
  } else {
    // For every second the motor is on, let it cool for a number of (configurable) seconds.
    unsigned long delta = time_elapsed / calvals.accumulated_motor_on_time_aging_rate;
    if (accumulated_motor_on_time < delta) {
      accumulated_motor_on_time = 0;
    } else {
      accumulated_motor_on_time -= delta;
    }
  }
  motor_might_be_hot = accumulated_motor_on_time > calvals.accumulated_motor_on_time_limit;
  if (motor_might_be_hot && (panels_going_up || panels_going_down)) {
    stop_driving_panels();
  }
}

/*
 * This is called back periodically when the panels are moving up.  Stop their upwards movement if the sun angle sensor
 * tell us that the sun is now normal to the plane of the panels.  If that is the case, disable this task.
 */
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback()
{
  if (panels_going_up == false) {
    fail("monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback(): panels_going_up == false");
  }
  if (sun_low == true) {
    stop_driving_panels();
  }
}

/*
 * Check to see if the sun angle sensor is showing that the sun has move to be higher in the sky and so the suns rays are no 
 * longer normal to the panels (and sensor).  In this case, start raising the panels.
 * 
 * Also check to see if it is now night time and we should lower the panels to their resting (down) position and start them moving
 * down if that is so.
 */
void control_hydraulics_callback()
{
  if (operational_mode) {
    // If we need to raise the panel and the panel is not at is position limit, turn on the relay to raise it.
    bool supply_voltage_ok = supply_voltage > calvals.supply_voltage_lower_limit;
    
    if (sun_high && at_upper_position_limit == false && 
      panels_going_up == false && 
      panels_going_down == false && 
      supply_voltage_ok && 
      motor_might_be_hot == false) {
      if (time_of_first_raise == 0ULL) {
        time_of_first_raise = time_now;
      }
      drive_panels_up();
      monitor_upward_moving_panels_and_stop_when_sun_angle_correct.enable();
    } 
    unsigned long time_since_first_raise = time_now - time_of_first_raise;
    bool bedtime = (time_since_first_raise > calvals.max_time_tilted_up) || dark;
    if (panels_going_up == false && panels_going_down == false && bedtime) {
      if (at_lower_position_limit == false) {
        drive_panels_down();
      }
    }
  }
}

/*
 * Function copied from Arduino tutorial, modified to use arbitrary pointer, calculates the CRC 
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
 * Write the structure 'calvals' to EEPROM starting at EEPROM location zero.  Calculate its
 * checksum and write that after 'calvals'.  Read back 'calvals' and verify it was correct.
 * This assumes that the RS-485 transmitter is enabled.
 */
void write_calvals_to_eeprom()
{
  unsigned eeprom_crc_value = 0;
  unsigned calvals_crc_value = 0;
  struct calvals_s temp_calvals;

  if (txenabled == false) {
    fail("txenabled is false in write_calvals_to_eeprom()");
  }
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
 * Print the calibration values to RS-485 output.  This assumes that the RS-485 transmitter is already enabled.
 */
void print_calvals()
{
  if (txenabled == false) {
    fail("txenabled is false in print_calvals...");
  }

  Serial.print(F("Build date: "));
  Serial.println(build_date);

  Serial.print(F("Build time: "));
  Serial.println(build_time);
  
  Serial.print(F("unsigned position_upper_limit: "));
  Serial.println(calvals.position_upper_limit);
  
  Serial.print(F("position_lower_limit: "));
  Serial.println(calvals.position_lower_limit);
  
  Serial.print(F("max_time_tilted_up: "));
  Serial.println(calvals.max_time_tilted_up);

  Serial.print(F("darkness_threshold: "));
  Serial.print(calvals.darkness_threshold);
  
  Serial.print(F("supply_voltage_lower_limit: "));
  Serial.println(calvals.supply_voltage_lower_limit);
  
  Serial.print(F("supply_voltage_charge_limit_low: "));
  Serial.print(calvals.supply_voltage_charge_limit_low);
  
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
 * Read the calibration values from the EEPROM.  If their checksum fails, then do not update the calibration values
 * in 'calvals'.  If the passed flag is set and the checksum doesn't match, then write the existing (default)
 * calibration values to EEPROM instead.
 * This function assumes that the rs-485 is enabled.
 */
void read_calvals_from_eeprom(bool fix_if_checksum_fails)
{
  unsigned eeprom_crc_value = 0;
  unsigned calvals_crc_value = 0;
  unsigned stored_eeprom_crc_value = 0;
  struct calvals_s temp_calvals;
  if (txenabled == false) {
    fail("txenabled is false in read_calvals...");
  }
  /*
   * Fetch the calvals structure from EEPROM and the stored CRC that follows it.
   */
  EEPROM.get(0 /* EEPROM address */, temp_calvals);
  EEPROM.get(sizeof(calvals), stored_eeprom_crc_value);

  /*
   * Calculate the CRC of what we just fetched from EEPROM
   */
  eeprom_crc_value = crc((unsigned char *)&temp_calvals, sizeof(temp_calvals));

  if (eeprom_crc_value != stored_eeprom_crc_value) {
    Serial.println(F("CRCs don't match"));

    if (fix_if_checksum_fails) {
      Serial.println(F("updating EERPOM with defaults"));
      write_calvals_to_eeprom();
    }
  } else {
    calvals = temp_calvals;
  }
}

/*
 * This is the function that the Arudino run time system calls once, just after start up.  We have to set the 
 * pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
 * our operation and emit a startup message.
 */
void setup() 
{
  analogReference(DEFAULT);
 
  pinMode(REL1, OUTPUT);  // declare the relay pin as an output
  pinMode(REL2, OUTPUT);  // declare the relay pin as an output
  pinMode(REL8, OUTPUT);  // declare the relay pin as an output
  
  pinMode(TXEN, OUTPUT);

  // Set the working calibration values to the defaults that are in this file
  set_calvals_to_defaults();

  lcd.begin(16, 2);
  lcd.print("Solartracker 1.0");
  lcd.setCursor(0, 1);
  lcd.print(build_date);

  Serial.begin(SERIAL_BAUD);
  UCSR0A=UCSR0A |(1 << TXC0); //Clear Transmit Complete Flag
     
  enable_rs485_output();
  Serial.print(F("Suntracker "));
  Serial.print(build_date);
  Serial.write(' ');
  Serial.println(build_time);

#ifdef SETTIME
  void setup_rtc();
  setup_rtc();
#endif

  // Try reading calibration values from EEPROM.  If that fails, write our default calibration values to EERPOM
  read_calvals_from_eeprom(true);
  backlight_timer = calvals.backlight_on_time;

  // Drop junk in receive buffer
  while (Serial.available() > 0) 
  {   
    (void)Serial.read();  
  }
  delay(INITIAL_DELAY);
  disable_rs485_output();         //Turn off transmit enable
}

/*
 * This is the function that the Arduino run time system calls repeatedly after it has called setup().  
 */
void loop() 
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}

/*
 * This serial code _was_ working, then it seemed to throw away most of the input characters and 
 * when it did receive some, they were often values like 0xff or other bytes with
 * most bits set.  Converting to using serialEvent() does not seem to have changed that.  
 */
void monitor_rs485_input_callback() 
{   
  if (txenabled == true) {
    fail("txenabled in monitor_rs485...");
  }
}

void serialEvent() 
{
  if (Serial.available() == 0) {
    fail("serialEvent()");
  }

  // This may fail, but I haven't see it fail yet.  I think it is just a matter of time, but in the meantime, I want
  // to ensure that the issues I am seeing are not due to entering this function with txenabled.
  if (txenabled == true) {
    fail("txenabled in serialEvent()");
  }
    char c = Serial.read();      // Get the waiting character
    
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
            operational_mode = !operational_mode;
            Serial.print(F("Operational mode "));
            if (operational_mode) {
              Serial.println(F("on"));
            } else {
              Serial.println(F("off"));
              stop_driving_panels();
            }
            break;
          
          default:
            Serial.println(F("Unrecognized command character (expected 'u', 'l', 'd', or 'h'): "));
            break;
        }
        break;
        
      default:
        Serial.println(F("Unrecog: "));
        Serial.println(c & 0xff, HEX);
        break;
    }
    disable_rs485_output();         //Turn off transmit enable
}



#ifdef SETTIME


tmElements_t tm;

void setup_rtc() 
{
  bool parse=false;
  bool config=false;

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
    fail("Could not parse time string");
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
