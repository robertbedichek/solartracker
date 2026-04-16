
/*
   This controls a SSR that powers a 100A power supply that is directly connected to a hydraulic motor.  The goal is
   to get more power out of four LG Neon-2 420 Watt panels than if they were fixed-mounted.  A secondary goal is to
   have them better looking.  If they were fixed-mounted, they'd need to be reversed-racked, which is not a great look.

    The core of the controls system is an Arduino: Sparkfun Redboard Qwiic 
    Earlier versions used the Qwiic connector to tie in I2C devices, this iteration is much simplier
    and no longer uses any I2C devices.

   Creative Commons Licence
   Robert Bedichek
*/

#include <string.h>  //Use the string Library
#include <ctype.h>
#include <assert.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <TimeLib.h>  // for update/display of time


#include <SoftwareSerial.h>

// This string variable is used by multiple functions below, but not at the same time
char cbuf[55];


const unsigned long max_solenoid_on_time = 1800 * 1000UL;
const unsigned long max_solenoid_off_time = 2700 * 1000UL;
unsigned long solenoid_power_supply_on_off_time;  // Value of millis() when we last turned on the solenoid on or off

bool time_of_day_valid = true;

/*
   Speed at which we run the USB serial connection 
*/
const unsigned long serial_baud = 115200;

const int position_hysteresis = 5;
/*
 * This global variable is set to the position value to which we are trying to raise the panels.
 */
int desired_position = 0;

const unsigned motor_current_threshold_low = 10;
const unsigned motor_current_threshold_high = 90;
bool panels_retracted;

//   We can operate in one of three modes.  The first is a kind of "off" mode where we stay alive, talk on the
//   serial port, update the LCD, but do not move the panels.   The two "on" modes are Sun-sensor mode and time-of-day/day-of-year mode.
enum mode_e { no_panel_movement_mode,
              position_mode,
              last_mode };

#define MOTOR_PS_SSR_ENABLE_PIN (8)  // Arduino output pin 8 on J4, writing '1' turns on 9V/100A power supply

#define SOLENOID_PS_SSR_ENABLE_PIN (7)

//    Arudino Analog In 0, measures the voltage from the draw-string position sensor
#define DRAW_STRING_IN (0)

// This taps a shunt resistor on the ground side of the high-current motor power, 75mV per 10 amps
#define MOTOR_CURRENT_SENSE_IN (1)

// This measures the voltage coming from a Hall Effect sensor that is clamped to the DC cable of one
// of the solar panels
#define PV_CURRENT_SENSE_IN (2)

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
  enum mode_e operation_mode;  // Mode to start in
} calvals;

//   Calculated from 'position_sensor_val' and the position limit calibration values.
bool at_upper_position_limit = false, at_lower_position_limit = false;
/*
   This is from the 1000mm pull-string sensor that tells us where the panels are.
*/
float position_sensor_val;                // Raw ADC values 0..1023 for 0..5V
float pv_current_sensor_val;              // Raw ADS values of PV current clamp sensor values
float pv_current;                    // Amps flowing past Hall Effect current sensor

unsigned last_position_sensor_val;        // Position value at last call to status print
unsigned last_position_sensor_val_stall;  // Position sensor value at last sample in monitor_motor_stall_callback()

//---------------------------------------------------------------------------------------------------

void read_time_and_sensor_inputs_callback();
Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
//---------------------------------------------------------------------------------------------------

void print_status_to_serial_callback();
Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
//---------------------------------------------------------------------------------------------------
void control_hydraulics_callback();
Task control_hydraulics(TASK_SECOND * 10, TASK_FOREVER, &control_hydraulics_callback, &ts, true);
//---------------------------------------------------------------------------------------------------
void monitor_position_limits_callback();
Task monitor_position_limits(50, TASK_FOREVER, &monitor_position_limits_callback, &ts, false);
//---------------------------------------------------------------------------------------------------

void monitor_stall_and_motor_current_callback();

//   We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
//   is unused.  Its current output goes to Arduino Analog input 3.

unsigned daily_stalls;                    // Zeroed at midnight, incremented on each motor stall
unsigned long stall_start_time;           // Incremented when no change in position while motor on
unsigned long under_current_start_time;

Task monitor_stall_and_motor_current(200, TASK_FOREVER, &monitor_stall_and_motor_current_callback, &ts, false);
//---------------------------------------------------------------------------------------------------
void monitor_serial_console_callback(void);
Task monitor_serial_console(TASK_SECOND, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);

/*
   Keep track of whether we are driving the panels up or down.  These two should never be true at the same time.
*/
bool panels_going_up = false;
bool panels_going_down = false;

/*
   Set the calibration values to their "factory default".
*/
void set_calvals_to_defaults() {
  calvals.position_upper_limit = 320;  // Max position value (i.e., fully tilted up) minus overshoot (real max is around 363)
  calvals.position_lower_limit = 70;
  calvals.operation_mode = position_mode;  // Mode in which we should start operation
}


void turn_on_motor_power_supply(void) 
{
  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(MOTOR_PS_SSR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, HIGH);
}

void turn_off_motor_power_supply(void)
{
  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(MOTOR_PS_SSR_ENABLE_PIN, INPUT);
  digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, LOW);  // We write the bit with a zero ust so we can query later to see the state 
}
void turn_on_solenoid_power_supply(void) 
{
  if (solenoid_power_supply_is_on() == false) {
    pinMode(SOLENOID_PS_SSR_ENABLE_PIN, OUTPUT);
    digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, HIGH);
    delay(2000);  // Wait for supply to develop powers
  }
  solenoid_power_supply_on_off_time = millis();
}

void turn_off_solenoid_power_supply(void) 
{
  if (solenoid_power_supply_is_on()) {
    // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
    // input.  That allows us to drive it high with a power-supply over ride switch without having that
    // conflict with the ATMega output driver.
    pinMode(SOLENOID_PS_SSR_ENABLE_PIN, INPUT);
    digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, LOW);  // Just so we can query later
    solenoid_power_supply_on_off_time = millis();
  }
}

bool solenoid_power_supply_is_on(void)
{
  return digitalRead(SOLENOID_PS_SSR_ENABLE_PIN) == HIGH;
}

/*
    Turn off both relays that drive 12VDC to the contactor inputs.  Since we aren't driving
    the panels, we also disable the task that monitors position and estimated temperature limits
*/
void stop_driving_panels(const __FlashStringHelper *who_called) 
{
  turn_off_motor_power_supply();

  if (who_called != (void *)0) {
    Serial.print(F("# stop_driving_panels(): "));
    Serial.println(who_called);
  }
  panels_going_down = false;
  panels_going_up = false;

  /*
   * We were called by one of these.  Ensure they are all disabled so we don't get called twice.
   */
  monitor_position_limits.disable();
  monitor_stall_and_motor_current.disable();
  // turn_off_solenoid_power_supply();
}

/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const __FlashStringHelper *fail_message) {
  stop_driving_panels(F("fail"));
  Serial.println(fail_message);

  delay(500);  // Give the serial link time to propogate the error message before execution ends
  abort();
}

// Value of millis() the last time we drove the panels up
unsigned long last_drive_panels_up_time;

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
    turn_on_motor_power_supply();
    panels_going_up = true;
    stall_start_time = 0;
    under_current_start_time = 0;
    panels_retracted = false;
    last_position_sensor_val_stall = position_sensor_val;
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
  const float ema_alpha = 0.1;
  float alpha;
  const int samples = 10;  // # of samples in arithmetic average

  float position_sensor_val_temp = 0.0;
  float pv_current_sensor_val_temp = 0.0;

  static bool first_time = true;

  if (first_time) {
    alpha = 1.0;
    first_time = false;
  } else {
    alpha = ema_alpha;
  }

  for (int sample = 0; sample < samples; sample++) {
   
        /*
        The values we read for the sun sensor and position sensors jump around, I guess due to noise.  To compensate and
        have more stable values average the last reading with this reading (and the 'last reading' is a running average)
    */
  //  int solar_raw = ads.readADC_SingleEnded(/* ADS1115 input */ 3);
//     solar_volts_temp += ads.computeVolts(solar_raw);

    unsigned position_sensor_raw = (unsigned)analogRead(DRAW_STRING_IN);
    position_sensor_val_temp += position_sensor_raw;

    unsigned pv_current_sensor_raw = (unsigned)analogRead(PV_CURRENT_SENSE_IN);
    pv_current_sensor_val_temp += pv_current_sensor_raw;
  }

  position_sensor_val_temp /= samples;
  position_sensor_val = alpha * position_sensor_val_temp + (1 - alpha) * position_sensor_val;

  pv_current_sensor_val_temp /= samples;
  pv_current_sensor_val = alpha * pv_current_sensor_val_temp + (1 - alpha) * pv_current_sensor_val;

  float pv_current_sensor_millivolts = (pv_current_sensor_val * 5000.0) / 1023.0;
  pv_current = (2476.4 - pv_current_sensor_millivolts) / 100.0;

  if (pv_current < 0) {
    pv_current = 0;
  }
  static bool verbose = false;
  if (verbose) {
    static int prints = 0;
    if (prints++ < 10){
      Serial.print(pv_current_sensor_val_temp); Serial.print(F(" "));
      Serial.print(pv_current_sensor_val); Serial.print(F(" "));
      Serial.println(pv_current_sensor_millivolts);
    }
  }

  at_upper_position_limit = position_sensor_val >= calvals.position_upper_limit;
  at_lower_position_limit = position_sensor_val < calvals.position_lower_limit;  // Panels are at a good lower position when position_sensor_val is 50
}

int motor_amps(void) 
{
  unsigned motor_current_sense_volts_raw = analogRead(/* Arduino analog input 1 */ MOTOR_CURRENT_SENSE_IN);
  const bool verbose = false;

  if (verbose && motor_current_sense_volts_raw > 0) {
    Serial.print(F("# current sense raw="));
    Serial.println(motor_current_sense_volts_raw);
  }
  // /* Empirical calibration: the raw value seems to never be less than four, even with no current */
//   if (motor_current_sense_volts_raw >= 4) {
//    motor_current_sense_volts_raw -= 4;
//  }
  float motor_current_sense_millivolts = motor_current_sense_volts_raw * 5000.0 / 1023.0;
  if (verbose && motor_current_sense_volts_raw > 0) {
    Serial.print(F("# current sense millivolts="));
    Serial.println((int)motor_current_sense_millivolts);
  }
  return (int)(motor_current_sense_millivolts / 7.5); // Shunt resistor is 75 mV per 10 amps
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
  // static float last_solar_volts;
  static bool last_at_upper_position_limit;
  static bool last_at_lower_position_limit;
  static bool last_panels_going_up;
  static bool last_panels_going_down;
  static int last_motor_amps;
  static float last_pv_current;
  static unsigned skipped_record_counter;
  
  int position_difference;  // Amount position changed since last call to status print
  if (last_position_sensor_val == 0) {
    last_position_sensor_val = position_sensor_val;
  }
  position_difference = position_sensor_val - last_position_sensor_val;

  if (abs(position_difference) > 5 || 
     last_operation_mode != calvals.operation_mode || 
     // abs(last_solar_volts - solar_volts) > 0.1 || 
     last_at_upper_position_limit != at_upper_position_limit || 
     last_at_lower_position_limit != at_lower_position_limit || 
     last_panels_going_up != panels_going_up || 
     last_panels_going_down != panels_going_down || 
     last_motor_amps != (int)motor_amps() ||
     (int)last_pv_current != (int)pv_current ||
    //  last_solenoid_power_supply_is_on != solenoid_power_supply_is_on() ||
     
     skipped_record_counter++ > 1000) {

    last_position_sensor_val = position_sensor_val;
    last_operation_mode = calvals.operation_mode;
//     last_solar_volts = solar_volts;
    last_at_upper_position_limit = at_upper_position_limit;
    last_at_lower_position_limit = at_lower_position_limit;
    last_panels_going_up = panels_going_up;
    last_panels_going_down = panels_going_down;
    last_motor_amps = (int)motor_amps();
    last_pv_current = pv_current;
    skipped_record_counter = 0;

    if (line_counter == 0) {
      Serial.println(F("# Date     Time     Md Pos  Amps UpL Dnl GUp GDn PV Amps"));
      line_counter = 20;
    } else {
      line_counter--;
    }

    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.
    {
      time_t t = now();
      snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ",
             year(t),
             month(t),
             day(t),
             hour(t),
             minute(t),
             second(t));
    }
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), "%d %4d %4d",
             calvals.operation_mode,
             (int)position_sensor_val,
             (int)motor_amps());
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), " %3d %3d %3d %3d ",
             at_upper_position_limit,
             at_lower_position_limit,
             panels_going_up,
             panels_going_down);
    Serial.print(cbuf);

    dtostrf(pv_current, 4, 1, cbuf); 

    Serial.println(cbuf);
  }
}

/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/
void monitor_position_limits_callback() 
{
  if (panels_going_up && position_sensor_val > (calvals.position_upper_limit + position_hysteresis)) {
    last_drive_panels_up_time = millis();
    stop_driving_panels((void *)0 /* F("upper limit reached") */);
  }
  if (panels_going_down) {
    if (position_sensor_val <= (calvals.position_lower_limit - position_hysteresis)) {
      stop_driving_panels((void *)0 /* F("lower limit reached") */);
    }
      if (hour(now()) == 8) {
        // By 8AM, give up letting the panels fall so that we can reset global flags and be ready to start raising the panels
        stop_driving_panels((void *)0 /* F("letting panels fall") */);
      }
      // Turn the solenoid on and off until the panels reach the lower limit or 8AM rolls around
      if (solenoid_power_supply_is_on() && (millis() - solenoid_power_supply_on_off_time) > max_solenoid_on_time) {
        turn_off_solenoid_power_supply();
      } else if (solenoid_power_supply_is_on() == false && (millis() - solenoid_power_supply_on_off_time) > max_solenoid_off_time) {
        turn_on_solenoid_power_supply();
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
 
  if (panels_going_up) {
    if (position_sensor_val <= last_position_sensor_val_stall) {
      if (stall_start_time == 0) {
        stall_start_time = millis();
      } else if ((millis() - stall_start_time) > 500) {
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
  } else {
    fail(F("stall yet panels not in motion"));
  }
  // Now check to see if the hydraulic motor is taking too little or too much current
  int amps = motor_amps();
  const bool verbose_motor = false;
  if (verbose_motor) {
    {
      time_t t = now();
      snprintf(cbuf, sizeof(cbuf), "# %02u:%02u:%02u position=%u amps=%d  ",
              hour(t),
              minute(t),
              second(t),
              (unsigned)position_sensor_val,
              amps);
    }
    
    Serial.println(cbuf);
  }
  if (amps < motor_current_threshold_low) {
    if (under_current_start_time == 0) {
      under_current_start_time = millis();
    } else if ((millis() - under_current_start_time) > 9000) {
      stop_driving_panels(F("low current for 9 seconds"));
      daily_stalls++;
      Serial.println(daily_stalls);
    }
  } else {
    under_current_start_time = 0; // Not taking too little, reset the start time of undercurrent

    // If the motor is taking too much current, stop it immediately
    if (amps > motor_current_threshold_high) {
      stop_driving_panels(F("high current"));
      Serial.print(F("# amps="));
      Serial.println(amps);
    }
  }
  last_position_sensor_val_stall = position_sensor_val;
}

#ifdef RAIN_SENSOR
const float rain_threshold = 3.0;  // Below this voltage, we say it is raining

// Returns true if the rain sensor is conducting enough.  This will turn on the
// rain sensor if it is off.  We seek to only turn it on if we are going to make
// a decision to raise the panels or if we are waiting to see if it has
// stopped raining.  There is a separate function to explicitly turn off
// the rain sensor.

bool is_raining(void)
{
//  if (quad_relay.getState(RELAY_RAIN_SENSOR) == false) {
//    quad_relay.turnRelayOn(RELAY_RAIN_SENSOR);
//    delay(100); // Hopefully the relay will close in less than 100 milliseconds
//  }


  // Take a number of samples to get a better estimate of actual voltage.
  unsigned long rain_sensor_raw = 0;
  const int samples = 10;
  for (int i = 0 ; i < samples ; i++) {
   rain_sensor_raw += analogRead(RAIN_SENSE_IN);
  }
  rain_sensor_raw /= samples;
  rain_sensor_volts = ((float)rain_sensor_raw * 5000.0) / 1023.0

  return rain_sensor_volts < rain_threshold;


  return false;
}

#endif

int panel_movement_start_hour(void);
int panel_movement_end_hour(void);

/*
   Check to see if the sun angle sensor is showing that the sun has move to be higher in the sky and so the suns rays are no
   longer normal to the panels (and sensor).  In this case, start raising the panels.

   Also check to see if it is now night time and we should lower the panels to their resting (down) position and start them moving
   down if that is so.
*/

int panel_movement_start_hour(void)
{
  time_t t = now();
  int raise_hour = 8;          // Hour to start raising panels (default for winter)

  switch (month(t)) {
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

  switch (month(now())) {
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
  time_t t = now();
  int h = hour(t);
  int minutes_of_hour = minute(t);
  int raise_hour = panel_movement_start_hour();          // Hour to start raising panels (default for winter)
  int top_position_hour = 10;  // Hour at which panels should be in top position
  int lower_hour = panel_movement_end_hour();         // Hour at which considering lowering panels, wait for dark to lower them

  // Normally, I'd use data structures to define the times to raise and lower the panels.  However, our little Arduino processors has
  // 7,000 bytes of instruction memory free, but only 310 bytes of data free.  I don't know how much of that free memory is used
  // by the stack, but I don't think I can afford to make it any smaller.  So I am using a code-intensive approach in this function.

  switch (month(t)) {
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
  
  // If it is the right time of day to raise the panels, we haven't stalled too many times today, and it has
  // been at least 30 minutes since the last time we raised the panels, then consider further raising them

  if (raise_hour <= h && h < lower_hour && daily_stalls < 5 && (millis() - last_drive_panels_up_time) > 1 * 60 * 1000UL) {
    desired_position = calvals.position_upper_limit - 10;  // This should be redundant with check above
    if (desired_position > (position_sensor_val + 10 /* hysterisis */)) {
      drive_panels_up();
    }
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
//  turn_off_motor_power_supply_if_idle();

  if (panels_going_up || panels_going_down) {
    return;
  }

  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      break;

    case position_mode:
      if (time_of_day_valid) {
        drive_panels_to_desired_position();
      }
      break;

    default:
      fail(F("?mode"));
      break;
  }
}

// Called frequently to poll for and act on received console input.  This recognizes characters
// that correspond to keys on the controller, 's' for select, '+' for the plus key, and '-' for the minus key.

void monitor_serial_console_callback(void)
{
  static char command_buf[20];

  while (Serial.available() > 0) {  // Check if data is available to read
    time_t t = now();
    char received_char = Serial.read();  // Read one character
    int l = strlen(command_buf);
    if (l >= sizeof(command_buf) - 1) {
      Serial.println(F("# command buffer overflow"));
      command_buf[0] = '\0';
      break;
    }
    if (received_char == '\n') {
      Serial.print(F("# Received: "));
      Serial.println(command_buf);

      switch (command_buf[0]) {   
        case 'd': // Set date command, "d year-month-day", e.g., "d 2025-05-23" to set the date
        {
          int host_year, host_month, host_day;
          sscanf(command_buf + 2, "%d-%d-%d", &host_year, &host_month, &host_day);
          setTime(hour(t), minute(t), second(t), host_day, host_month, host_year);
        }
        break; 

        case 't': // Set time command, "t hh:mm:ss", e.g., "t 9:23:33" to set the time.
        {
          int host_hour, host_minute, host_second;
          sscanf(command_buf + 2, "%d:%d:%d", &host_hour, &host_minute, &host_second);
          setTime(host_hour, host_minute, host_second, day(t), month(t), year(t));
        }
        break;

        default:
          Serial.print(F("# Unknown command: "));
          Serial.println(command_buf);
          Serial.println(F("# choices are: 'd year-month-day', 't hour:minute:second','s', +, -"));
          break;
      }
      command_buf[0] = '\0';
    } else {
      command_buf[l] = received_char;
      command_buf[l+1] = '\0';
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

  Serial.begin(serial_baud);
  UCSR0A = UCSR0A | (1 << TXC0);  //Clear Transmit Complete Flag
  
  Serial.print(F("\n\r# reboot SolarTracker "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));
 
  
 // Set the working calibration values to the defaults that are in this file
  set_calvals_to_defaults();

  /*
     Set a valid initial value for lots of globals.  We need this in order to get the tm structure set,
     s that we can set today's cron string.  We need to do that before running in time_mode.
  */
  read_time_and_sensor_inputs_callback();
  setTime(0, 0, 0, 1, 1, 2026);  // So that we never get an invalid time due to library caching
  Serial.println(F("# Init done"));
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

    default:
      fail(F("??mode"));
      break;
  }
}

  

