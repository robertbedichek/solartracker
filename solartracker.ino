
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



// This string variable is used by multiple functions below, but not at the same time
char cbuf[55];


const unsigned long max_solenoid_on_time = 1800 * 1000UL;
const unsigned long max_solenoid_off_time = 2700 * 1000UL;
unsigned long solenoid_power_supply_on_off_time;  // Value of millis() when we last turned on the solenoid on or off

// Value of millis() the last time we drove the panels up
unsigned long last_drive_panels_up_time;


/*
   Speed at which we run the USB serial connection 
*/
const unsigned long serial_baud = 115200;

const int position_hysteresis = 5;

const unsigned motor_current_threshold_low = 10;
const unsigned motor_current_threshold_high = 90;

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
float pv_current(void);                   // Amps flowing past Hall Effect current sensor clamped to a solar panel DC cable
float peak_pv_current;                    // Highest recorded PV current during last panel raise 

unsigned last_position_sensor_val;        // Position value at last call to status print
unsigned last_position_sensor_val_stall;  // Position sensor value at last sample in monitor_motor_stall_callback()


//---------------------------------------------------------------------------------------------------

void read_time_and_sensor_inputs_callback();
Task read_time_and_sensor_inputs(TASK_SECOND, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
//---------------------------------------------------------------------------------------------------

void emit_telemetry_callback();
Task emit_telemetry(TASK_SECOND, TASK_FOREVER, &emit_telemetry_callback, &ts, true);
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
  calvals.position_upper_limit = 330;  // Max position value (i.e., fully tilted up) minus overshoot (real max is around 363)
  calvals.position_lower_limit = 70;
  calvals.operation_mode = position_mode;  // Mode in which we should start operation
}

// Return true if the Arduino is commanding the SSR that controls the motor power supply to be on.  However,
// for the motor power supply to be energized, it also needs to be the case that the limit switch is still conducting
// (and thus that the panels are not at their upper limit).

bool motor_power_supply_is_on = false;

void turn_on_motor_power_supply(void) 
{
  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(MOTOR_PS_SSR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, HIGH);
  motor_power_supply_is_on = true;
}

void turn_off_motor_power_supply(void)
{
  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(MOTOR_PS_SSR_ENABLE_PIN, INPUT);
  digitalWrite(MOTOR_PS_SSR_ENABLE_PIN, LOW);  // We write the bit with a zero ust so we can query later to see the state 
  motor_power_supply_is_on = false;
}


bool solenoid_power_supply_is_on = false;

void turn_on_solenoid_power_supply(void) 
{
  pinMode(SOLENOID_PS_SSR_ENABLE_PIN, OUTPUT);
  digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, HIGH);
  solenoid_power_supply_is_on = true;
  solenoid_power_supply_on_off_time = millis();
}

void turn_off_solenoid_power_supply(void) 
{
  // Instead of driving the output low, we turn it off by telling the Arudino runtime that it is an
  // input.  That allows us to drive it high with a power-supply over ride switch without having that
  // conflict with the ATMega output driver.
  pinMode(SOLENOID_PS_SSR_ENABLE_PIN, INPUT);
  digitalWrite(SOLENOID_PS_SSR_ENABLE_PIN, LOW);  // Just so we can query later
  solenoid_power_supply_is_on = false;
  solenoid_power_supply_on_off_time = millis();
}

/*
    Turn off both relays that drive 12VDC to the contactor inputs.  Since we aren't driving
    the panels, we also disable the task that monitors position and estimated temperature limits
*/
void stop_driving_panels(const __FlashStringHelper *who_called) 
{
  turn_off_motor_power_supply();
  turn_off_solenoid_power_supply();
  read_time_and_sensor_inputs.setInterval(TASK_SECOND);
  emit_telemetry.setInterval(TASK_SECOND);

  if (who_called != (void *)0) {
    Serial.print(F("# stop_driving_panels() secs=: "));
    Serial.print((unsigned)((millis() - last_drive_panels_up_time) / 1000ULL));
    Serial.print(F(", why="));
    Serial.println(who_called);
  }
  panels_going_down = false;
  panels_going_up = false;

  /*
   * We were called by one of these.  Ensure they are all disabled so we don't get called twice.
   */
  monitor_position_limits.disable();
  monitor_stall_and_motor_current.disable();
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
   Start the panels moving down and enable the task that monitors position and estimated temperature.  It is a fatal
   error if the panels were going up when this was called.
*/
void drive_panels_down(const __FlashStringHelper *why) 
{
  if (at_lower_position_limit) {
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
    monitor_position_limits.enable();
  }
}

// --- Insertion sort (in-place, ascending) --------------------------------
static void insertionSort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j   = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback() 
{
#define NUM_SAMPLES (16)
#define TRIM_COUNT  (4)       // discard this many from each end
#define MAX_RETRIES (20)
  static int warnings = 0;

  int samples[NUM_SAMPLES];  // Array of

  for (int i = 0; i < NUM_SAMPLES; i++) {
    for (int j = 0 ; j < MAX_RETRIES ; j++) {
      samples[i] = analogRead(DRAW_STRING_IN);
      delay(1); // Ensure that we take samples over more than one 60 Hz interval
      // If we have an out-of-range sample, sample again.
      if (samples[i] > 40 && samples[i] < 380) {
        break;
      } else {
        static unsigned bad_samples = 0;
        if (bad_samples++ < 10) { // ok for this to wrap-around
          Serial.print(F("# bad position sample: "));
          Serial.println(samples[i]);
        }
      }
    }
    if (samples[i] >= 380) {
      if (warnings++ < 100) {
        Serial.print(F("# out of range: "));
        Serial.println(samples[i]);
      }
      // We know that this sample is bad, but we tried to get a good value many times.  So
      // substitute the last known value.
      samples[i] = last_position_sensor_val; 
    }
  }
  insertionSort(samples, NUM_SAMPLES);

  // Sum the middle samples, skipping TRIM_COUNT on each end
  long sum = 0;
  const int keepCount = NUM_SAMPLES - 2 * TRIM_COUNT;
  for (int i = TRIM_COUNT; i < NUM_SAMPLES - TRIM_COUNT; i++) {
    sum += samples[i];
  }
 
  position_sensor_val = (float)sum / keepCount;

  at_upper_position_limit = position_sensor_val >= (calvals.position_upper_limit - 20);
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

// Return the number of amps that one of the panels is producing.  We use this to help determine
// the optimal panel position.  

float pv_current(void)
{
  unsigned pv_current_sensor_raw = 0.0;

  const int samples = 10;
  for (int sample = 0; sample < samples; sample++) {
    pv_current_sensor_raw += (unsigned)analogRead(PV_CURRENT_SENSE_IN);
    delay(1);
  }

  pv_current_sensor_raw /= samples;

  float pv_current_sensor_millivolts = (pv_current_sensor_raw * 5000.0) / 1023.0;

  // The Hall Effect current sensor outputs a signal that is nominally 2.5 +/- 2V.  So
  // a voltage of 0.5 volts represents a current of 10A.  A voltage of 4.5 also represents
  // 10A, but in the opposite direction.  Since current only goes in one direction in the PV
  // leads, but the current clamp could be put on in either of two orientations, we don't
  // know in which direction the current will be sensed.  We don't care which way the clamp
  // was put on, so we just take the absolute value of the difference between the sense
  // voltage and 2.5VDC.

  float pv_current_val = (2476.4 - pv_current_sensor_millivolts) / 200.0;

  if (pv_current_val < 0) {
    pv_current_val = -pv_current_val;
  }
  const bool debug_print = false;
  if (debug_print) {
    static int cnt = 0 ;
    if (cnt++ < 10) {
      Serial.print(F("# raw=")); Serial.print(pv_current_sensor_raw); 
      Serial.print(F(" millivolts=")); Serial.print(pv_current_sensor_millivolts); 
      Serial.print(F(" val=")); Serial.println(pv_current_val);
    }
  }
  
  return pv_current_val;
}

/*
   This is the main system tracing function.  It emits a line of ASCII to USB serial line with lots of information.
   We display this information in a form that plotyJS can readily absorb.

  2025-04-06 01:01:07 1   58    0    0    0  328  11.534   0   0   1   0   1   0   0   0   0  0.0

*/

bool force_status_line = false;

// Called periodically.  Sends relevant telemetry back over one or both of the serial channels
void emit_telemetry_callback(void)
{
  static char line_counter = 0;
  static enum mode_e last_operation_mode = last_mode;  // Force a difference the first time
  static bool last_panels_going_up;
  static bool last_panels_going_down;
  static bool last_solenoid_is_on;
  static bool last_motor_is_on;
  static int last_motor_amps;
  static float last_pv_current;
  static int skipped_record_counter = 0;
  
  int cur_motor_amps = motor_amps();
  int position_difference;  // Amount position changed since last call to status print
  if (last_position_sensor_val == 0) {
    last_position_sensor_val = position_sensor_val;
  }
  position_difference = position_sensor_val - last_position_sensor_val;

  if (force_status_line ||
    abs(position_difference) > 10 ||
     last_operation_mode != calvals.operation_mode ||
     last_panels_going_up != panels_going_up ||
     last_panels_going_down != panels_going_down ||
     last_solenoid_is_on != solenoid_power_supply_is_on ||
     last_motor_is_on != motor_power_supply_is_on ||
     last_motor_amps != cur_motor_amps ||
     (panels_going_up || skipped_record_counter-- <= 0)) {

    last_position_sensor_val = position_sensor_val;
    last_operation_mode = calvals.operation_mode;
    last_panels_going_up = panels_going_up;
    last_panels_going_down = panels_going_down;
    last_solenoid_is_on = solenoid_power_supply_is_on;
    last_motor_is_on = motor_power_supply_is_on;
    last_motor_amps = cur_motor_amps;
    last_pv_current = pv_current();
    skipped_record_counter = at_lower_position_limit ? 900 : 180;
    force_status_line = false;
    if (line_counter == 0) {
      Serial.println(F("# Date     Time     Md Pos  Amps UpL Dnl GUp GDn Sol Mot PV Amps"));
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
             cur_motor_amps);
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), " %3d %3d %3d %3d",
             at_upper_position_limit,
             at_lower_position_limit,
             panels_going_up,
             panels_going_down);
    Serial.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), " %3d %3d  ",
             solenoid_power_supply_is_on,
             motor_power_supply_is_on);
    Serial.print(cbuf);

    dtostrf(last_pv_current, 4, 1, cbuf);

    Serial.println(cbuf);
  }
}

/*
     Check to see if the system has reached its high or low limits and stop the panels from moving if so.
*/
void monitor_position_limits_callback()
{
  if (panels_going_up && (millis() - last_drive_panels_up_time) > 5000UL) {
    stop_driving_panels(F("5s raise limit"));
  }
  if (panels_going_up && position_sensor_val > (calvals.position_upper_limit + position_hysteresis)) {
    stop_driving_panels(F("upper limit reached"));
  }
  if (panels_going_down) {
    if (position_sensor_val <= (calvals.position_lower_limit - position_hysteresis)) {
      stop_driving_panels(F("lower limit reached"));
    } else {
      if (hour(now()) == 8) {
        // By 8AM, give up letting the panels fall so that we can reset global flags and be ready to start raising the panels
        stop_driving_panels(F("letting panels fall"));
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
  // First check to see that the panels are moving (and thus not stalled)
 int amps = motor_amps();
  if (panels_going_up) {
    if (amps > 1 && position_sensor_val <= last_position_sensor_val_stall) {
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
    } else if ((millis() - under_current_start_time) > 2000) {
      if (position_sensor_val < 150) {
        stop_driving_panels(F("low hydraulic motor current at low panel angle"));
        daily_stalls++;
      } else {
        stop_driving_panels(F("low hydaulic motor current, probably at upper stop"));
        Serial.print(F("# setting upper position limit to current position: "));
        Serial.println(position_sensor_val);
        calvals.position_upper_limit = position_sensor_val;
      }      
    }
  } else {
    under_current_start_time = 0; // Not taking too little, reset the start time of undercurrent

    // If the motor is taking too much current, stop it immediately
    if (amps > motor_current_threshold_high) {
      stop_driving_panels(F("high hydraulic motor current"));
      Serial.print(F("# amps="));
      Serial.println(amps);
    }
  }
  float pv = pv_current();
  if (peak_pv_current < pv) {
    peak_pv_current = pv;

    // If we have been driving the panels for at least one second and the pv current is decreasing from
    // the peak, then stop driving the panels up
  } else if ((millis() - last_drive_panels_up_time) > 1000UL && pv < (peak_pv_current - 0.1)) {
    stop_driving_panels(F("decreasing panel current"));
    Serial.print(F("# pv amps=")); 
    Serial.println(pv);
    Serial.print(F(", peak="));
    Serial.println(peak_pv_current);
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

void drive_panels_to_desired_position(void)
{
  static unsigned long low_pv_start_time = 0;
  static bool was_pv_low = true;  // start true so dawn detection fires on first sunrise

  float pv = pv_current();
  bool pv_is_low = (pv < 0.2);

  // Dawn: when sun rises after a dark period and panels are down, reset daily stall count
  if (was_pv_low && !pv_is_low && at_lower_position_limit) {
    daily_stalls = 0;
  }
  was_pv_low = pv_is_low;

  if (pv_is_low) {
    if (low_pv_start_time == 0) {
      low_pv_start_time = millis();
    } else if ((millis() - low_pv_start_time) > 45UL * 60 * 1000) {
      drive_panels_down(F("pv low 45 min"));
      low_pv_start_time = 0;
    }
  } else {
    low_pv_start_time = 0;
    bool sufficient_delay = (millis() - last_drive_panels_up_time) > 30UL * 60 * 1000;
    if (last_drive_panels_up_time == 0) {
      sufficient_delay = true;
    }
    if (daily_stalls < 5 && sufficient_delay && !at_upper_position_limit) {
      drive_panels_up();
    }
  }
}

void control_hydraulics_callback() 
{
  if (panels_going_up || panels_going_down) {
    return;
  }

  switch (calvals.operation_mode) {
    case no_panel_movement_mode:
      break;

    case position_mode:
      drive_panels_to_desired_position();
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
          force_status_line = true;
        }
        break; 

        case 't': // Set time command, "t hh:mm:ss", e.g., "t 9:23:33" to set the time.
        {
          int host_hour, host_minute, host_second;
          sscanf(command_buf + 2, "%d:%d:%d", &host_hour, &host_minute, &host_second);
          setTime(host_hour, host_minute, host_second, day(t), month(t), year(t));
          force_status_line = true;
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
  turn_off_motor_power_supply();
  turn_off_solenoid_power_supply();
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


