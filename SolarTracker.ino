

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

Scheduler ts;

// The next three forward declarations are hold-overs from the example code from OC.
void writedops(void);
void printaddr(char x);
void setbaud(char Mybaud);

// REL8 controls the AC input of the power supply that charges the battery
#define REL8 9  // Relay 8 is connected to Arduino Digital 9 PWM

#define TXEN 19 // RS-485 Transmit Enable is connected to Arduino Analog 5 which is Digital 19

// When true, display extra information on the serial output.  This should be controllable by the serial input stream
bool verbose = false;

// Forward definitions of the call-back functions that we pass to the task scheduler.

void read_inputs_callback();
void print_status_callback();
void control_battery_charger_callback();
void control_hydraulics_callback();
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback();
void monitor_position_limits_callback();
void monitor_motor_on_time_limits_callback();
void monitor_rs485_input_callback();

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
Task monitor_motor_on_time_limits(TASK_SECOND * 2, TASK_FOREVER, &monitor_motor_on_time_limits_callback, &ts, true);
Task monitor_rs485_input(100, TASK_FOREVER, &monitor_rs485_input_callback, &ts, true);


void check_rs485(void);

bool panels_going_up = false;
bool panels_going_down = false;

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
}

/*
 * Our RS-485 output is only active when we are transmitting.  Otherwise, we have this half-duplex channel
 * in a listening mode so that we can receive commands.
 */
void enable_rs485_output()
{
  digitalWrite(TXEN, HIGH);   //Enable Transmit
  delay(1);                   //Let 485 chip go into Transmit Mode
}

void disable_rs485_output()
{
  while (!(UCSR0A & (1 << TXC0))); 
  digitalWrite(TXEN,LOW);               //Turn off transmit enable
}

/*
 * Start the panels moving up and enable the task that monitors position and estimated temperature.  It is a fatal
 * error if the panels were going down when this was called.
 */
void drive_panels_up(void)
{
  if (panels_going_down) {
    enable_rs485_output();
    Serial.println("drive_panels_up() called while panels_going_down is true");
    stop_driving_panels();
    delay(500);  // Give the serial link time to propogate the error message before execution ends
    abort();
  } else {
    digitalWrite(REL1,HIGH); // Turn on go-up relay
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
    Serial.println("drive_panels_down() called while panels_going_up is true");
    stop_driving_panels();
    delay(500); // Give the serial link time to propogate the error message before execution ends
    abort();
  } else {
    digitalWrite(REL2,HIGH);  // Turn on go-down relay
    panels_going_down = true;
    monitor_position_limits.enable();
  }
}

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
 * These are values that may need to be tweaked.  They should be stored in EEPROM and changable through the RS-485
 * interface and possibly by a future keypad or other direct, at the device, interface.
 */
unsigned position_upper_limit = 350;                     // Max position value (i.e., fully tilted up)
unsigned position_lower_limit = 200;
unsigned long total_on_time_limit = 300ULL * 1000ULL; // Max milliseconds driving piston up.
unsigned long max_time_tilted_up = 10ULL * 3600ULL * 1000ULL; // Lower the panels after 10 hours, maximum.
unsigned darkness_threshold = 200;

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
unsigned supply_tap_volts_raw;  // Raw ADC value: 0..1023 for 0..5V
unsigned long supply_tap_volts; // Raw value converted to millivolts (of the tap)
unsigned long supply_volts;     // Tap voltage converted to supply voltage

/*
 * These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is in the middle of the pack.
 */
unsigned battery_temperature_volts_raw;          // raw ADC value: 0..1023 for 0..5V
unsigned long battery_temperature_millivolts;    // raw value converted to millivolts
long battery_temperature_C_x10;                  // millivolts converted to degrees Celsiuis
unsigned long battery_temperature_F;             // Degrees C converter to Farenheight

/*
 * We have an Attopilot current sensor on the 4V line going from the Li-ion pack to the contactor.  Its voltage output
 * is unused.  Its current output goes to Arduino Analog input 3.
 */
unsigned current_sense_4V_volts_raw;         // What is read from input 3, 0..1023 for 0..5V
unsigned long current_sense_4V_millivolts;   // Raw ADC value converted to millivolts
unsigned long current_sense_4V_milliamps;    // Millivolts from the Attopilot device converter to milliamps it is sensing

/*
 * These two are from the sun angle sensor and are the raw values from the two tiny solar cells in that sensor
 */
unsigned lower_solar_val;  // Raw ADC values 0..1023 for 0..5V
unsigned upper_solar_val;  // Raw ADC values 0..1023 for 0..5V

/*
 * This is from the 1000mm pull-string sensor that tells us where the panels are.
 */
unsigned position_sensor_val;// Raw ADC values 0..1023 for 0..5V
  
unsigned long time_of_first_raise = 0ULL;

/*
 * These are calculated from the values read from sensor.
 */
bool sun_high, sun_low, dark;


/* 
 * Calculated from 'position_sensor_val'
 */
bool at_upper_position_limit, at_lower_position_limit;

/*
 * Set when we have spent more than 1% of the time with the motor and contactor on.
 */
bool at_time_limit;


/*
 * The modeled temperature (based on on-times vs off-times)
 */
unsigned long synthetic_temperature = 0;

/*
 * This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
 */
void read_inputs_callback()
{  
  time_now = millis();
  seconds = time_now / 1000;
  milliseconds = time_now % 1000;

  supply_tap_volts_raw = analogRead(ANIN4);
  supply_tap_volts = ((unsigned long)supply_tap_volts_raw * 5036ULL) / 1023ULL;
  supply_volts = (supply_tap_volts * 10000UL) / 3245UL;
  

  if (supply_volts < 12050) {
    // Turn on power supply to charge our batteries
    turn_on_power_supply();
  } else if (supply_volts > 12300) {
    turn_off_power_supply();
  }
  battery_temperature_volts_raw = analogRead(ANIN5);
  battery_temperature_millivolts = ((battery_temperature_volts_raw * 5000ULL) / 1023) + 20 /* Calibration value */;
  battery_temperature_C_x10 = battery_temperature_millivolts - 500;
  battery_temperature_F = (((battery_temperature_C_x10 * 90) / 50) + 320) / 10;
  
  current_sense_4V_volts_raw = analogRead(ANIN6);
  current_sense_4V_millivolts = (current_sense_4V_volts_raw * 5000ULL) / 1023;
  current_sense_4V_milliamps = (current_sense_4V_millivolts * 1000ULL) / 36600ULL;

  // Average the last reading with this reading (and the 'last reading' is really the averages from before that
  lower_solar_val = (analogRead(ANIN1) + lower_solar_val) / 2;
  upper_solar_val = (analogRead(ANIN2) + upper_solar_val) / 2;
  position_sensor_val = (analogRead(ANIN3) + position_sensor_val) / 2;

  dark = lower_solar_val < darkness_threshold && upper_solar_val < darkness_threshold;
  sun_high = (dark == false) && ((lower_solar_val + 10) < upper_solar_val);
  sun_low = dark || (lower_solar_val > (upper_solar_val + 10));
  at_upper_position_limit = position_sensor_val >= position_upper_limit;
  at_lower_position_limit = position_sensor_val < position_lower_limit;
}

/*
 * This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
 */
void print_status_callback()
{
  enable_rs485_output();
  Serial.print("[");
  Serial.print(seconds);
  Serial.print(":");
  Serial.print(milliseconds);
  Serial.print("] ");
  Serial.print("<");
  if (verbose) {
    Serial.print(supply_tap_volts_raw);
    Serial.print(",");
    Serial.print(supply_tap_volts);
    Serial.print(",");
  }
  Serial.print(supply_volts / 1000);
  Serial.print(".");
  Serial.print(supply_volts % 1000);
  Serial.print("V>");
 
  Serial.print("<");
  if (verbose) {
    Serial.print(battery_temperature_millivolts);
    Serial.print("VDC,");
  }
  if (power_supply_on) {
    Serial.print("<power supply on>");
  }
  Serial.print(battery_temperature_F);
  Serial.print("F>");

  
  Serial.print("<");
  if (verbose) {
    Serial.print(current_sense_4V_millivolts);
    Serial.print("mV,");
  }
  Serial.print(current_sense_4V_milliamps);
  Serial.print("mA><");
  Serial.print(synthetic_temperature);
  Serial.print("ST>Position: ");
  
    Serial.print(position_sensor_val);
    
    Serial.print("  Lower Solar: ");
    Serial.print(lower_solar_val);

    Serial.print("  Upper Solar: ");
    Serial.print(upper_solar_val);

    if (dark) {
      Serial.print("<Night>");
    } else {
      Serial.print("<Day>");
    }
    if (sun_high) {
      Serial.print("<Sun-high>");
    }
    if (sun_low) {
      Serial.print("<Sun-low>");
    }
    if (at_upper_position_limit) {
      Serial.print("<Upper-position-limit>");
    }
    if (at_lower_position_limit) {
      Serial.print("<Lower-position-limit>");
    }
    if (at_time_limit) {
      Serial.print("<Time-limit>");
    }
    if (panels_going_up) {
      Serial.print("<Up>");
    }
    if (panels_going_down) {
      Serial.print("<Down>");
    }
    Serial.println("");
    disable_rs485_output();
}

/*
 * Check to see if the battery charger needs to be turned on or off.
 */
void control_battery_charger_callback()
{
   if (supply_volts < 12100) {
    // Turn on power supply to charge our batteries
    turn_on_power_supply();
  } else if (supply_volts > 12250) {
    turn_off_power_supply();
  }
}

/*
 * Check to see if the system has reached its high or low limits and stop the panels from moving if so.
 */
void monitor_position_limits_callback()
{
  if (panels_going_up && at_upper_position_limit && position_sensor_val > (position_upper_limit + 20)) {
    stop_driving_panels();
    enable_rs485_output();
    Serial.print("monitor_position_limits(): at upper limit, stopping");
    disable_rs485_output();
  }
  if (panels_going_down && at_lower_position_limit && position_sensor_val <= (position_lower_limit - 20)) {
    stop_driving_panels();
    enable_rs485_output();
    Serial.print("monitor_position_limits(): at lower limit, stopping");
    disable_rs485_output();
  }
}

/*
 * Model the temperature of the hydraulic motor and the contactor and the check valve solenoid by keeping track
 * of how long they are on vs. how long they are off. This callback doesn't directly control the motors, it runs
 * all the time doing this modeling.  Other functions use the 'synthetic_temperature' that this task/function
 * calculates to decide whether the system has been on too long.
 */
void monitor_motor_on_time_limits_callback()
{
  unsigned long time_now = millis();
  static unsigned long last_time_now = 0;
  static bool initialized = false;

  if (initialized == false) {
    initialized = true;
    last_time_now = time_now;
  }
  unsigned long time_elapsed = time_now - last_time_now;  // Works even if we had a roll-over
  last_time_now = time_now;

  if (panels_going_up || panels_going_down) {
    synthetic_temperature += time_elapsed;
  } else {
    if (synthetic_temperature > 0) {
      // For every second the motor is on, let it cool for 100 seconds.
      unsigned long delta = time_elapsed / 100;
      if (delta == 0) {
        delta = 1;
      }
      synthetic_temperature -= delta;
    }
  }
}

/*
 * This is called back periodically when the panels are moving up.  Stop their upwards movement if the sun angle sensor
 * tell us that the sun is now normal to the plane of the panels.  If that is the case, disable this task.
 */
void monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback()
{
  if (panels_going_up == false) {
    enable_rs485_output();
    Serial.println("monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback(): panels_going_up == false");
    delay(500);
    abort();
  }
  if (sun_low == true) {
    stop_driving_panels();
    enable_rs485_output();
    Serial.print("monitor_upward_moving_panels_and_stop_when_sun_angle_correct_callback(): sun_low");

    Serial.print(" total_on_time_limit: ");
    Serial.println(total_on_time_limit);
    disable_rs485_output();
    monitor_upward_moving_panels_and_stop_when_sun_angle_correct.disable();
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
  // If we need to raise the panel and the panel is not at is position limit, turn on the relay to raise it.
  if (sun_high && at_upper_position_limit == false && panels_going_up == false && panels_going_down == false && at_time_limit == false) {
    if (time_of_first_raise == 0ULL) {
      time_of_first_raise = time_now;
    }
    enable_rs485_output();
    Serial.println("  raise panels ");
    disable_rs485_output();
    monitor_upward_moving_panels_and_stop_when_sun_angle_correct.enable();
    drive_panels_up();
  } 
  unsigned long time_since_first_raise = time_now - time_of_first_raise;
  bool bedtime = (time_since_first_raise > max_time_tilted_up) || dark;
  if (panels_going_up == false && panels_going_down == false && bedtime) {
    if (at_lower_position_limit == false) {
      // Open the check valve so that the panels can go down
      drive_panels_down();
    }
  }
}

/*
 * This is the value in 0..99 of this device's RS-485 address.  It is used by the needs-work RS-485 code at the end of this file.
 */
char Unitaddress;

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

  // Redo the use of EEPROM to use it for all the calibration values
  //Read Address, Baud and Parity from EEPROM Here  
  Unitaddress = EEPROM.read(0);
  if ((Unitaddress > 99) || (Unitaddress < 0))
  {
    Unitaddress = 0;  
  }
  Serial.begin(9600);
   
  UCSR0A=UCSR0A |(1 << TXC0); //Clear Transmit Complete Flag
  enable_rs485_output();
  Serial.println("Ocean Controls (Suntracker)");
  while (!(UCSR0A & (1 << TXC0))); 
  Serial.println("KTA-223 v3.3");
  while (!(UCSR0A & (1 << TXC0)));    //Wait for Transmit to finish
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

// The next block of variables are hold overs. Ugly code with globals that don't need to be global.
char  Rxchar, Rxenable, Rxptr, Cmdcomplete, R;
char Rxbuf[15];
char adrbuf[3], cmdbuf[3], valbuf[12];
char rxaddress, Unitbaud;
char Dip[5], Dop[9];
char Hold, Mask, Analog =1, Imax;     
int anreadings[3];
unsigned long RelayStartTime[8], WatchdogStartTime;
unsigned char RelayOnTime[8], WatchdogOnTime;

/*
 * The code from here to the end of the file is not working yet, needs major rework.  It is derived
 * from the original Ocean Controls example code (which probably worked fine).
 */
void monitor_rs485_input_callback()
{   
  if (Serial.available() > 0)    // Is a character waiting in the buffer?
  {
    Serial.println("Serial avaiable");
    Rxchar = Serial.read();      // Get the waiting character
    Serial.println(Rxchar);
    if (Rxchar == '@')      // Can start recording after @ symbol
    {
      if (Cmdcomplete != 1)
      {
        Rxenable = 1;
        Rxptr = 1;
      }//end cmdcomplete
    }//end rxchar
    if (Rxenable == 1)           // its enabled so record the characters
    {
      if ((Rxchar != 32) && (Rxchar != '@')) //dont save the spaces or @ symbol
      {
        Rxbuf[Rxptr] = Rxchar;
        
        Rxptr++;
        if (Rxptr > 13) 
        {
          Rxenable = 0;
        }//end rxptr
      }//end rxchar
      if (Rxchar == 13) 
      {
        Rxenable = 0;
        Cmdcomplete = 1;
      }//end rxchar
    }//end rxenable

  }// end serial available


   //we should now have AACCXXXX in the rxbuf array, with a cr at rxptr
   //we should take rxbuf(1) and rxbuf(2) and turn them into a number

  if (Cmdcomplete == 1)
  {
     adrbuf[0] = Rxbuf[1];
     adrbuf[1] = Rxbuf[2];
     adrbuf[2] = 0; //null terminate Mystr = Chr(rxbuf(1)) + Chr(rxbuf(2))
     rxaddress = atoi(adrbuf);//    Address = Val(mystr)
  //Serial.println(adrbuf);
     cmdbuf[0] = toupper(Rxbuf[3]); //copy and convert to upper case
     cmdbuf[1] = toupper(Rxbuf[4]); //copy and convert to upper case
     cmdbuf[2] = 0; //null terminate        Command = Chr(rxbuf(3)) + Chr(rxbuf(4))
     //   Command = Ucase(command)
  //Serial.println(cmdbuf);
     valbuf[0] = Rxbuf[5]; //        Mystr = Chr(rxbuf(5))
        R = Rxptr - 1;
            for (int i = 6 ; i <= R ; i++)//For I = 6 To R
            {
                valbuf[i-5] = Rxbuf[i]; //Mystr = Mystr + Chr(rxbuf(i))
            }
     valbuf[R+1] = 0; //null terminate
     int Param = atoi(valbuf);//   Param = Val(mystr)

     //Serial.println(Param); //   'Print "Parameter: " ; Param

       if ((rxaddress == Unitaddress) || (rxaddress == 0)) //0 is wildcard address, all units respond
       {

         //switch (cmdbuf) //Select Case Command
         //{
              if (strcmp(cmdbuf,"ON")==0)                                   //'turn relay x ON
              {
                   //'Print "command was ON"
                   if ((Param <= 8) && (Param >= 0)) 
                   {
                     if (Param == 0)
                     { 
                        for (int i = 1 ; i<=8 ; i++)
                        {
                          Dop[i-1] = 1;
                          RelayOnTime[i-1]= 0;//override the timing
                        }
                     }
                     else
                     {
                        Dop[Param-1] = 1;
                        RelayOnTime[Param-1]= 0;//override the timing
                     }
                     writedops();
                     printaddr(1);                     
                   }
                   else
                   {
                        //'Print "out of range"
                   }
              }
              if (strcmp(cmdbuf,"OF")==0)                                   //'turn relay x OFF
              {
                   //'Print "command was OFF"
                   if ((Param <= 8) && (Param >= 0)) 
                   {
                     if (Param == 0)
                     {
                        for (int i = 1 ; i<=8 ; i++)
                        {
                          Dop[i-1] = 0;
                        }
                     }
                     else
                     {
                        Dop[Param-1] = 0;
                     }
                     writedops();
                     printaddr(1);
                   }
                   else
                   {
                        //'Print "out of range"
                   }
              }    
              /*
              if (strcmp(cmdbuf,"PW")==0)        // PWM out on Relays 2, 4, 5 or 8 only
              {                                  // Value of parameter is Relay number and 0-255 value ie 2127 will make channel 2 output 127 (50%)
                   for ( i=0 ; i<=3 ; i++)       //
                   {
                     x = Param / 1000;
                     if ( x==PWMAble[i]) //Is is on the list?
                     {
                       
                       outval = Param % 1000; //Get the 0-255 value
                       if (outval>=0 && outval <=255) //within range?
                       {
                         analogWrite( Relays[x-1] , outval);
                         if (outval == 0)
                         {
                           Dop[x-1]=0;     //If it is 0 then we will say the Relay is OFF
                         }
                         else
                         {
                           Dop[x-1]=1;     //If it is not 0 then we will say the Relay is ON
                         }
                         printaddr(1);
                       }
                     }
                   }
              }
              */
              if (strcmp(cmdbuf,"TR")==0)        // Timed Relay
              {                                  // Value of parameter is Relay number and 1-255 time in 0.1s increments 2127 is relay 2 for 12.7 sec
                 int x = Param / 1000; //Get the relay number
                 int outval = Param % 1000; //Get the 0-255 value
                 if (outval>=1 && outval <=255) //within range?
                 {
                     RelayStartTime[x-1] = millis(); //save the current time
                     RelayOnTime[x-1] = outval;
                     Dop[x-1]=1;     //Relay on 
                     writedops();    //Write outputs
                     printaddr(1);
                 }
              }
              
              if (strcmp(cmdbuf,"KA")==0)        // Keep Alive
              {                                  // Value of parameter is seconds to stay alive for
                 
                 if (Param>=0 && Param <=255) //within range?
                 {
                     WatchdogStartTime = millis(); //save the current time
                     WatchdogOnTime = Param;
                     printaddr(1);
                 }
              }
              
              if (strcmp(cmdbuf,"WR")==0)                                    //'turn relay on/off according to binary value of x
              {
                    //'Print "command was WRITE"
                   if ((Param <= 255) && (Param >= 0)) 
                   {
                    for (int i=0 ; i<8 ; i++)
                    {
                       Mask = 1<<i;//Shift Mask , Left , I
                       Hold = Param & Mask;
                       /*'Print "mask=" ; Mask
'                                Print " I= " ; I
'                                Print "param=" ; Param
'                                print "hold=" ; Hold
*/
                       if (Hold == Mask)
                       {
                          Dop[i] = 1;
                          RelayOnTime[i]= 0; //override timing
                       }
                       else
                       {
                          Dop[i] = 0;
                       }
                    }
                   }
                   else
                   {
                   }
                   writedops();
                   printaddr(1);
                   
              }
              
              if (strcmp(cmdbuf,"RS")==0)                                   // 'relay status
              {
                    //'Print "command was RELAY STATUS"
                    if ((Param > 0) && (Param <= 8))
                   {
                         printaddr(2);
                         Serial.println(Dop[Param-1], DEC);
                   }
                   else if (Param == 0) 
                   {
                         int N = 0;
                         for (int i=0 ; i<8 ; i++)
                         {
                            if(Dop[i] == 1)
                            {
                               N = N|(1<<i);
                            }
                         }
                         printaddr(2);
                         Serial.println(N, DEC);
                   }
                   else
                   {
                     //'Print "out of range"
                   }
              }   

              if (strcmp(cmdbuf,"IS")==0)                                   // 'input status
              {
               // readdips();                 
                     if ((Param > 0) && (Param <= 4)) 
                     {
                       printaddr(2);
                       Serial.println(Dip[Param-1], DEC);
                       
                     }
                     else if (Param == 0)
                     {
                        int N = 0;
                        if (Analog == 1)
                        {
                          Imax = 3;
                        }
                        else 
                        {
                          Imax = 8;
                        }

                        for (int i=0 ; i<=Imax ; i++)
                        {
                          if (Dip[i] == 1)
                          {
                           N = N|(1<<i);
                          }
                        }
                        printaddr(2);
                        Serial.println(N,DEC);
          
                     }
                     else
                     {
                      //        'Print "out of range"
                     }
              }

              if (strcmp(cmdbuf,"AI")==0)                                   // 'return analog input
              {                                   
                    if (Analog == 1)
                    {
                       if ((Param >= 0) && (Param <= 3))
                       {
                         for (int i=0 ; i<3 ; i++)
                         {
                          // anreadings[i] = analogRead(Analogs[i]);
                         }
                         printaddr(2);
                         if (Param == 0)
                         {
                             Serial.print(anreadings[0], DEC);
                             Serial.print(" ");
                             Serial.print(anreadings[1], DEC);
                             Serial.print(" ");
                             Serial.println(anreadings[2], DEC);
                             
                         }
                         else
                         {
                             Serial.println(anreadings[Param-1], DEC);
                         }
                       }
                    }
                    else
                    {
                    }
              }
              if (strcmp(cmdbuf,"SS")==0)                                   // System Status
              {
                    
                   if (Param == 0)
                   {
                         int N = 0;
                         for (int i=0 ; i<8 ; i++) //Read Relays
                         {
                            if(Dop[i] == 1)
                            {
                               N = N|(1<<i);
                            }
                         }
                         printaddr(2);
                         Serial.print(N, DEC); //Print Relays
                         Serial.print(" ");
                         
                         // readdips(); //Read Inputs
                          N = 0;
                          if (Analog == 1)
                          {
                            Imax = 3;
                          }
                          else 
                          {
                            Imax = 8;
                          }
  
                          for (int i=0 ; i<=Imax ; i++)
                          {
                            if (Dip[i] == 1)
                            {
                             N = N|(1<<i);
                            }
                          }
                         
                          Serial.print(N,DEC); //Print Inputs
                          Serial.print(" ");
                          
                          if (Analog == 1)
                          {
                             for (int i=0 ; i<3 ; i++)
                             {
                               //anreadings[i] = analogRead(Analogs[i]); // Read Analogs
                             }
                             
                             
                                 Serial.print(anreadings[0], DEC); //Print Analogs
                                 Serial.print(" ");
                                 Serial.print(anreadings[1], DEC);
                                 Serial.print(" ");
                                 Serial.println(anreadings[2], DEC);
                                 
                          }
                   }
                   else
                   {
                     //'Print "out of range"
                   }
              }   
              if (strcmp(cmdbuf,"SA")==0)                                   // Set Address and save to EEP
              {
                    //'Print "command was Set Address"
                   if ((Param >= 0) && (Param <= 99))
                   {
                     Unitaddress = Param;   //make it the address
                     EEPROM.write(0, Unitaddress);//save to eep                       
                     printaddr(1);                         
                   }
                   else
                   {
                     //'Print "out of range"
                   }
              } 
              if (strcmp(cmdbuf,"SB")==0)                                   // Set Baud and save to EEP
              {
                    //'Print "command was Set Baud"
                   if ((Param > 0) && (Param <= 10))
                   {
                     Unitbaud = Param;   
                     EEPROM.write(1, Unitbaud);//save to eep          

                     setbaud(Unitbaud);// start serial port             
                     printaddr(1);                         
                   }
                   else
                   {
                     //'Print "out of range"
                   }
              } 

         //}//end switch cmdbuf
       }//end address


      Cmdcomplete = 0;
  }//end cmdcomplete 
    while (!(UCSR0A & (1 << TXC0)));    //Wait for Transmit to finish
    digitalWrite(TXEN,LOW);               //Turn off transmit enable
}//end check_rs485()


void writedops(void)
{
  for (int i=0 ; i<8 ; i++)
  {
    if (Dop[i]==1)
    {
      // digitalWrite(Relays[i],HIGH);
    }
    else
    {
      //digitalWrite(Relays[i],LOW);
    }
  }
}

void printaddr(char x) //if x=1 then it prints an enter, if x=2 then it prints a space after the address
{
  UCSR0A=UCSR0A |(1 << TXC0); //Clear Transmit Complete Flag
  digitalWrite(TXEN, HIGH);   //Enable Transmit
  delay(1);                   //Let 485 chip go into Transmit Mode
  
  if (Unitaddress < 10)
  {
    Serial.print("#0");
    Serial.print(Unitaddress, DEC);
  }
  else
  {
    Serial.print("#"); 
    Serial.print(Unitaddress, DEC);
  }
  switch(x)
  {
    case 1:
        Serial.println(); //print enter
      break;
    case 2:
        Serial.print(" "); //print space
      break;
  
  }
}

void setbaud(char Mybaud)
{
   switch (Mybaud)
   {
    case 1 : Serial.begin(1200);
      break;
    case 2 : Serial.begin(2400);
      break;     
    case 3 : Serial.begin(4800);
      break;
    case 4 : Serial.begin(9600);
      break;
    case 5 : Serial.begin(14400);
      break;
    case 6 : Serial.begin(19200);
      break;
    case 7 : Serial.begin(28800);
      break;
    case 8 : Serial.begin(38400);
      break;
    case 9 : Serial.begin(57600);
      break;
    case 10 : Serial.begin(115200);
      break;
    default:  Serial.begin(9600);
      break;
   }
}
