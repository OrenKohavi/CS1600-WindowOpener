#include <Adafruit_SleepyDog.h>

/*
TODO:
FSM Implementation
Demo_mode definitions
Timer for motor sanity check
Automatic threshold recalibration (Use WiFi and real-world time)
*/

/*
Use wifi to get the real world time, and get sunrise/sunset times
Recalibrate the thresholds for 
*/

//For demo/debugging
#define DEMO_MODE false
#define VERBOSE true

//For integration and adjustments
#define INVERT_MOTOR_OFF_STATE false
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 500
#define INVERT_MOTOR_DIR false
#define UP_PIN 1
#define DOWN_PIN 0
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6

//For changing fundamental system behavior
#define LOOP_INTERVAL 5 //ms delay between loops
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1000
#define LOOPS_UNTIL_DOUBLEPRESS_REGISTERED MS_UNTIL_DOUBLEPRESS_REGISTERED / LOOP_INTERVAL
#define PHOTORESISTOR_SAMPLE_INTERVAL_MS 100 //ms between light sensor samples
#define MS_UNTIL_BUTTON_PRESS_REGISTERED 50
#define LOOPS_UNTIL_BUTTON_PRESS_REGISTERED MS_UNTIL_BUTTON_PRESS_REGISTERED / LOOP_INTERVAL
#define LOOPS_PER_PHOTORESISTOR_SAMPLE PHOTORESISTOR_SAMPLE_INTERVAL_MS / LOOP_INTERVAL
#define MINS_PER_SAMPLE_INTERVAL 5 //How long do we record data for our moving average
#define SAMPLES_TO_KEEP (1000 * 60 * MINS_PER_SAMPLE_INTERVAL) / PHOTORESISTOR_SAMPLE_INTERVAL_MS //How many ints do we need to store

enum FSM_State {
  S_INIT,
  S_CALIBRATE_UP,
  S_CALIBRATE_DOWN,
  S_WAITING,
  S_MOTOR_MOVING_UP,
  S_MOTOR_MOVING_DOWN,
};

enum Motor_Direction {
  Motor_OFF = 0,
  Motor_UP = -1,
  Motor_DOWN = 1,
};

//Globals
FSM_State curr_system_state = S_INIT;
int16_t sample_arr[SAMPLES_TO_KEEP];
size_t sample_arr_idx = 0;
uint32_t mils_at_last_physical_interaction;
uint32_t micros_at_motor_start;
uint32_t micros_at_motor_end;
uint32_t micros_motor_raised; //counts the number of microseconds that the motor has been lifting (i.e. this tracks how far above the bottom we are)
int up_button_state;
int down_button_state;
Motor_Direction last_motor_command = Motor_OFF;
uint32_t last_motor_command_time;

void setup() {
  Serial.begin(9600);
  while(!Serial); //Wait for Serial to initialize

  if (true){
    //Print values of some definitions, for sanity checking
    Serial.print("LOOPS_UNTIL_BUTTON_PRESS_REGISTERED: ");
    Serial.println(LOOPS_UNTIL_BUTTON_PRESS_REGISTERED, DEC);

    Serial.print("LOOPS_UNTIL_DOUBLEPRESS_REGISTERED: ");
    Serial.println(LOOPS_UNTIL_DOUBLEPRESS_REGISTERED, DEC);

    Serial.print("SAMPLES_TO_KEEP: ");
    Serial.println(SAMPLES_TO_KEEP, DEC);

    Serial.print("LOOPS_PER_PHOTORESISTOR_SAMPLE: ");
    Serial.println(LOOPS_PER_PHOTORESISTOR_SAMPLE, DEC);
  }

  //Initialize all pins with correct pinmodes
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  //Clear arrays for samples and supersamples
  memset(sample_arr, -1, sizeof(int16_t) * SAMPLES_TO_KEEP);

  //Setup button interrupts
  //Due to details in the interrupt handler, these cannot be different functions (Since both pins may be on the same pin bank)
  attachInterrupt(digitalPinToInterrupt(UP_PIN), Button_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), Button_ISR, CHANGE);

  //Connect to WiFi and sync time
  //TODO

  //Enable watchdog (So that things like WiFi connection don't cause the WDT to accidentaly trigger)
  int watchdog_time = Watchdog.enable(LOOP_INTERVAL * 2); //ms before watchdog kicks
  //int watchdog_time = -1;
  Serial.print("Enabled the watchdog with a time of: ");
  Serial.print(watchdog_time, DEC);
  Serial.println();
}

void loop() {
  //Static Stuff
  static Motor_Direction motor_action = Motor_OFF;
  static uint32_t up_active_loop_count = 0;
  static uint32_t down_active_loop_count = 0;
  static bool should_print = true; //helps for printing button presses in verbose mode

  //Count loops active for each button
  up_active_loop_count++;
  up_active_loop_count *= (up_button_state == HIGH); //Resets to zero if button is not pressed
  down_active_loop_count++;
  down_active_loop_count *= (up_button_state == HIGH); //Resets to zero if button is not pressed
  bool up_active = up_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool down_active = down_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool buttons_changing = (up_active_loop_count < LOOPS_UNTIL_BUTTON_PRESS_REGISTERED || down_active_loop_count < LOOPS_UNTIL_BUTTON_PRESS_REGISTERED);

  //FSM Logic
  if (VERBOSE) {
    Serial.print("Entering FSM Logic with state: ");
    Serial.println(curr_system_state);
  }


  /*
  if (buttons_changing) { //We only want to take actions after buttons have stabilized
    Serial.println("Buttons Changing -- Taking no action");
  } else {
    if (up_active && down_active) {
      if (should_print) {
        Serial.println("Both Buttons Clicked!");
        should_print = false;
      }
      if (up_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && down_active_loop_count > LOOPS_UNTIL_BUTTON_PRESS_REGISTERED) {
        //Both buttons have been pressed long enough to trigger setup mode
        //Enter setup mode
        Serial.println("Entering Setup Mode!");
      }
    } else if (up_active) {
      if (should_print){
        Serial.println("Up Clicked");
        should_print = false;
      }
      motor_action = Motor_UP;
      direction_manually_modified = true;
    } else if (down_active) {
      if (should_print){
        Serial.println("Down Clicked");
        should_print = false;
      }
      motor_action = Motor_DOWN;
      direction_manually_modified = true;
    } else {
      //No need to set motor_action, because default is off.
      should_print = true;
    }
  }
  */

  setMotor(motor_action);
  Watchdog.reset();
  delay(LOOP_INTERVAL);
}

void setMotor(Motor_Direction direction){
  if (INVERT_MOTOR_DIR) {
    direction = (Motor_Direction)((int)direction * -1); //flip from UP to DOWN without modifying the zero state (which is OFF)
  }
  if (last_motor_command == Motor_OFF && direction != Motor_OFF){
    //Motor just changed from off into some other state, log it.
    Serial.print("Motor Command Recieved!: ");
    Serial.println(direction);
  }
  last_motor_command = direction;
  last_motor_command_time = millis();
  switch (direction) {
    case Motor_UP:
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      break;
    case Motor_DOWN:
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      break;
    case Motor_OFF:
    default:
      if (INVERT_MOTOR_OFF_STATE){
        digitalWrite(MOTOR_PIN_1, HIGH);
        digitalWrite(MOTOR_PIN_2, HIGH);
      } else {
        digitalWrite(MOTOR_PIN_1, LOW);
        digitalWrite(MOTOR_PIN_2, LOW);
      }

      break;
  };
}

void motorSanityCheck() {
  //Make sure that if the motor is on, it had a recent command instructing it to turn on/off

}

void Button_ISR() {
  up_button_state = digitalRead(UP_PIN);
  down_button_state = digitalRead(DOWN_PIN);
  mils_at_last_physical_interaction = millis();
  //Print statements in ISRs are buggy/weird, so these are commented out. Uncomment with care.
  //Serial.print("In ISR: ");
  //Serial.print("up_button_state and down_button_state are: ");
  //Serial.print(up_button_state);
  //Serial.println(down_button_state);
  /*
  As a safety feature, when buttons change, we always turn the motor off.
  There are a few potential scenarios:
  1. The button was pressed before, and is now released
  --> In this case, we should turn the motor off anyway, so it's best to do it as soon as possible
  2. The button was not pressed before, and is now being pressed
  --> In this case, the motor is already off. No harm turning it off again (it will later get turned on by the main loop after the ISR)
  3. There are multiple buttons pushed, or some other strange configuration is happening
  --> If there are multiple buttons pushed, the motor should be off anyway. Strange configurations should stop the motor no matter what
  Therefore there's no harm in just calling setMotor(Motor_OFF) every time 
  */
  //setMotor(Motor_OFF);
}