#include <Adafruit_SleepyDog.h>

/*
TODO:
FSM Implementation
Buttons_Changing 
Timer for motor sanity check
Automatic threshold recalibration
*/

#define DEMO_MODE

//User-configurable actions
#define USE_TIMEBASED_ACTIONS false
#define USE_LIGHTBASED_ACTIONS true

//For integration and adjustments
#define INVERT_MOTOR_OFF_STATE false
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 800
#define INVERT_MOTOR_DIR false
#define UP_PIN 1
#define DOWN_PIN 0
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6

//For changing fundamental system behavior
#define LOOP_INTERVAL 5 //ms delay between loops -- DO NOT SET LOWER THAN 4MS (so that LOOPS_UNTIL_DOUBLEPRESS_REGISTERED < 255)
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1000 //DO NOT SET ABOVE 1000 UNLESS LOOP_INTERVAL IS >=10 (so that LOOPS_UNTIL_DOUBLEPRESS_REGISTERED < 255)
#define LOOPS_UNTIL_DOUBLEPRESS_REGISTERED MS_UNTIL_DOUBLEPRESS_REGISTERED / LOOP_INTERVAL
#define PHOTORESISTOR_SAMPLE_INTERVAL_MS 100 //ms between light sensor samples
#define MS_UNTIL_BUTTON_PRESS_REGISTERED 50
#define LOOPS_UNTIL_BUTTON_PRESS_REGISTERED MS_UNTIL_BUTTON_PRESS_REGISTERED / LOOP_INTERVAL
#define LOOPS_PER_PHOTORESISTOR_SAMPLE PHOTORESISTOR_SAMPLE_INTERVAL_MS / LOOP_INTERVAL
#define MINS_PER_SAMPLE_INTERVAL 5 //How long before we discard historical data
#define SAMPLES_TO_KEEP (1000 * 60 * MINS_PER_SAMPLE_INTERVAL) / PHOTORESISTOR_SAMPLE_INTERVAL_MS //How many ints do we need to store
#define SUPERSAMPLES_TO_KEEP (60 * 6) / MINS_PER_SAMPLE_INTERVAL //How many sample interval averages should we store (6hrs of historical data)
#define BUTTON_HISTORY_ARR_SIZE 255 //DON'T CHANGE - How far back should we store the state of the buttons (for detecting double-presses and such)
//BUTTON_HISTORY_ARR_SIZE *must* stay at 255 (or higher, I guess) unless you know what you're doing, as the circular buffer relies on unsigned integer overflow/underflow (which is defined behavior)

enum FSM_State {
  S_INIT,
  S_CALIBRATE_UP,
  S_CALIBRATE_DOWN,
  S_WAITING,
  S_MOTOR_MOVING_UP,
  S_MOTOR_MOVING_DOWN,
  S_MOVEMENT_DEFERRED_BY_MANUAL_PRESS, //Safety thing, don't want automatic movements while manual movements are happening
};

enum Motor_Direction {
  Motor_OFF = 0,
  Motor_UP = -1,
  Motor_DOWN = 1,
};

//Globals
FSM_State curr_system_state = S_INIT;
int16_t sample_arr[SAMPLES_TO_KEEP]; //These arrays use about 6KB (20%) of SRAM, but we have 32KB so it's no problem
int16_t supersample_arr[SUPERSAMPLES_TO_KEEP];
size_t sample_arr_idx = 0;
size_t supersample_arr_idx = 0;
uint32_t mils_at_last_time_update;
uint32_t mils_at_last_physical_interaction;
uint32_t micros_at_motor_start;
uint32_t micros_at_motor_end;
uint32_t micros_motor_raised; //counts the number of microseconds that the motor has been lifting (i.e. this tracks how far above the bottom we are)
int up_button_state;
int down_button_state;
uint8_t button_history_arr[BUTTON_HISTORY_ARR_SIZE + 16]; //Circular buffer for storing historical button press data and recognizing when both buttons are pushed
//16 bytes longer than it needs to be, because there's REALLY STRANGE memory issues here, so this is just padding.
uint8_t button_history_arr_idx = 0;
Motor_Direction last_motor_command;
uint32_t last_motor_command_time;
bool should_print = true;

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

    Serial.print("SUPERSAMPLES_TO_KEEP: ");
    Serial.println(SUPERSAMPLES_TO_KEEP, DEC); 

    Serial.print("LOOPS_PER_PHOTORESISTOR_SAMPLE: ");
    Serial.println(LOOPS_PER_PHOTORESISTOR_SAMPLE, DEC);

    Serial.print("BUTTON_HISTORY_ARR_SIZE: ");
    Serial.println(BUTTON_HISTORY_ARR_SIZE, DEC);
  }

  //Initialize all pins with correct pinmodes
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  //Clear arrays for samples and supersamples
  memset(sample_arr, -1, sizeof(int16_t) * SAMPLES_TO_KEEP);
  memset(supersample_arr, -1, sizeof(int16_t) * SUPERSAMPLES_TO_KEEP);
  //Clear history array
  memset(button_history_arr, 0x0, BUTTON_HISTORY_ARR_SIZE);

  //Setup button interrupts
  //Due to details in the interrupt handler, these cannot be different functions (Since both pins may be on the same pin bank)
  attachInterrupt(digitalPinToInterrupt(UP_PIN), Button_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), Button_ISR, CHANGE);

  if (USE_TIMEBASED_ACTIONS) {
    //Connect to WiFi and sync time
    //TODO
    mils_at_last_time_update = millis();
  }

  //Enable watchdog (So that things like WiFi connection don't cause the WDT to accidentaly trigger)
  //int watchdog_time = Watchdog.enable(1000); //ms before watchdog kicks
  int watchdog_time = -1;
  Serial.print("Enabled the watchdog with a time of: ");
  Serial.print(watchdog_time, DEC);
  Serial.println();
}

void loop() {
  Motor_Direction motor_action = Motor_OFF;
  bool direction_manually_modified = false;

  uint8_t history = 0x0;
  /*
  history is a bitfield
  [0:5] -> Unmapped/Reserved
  [6] -> down_button pressed in current state
  [7] -> up_button pressed in current state
  */
  if (up_button_state == HIGH) {
    history |= 0b00000001;
  }
  if (down_button_state == HIGH) {
    history |= 0b00000010;
  }

  button_history_arr[button_history_arr_idx] = history;

  //Check the history to determine button behavior
  uint8_t active_buttons = 0b00000011;
  uint8_t loop_limit = button_history_arr_idx - LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  //Serial.print("Before Loop: idx=");
  //Serial.println(button_history_arr_idx, DEC); 
  for (uint8_t i = button_history_arr_idx; i != loop_limit; i--){
    int curr_data = button_history_arr[i];
    active_buttons &= curr_data;
    /*
    Serial.print("Inloop, i / idx / limit = ");
    Serial.print(i, DEC);
    Serial.print(" | ");
    Serial.print(button_history_arr_idx, DEC);
    Serial.print(" | ");
    Serial.println(loop_limit, DEC);
    */
  }
  bool up_active = active_buttons & 0b00000001;
  bool down_active = (active_buttons & 0b00000010) >> 1;
  bool buttons_changing = active_buttons != (history & 0b00000011); //if buttons are in the process of being pressed, buttons_changing should be true.
  //Serial.print("After Loop: idx=");
  //Serial.println(button_history_arr_idx, DEC);  
  if (!buttons_changing) { //We only want to take actions after buttons have stabilized
    if (up_active && down_active) {
      if (should_print) {
        Serial.println("Both Buttons Clicked!");
        should_print = false;
      }
      //Do a deeper dive into the history to see if both buttons have been pressed for at least 
      active_buttons = 0b00000011; //reset active_buttons
      loop_limit = button_history_arr_idx - LOOPS_UNTIL_DOUBLEPRESS_REGISTERED;
      for (uint8_t i = button_history_arr_idx; i != loop_limit; i--){
        uint8_t curr_data = button_history_arr[i];
        active_buttons &= curr_data;
      }
      if (active_buttons == 0b00000011) {
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
  } else {
    Serial.println("Buttons Changing");
    /*
    Serial.print("Buttons Changing -- active_buttons / history / idx is: ");
    Serial.print(active_buttons, BIN);
    Serial.print(" | ");
    Serial.print(history, BIN);
    Serial.print(" | ");
    Serial.print(button_history_arr_idx, DEC);
    Serial.print(" | ");
    Serial.print((uint8_t)(button_history_arr_idx - (uint8_t)LOOPS_UNTIL_DOUBLEPRESS_REGISTERED), DEC);
    Serial.println();
    */

    //setMotor(Motor_OFF) //Motor should be off when buttons are changing
  }


  //Automatic actions should really be handled by the FSM system - Otherwise there's way too much interaction to deal with manually
  if (direction_manually_modified){
    //Skip the photoresistor crap, because manual input overrides it
    //Serial.println("Skipping Photoresistor checks");
  } else {
    int16_t photoresistor_reading = analogRead(PHOTORESISTOR_PIN);
    //Serial.print("Photoresistor Reading: ");
    //Serial.println(photoresistor_reading, DEC);
    if (photoresistor_reading > INITIAL_HIGH_LIGHT_THRESHOLD){
      motor_action = Motor_UP;
    } else if (photoresistor_reading < INITIAL_LOW_LIGHT_THRESHOLD) {
      motor_action = Motor_DOWN;
    }
  }

  //Watchdog.reset();
  //Serial.println("--Loop--");
  if (button_history_arr_idx == 0){
    Serial.println("Heartbeat");
  }

  setMotor(motor_action);
  button_history_arr_idx++;
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