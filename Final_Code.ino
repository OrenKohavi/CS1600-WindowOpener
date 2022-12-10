#include <Adafruit_SleepyDog.h>

/*
TODO:
FSM Implementation
Demo_mode definitions
Automatic threshold recalibration (Use WiFi and real-world time)
*/

/*
Use wifi to get the real world time, and get sunrise/sunset times
Recalibrate the thresholds for 
*/

//For demo/debugging
#define DEMO_MODE false
#define VERBOSE 2 //Higher number is more verbose. Ranges from [0-3], with 0 being only errors shown, and 3 being insane amounts of output.

//For integration and adjustments
#define CALIBRATE_AT_SUNRISE_SUNSET true
#define INVERT_MOTOR_OFF_STATE false
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 500
#define INVERT_MOTOR_DIR false
#define UP_PIN 0
#define DOWN_PIN 1
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6
#define SETUP_LED_PIN 14

//For changing fundamental system behavior
#define USE_WATCHDOG false
#define LOOP_INTERVAL 10 //ms delay between loops
#define MINS_PER_SAMPLE_INTERVAL 5 //How long do we record data for our moving average
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1500
#define PHOTORESISTOR_SAMPLE_INTERVAL_MS 100 //ms between light sensor samples
#define MS_UNTIL_BUTTON_PRESS_REGISTERED 50
#define LOOPS_UNTIL_DOUBLEPRESS_REGISTERED (MS_UNTIL_DOUBLEPRESS_REGISTERED / LOOP_INTERVAL)
#define LOOPS_UNTIL_BUTTON_PRESS_REGISTERED (MS_UNTIL_BUTTON_PRESS_REGISTERED / LOOP_INTERVAL)
#define LOOPS_PER_PHOTORESISTOR_SAMPLE (PHOTORESISTOR_SAMPLE_INTERVAL_MS / LOOP_INTERVAL)
#define SAMPLES_TO_KEEP ((1000 * 60 * MINS_PER_SAMPLE_INTERVAL) / PHOTORESISTOR_SAMPLE_INTERVAL_MS)
#define MOTOR_MICROS_MARGIN (1000 * LOOP_INTERVAL)

enum FSM_State {
  S_INIT,
  S_SETUP_MAX,
  S_SETUP_MIN,
  S_WAIT,
  S_BUTTON_UP,
  S_BUTTON_DOWN,
  S_LIGHT_UP,
  S_LIGHT_DOWN,
  S_UNRECOVERABLE_ERROR,
};

enum Motor_Direction {
  Motor_OFF = 0,
  Motor_UP = -1,
  Motor_DOWN = 1,
};

//Globals
FSM_State curr_system_state = S_INIT;
uint16_t sample_arr[SAMPLES_TO_KEEP];
uint32_t sample_arr_idx = 0;
uint32_t mils_at_last_physical_interaction;
uint32_t micros_at_motor_start;
uint32_t micros_at_motor_end;
uint32_t micros_motor_lowered; //counts the number of microseconds that the motor has been lowering (i.e. this tracks how far from the top we are)
uint32_t micros_motor_lowered_max; //Maximum micros of height the motor can be at (calibrated during setup)
PinStatus up_button_state;
PinStatus down_button_state;
Motor_Direction last_motor_command = Motor_OFF;

void setup() {
  Serial.begin(9600);
  while(!Serial); //Wait for Serial to initialize

  log(2, "LOOPS_UNTIL_BUTTON_PRESS_REGISTERED: %d\n", LOOPS_UNTIL_BUTTON_PRESS_REGISTERED);
  log(2, "LOOPS_UNTIL_DOUBLEPRESS_REGISTERED: %d\n", LOOPS_UNTIL_DOUBLEPRESS_REGISTERED);
  log(2, "LOOPS_PER_PHOTORESISTOR_SAMPLE: %d\n", LOOPS_PER_PHOTORESISTOR_SAMPLE);
  log(2, "SAMPLES_TO_KEEP: %d\n", SAMPLES_TO_KEEP);
  log(2, "MOTOR_MICROS_MARGIN: %d\n", MOTOR_MICROS_MARGIN);
  

  //Initialize all pins with correct pinmodes
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(SETUP_LED_PIN, OUTPUT);

  //Clear arrays for samples and supersamples
  memset(sample_arr, UINT16_MAX, sizeof(int16_t) * SAMPLES_TO_KEEP);

  //Setup button interrupts
  //Due to details in the interrupt handler, these cannot be different functions (Since both pins may be on the same pin bank)
  attachInterrupt(digitalPinToInterrupt(UP_PIN), Button_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), Button_ISR, CHANGE);

  //Connect to WiFi and sync time
  //TODO

  //Enable watchdog (So that things like WiFi connection don't cause the WDT to accidentaly trigger)
  if (USE_WATCHDOG) {
    int watchdog_time = Watchdog.enable(LOOP_INTERVAL * 2); //ms before watchdog kicks
    Serial.print("Enabled the watchdog with a time of: ");
    Serial.print(watchdog_time, DEC);
    Serial.println();
  }
}

void loop() {
  //Static Stuff
  static uint32_t up_active_loop_count = 0;
  static uint32_t down_active_loop_count = 0;
  static uint32_t loops_since_last_photoresistor_reading = UINT32_MAX; //set to INT_MAX at first, so that it always takes a photoresistor reading on the first loop
  static uint32_t photoresistor_avg;
  static bool doublepress_happened; //gets set to true when a doublepress occurrs, and should prevent motor movement if true.
  static bool doublepress_transition_allowed; //Set to false when a doublepress triggers a transition, and is only reset to true when no buttons are being clicked.

  //Count loops active for each button
  up_active_loop_count++;
  up_active_loop_count *= (up_button_state == HIGH); //Resets to zero if button is not pressed
  down_active_loop_count++;
  down_active_loop_count *= (down_button_state == HIGH); //Resets to zero if button is not pressed
  bool up_active = up_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool down_active = down_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool buttons_changing = (!up_active && up_active_loop_count > 0) || (!down_active && down_active_loop_count > 0);
  Motor_Direction motor_action = Motor_OFF;
  
  if (!up_active && !down_active) {
    doublepress_transition_allowed = true;
    doublepress_happened = false;
  }
  if (up_active && down_active) {
    doublepress_happened = true;
  }

  //Get reading from photoresistor, and calculate moving average
  if (loops_since_last_photoresistor_reading >= LOOPS_PER_PHOTORESISTOR_SAMPLE) {
    //Time to take a new sample (otherwise, just skip it and leave all variables unchanged)
    loops_since_last_photoresistor_reading = 0;
    uint16_t new_sample = analogRead(PHOTORESISTOR_PIN);
    sample_arr[sample_arr_idx] = new_sample;
    log(3, "New photoresistor sample is: %d | placed into sample_arr[%d] -- ", new_sample, sample_arr_idx);
    sample_arr_idx++; //increment circular buffer pointer
    sample_arr_idx = sample_arr_idx % SAMPLES_TO_KEEP;
    //Now calculate the average reading over the whole array (uninitialized values are 0xFFFF, which is not a valid photoresistor reading)
    uint32_t valid_samples = 0;
    uint32_t ignored_samples = 0;
    uint32_t sample_sum = 0; //accumulator variable (Will not overflow unless we have 2^16 samples, which memory can't even hold.)
    for (int i = 0; i < SAMPLES_TO_KEEP; i++) {
      uint16_t this_sample = sample_arr[i];
      if (this_sample == UINT16_MAX){
        ignored_samples++;
        continue;
      } else {
        valid_samples++;
        sample_sum += this_sample;
      }
    }
    photoresistor_avg = sample_sum / valid_samples;
    log(3, "Finished finding average of photoresistor readings. photoresistor_avg: %d | valid_samples: %d | ignored_samples: %d\n", photoresistor_avg, valid_samples, ignored_samples);
  } else {
    loops_since_last_photoresistor_reading++;
    log(3, "Not taking photoresistor sample. loops_since_last_photoresistor_reading: %d | photoresistor_avg: %d\n", loops_since_last_photoresistor_reading, photoresistor_avg);
  }

  //Time-calibration logic. 

  //FSM Logic
  motor_action = Motor_OFF;
  log(3, "Entering FSM... Current State: %d | up_active: %d | down_active: %d | buttons_changing: %d | photoresistor_avg: %d | micros_motor_lowered: %d | max_lower: %d\n",
    curr_system_state, up_active, down_active, buttons_changing, photoresistor_avg, micros_motor_lowered, micros_motor_lowered_max);

  switch (curr_system_state) {
    case S_INIT: //State 1
      //There's no logic here, INIT always goes into SETUP_MAX
      curr_system_state = S_SETUP_MAX;
      digitalWrite(SETUP_LED_PIN, HIGH);
      log(1, "Transitioning from S_INIT to S_SETUP_MAX\n");
      break;
    case S_SETUP_MAX:
    case S_SETUP_MIN:
      //Within the state, react to buttons
      //This button reaction is done here (rather than switching to S_BUTTON_UP/DOWN), because during setup there are no safeties on motor movement (since there are no limits yet)
      if (buttons_changing) {
        log(3, "Buttons Changing: Skipping SETUP logic\n");
        break; //While buttons are changing, nothing should be happening
      }
      if (up_active && !down_active && !doublepress_happened) {
        motor_action = Motor_UP;
      } else if (!up_active && down_active && !doublepress_happened) {
        motor_action = Motor_DOWN;
      }
      if (up_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && down_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && doublepress_transition_allowed) {
        //DOUBLEPRESS!
        if (curr_system_state == S_SETUP_MAX) {
          //Transition to S_SETUP_MIN
          log(1, "Transitioning from S_SETUP_MAX to S_SETUP_MIN\n");
          micros_motor_lowered = 0;
          doublepress_transition_allowed = false; //To prevent us from immidiatley switching out of S_SETUP_MIN right after this transition
          led_quick_flash(); //let the user know their action was successful
          digitalWrite(SETUP_LED_PIN, HIGH); //LED should still stay on for S_SETUP_MIN
          curr_system_state = S_SETUP_MIN;
          //Not much to record here, because the minimum is implicitly zero, and we are at the minimum.
        } else if (curr_system_state == S_SETUP_MIN) {
          //Transition to S_WAIT
          log(2, "Attempting to transition from S_SETUP_MIN to S_WAIT\n");
          doublepress_transition_allowed = false;
          micros_motor_lowered_max = micros_motor_lowered;
          log(1, "Setting micros_motor_lowered_max to: %d\n", micros_motor_lowered);
          log(2, "Conducting sanity check of limits...\n");
          if (MOTOR_MICROS_MARGIN >= micros_motor_lowered_max) { //MOTOR_MICROS_MARGIN is basically our zero -- it's the smallest step of motor precision that we have
            //Seems like the high and low limits are either the same, or inverted -- Not good, switch into error state.
            log(0, "Maximum and Minimum height not set correctly -- Power cycle to redo setup\n");
            curr_system_state = S_UNRECOVERABLE_ERROR;
          } else {
            log(2, "Safety check passed\n");
            log(1, "Transitioning from S_SETUP_MIN to S_WAIT\n");
            led_quick_flash(); //let the user know their action was successful
            digitalWrite(SETUP_LED_PIN, LOW); //LED should now be off for S_WAIT
            curr_system_state = S_WAIT;
          }
        } else {
          //Something's gone wrong, because we're only supposed to be in one of these states.
          log(0, "ERROR: Unexpected state of %d in setup logic\n", curr_system_state);
        }
      } else {
        log(3, "Not transitioning out of setup. Current (and next) state is: %d\n", curr_system_state);
      }
      break;
    case S_WAIT:
      //Motor should never be moving in the wait state
      motor_action = Motor_OFF;
      if (buttons_changing) {
        log(3, "Buttons changing, skipping S_WAIT logic\n");
        break;
      }
      if (up_active && !down_active){
        log(2, "Up button pressed\n");
        if (micros_motor_lowered > MOTOR_MICROS_MARGIN) {
          //There's still room to go before we hit the top, so all good.
          log(2, "Safety check passed\n");
          log(1, "Transitioning from S_WAIT to S_BUTTON_UP\n");
          curr_system_state = S_BUTTON_UP;
        }
      } else if (!up_active && down_active) {
        log(2, "Down button pressed\n");
        if (micros_motor_lowered + MOTOR_MICROS_MARGIN < micros_motor_lowered_max) {
          //There's still room to go before we hit the bottom, so all good.
          log(2, "Safety check passed\n");
          log(1, "Transitioning from S_WAIT to S_BUTTON_DOWN\n");
          curr_system_state = S_BUTTON_DOWN;
        }
      }
      break;
    case S_BUTTON_UP:
      if (doublepress_happened || !up_active) {
        //Go back to the wait state (wait state can figure out stuff like double-presses, so no need to duplicate that logic here)
        log(1, "Transitioning from S_BUTTON_UP to S_WAIT");
        motor_action = Motor_OFF;
        curr_system_state = S_WAIT;
      } else {
        //Remain in the S_BUTTON_UP state, and command the motor to go up
        log(2, "Staying in S_BUTTON_UP");
        motor_action = Motor_UP;
        curr_system_state = S_BUTTON_UP; //Probably redundant, but whatever it doesn't hurt
      }
      break;
    case S_BUTTON_DOWN:
      break;
    case S_LIGHT_UP:
      break;
    case S_LIGHT_DOWN:
      break;
    case S_UNRECOVERABLE_ERROR:
      //For cases where the device was setup wrong, or otherwise needs to be power cycled
      //Just blink the LED and disable the device by going into an infinite loop
      while(true) {
        digitalWrite(SETUP_LED_PIN, LOW);
        delay(250);
        digitalWrite(SETUP_LED_PIN, HIGH);
        delay(250);
      }
      break;
    default:
      log(0, "ERROR: Unrecognized State of: %d\n", curr_system_state);
  };
  log(3, "After FSM, state is: %d\n", curr_system_state);

  //End of the loop -- Reset the Watchdog and apply motor action.
  setMotor(motor_action);
  if (USE_WATCHDOG) {
    Watchdog.reset();
  }
  delay(LOOP_INTERVAL);
  log(3, "---- LOOP END ----\n");
}

void setMotor(Motor_Direction direction){
  if (INVERT_MOTOR_DIR) {
    direction = (Motor_Direction)((int)direction * -1); //flip from UP to DOWN without modifying the zero state (which is OFF)
  }
  if (last_motor_command == Motor_OFF && direction != Motor_OFF){
    //Motor just changed from off into some other state, log it with less verbosity required
    log(1, "Motor Recieved New Command: %d\n", direction);
  } else {
    log(3, "Motor Command Recieved: %d\n", direction);
  }
  last_motor_command = direction;
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

void Button_ISR() {
  up_button_state = digitalRead(UP_PIN);
  down_button_state = digitalRead(DOWN_PIN);
  mils_at_last_physical_interaction = millis();
}