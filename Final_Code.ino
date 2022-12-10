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
#define DEMO_MODE
#define VERBOSE 2 //Higher number is more verbose. Ranges from [0-3], with 0 being only errors shown, and 3 being insane amounts of output.

//For integration and adjustments
#define CALIBRATE_AT_SUNRISE_SUNSET true
#define INVERT_MOTOR_OFF_STATE false
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 500
#define UP_PIN 1
#define DOWN_PIN 0
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6
#define SETUP_LED_PIN 14

//For changing fundamental system behavior
#ifdef DEMO_MODE
//Special definitions so that the product can be demo'ed easily
#define MINS_PER_SAMPLE_INTERVAL 1
#define PHYSICAL_INTERACTION_LOCKOUT_SECONDS 30
#else
//Normal operation
#define MINS_PER_SAMPLE_INTERVAL 5 //How long do we record data for our moving average
#define PHYSICAL_INTERACTION_LOCKOUT_SECONDS 60 * 60 //One hour
#endif

#define USE_WATCHDOG false
#define LOOP_INTERVAL 5 //ms delay between loops
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1500
#define PHOTORESISTOR_SAMPLE_INTERVAL_MS 100 //ms between light sensor samples
#define MS_UNTIL_BUTTON_PRESS_REGISTERED 60
#define LOOPS_UNTIL_DOUBLEPRESS_REGISTERED (((MS_UNTIL_DOUBLEPRESS_REGISTERED - LOOP_INTERVAL) / LOOP_INTERVAL) + 1)
#define LOOPS_UNTIL_BUTTON_PRESS_REGISTERED (((MS_UNTIL_BUTTON_PRESS_REGISTERED - LOOP_INTERVAL) / LOOP_INTERVAL) + 1)
#define LOOPS_PER_PHOTORESISTOR_SAMPLE (PHOTORESISTOR_SAMPLE_INTERVAL_MS / LOOP_INTERVAL)
#define SAMPLES_TO_KEEP ((1000 * 60 * MINS_PER_SAMPLE_INTERVAL) / PHOTORESISTOR_SAMPLE_INTERVAL_MS)
#define MOTOR_MICROS_MARGIN (1000 * LOOP_INTERVAL * 2)

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
  Motor_UP = 1,
  Motor_DOWN = -1,
  Motor_ERR = 0xFF,
};

//Globals
FSM_State curr_system_state = S_INIT;
uint16_t sample_arr[SAMPLES_TO_KEEP];
uint32_t sample_arr_idx = 0;
uint32_t mils_at_last_physical_interaction;
uint32_t micros_at_motor_start;
uint32_t micros_at_motor_end;
int32_t micros_motor_lowered; //counts the number of microseconds that the motor has been lowering (i.e. this tracks how far from the top we are)
int32_t micros_motor_lowered_max; //Maximum micros of height the motor can be at (calibrated during setup)
PinStatus up_button_state;
PinStatus down_button_state;

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
  log(3, "Up active loop count: %d | down_active_loop_count: %d\n", up_active_loop_count, down_active_loop_count);
  //Set some variables based on which buttons are up/down
  if (!up_active && !down_active) {
    doublepress_transition_allowed = true;
    doublepress_happened = false;
    log(3, "Both buttons free\n");
  }
  if (up_active && down_active) {
    doublepress_happened = true;
    log(3, "Doublepress detected\n");
  }

  uint32_t photoresistor_avg = readPhotoresistor();

  //Time-calibration logic.
  if (CALIBRATE_AT_SUNRISE_SUNSET) {
    timeCalibrate();
  }

  log(3, "Before FSM, Current State: %d | up_active: %d | down_active: %d | buttons_changing: %d | photoresistor_avg: %d | micros_motor_lowered: %d | max_lower: %d\n",
    curr_system_state, up_active, down_active, buttons_changing, photoresistor_avg, micros_motor_lowered, micros_motor_lowered_max);
  motor_action = fsmUpdate(up_active, down_active, buttons_changing, photoresistor_avg, doublepress_happened, doublepress_transition_allowed, up_active_loop_count, down_active_loop_count);
  log(3, "After FSM, state is: %d\n", curr_system_state);
  setMotor(motor_action);

  //End of the loop -- Reset the Watchdog and delay
  if (USE_WATCHDOG) {
    Watchdog.reset();
  }
  delay(LOOP_INTERVAL);
  log(3, "---- LOOP END ----\n");
}

void timeCalibrate() {
  //Use the clock to determine if the time of day is sunset or sunrise. If so, set appropriate photoresistor threshold to current average.
}

Motor_Direction fsmUpdate(bool up_active, bool down_active, bool buttons_changing,
    uint32_t photoresistor_avg, bool doublepress_happened, bool doublepress_transition_allowed, uint32_t up_active_loop_count, uint32_t down_active_loop_count) {
  Motor_Direction motor_action = Motor_OFF;
  FSM_State next_system_state = curr_system_state;
  switch (curr_system_state) {
    case S_INIT: //State 1
      //There's no logic here, INIT always goes into SETUP_MAX
      next_system_state = S_SETUP_MAX;
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
          next_system_state = S_SETUP_MIN;
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
            next_system_state = S_UNRECOVERABLE_ERROR;
          } else {
            log(2, "Safety check passed\n");
            log(1, "Transitioning from S_SETUP_MIN to S_WAIT\n");
            led_quick_flash(); //let the user know their action was successful
            digitalWrite(SETUP_LED_PIN, LOW); //LED should now be off for S_WAIT
            next_system_state = S_WAIT;
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
          next_system_state = S_BUTTON_UP;
        } else {
          log(2, "Shade at maximum height\n");
          next_system_state = S_WAIT;
        }
      } else if (!up_active && down_active) {
        log(2, "Down button pressed\n");
        if (micros_motor_lowered + MOTOR_MICROS_MARGIN < micros_motor_lowered_max) {
          //There's still room to go before we hit the bottom, so all good.
          log(2, "Safety check passed\n");
          log(1, "Transitioning from S_WAIT to S_BUTTON_DOWN\n");
          next_system_state = S_BUTTON_DOWN;
        } else {
          log(2, "Shade at minimum height\n");
          next_system_state = S_WAIT;
        }
      } else {
        log(3, "No buttons active, staying in S_WAIT\n");
        next_system_state = S_WAIT;
      }
      break;
    case S_BUTTON_UP:
      if (doublepress_happened || !up_active) {
        //Go back to the wait state (wait state can figure out stuff like double-presses, so no need to duplicate that logic here)
        log(1, "Transitioning from S_BUTTON_UP to S_WAIT");
        motor_action = Motor_OFF;
        next_system_state = S_WAIT;
      } else {
        //Check shade position to make sure it's safe to keep going up:
        if (micros_motor_lowered > MOTOR_MICROS_MARGIN){
          //Remain in the S_BUTTON_UP state, and command the motor to go up
          log(2, "Staying in S_BUTTON_UP");
          motor_action = Motor_UP;
          next_system_state = S_BUTTON_UP; //Probably redundant, but whatever it doesn't hurt
        } else {
          log(1, "Shade at maximum height, transitioning from S_BUTTON_UP to S_WAIT\n");
          log(2, "micros_motor_lowered: %d\n", micros_motor_lowered);
          motor_action = Motor_OFF;
          next_system_state = S_WAIT;
        }
      }
      break;
    case S_BUTTON_DOWN:
      //Basically the same as S_BUTTON_UP
      if (doublepress_happened || !down_active) {
        //Go back to the wait state (wait state can figure out stuff like double-presses, so no need to duplicate that logic here)
        log(1, "Transitioning from S_BUTTON_DOWN to S_WAIT");
        motor_action = Motor_OFF;
        next_system_state = S_WAIT;
      } else {
        //Check shade position to make sure it's safe to keep going down:
        if (micros_motor_lowered + MOTOR_MICROS_MARGIN < micros_motor_lowered_max){
          //Remain in the S_BUTTON_DOWN state, and command the motor to go down
          log(2, "Staying in S_BUTTON_DOWN");
          motor_action = Motor_DOWN;
          next_system_state = S_BUTTON_DOWN; //Probably redundant, but whatever it doesn't hurt
        } else {
          log(1, "Shade at maximum height, transitioning from S_BUTTON_DOWN to S_WAIT\n");
          log(2, "micros_motor_lowered: %d | threshold (micros_motor_lowered_max): %d\n", micros_motor_lowered, micros_motor_lowered_max);
          motor_action = Motor_OFF;
          next_system_state = S_WAIT;
        }
      }
      break;
    case S_LIGHT_UP:
      break;
    case S_LIGHT_DOWN:
      break;
    case S_UNRECOVERABLE_ERROR:
      //For cases where the device was setup wrong, or otherwise needs to be power cycled
      //Just blink the LED and disable the device by going into an infinite loop
      setMotor(Motor_OFF); //Make sure motor is not still running
      errorLoop();
      break;
    default:
      log(0, "ERROR: Unrecognized State of: %d\n", curr_system_state);
  };
  curr_system_state = next_system_state;
  return motor_action;
}

int readPhotoresistor() {
  static uint32_t loops_since_last_photoresistor_reading = UINT32_MAX; //set to INT_MAX at first, so that it always takes a photoresistor reading on the first loop
  static uint32_t photoresistor_avg;
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
  return photoresistor_avg;
}

void setMotor(Motor_Direction direction){
  static bool tracking_movement = false;
  static uint32_t micros_at_movement;
  static Motor_Direction last_motor_command = Motor_OFF;
  if (last_motor_command == Motor_OFF && direction != Motor_OFF){
    //Motor just changed from off into some other state, log it with less verbosity required
    log(1, "Motor Recieved New Command: %d\n", direction);
    //set micros_at_movement_start
  } else {
    log(3, "Motor Command Recieved: %d\n", direction);
  }
  //Some sanity checking to make sure the motor stops before it turns in the opposite direction
  //This also lets us greatly simplify the micros-based logic
  if (abs(last_motor_command) == 1 && abs(direction) == 1 && last_motor_command != direction) {
    //Bad, because this means the motor is switching directions without stopping
    //Not only will this mess up the micros-based counter, but this behavior never happens in the FSM, so it's an error
    log(0, "Motor recieved opposite-direction commands without a stop inbetweeen (last command: %d, this command: %d) | Unrecoverable Error\n", last_motor_command, direction);
    direction = Motor_ERR; //Make sure it's not a defined direction, so that we go into the error case 
  }
  if (tracking_movement) {
    log(2, "Moving in direction %d, calculating offset change\n", last_motor_command);
    uint32_t time_now = micros();
    uint32_t motor_on_time = time_now - micros_at_movement;
    //TODO: Handle overflow of micros(), which happens every hour-ish
    log(2, "Calculated that this movement step has been ocurring for %d micros\n", motor_on_time);
    if (abs((int)last_motor_command) != 1) {
      log(0, "ERROR: last_motor_command was not a direction, so cannot calculate motor offset | Unrecoverable Error\n");
      tracking_movement = false;
      setMotor(Motor_ERR);
    }
    //Now we can assume that last_motor_command is either -1 or 1, so we can do math! (-1 is down, 1 is up)
    int motor_offset = motor_on_time * -1 * (int)last_motor_command; //If commmand was up, then offset will be negative. (otherwise unchanged)
    log(2, "Applying offset of %d | old micros_motor_lowered: %d | new micros_motor_lowered: %d\n",motor_offset, micros_motor_lowered, micros_motor_lowered + motor_offset);
    micros_motor_lowered += motor_offset;
  }
  switch (direction) {
    case Motor_UP:
      tracking_movement = true;
      micros_at_movement = micros(); //Potentially, put some logic in here that decrements micros_at_movement by the amount of time it takes for the tracking_movement block to execute?
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      break;
    case Motor_DOWN:
      tracking_movement = true;
      micros_at_movement = micros();
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      break;
    case Motor_OFF:
      tracking_movement = false;
      if (INVERT_MOTOR_OFF_STATE){
        digitalWrite(MOTOR_PIN_1, HIGH);
        digitalWrite(MOTOR_PIN_2, HIGH);
      } else {
        digitalWrite(MOTOR_PIN_1, LOW);
        digitalWrite(MOTOR_PIN_2, LOW);
      }
      break;
    default:
      log(0, "ERROR: Unrecognized motor state of %d | Unrecoverable error\n", direction);
      //no break, we want to pass into the error case
    case Motor_ERR:
      log(1, "Unrecoverable Error in Motor\n");
      //Make sure motor is off before going into error state
      if (INVERT_MOTOR_OFF_STATE){
        digitalWrite(MOTOR_PIN_1, HIGH);
        digitalWrite(MOTOR_PIN_2, HIGH);
      } else {
        digitalWrite(MOTOR_PIN_1, LOW);
        digitalWrite(MOTOR_PIN_2, LOW);
      }
      curr_system_state = S_UNRECOVERABLE_ERROR;
      errorLoop();
      break;
  };
  last_motor_command = direction;
}

void Button_ISR() {
  up_button_state = digitalRead(UP_PIN);
  down_button_state = digitalRead(DOWN_PIN);
  mils_at_last_physical_interaction = millis();
}