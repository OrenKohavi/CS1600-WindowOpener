#include <Adafruit_SleepyDog.h>
#include <time.h>

//For demo/debugging
#define DEMO_MODE
#define VERBOSE 1 //Higher number is more verbose. Ranges from [0-3], with 0 being only errors shown, and 3 being insane amounts of output.

//For integration and adjustments
#define CALIBRATE_AT_SUNRISE_SUNSET true
#define HOURS_AFTER_SUNSET_CALIBRATION 2
#define HOURS_BEFORE_SUNRISE_CALIBRATION 0
#define INVERT_MOTOR_OFF_STATE true
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 500
#define UP_PIN 1
#define DOWN_PIN 0
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6
#define SETUP_LED_PIN 14
#define MOTOR_STRENGTH_ADJUSTMENT_CONSTANT 1.02 //Numbers above 1 counteract the 'top' getting lower, numbers below 1 counteract the 'top' getting higher.

//For changing fundamental system behavior
#ifdef DEMO_MODE
//Special definitions so that the product can be demo'ed easily
#define MOVING_AVERAGE_INTERVAL_SECONDS 30
#define PHYSICAL_INTERACTION_LOCKOUT_SECONDS 30
#endif

#ifndef DEMO_MODE
//Normal operation
#define MOVING_AVERAGE_INTERVAL_SECONDS (10 * 60) //10 mins moving average
#define PHYSICAL_INTERACTION_LOCKOUT_SECONDS (60 * 60) //One hour
#endif

#define SUNRISE_LAT 41.82 //Lat and Long of Providence, RI
#define SUNRISE_LON -71.41 //Doesn't have to be perfectly precise, because it's just the sunrise/sunset time: It can vary by a few mins without issue.
#define TIMEZONE_OFFSET 7
#define INVERT_SHADE_LIGHT_BEHAVIOR false //when false, shades will raise when bright, and lower when dim. When true, shades will lower when bright, and raise when dim.
#define USE_WATCHDOG true
#define LOOP_INTERVAL 5 //ms delay between loops (At 2ms and below, behavior starts getting funky)
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1500
#define PHOTORESISTOR_SAMPLE_INTERVAL_MS 100 //ms between light sensor samples
#define MS_UNTIL_BUTTON_PRESS_REGISTERED 60
#define LOOPS_UNTIL_DOUBLEPRESS_REGISTERED (((MS_UNTIL_DOUBLEPRESS_REGISTERED - LOOP_INTERVAL) / LOOP_INTERVAL) + 1)
#define LOOPS_UNTIL_BUTTON_PRESS_REGISTERED (((MS_UNTIL_BUTTON_PRESS_REGISTERED - LOOP_INTERVAL) / LOOP_INTERVAL) + 1)
#define LOOPS_PER_PHOTORESISTOR_SAMPLE (PHOTORESISTOR_SAMPLE_INTERVAL_MS / LOOP_INTERVAL)
#define SAMPLES_TO_KEEP ((1000 * MOVING_AVERAGE_INTERVAL_SECONDS) / PHOTORESISTOR_SAMPLE_INTERVAL_MS)
#define MOTOR_MICROS_MARGIN (1000 * LOOP_INTERVAL * 2)

enum FSM_State {
  S_INIT = 1,
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
uint16_t photoresistor_bright_threshold = INITIAL_HIGH_LIGHT_THRESHOLD;
uint16_t photoresistor_dim_threshold = INITIAL_LOW_LIGHT_THRESHOLD;
uint16_t sample_arr[SAMPLES_TO_KEEP];
uint32_t sample_arr_idx = 0;
uint32_t mils_at_last_physical_interaction;
uint32_t micros_at_motor_start;
uint32_t micros_at_motor_end;
int32_t micros_motor_lowered; //counts the number of microseconds that the motor has been lowering (i.e. this tracks how far from the top we are)
int32_t micros_motor_lowered_max; //Maximum micros of height the motor can be at (calibrated during setup)
PinStatus up_button_state;
PinStatus down_button_state;
bool use_wifi = CALIBRATE_AT_SUNRISE_SUNSET;
time_t unix_epoch_time_at_startup;
uint32_t mils_at_startup;
tm sunset_time;
tm sunrise_time;
tm current_time;

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
  setMotor(Motor_OFF); //Make sure the motor doesn't initialize into a moving state

  //Clear arrays for samples and supersamples
  memset(sample_arr, UINT16_MAX, sizeof(uint16_t) * SAMPLES_TO_KEEP);

  //Setup button interrupts
  //Due to details in the interrupt handler, these cannot be different functions (Since both pins may be on the same pin bank)
  //Still vital to set them both, because they *might* be on pins that support two different interrupts, we just don't know.
  attachInterrupt(digitalPinToInterrupt(UP_PIN), Button_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), Button_ISR, CHANGE);

  /* WIFI Stuff */
  //Connect to WiFi
  if(use_wifi && setup_wifi()) {
    log(1, "Wifi Connection Completed\n");
  } else {
    log(0, "ERROR: Could not connect to wifi\n");
    use_wifi = false;
  }
  //Now get the current time
  if(use_wifi && get_current_time()) {
    log(1, "Current Time Fetched\n");
    //Adjust for timezone
    unix_epoch_time_at_startup += (TIMEZONE_OFFSET * 60 * 60);
  } else {
    log(0, "ERROR: Could not fetch current time\n");
    use_wifi = false;
  }
  //Now get sunset and sunrise times:
  if(use_wifi && get_sunrise_sunset_times()) {
    log(1, "Sunrise and Sunset fetched\n");
  } else {
    log(0, "ERROR: Could not fetch sunrise and sunset times\n");
    use_wifi = false;
  }

  if (use_wifi) {
    log(1, "Wifi Setup Succeeded!\n");
    log(2, "Sunrise is at: %d:%d\n", sunrise_time.tm_hour, sunrise_time.tm_min);
    log(2, "Sunset is at: %d:%d\n", sunset_time.tm_hour, sunset_time.tm_min);
    current_time = *localtime(&unix_epoch_time_at_startup);
    log(2, "Time at startup is: %d:%d\n", current_time.tm_hour, current_time.tm_min);
  }

  //Enable watchdog (at the end of setup so that things like WiFi connection don't cause the WDT to accidentaly trigger)
  if (USE_WATCHDOG) {
    int watchdog_time = Watchdog.enable(max(1000, LOOP_INTERVAL * 4));
    log(1, "Enabled WDT with period of %d\n", watchdog_time);
  }
  log(1, "Startup Completed\n");
}

void loop() {
  //Static Stuff
  static uint32_t up_active_loop_count = 0;
  static uint32_t down_active_loop_count = 0;
  static uint32_t loop_counter = 0; //Just for printing, you can ignore this.

  loop_counter++;
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
  if (up_active_loop_count > 0 || down_active_loop_count > 0) {
    //This is already done in the ISR, but this case also deals with long button presses (where the ISR only triggers on a button change)
    mils_at_last_physical_interaction = millis();
  }

  uint32_t photoresistor_avg = readPhotoresistor();

  //Time-calibration logic.
  time_t epoch_time_now = unix_epoch_time_at_startup + ((millis() - mils_at_startup) / 1000);
  current_time = *localtime(&epoch_time_now); //This is outside the timeCalibrate function because it's used for timestamps on printouts
  if (use_wifi) {
    timeCalibrate(current_time, photoresistor_avg);
  }

  log(3, "Before FSM, Current State: %d | up_active: %d | down_active: %d | buttons_changing: %d | photoresistor_avg: %d | micros_motor_lowered: %d | max_lower: %d\n",
    curr_system_state, up_active, down_active, buttons_changing, photoresistor_avg, micros_motor_lowered, micros_motor_lowered_max);
  motor_action = fsmUpdate(up_active, down_active, buttons_changing, photoresistor_avg, up_active_loop_count, down_active_loop_count);
  log(3, "After FSM, state is: %d\n", curr_system_state);
  setMotor(motor_action);

  //End of the loop -- Reset the Watchdog and delay
  if (USE_WATCHDOG) {
    Watchdog.reset();
    log(3, "Reset Watchdog\n");
  }
  if (loop_counter % 200 == 0 && true) {
    log(1, "[%d:%d] photoresistor avg: %d | time since last physical interaction: %d\n", current_time.tm_hour, current_time.tm_min, photoresistor_avg, (millis() - mils_at_last_physical_interaction) / 1000);
  }
  delay(LOOP_INTERVAL);
  log(3, "---- LOOP END ----\n");
}

void timeCalibrate(tm current_time, uint32_t photoresistor_avg) {
  //Use the clock to determine if the time of day is sunset or sunrise. If so, set appropriate photoresistor threshold to current average.
  if (same_hour_minute(current_time, sunrise_time)) {
    //Reset thresholds for bright light
    photoresistor_bright_threshold = photoresistor_avg;
  } else if (same_hour_minute(current_time, sunset_time)) {
    //Reset thresholds for low light
    photoresistor_dim_threshold = photoresistor_avg;
  }
}

Motor_Direction fsmUpdate(bool up_active, bool down_active, bool buttons_changing, uint32_t photoresistor_avg, uint32_t up_active_loop_count, uint32_t down_active_loop_count) {
  static bool doublepress_transition_allowed = false;
  static bool doublepress_happened = false;
  Motor_Direction motor_action = Motor_OFF;
  FSM_State next_system_state = curr_system_state;

  if (!up_active && !down_active) {
    doublepress_transition_allowed = true;
    doublepress_happened = false;
    log(3, "Both buttons free\n");
  }
  if (up_active && down_active) {
    doublepress_happened = true;
    log(3, "Doublepress detected\n");
  }

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
        if (curr_system_state == S_SETUP_MIN && !(shade_not_at_max())) {
          //No room to go up
          log(2, "Maximum height -- Cannot go up\n");
        } else {
          motor_action = Motor_UP;
        }
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
      if (up_active && down_active) {
        log(3, "Both buttons pushed (transition allowed? %d)\n", doublepress_transition_allowed);
        if (up_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && down_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && doublepress_transition_allowed) {
          //Double-press in the wait state means we need to go back to setup
          log(1, "Transitioning from S_WAIT to S_SETUP_MAX due to button double-press\n");
          doublepress_transition_allowed = false; //To prevent us from immidiatley switching out of the next state right after this transition
          digitalWrite(SETUP_LED_PIN, HIGH); //turn on LED so that user knows they're back in setup mode
          next_system_state = S_SETUP_MAX;
        } else {
          log(3, "Doublepress detected, waiting for time requirement (pressed for %d / %d loops)\n", min(up_active_loop_count, down_active_loop_count), LOOPS_UNTIL_DOUBLEPRESS_REGISTERED);
        }        
      } else if (up_active && !down_active && !doublepress_happened){
        log(2, "Up button pressed\n");
        if (shade_not_at_max()) {
          //There's still room to go before we hit the top, so all good.
          log(2, "Safety check passed\n");
          log(1, "Transitioning from S_WAIT to S_BUTTON_UP\n");
          next_system_state = S_BUTTON_UP;
        } else {
          log(2, "Shade at maximum height\n");
          next_system_state = S_WAIT;
        }
      } else if (!up_active && down_active && !doublepress_happened) {
        log(2, "Down button pressed\n");
        if (shade_not_at_min()) {
          //There's still room to go before we hit the bottom, so all good.
          log(2, "Safety check passed\n");
          log(1, "Transitioning from S_WAIT to S_BUTTON_DOWN\n");
          next_system_state = S_BUTTON_DOWN;
        } else {
          log(2, "Shade at minimum height\n");
          next_system_state = S_WAIT;
        }
      } else {
        log(3, "No buttons active\n");
      }
      //Logic for light-based transitions:
      if ((millis() - mils_at_last_physical_interaction) / 1000 < PHYSICAL_INTERACTION_LOCKOUT_SECONDS) {
        //In lockout period, because a physical interaction has happened recently, so do nothing
        log(3, "Skipping light-based logic due to lockout period. Seconds since interaction: %d\n", (millis() - mils_at_last_physical_interaction) / 1000);
        break;
      }
      //Lockout is no longer a concern if we get here
      if (!INVERT_SHADE_LIGHT_BEHAVIOR){
        if (photoresistor_avg > photoresistor_bright_threshold && shade_not_at_max()) {
          //raise blind when bright
          log(1, "Transitioning from S_WAIT to S_LIGHT_UP\n");
          log(2, "photoresistor_avg is: %d\n", photoresistor_avg);
          next_system_state = S_LIGHT_UP;
        } else if (photoresistor_avg < photoresistor_dim_threshold && shade_not_at_min()) {
          //lower blind when dark
          log(1, "Transitioning from S_WAIT to S_LIGHT_DOWN\n");
          log(2, "photoresistor_avg is: %d\n", photoresistor_avg);
          next_system_state = S_LIGHT_DOWN;
        }
      } else {
        if (photoresistor_avg > photoresistor_bright_threshold && shade_not_at_min()) {
          //lower blind when bright
          log(1, "Transitioning from S_WAIT to S_LIGHT_DOWN\n");
          log(2, "photoresistor_avg is: %d\n", photoresistor_avg);
          next_system_state = S_LIGHT_DOWN;
        } else if (photoresistor_avg < photoresistor_dim_threshold && shade_not_at_max()) {
          //raise blind when dark
          log(1, "Transitioning from S_WAIT to S_LIGHT_UP\n");
          log(2, "photoresistor_avg is: %d\n", photoresistor_avg);
          next_system_state = S_LIGHT_UP;
        }
      }
      break;
    case S_BUTTON_UP:
      if (doublepress_happened || !up_active) {
        //Go back to the wait state (wait state can figure out stuff like double-presses, so no need to duplicate that logic here)
        log(1, "Transitioning from S_BUTTON_UP to S_WAIT\n");
        motor_action = Motor_OFF;
        next_system_state = S_WAIT;
      } else {
        //Check shade position to make sure it's safe to keep going up:
        if (shade_not_at_max()){
          //Remain in the S_BUTTON_UP state, and command the motor to go up
          log(3, "Staying in S_BUTTON_UP\n");
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
        log(1, "Transitioning from S_BUTTON_DOWN to S_WAIT\n");
        motor_action = Motor_OFF;
        next_system_state = S_WAIT;
      } else {
        //Check shade position to make sure it's safe to keep going down:
        if (shade_not_at_min()){
          //Remain in the S_BUTTON_DOWN state, and command the motor to go down
          log(3, "Staying in S_BUTTON_DOWN\n");
          motor_action = Motor_DOWN;
          next_system_state = S_BUTTON_DOWN; //Probably redundant, but whatever it doesn't hurt
        } else {
          log(1, "Shade at minimum height, transitioning from S_BUTTON_DOWN to S_WAIT\n");
          log(2, "micros_motor_lowered: %d | threshold (micros_motor_lowered_max): %d\n", micros_motor_lowered, micros_motor_lowered_max);
          motor_action = Motor_OFF;
          next_system_state = S_WAIT;
        }
      }
      break;
    case S_LIGHT_UP:
    case S_LIGHT_DOWN:
      if (up_active || down_active){
        //Physical input overrides anything, go back to waiting
        log(1, "Transitioning from S_LIGHT_(UP/DOWN) to S_WAIT due to user input\n");
        motor_action = Motor_OFF;
        next_system_state = S_WAIT;
      } else {
        //Check if the shade is at the max/min
        if (curr_system_state == S_LIGHT_UP){
          if (shade_not_at_max()) {
            //All good, there's still room to raise
            log(3, "Staying in S_LIGHT_UP, there's still room to raise\n");
            motor_action = Motor_UP;
            next_system_state = S_LIGHT_UP;
          } else {
            log(1, "Transitioning from S_LIGHT_UP to S_WAIT due to blind reaching limit\n");
            motor_action = Motor_OFF;
            next_system_state = S_WAIT;
          }
        } else {
          if (shade_not_at_min()) {
            //All good, there's still room to lower
            log(3, "Staying in S_LIGHT_DOWN, there's still room to lower\n");
            motor_action = Motor_DOWN;
            next_system_state = S_LIGHT_DOWN;
          } else {
            log(1, "Transitioning from S_LIGHT_DOWN to S_WAIT due to blind reaching limit\n");
            motor_action = Motor_OFF;
            next_system_state = S_WAIT;
          }
        }
      }
      break;
    case S_UNRECOVERABLE_ERROR:
      //For cases where the device was setup wrong, or otherwise needs to be power cycled
      //Just blink the LED and disable the device by going into an infinite loop
      log(0, "ERROR: Unrecoverable Error State Reached\n");
      setMotor(Motor_OFF); //Make sure motor is not still running
      errorLoop();
      break;
    default:
      log(0, "ERROR: Unrecognized State of: %d\n", curr_system_state);
  };
  log(3, "Setting state from: %d to: %d\n", curr_system_state, next_system_state);
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
      if (this_sample >= 1024){ //Photoresistor outputs a 10-bit number, so 1023 is the highest it can go
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
  static uint32_t motor_cycle_counter = 0; //Used only for printing stuff nicely, can be ignored.
  motor_cycle_counter++;
  if (last_motor_command == Motor_OFF && direction != Motor_OFF){
    //Motor just changed from off into some other state, log it with less verbosity required
    log(2, "Motor Recieved New Command: %d\n", direction);
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
    log(3, "Moving in direction %d, calculating offset change\n", last_motor_command);
    uint32_t time_now = micros();
    uint32_t motor_on_time = time_now - micros_at_movement;
    //TODO: Handle overflow of micros(), which happens every hour-ish
    if (motor_on_time > 0xF0000000) {
      //Assume this is overflow -- We got unlucky :(
      //To reach this case without overflow, the motor must be on for over 4 minutes, which seems incredibly unlikely.
      //No handling of this case yet, so just error unfortunately
      log(0, "ERROR: micros variable overflowed and we didn't have time to put in a special case for this\n");
      tracking_movement = false;
      setMotor(Motor_ERR);
    }

    log(3, "Calculated that this movement step has been ocurring for %d micros\n", motor_on_time);
    if (abs((int)last_motor_command) != 1) {
      log(0, "ERROR: last_motor_command was not a direction, so cannot calculate motor offset | Unrecoverable Error\n");
      tracking_movement = false;
      setMotor(Motor_ERR);
    }
    //Now we can assume that last_motor_command is either -1 or 1, so we can do math! (-1 is down, 1 is up)
    int motor_offset = motor_on_time * -1 * (int)last_motor_command; //If commmand was up, then offset will be negative. (otherwise unchanged)
    log(3, "Applying offset of %d | old micros_motor_lowered: %d | new micros_motor_lowered: %d\n",motor_offset, micros_motor_lowered, micros_motor_lowered + motor_offset);
    micros_motor_lowered += (motor_offset * MOTOR_STRENGTH_ADJUSTMENT_CONSTANT);
    if (motor_cycle_counter % 100 == 0) {
      log(2, "Motor Update | micros_motor_lowered = %d\n", micros_motor_lowered);
    }
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