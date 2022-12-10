#include <Adafruit_SleepyDog.h>

/*
TODO:
FSM Implementation
Demo_mode definitions
Timer for motor sanity check???
Automatic threshold recalibration (Use WiFi and real-world time)
*/

/*
Use wifi to get the real world time, and get sunrise/sunset times
Recalibrate the thresholds for 
*/

//For demo/debugging
#define DEMO_MODE false
#define VERBOSE 3 //Higher number is more verbose. Ranges from [0-3], with 0 being no output at all, and 3 being insane amounts of output.

//For integration and adjustments
#define CALIBRATE_AT_SUNRISE_SUNSET true
#define INVERT_MOTOR_OFF_STATE false
#define INITIAL_LOW_LIGHT_THRESHOLD 50
#define INITIAL_HIGH_LIGHT_THRESHOLD 500
#define INVERT_MOTOR_DIR false
#define UP_PIN 1
#define DOWN_PIN 0
#define MOTOR_PIN_1 4
#define MOTOR_PIN_2 5
#define PHOTORESISTOR_PIN A6
#define SETUP_LED_PIN 14

//For changing fundamental system behavior
#define USE_WATCHDOG false
#define LOOP_INTERVAL 10 //ms delay between loops
#define MINS_PER_SAMPLE_INTERVAL 5 //How long do we record data for our moving average
#define MS_UNTIL_DOUBLEPRESS_REGISTERED 1000
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
uint32_t max_motor_lower_micros; //Maximum micros of height the motor can be at (calibrated during setup)
PinStatus up_button_state;
PinStatus down_button_state;
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

  //Count loops active for each button
  up_active_loop_count++;
  up_active_loop_count *= (up_button_state == HIGH); //Resets to zero if button is not pressed
  down_active_loop_count++;
  down_active_loop_count *= (down_button_state == HIGH); //Resets to zero if button is not pressed
  bool up_active = up_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool down_active = down_active_loop_count >= LOOPS_UNTIL_BUTTON_PRESS_REGISTERED;
  bool buttons_changing = (!up_active && up_active_loop_count > 0) || (!down_active && down_active_loop_count > 0);
  Motor_Direction motor_action = Motor_OFF;

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
  log(2, "Entering FSM... Current State: %d | up_active: %d | down_active: %d | buttons_changing: %d | photoresistor_avg: %d | micros_motor_lowered: %d | max_lower: %d\n",
    curr_system_state, up_active, down_active, buttons_changing, photoresistor_avg, micros_motor_lowered, max_motor_lower_micros);
  switch (curr_system_state) {
    case S_INIT: //State 1
      //There's no logic here, INIT always goes into SETUP_MAX
      curr_system_state = S_SETUP_MAX;
      digitalWrite(SETUP_LED_PIN, HIGH);
      log(1, "Transitioning from S_INIT to S_SETUP_MAX\n");
      break;
    case S_SETUP_MAX:
      //Within the state, react to buttons
      if (buttons_changing) {
        break; //While buttons are changing, nothing should be happening
      }
      if (up_active && !down_active) {
        motor_action = Motor_UP;
      } else if (!up_active && down_active) {
        motor_action = Motor_DOWN;
      }
      //Only one transition out: transition to S_SETUP_MIN when doublepress is detected
      if (up_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && down_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED) {
        log(1, "Transitioning from S_SETUP_MAX to S_SETUP_MIN\n");
        curr_system_state = S_SETUP_MIN;
        //Not much to record here, because the minimum is implicitly zero, and we are at the lowering minimum.
      } else {
        log(3, "Not transitioning out of S_SETUP_MAX\n");
      }
      break;
    case S_SETUP_MIN:
      //Only one transition out: transition to S_WAIT when doublepress is detected
      if (buttons_changing == false && up_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED && down_active_loop_count > LOOPS_UNTIL_DOUBLEPRESS_REGISTERED) {
        log(1, "Transitioning from S_SETUP_MIN to S_WAIT\n");
        curr_system_state = S_WAIT;
        max_motor_lower_micros = micros_motor_lowered; //Record the minimum height
      } else {
        log(3, "Not transitioning out of S_SETUP_MIN\n");
      }
      break;
    case S_WAIT:
      break;
  };
  log(2, "After FSM state is: %d\n", curr_system_state);

  //End of the loop -- Reset the Watchdog and apply motor action.
  log(2, "Applying Motor Action: %d\n", motor_action);
  setMotor(motor_action);
  if (USE_WATCHDOG) {
    Watchdog.reset();
  }
  delay(LOOP_INTERVAL);
  log(2, "---- LOOP END ----\n");
}

void setMotor(Motor_Direction direction){
  if (INVERT_MOTOR_DIR) {
    direction = (Motor_Direction)((int)direction * -1); //flip from UP to DOWN without modifying the zero state (which is OFF)
  }
  if (last_motor_command == Motor_OFF && direction != Motor_OFF){
    //Motor just changed from off into some other state, log it with less verbosity
    log(1, "Motor Recieved New Command: %d\n", direction);
  } else {
    log(3, "Motor Command Recieved: %d\n", direction);
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

void Button_ISR() {
  up_button_state = digitalRead(UP_PIN);
  down_button_state = digitalRead(DOWN_PIN);
  mils_at_last_physical_interaction = millis();
}