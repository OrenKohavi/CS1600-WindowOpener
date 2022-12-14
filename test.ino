// FSM_State test_state;
Motor_Direction test_direction;
bool test_up_button;
bool test_down_button;
uint32_t test_photo_resister_avg;
uint32_t test_up_loop_count;
uint32_t test_down_loop_count;

void initializeInput()
{
  test_direction = Motor_Direction::Motor_OFF;
  curr_system_state = FSM_State::S_INIT;
  test_up_button = false;
  test_down_button = false;
  test_photo_resister_avg = 500;
  test_up_loop_count = LOOPS_UNTIL_DOUBLEPRESS_REGISTERED;
  test_down_loop_count = LOOPS_UNTIL_DOUBLEPRESS_REGISTERED;
}

void runAllTests()
{
  bool testPassFlag = true;
  initializeInput();

  // state 1-2
  test_up_button = true;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count + 10);
  if (curr_system_state != S_SETUP_MAX) {
    log(0, "TEST: test 1-2 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 2-3
  // both buttons released, loop count needs to be greater than threshold
  test_up_button = false;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count + 10);
  if (curr_system_state != S_SETUP_MIN) {
    log(0, "TEST: test 2-3 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 3-4
  // making sure the blind is not below our lowest bottom
  micros_motor_lowered = MOTOR_MICROS_MARGIN + 1000;
  test_up_button = false;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count + 10);
  if (curr_system_state != S_WAIT) {
    log(0, "TEST: test 3-4 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 4-5
  // blinder should go up when UP button is pressed
  test_up_button = true;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count - 10);
  if (curr_system_state != S_BUTTON_UP) {
    log(0, "TEST: test 4-5 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 5-4
  // press the up button again, or the position is MAX; blinder should go to S_WAIT
  test_up_button = true;
  test_down_button = true;
  // make double press to true
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count + 10);
  test_up_button = false;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count - 10, test_down_loop_count - 10);
  if (curr_system_state != S_WAIT) {
    log(0, "TEST: test 5-4 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 4-6
  // blinder should go down when DOWN button is pressed
  test_up_button = false;
  test_down_button = true;
  micros_motor_lowered = 0;

  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count - 10, test_down_loop_count + 10);
  if (curr_system_state != S_BUTTON_DOWN) {
    log(0, "TEST: test 4-6 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  // state 6-4
  // press the up button again, or the position is MAX; blinder should go to S_WAIT
  test_up_button = true;
  test_down_button = true;
  // make double press to true
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count + 10, test_down_loop_count + 10);
  test_up_button = false;
  test_down_button = false;
  fsmUpdate(test_up_button, test_down_button, false, test_photo_resister_avg, test_up_loop_count - 10, test_down_loop_count + 10);
  if (curr_system_state != S_WAIT) {
    log(0, "TEST: test 6-4 failed, current FSM state: %d\n", curr_system_state);
    testPassFlag = false;
  }

  if (testPassFlag) {
      log(0, "TEST: all tests passed\n");
  } else {
      log(0, "TEST: test suite failed\n");
  }
}
