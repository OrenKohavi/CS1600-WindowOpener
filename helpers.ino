#include <stdarg.h>
#define LOG_BUFSIZE 512

void log(int logging_level, const char* format, ...) {
    if (!(VERBOSE >= logging_level)) {
      return;
    }
    if (strlen(format) > LOG_BUFSIZE * 0.8) {
      Serial.println("WARNING: Command is approaching maximum logging buffer size");
      //If you ever see this warning, consider increasing LOG_BUFSIZE or splitting your log message into two messages
    }
    //Not sure how to pass variable args to my_printf, so I just copy the code like a doofus
    char buffer[LOG_BUFSIZE];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);
}

void led_quick_flash() {
  log(2, "Flashing LED\n");
  digitalWrite(SETUP_LED_PIN, LOW);
  delay(100);
  digitalWrite(SETUP_LED_PIN, HIGH);
  delay(100);
  digitalWrite(SETUP_LED_PIN, LOW);
  delay(100);
  digitalWrite(SETUP_LED_PIN, HIGH);
  delay(100);
  digitalWrite(SETUP_LED_PIN, LOW);
  delay(100);
}

void errorLoop() {
  while(true) {
    digitalWrite(SETUP_LED_PIN, LOW);
    delay(250);
    digitalWrite(SETUP_LED_PIN, HIGH);
    delay(250);
    Watchdog.reset(); //Better to keep the system alive, so the user knows there's been an error
  }
}

bool shade_not_at_max() {
  return micros_motor_lowered > MOTOR_MICROS_MARGIN;
}

bool shade_not_at_min() {
  return (micros_motor_lowered + MOTOR_MICROS_MARGIN) < micros_motor_lowered_max;  
}