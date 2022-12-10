#include <stdarg.h>

void log(int logging_level, const char* format, ...) {
    if (!(VERBOSE >= logging_level)) {
      return;
    }
    //Not sure how to pass variable args to my_printf, so I just copy the code like a doofus
    char buffer[512];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);
}