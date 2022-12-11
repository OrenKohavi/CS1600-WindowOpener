#include <stdarg.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#define LOG_BUFSIZE 512
#define WIFI_BUFSIZE 1024 //We have extra memory so why not

void log(int logging_level, const char* format, ...) {
    if (!(VERBOSE >= logging_level)) {
      return;
    }
    if (strlen(format) > LOG_BUFSIZE * 0.8) {
      Serial.println("WARNING: Command is approaching maximum logging buffer size");
      //If you ever see this warning, consider increasing LOG_BUFSIZE or splitting your log message into two messages
    }
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

//Wifi Code (Stolen mostly verbatim from lab7)

char ssid[] = "Brown-Guest";  // network SSID (name)
char pass[] = ""; // for networks that require a password
int status = WL_IDLE_STATUS;  // the WiFi radio's status

WiFiClient client;
char buffer[WIFI_BUFSIZE];

//Many of this is taken from: https://docs.arduino.cc/tutorials/mkr-1000-wifi/wifi-101-library-examples#wifi101-udp-ntp-client
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP Udp;

const char* host = "api.sunrise-sunset.org";
const int httpPort = 80;

bool setup_wifi() {
  // attempt to connect to WiFi network:
  int attempts = 0;
  while ( status != WL_CONNECTED) {
    attempts++;
    log(1, "Attempting to connect to: %s\n", ssid);
    status = WiFi.begin(ssid); // WiFi.begin(ssid, pass) for password
    delay(10000);
    if (attempts > 6) {
      //If we've tried for a whole minute, just move on
      return false;
    }
  }
  log(1, "Connected to WiFi!\n");
  if (VERBOSE >= 3) {
    Serial.println("Verbose WIFI Data:");
    printCurrentNet();
    printWiFiData();
  }
  Udp.begin(localPort);
  log(1, "UDP Started\n");
  return true;
}

bool get_current_time() {
  int attempts = 0;
  while(attempts < 5){
    attempts++;
    log(2, "Attempt %d at getting network time\n", attempts);
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    delay(1000);
    if ( Udp.parsePacket() ) {
      log(2, "UDP packet received\n");
      // We've received a packet, read the data from it
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      log(2, "Seconds since Jan 1 1900 = %u\n", secsSince1900);
      // now convert NTP time into everyday time:
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      log(2, "Unix time = %u\n", epoch);
      // print the hour, minute and second:'
      if (VERBOSE >= 3) {
        Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if ( ((epoch % 3600) / 60) < 10 ) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ( (epoch % 60) < 10 ) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.println(epoch % 60); // print the second
      }
      //Set global variable
      unix_epoch_time = epoch;
      return true;
    } else {
      log(2, "No UDP Packet recieved in time on attempt %d\n", attempts);
    }
  }
  log(1, "Could get get current time :(\n");
  return false;
}

bool get_sunrise_sunset_times() {
  log(2, "Connecting to %s...\n", host);
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return false;
  }

  // Submit GET request with latitude and longitude parameters
  float lat = SUNRISE_LAT;
  float lon = SUNRISE_LON;
  String url = "/json?lat=" + String(lat) + "&lon=" + String(lon);
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: ArduinoWiFi/1.1\r\n" +
               "Connection: close\r\n\r\n");

  // Print response
  Serial.println("Response: ");
  bool read_successful = false;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    //This is an incredibly hacky way to read the response, but it works (until the API goes down or changes).
    if (line.startsWith("{\"results\":")) {
      //We've found the relevant line with all the api data
      read_successful = parse_sun_times(line);
      return read_successful;
    }
    if (VERBOSE >= 3) {
      Serial.println(line);
    }
  }
}

bool parse_sun_times(String json) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    log(1, "Failed to parse JSON for sun times\n");
    return false;
  }
  const char* status = doc["status"];
  log(2, "JSON Status is: %s\n", status);
  if (String(status).compareTo(String("OK")) != 0) {
    //Response was not 'OK'
    log(1, "Response was not 'OK', so cannot use WiFi :(\n");
    return false;
  }

  return true;
}

//Functions below are helpers from https://docs.arduino.cc/tutorials/mkr-1000-wifi/wifi-101-library-examples 

unsigned long sendNTPpacket(IPAddress& address) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  //Serial.println("3");
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}

void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);
  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);
  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}