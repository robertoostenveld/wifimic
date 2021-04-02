/*
   Sketch for ESP32 board, like the NodeMCU 32S or the Adafruit Huzzah32
   connected to a INMP411 I2S microphone

   See https://diyi0t.com/i2s-sound-tutorial-for-esp32/
   and https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html

*/

#include <WiFi.h>
#include <driver/i2s.h>
#include <endian.h>
#include <math.h>
#include <limits.h>
#include <WiFiUdp.h>
#include <AsyncUDP.h>

#include "secret.h" // this contains the ssid and password
#include "RunningStat.h"

#define LED_BUILTIN 22
#define USE_DHCP
#define USE_UDP
// #define USE_TCP
// #define DO_RECONNECT
// #define DO_FILTER
// #define DO_LIMITER
// #define DO_THRESHOLD
#define DO_CLOCKSYNC
// #define PRINT_VALUE
// #define PRINT_RANGE
// #define PRINT_VOLUME
// #define PRINT_FREQUENCY
// #define PRINT_HEADER
// #define PRINT_ELAPSED

#ifdef DO_FILTER
#include <IIRFilter.h>    // See https://github.com/tttapa/Filters
const double b_coefficients[] = {0.049261, -0.000000, -0.147783, -0.000000, 0.147783, -0.000000, -0.049261};
const double a_coefficients[] = {1.000000, -4.038586, 6.833250, -6.350717, 3.497524, -1.079855, 0.138470};
IIRFilter iir(b_coefficients, a_coefficients);
#endif

#ifndef USE_DHCP
IPAddress localAddress(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
#endif

IPAddress serverAddress(192, 168, 1, 33);

const unsigned int tcpPort = 4000;
const unsigned int udpPort = 4001;

WiFiClient Tcp;
WiFiUDP Udp;

unsigned long blinkInterval = 250;
unsigned long thresholdInterval = 500;
unsigned int maintenanceInterval = 5000;
unsigned long lastBlink = 0, lastThreshold = 0, lastConnect = 0, lastClock = 0;
unsigned long offset = 0;
unsigned long thisPacket, lastPacket;
bool wifiConnected = false, tcpConnected = false, udpConnected = false;
const unsigned int sampleRate = 44100;
const unsigned int nMessage = 720;      // this can be up to 720 and still fit within the MTU of 1500 bytes
const unsigned int nBuffer = 720;       // minimum 8, maximum 1024
bool meanInitialized = 0;
const double alpha = 10. / sampleRate;  // if the sampling time dT is much smaller than the time constant T, then alpha=1/(T*sampleRate) and T=1/(alpha*sampleRate)
double signalMean = sqrt(-1);           // initialize as not-a-number
const double dbMax = 90.3;              // this is the loudest that an int16 signal can get
const double volumeThreshold = -40;     // relative to the maxiumum
const double signalDivider = pow(2, 12);
/*
    With a divider of 2^16=65536 blowing hard into the mic still does not clips. In this case the limiter is not needed.
    With a divider of 2^13=8192 normal speech never clips. In this case the limiter is not needed.
    With a divider of 2^12=4096 the signal does not clip often, but it still happens occasionally.
    With lower values for the divider, the limiter is certainly needed.
*/

struct message_t {
  uint32_t version = 1;
  uint32_t id = 0;
  uint32_t counter;
  uint32_t samples;
  int16_t data[nMessage];
} message __attribute__((packed));

esp_err_t err;
uint32_t bytes_read;
int32_t buffer[nBuffer];

const i2s_port_t I2S_PORT = I2S_NUM_0;

RunningStat shortstat;
RunningStat longstat;

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
#ifdef USE_UDP
      // this sets up the local UDP port
      Udp.begin(WiFi.localIP(), udpPort);
#endif
      // use the last number of the IP address the mesage identifier
      message.id = WiFi.localIP()[3];
      wifiConnected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.");
      // do not use the last number of the IP address as identifier any more
      message.id = 0;
      wifiConnected = false;
#ifdef USE_UDP
      Udp.stop();
#endif
      ESP.restart();
      break;
    default:
      break;
  }
}

long timestampOffset = 0;
float timestampSlope = 1;

unsigned long getTimestamp() {
  float timestamp = millis();
  timestamp += timestampOffset;
  timestamp *= timestampSlope;
  return timestamp; // typecast to unsigned long
}

#ifdef DO_CLOCKSYNC
// the server sends a UDP broadcast packet with its timestamp
// this is used to update the linear mapping between teh local clock and that of the server

AsyncUDP Sync;
const unsigned int syncPort = 4002;

void SyncHandler(AsyncUDPPacket packet) {
  if (packet.length() == 4) {
    uint8_t *ptr = packet.data();
    unsigned long timestamp = ((unsigned long *)ptr)[0];
    timestamp = ntohl(timestamp);
    timestampOffset = (timestamp - millis());
    Serial.print("offset = ");
    Serial.println(timestampOffset);

  }
}
#endif

/**************************************************************************************************/

void setup() {
  esp_err_t err;

  Serial.begin(115200);

  pinMode(32, OUTPUT); digitalWrite(32, HIGH);  // L/R
  pinMode(35, OUTPUT); digitalWrite(35, HIGH);  // VDD

  const i2s_pin_config_t pin_config = {
    .bck_io_num = 25,                   // Serial Clock (SCK on the INMP441)
    .ws_io_num = 33,                    // Word Select  (WS on the INMP441)
    .data_out_num = I2S_PIN_NO_CHANGE,  // not used     (only for speakers)
    .data_in_num = 26                   // Serial Data  (SD on the INMP441)
  };

  // initialize status LED
  pinMode(LED_BUILTIN, OUTPUT);
  lastBlink = millis();

  // delete old config
  WiFi.disconnect(true);

  //register event handler
  WiFi.onEvent(WiFiEvent);

#ifndef USE_DHCP
  if (!WiFi.config(localAddress, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
    while (true);
  }
#endif

  //Initiate connection
  Serial.println("Connecting to WiFi network: " + String(ssid));
  WiFi.begin(ssid, password);
  Serial.println("Waiting for WIFI connection...");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }

#ifdef DO_CLOCKSYNC
  if (Sync.listen(syncPort)) {
    Serial.print("Listening for clock synchronization on ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(syncPort);
    Sync.onPacket(SyncHandler);
  }
#endif

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),  // receive, not transfer
    .sample_rate = sampleRate,                          // sampling rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,       // could only get it to work with 32bits
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,        // channel to use
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // interrupt level 1
    .dma_buf_count = 16,                                // number of buffers, 128 max.
    .dma_buf_len = nBuffer,                             // samples per buffer (minimum is 8)
    .use_apll = false
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S pins set.");

  for (int i; i < 2 * sampleRate; i++) {
    err = i2s_read(I2S_PORT, buffer, 4, &bytes_read, 0);
  }

} // setup

/**************************************************************************************************/

void loop() {

  if ((millis() - lastBlink) > 2000) {
    digitalWrite(LED_BUILTIN, LOW);
    lastBlink = millis();
  }
  else if ((millis() - lastBlink) > 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // how many audio samples do we still need to fill the next message?
  size_t samples = nMessage - message.samples;
  if (samples > nBuffer) {
    // do not read more than nBuffer at a time
    samples = nBuffer;
  }

  err = i2s_read(I2S_PORT, buffer, samples * 4, &bytes_read, 0);

  if (err == ESP_OK) {
    for (unsigned int sample; sample < bytes_read / 4; sample++) {

      double value = buffer[sample];

      // compute a smooth running mean with https://en.wikipedia.org/wiki/Exponential_smoothing
      if (isnan(signalMean)) {
        signalMean = value;
      }
      else {
        signalMean = alpha * value + (1 - alpha) * signalMean;
      }
      value -= signalMean;
      value /= signalDivider;

#ifdef DO_FILTER
      value = iir.filter(value);
#endif

#ifdef DO_LIMITER
      // https://en.wikipedia.org/wiki/Sigmoid_function
      value /= 32768;
      value /= (1 + fabs(value));
      value *= 32768;
#endif

#ifdef PRINT_VALUE
      Serial.println(value);
#endif

      message.data[message.samples] = value; // this casts the value to an int16
      message.samples++;

      shortstat.Push(value);
      longstat.Push(value);
    }

    if (message.samples == nMessage) {

#ifdef PRINT_RANGE
      Serial.print(shortstat.Min());
      Serial.print(", ");
      Serial.print(shortstat.Mean());
      Serial.print(", ");
      Serial.println(shortstat.Max());
#endif

#ifdef PRINT_FREQUENCY
      Serial.println(shortstat.ZeroCrossingRate() * sampleRate);
#endif

#ifdef PRINT_VOLUME
      Serial.println(10 * log10(shortstat.Variance()) - dbMax);
#endif

#ifdef PRINT_HEADER
      Serial.print(message.version); Serial.print(", ");
      Serial.print(message.id); Serial.print(", ");
      Serial.print(message.counter); Serial.print(", ");
      Serial.print(message.samples); Serial.println();
#endif

#ifdef PRINT_ELAPSED
      thisPacket = millis();
      Serial.println(thisPacket - lastPacket);
      lastPacket = thisPacket;
#endif

#ifdef DO_RECONNECT
      if  (lastConnect == 0 || (millis() - lastConnect) > maintenanceInterval) {
        // reconnect to TCP, but don't try to reconnect too often
        lastConnect = millis();

        // turn the status LED on
        digitalWrite(LED_BUILTIN, LOW);
        lastBlink = millis();

        if (wifiConnected && !tcpConnected) {
          tcpConnected = Tcp.connect(serverAddress, tcpPort);
          if (tcpConnected) {
            Serial.print("Connected to ");
            Serial.println(serverAddress);
          }
          else {
            Serial.print("Failed to connect to ");
            Serial.println(serverAddress);
          }
        }
      }
#endif

      if (wifiConnected) {
        blinkInterval = 1000;

#ifdef DO_THRESHOLD
        bool aboveThreshold = (10 * log10(shortstat.Variance() - dbMax) > volumeThreshold);
        if (aboveThreshold)
          lastThreshold = millis();
        else if ((millis() - lastThreshold) < thresholdInterval)
          aboveThreshold = 1;
#else
        bool aboveThreshold = 1;
#endif

        if (aboveThreshold) {
          int count;
#ifdef USE_UDP
          Udp.beginPacket(serverAddress, udpPort);
          count = Udp.write((uint8_t *)(&message), sizeof(message));
          Udp.endPacket();
          udpConnected = (count == sizeof(message));
          if (!udpConnected) {
            Serial.println("not connected to udp");
            blinkInterval = 250;
          }
#endif
#ifdef USE_TCP
          count = Tcp.write((uint8_t *)(&message), sizeof(message));
          tcpConnected = (count == sizeof(message));
          if (!tcpConnected) {
            Serial.println("not connected to tcp");
            blinkInterval = 250;
          }
#endif
        } // if aboveThreshold
      } // if wifiConnected

      else {
        Serial.println("not connected to wifi");
        blinkInterval = 250;
      }

      shortstat.Clear();
      message.counter++;
      message.samples = 0;
    }
  }


} // loop
