/*
   Sketch for ESP32 board, like the NodeMCU 32S, LOLIN32, or the Adafruit Huzzah32
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
#include "g711.h"

// see https://en.wikipedia.org/wiki/Real-time_Transport_Protocol, one of these should be defined
#define RTP_ULAW
// #define RTP_ALAW
// #define RTP_L16

#define LED_BUILTIN 22

#define USE_WIFI        // required for TCP and UDP
#define USE_UDP
// #define USE_TCP
// #define DO_RECONNECT // not needed for UDP
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

// see https://en.wikipedia.org/wiki/Real-time_Transport_Protocol
#if defined(RTP_ULAW)
const unsigned char rtpType = 0;
const unsigned int sampleRate = 8000;
#elif defined(RTP_ALAW)
const unsigned char rtpType = 1;
const unsigned int sampleRate = 8000;
#elif defined(RTP_L16)
const unsigned char rtpType = 11;
const unsigned int sampleRate = 44100;
#endif

const unsigned int nMessage = 400;      // this can be up to 720 and still fit within the MTU of 1500 bytes
const unsigned int nBuffer = 1024;  // minimum 8, maximum 1024
unsigned int samplesReady = 0;
long timestampOffset = 0;
float timestampSlope = 1;
bool meanInitialized = 0;
const double alpha = 10. / sampleRate;  // if the sampling time dT is much smaller than the time constant T, then alpha=1/(T*sampleRate) and T=1/(alpha*sampleRate)
double signalMean = sqrt(-1);           // initialize as not-a-number
const double dbMax = 90.3;              // this is the loudest that an int16 signal can get
const double volumeThreshold = -40;     // relative to the maxiumum
const double signalDivider = pow(2, 12);
/*
    With a divider of 2^16=65536 blowing hard into the mic still does not clips. In this case the limiter is not needed.
    With a divider of 2^13=8192 normal speech does not clips. In this case the limiter is not needed.
    With a divider of 2^12=4096 the signal occasionally clips.
    With lower values for the divider, the limiter is certainly needed.
*/

unsigned long getTimestamp() {
  float timestamp = millis();
  timestamp += timestampOffset;
  timestamp *= timestampSlope;
  return timestamp; // typecast to unsigned long
}

// see https://en.wikipedia.org/wiki/Real-time_Transport_Protocol
struct message_t {
  // the format of the packet header is always the same
  uint8_t version = (2 << 6);  // the RTP version is 2
  uint8_t type = rtpType;      // the RTP type is 0, 1, or 11
  uint16_t counter = 0;
  uint32_t timestamp = getTimestamp();
  uint32_t id = 0;
  // the data is represented according to one of these
#if defined(RTP_ULAW)
  // type=0  PCMU  audio 1 8000  any 20  ITU-T G.711 PCM μ-Law audio 64 kbit/s               RFC 3551
  uint8_t data[nMessage];
#elif defined(RTP_ALAW)
  // type=8  PCMA  audio 1 8000  any 20  ITU-T G.711 PCM A-Law audio 64 kbit/s               RFC 3551
  uint8_t data[nMessage];
#elif defined(RTP_L16)
  // type=11 L16   audio 1 44100 any 20  Linear PCM 16-bit audio 705.6 kbit/s, uncompressed  RFC 3551, Page 27
  int16_t data[nMessage];
#endif
} message __attribute__((packed));

esp_err_t err;
uint32_t bytes_read;
int32_t buffer[nBuffer];
const i2s_port_t I2S_PORT = I2S_NUM_0;

RunningStat shortStat;

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

#ifdef USE_WIFI
  //register event handler
  WiFi.onEvent(WiFiEvent);

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
#else
  WiFi.mode(WIFI_OFF);
#endif

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
  size_t samplesNeeded = nMessage - samplesReady;
  if (samplesNeeded > nBuffer) {
    // do not read more than nBuffer at a time
    samplesNeeded = nBuffer;
  }

  err = i2s_read(I2S_PORT, buffer, samplesNeeded * 4, &bytes_read, 0);

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

#if defined(RTP_ULAW)
      message.data[samplesReady] = linear2ulaw(value); // this casts the value to an int16, and then converts to uint8
#elif defined(RTP_ALAW)
      message.data[samplesReady] = linear2alaw(value); // this casts the value to an int16, and then converts to uint8
#elif defined(RTP_L16)
      message.data[samplesReady] = value; // this casts the value to an int16
#endif

      samplesReady++;
      shortStat.Push(value);
    }

    if (samplesReady == nMessage) {

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
      Serial.print(message.type); Serial.print(", ");
      Serial.print(message.counter); Serial.print(", ");
      Serial.print(message.timestamp); Serial.print(", ");
      Serial.println(message.id);
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
#ifdef USE_WIFI
        Serial.println("not connected to wifi");
#endif
        blinkInterval = 250;
      }

      shortStat.Clear();
      samplesReady = 0;
      message.counter++;
      message.timestamp = getTimestamp();
    }
  }


} // loop
