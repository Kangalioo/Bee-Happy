#include <HX711.h>
#include <lmic.h>
#include <hal/hal.h>

#define MEASURE_CYCLES 1
#define VOLUME_MEASURE_CYCLES 50
#define MEASURE_DELAY 10000
#define VOLUME_MEASURE_RATE 0.7  // e.g. 0.9 = 90% of the time, volume is measured, 10% of the time, the other stuff is done
#define IDUINO_TIMEOUT 1000
#define DATA_BITS 11

int dhPin = 8;
byte dat[4];

// A3 = DAT, A2 = CLK (Correct me if I am wrong)
HX711 scale(A3, A2);

static const u1_t NWKSKEY[16] = { 0xE6, 0x74, 0x0F, 0xFD, 0xF8, 0xA2, 0x8F, 0xF8, 0x51, 0x59, 0x16, 0xAE, 0x7B, 0x7F, 0x11, 0xA8 };
static const u1_t APPSKEY[16] = { 0xF5, 0x15, 0xEB, 0x2F, 0x99, 0x9D, 0xE0, 0x72, 0xDF, 0x37, 0x2B, 0x16, 0x01, 0x96, 0x45, 0x7A };
static const u4_t DEVADDR = 0x2601189E;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print("Message event: ");
    Serial.println(ev);
    if (ev == EV_TXCOMPLETE) {
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // Schedule next transmission
//        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    }
}

void do_send(osjob_t* j, uint8_t message[]){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        message = "hello world!!!";
        LMIC_setTxData2(1, message, sizeof(message)-1, 0);
        Serial.println(F("Sending uplink packet..."));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

byte readData() {
  byte data;
  unsigned long millisStart = millis();
  unsigned long millisCurr = millisStart;

  for (int i = 0; i < 8; i++) {
//    Serial.println(digitalRead(dhPin));
    if (digitalRead(dhPin) == LOW) {
      while (digitalRead(dhPin) == LOW && millisCurr - millisStart < IDUINO_TIMEOUT) {
        millisCurr = millis();
      }
      delayMicroseconds(30);
      
      // Fehlerbehandlung
      if (millisCurr - millisStart > IDUINO_TIMEOUT) {
        data = 0;
      }
      
      // Datenlesen erfolgreich
      if (digitalRead(dhPin) == HIGH) {
        data |= (1 << (7 - i));
      }
      
      while (digitalRead(dhPin) == HIGH);
    }
  }
  return data;
}

void startTest() {
  Serial.println("Start measuring humidity & temp");

  pinMode(dhPin, OUTPUT);
  digitalWrite(dhPin, LOW);
  delay(30);
  digitalWrite(dhPin, HIGH);
  delayMicroseconds(40);

  pinMode(dhPin, INPUT);
  
  unsigned long millisStart = millis();
  unsigned long millisCurr = millisStart;
  while (digitalRead(dhPin) == HIGH && millisCurr - millisStart < IDUINO_TIMEOUT) {
    Serial.print(".");
    Serial.print(millisCurr);
    millisCurr = millis();
  }
  delayMicroseconds(80);
//  if (digitalRead(dhPin) == LOW);
//  delayMicroseconds(80);

  for (int i = 0; i < 4; i++) {
    dat[i] = readData();
  }

  pinMode(dhPin, OUTPUT);
  digitalWrite(dhPin, HIGH);
//  Serial.println("Finished test");
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Initializing");

  // Waage kalibrieren
  scale.set_offset(161309);
  scale.set_scale(29.95);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable every channel but the first one, as our gateway only receives channel 0 messages
  for (int i = 1; i <= 8; i++) LMIC_disableChannel(i);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7 ,14);
}

uint8_t dataChunk[MEASURE_CYCLES * DATA_BITS];

void loop() {
  os_runloop_once();
  
  Serial.println("Measuring data...");
  for (short i = 0; i < MEASURE_CYCLES; i++) {

    // Read humidity and temperature
    startTest();
    dataChunk[i * DATA_BITS] = dat[0];
    dataChunk[i * DATA_BITS + 1] = dat[2];
    Serial.print("Humidity: ");
    Serial.println(dat[1]);
    Serial.print("Temperature: ");
    Serial.println(dat[3]);

    // Read (average) of loundness
    short delayMicro = 1000 * VOLUME_MEASURE_RATE * MEASURE_DELAY / VOLUME_MEASURE_CYCLES; // delay time in microseconds
    short minVolume = 1024, maxVolume = 0, value;
    for (short j = 0; j < VOLUME_MEASURE_CYCLES; j++) {
      value = analogRead(A0);
      if (value > maxVolume) maxVolume = value;
      if (value < minVolume) minVolume = value;
      delayMicroseconds(delayMicro);
    }
    Serial.print("Min volume: ");
    Serial.println(minVolume);
    Serial.print("Max volume: ");
    Serial.println(maxVolume);
    dataChunk[i * DATA_BITS + 2] = minVolume >> 8;
    dataChunk[i * DATA_BITS + 3] = minVolume & 0b11111111;
    dataChunk[i * DATA_BITS + 4] = maxVolume >> 8;
    dataChunk[i * DATA_BITS + 5] = maxVolume & 0b11111111;

    // Read weight
    float rawval = scale.get_units(10);
    long val = (long) rawval;
    Serial.print("Weight: ");
    Serial.println(rawval);
    dataChunk[i * DATA_BITS + 6] = (val >> 24) & 0b11111111;
    dataChunk[i * DATA_BITS + 7] = (val >> 16) & 0b11111111;
    dataChunk[i * DATA_BITS + 8] = (val >> 8) & 0b11111111;
    dataChunk[i * DATA_BITS + 9] = (val >> 0) & 0b11111111;
    dataChunk[i * DATA_BITS + 10] = (uint8_t) ((rawval - val) * 256);
    delayMicroseconds(1000 * (1 - VOLUME_MEASURE_RATE) * MEASURE_DELAY);
  }
  Serial.println();

  Serial.print("Data: ");
  for (int i = 0; i < MEASURE_CYCLES * DATA_BITS; i++) {
    Serial.print(dataChunk[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Sending data (");
  Serial.print(MEASURE_CYCLES * DATA_BITS);
  Serial.println(" bytes)...");
  
  do_send(&sendjob, dataChunk);
}
