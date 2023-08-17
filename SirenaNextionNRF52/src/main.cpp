#include <Arduino.h>
#include <bluefruit.h>
#include <nrfx_twis.h>

//#define SERIAL_DEBUG_ENABLED

#ifdef SERIAL_DEBUG_ENABLED
#define DEBUG_PRINT(x)     Serial.print (x)
#define DEBUG_PRINTF     Serial.printf
#define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define DEBUG_PRINTLNF(x,y)  Serial.println (x,y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x,y)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x,y)
#endif


#define CENTRAL_NAME "Sirena"
#define UUID16_SVC_SCALES 0xFFF0
#define UUID16_CHR_SCALES_READ 0x36F5
#define UUID16_CHR_SCALES_WRITE 0xFFF4

#define POWER_SW    18  // P0.02 (2)
#define BREW1_SW    21  // P0.31 (31)
#define BREW2_SW    16  // P0.30 (30)
#define STEAM_SW    20  // P0.29 (29)
#define STEAM_KNOB  24  // P0.15 (15)

#define POWER_LED   7   // P1.02 (34)
#define BREW1_LED   6   // P0.07 (7)
#define BREW2_LED   17  // P0.28 (28)
#define STEAM_LED   19  // P0.03 (3)
#define WATER_LED   4   // P1.10 (42)

#define SD1 29  // P0.17 (17)
#define SD2 30  // P0.22 (22)
#define SC  28  // P0.20 (20)
#define SB  25  // P0.13 (13) // SDA
#define SS  1   // P0.24 (24) // SCL

BLEClientService ss = BLEClientService(UUID16_SVC_SCALES);
BLEClientCharacteristic rc = BLEClientCharacteristic(UUID16_CHR_SCALES_READ);
BLEClientCharacteristic wc = BLEClientCharacteristic(UUID16_CHR_SCALES_WRITE);
BLEClientDis  clientDis;  // device information client
BLEClientBas  clientBas;  // battery client

const uint8_t tareRequest[7] = { 0x03, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0c };
volatile int16_t weight = 0;
volatile bool tareRequested = false;

volatile bool standardButtonsRequested = false;

bool powerLedState = false;
bool brew1LedState = false;
bool brew2LedState = false;
bool steamLedState = false;

bool powerState = false;
bool brew1State = false;
bool brew2State = false;
bool steamState = false;

void bleScanCallback(ble_gap_evt_adv_report_t* report) {
  Bluefruit.Central.connect(report);

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  //Bluefruit.Scanner.resume();
}

void connectCallback(uint16_t conn_handle) {
  if (!ss.discover(conn_handle)) {
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if (!rc.discover()) {
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if (!wc.discover()) {
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if (!wc.enableNotify()) {
    Bluefruit.disconnect(conn_handle);
    return;
  }
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  weight = 0;
}

void wcNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 7) {
    if (data[0] == 0x03 && data[1] == 0xca) {
      weight = (data[2] << 8) | data[3];
      DEBUG_PRINT("weight ");
      DEBUG_PRINTLN(weight);
    }
  }
}

void bleSetup() {
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(CENTRAL_NAME);

  clientDis.begin();
  clientBas.begin();

  ss.begin();

  rc.begin();

  wc.setNotifyCallback(wcNotifyCallback);
  wc.begin();

  // Start Central Scan
  Bluefruit.setConnLedInterval(250);

  Bluefruit.Central.setDisconnectCallback(disconnectCallback);
  Bluefruit.Central.setConnectCallback(connectCallback);

  Bluefruit.Scanner.setRxCallback(bleScanCallback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(ss.uuid);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
}

void parseLedState(uint8_t ledState) {
  brew1LedState = (ledState >> 0) & 1u;
  brew2LedState = (ledState >> 1) & 1u;
  steamLedState = (ledState >> 4) & 1u;
}

void parseLedStateEx(uint8_t ledState) {
  powerLedState = (ledState >> 2) & 1u;
}

const nrfx_twis_t twis = NRFX_TWIS_INSTANCE(0);

static uint8_t twisRxBuffer[4];
static uint8_t twisTxBuffer[4];

void wireReceiveEvent(uint32_t howMany) {
  DEBUG_PRINTF("wireReceiveEvent: %d\n\r", howMany);
  if (howMany == 1) { // register request
    if (twisRxBuffer[0] == 0x01u) {
      uint8_t statusByte = brew1State << 0 | brew2State << 1;
      statusByte ^= statusByte << 4;
      twisTxBuffer[0] = statusByte;
    }
    else if (twisRxBuffer[0] == 0x02u) {
      uint8_t statusByte = steamState << 0 | powerState << 2;
      statusByte ^= statusByte << 4;
      twisTxBuffer[0] = statusByte;
    }
  }
  else if (howMany == 2) {
    if (twisRxBuffer[0] == 0x00u) {
      parseLedState(twisRxBuffer[1]);
    }
    else if (twisRxBuffer[0] == 0x03u) { // scales comands
      if (twisRxBuffer[1] == 0xcau) {
        twisTxBuffer[0] = highByte(weight);
        twisTxBuffer[1] = lowByte(weight);
      }
      else if (twisRxBuffer[1] == 0x0fu) {
        tareRequested = true;
      }
    }
  }
  else if (howMany == 3) {
    if (twisRxBuffer[0] == 0x00u) {
      parseLedState(twisRxBuffer[1]);
      parseLedStateEx(twisRxBuffer[2]);
    }
  }
}

void twis_event_handler(nrfx_twis_evt_t const* const p_event) {
  DEBUG_PRINTF("twis_evt: %d\n\r", p_event->type);
  switch (p_event->type) {
  case NRFX_TWIS_EVT_READ_REQ:
    if (p_event->data.buf_req) {
      (void)nrfx_twis_tx_prepare(&twis, twisTxBuffer, sizeof(twisTxBuffer));
    }
    break;
  case NRFX_TWIS_EVT_READ_DONE:
    memset(twisTxBuffer, 0x00, sizeof(twisTxBuffer));
    break;
  case NRFX_TWIS_EVT_WRITE_REQ:
    if (p_event->data.buf_req) {
      (void)nrfx_twis_rx_prepare(&twis, twisRxBuffer, sizeof(twisRxBuffer));
    }
    break;
  case NRFX_TWIS_EVT_WRITE_DONE:
    wireReceiveEvent(p_event->data.rx_amount);
    break;
  default:
    break;
  }
}

void wireSetup() {
  nrfx_twis_config_t config = NRFX_TWIS_DEFAULT_CONFIG(24, 13, 0);

  if (NRFX_SUCCESS == nrfx_twis_init(&twis, &config, twis_event_handler)) {
    nrfx_twis_enable(&twis);
  }
}

void setup() {

#ifdef SERIAL_DEBUG_ENABLED
  Serial.begin(115200);
  while (!Serial) delay(10);   // for nrf52840 with native usb
  delay(5000);
  DEBUG_PRINTLN("starting...");
#endif

  pinMode(POWER_SW, INPUT);
  pinMode(BREW1_SW, INPUT);
  pinMode(BREW2_SW, INPUT);
  pinMode(STEAM_SW, INPUT);

  pinMode(POWER_LED, OUTPUT);
  pinMode(BREW1_LED, OUTPUT);
  pinMode(BREW2_LED, OUTPUT);
  pinMode(STEAM_LED, OUTPUT);
  pinMode(WATER_LED, OUTPUT);
  //pinMode(SB, OUTPUT);
  //pinMode(SS, OUTPUT);

  bleSetup();

  DEBUG_PRINTLN("bleSetup done");

  wireSetup();

  DEBUG_PRINTLN("wireSetup done");
}

void loop() {
  digitalWrite(POWER_LED, powerLedState ? HIGH : LOW);
  digitalWrite(BREW1_LED, brew1LedState ? HIGH : LOW);
  digitalWrite(BREW2_LED, brew2LedState ? HIGH : LOW);
  digitalWrite(STEAM_LED, steamLedState ? HIGH : LOW);

  powerState = digitalRead(POWER_SW);
  brew1State = digitalRead(BREW1_SW);
  brew2State = digitalRead(BREW2_SW);
  steamState = digitalRead(STEAM_SW);

  //digitalWrite(SB, brew1State ? LOW : HIGH);
  //digitalWrite(SS, steamState ? LOW : HIGH);

  if (tareRequested) {
    tareRequested = false;
    if (Bluefruit.Central.connected()) {
      rc.write(tareRequest, 7);
    }
  }
}
