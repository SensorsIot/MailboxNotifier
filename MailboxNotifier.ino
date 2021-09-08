/*

   Based on this example: https://github.com/JackGruber/ESP32-LMIC-DeepSleep-example
   https://jackgruber.github.io/2020-04-13-ESP32-DeepSleep-and-LoraWAN-OTAA-join/

   Use TTGO LoRa V1 Board

*/

#define TTN_APPEUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TTN Application EUI with "lsb"
#define TTN_DEVEUI { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TTN Device EUI with "lsb"
#define TTN_APPKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // TTN App Key with "msb"

#define POWERPIN 4
#define BATTPIN 39
#define BATTFACTOR  0.0857


#include <arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>
#include "EEPROM_routines.h"
//#include <ttn_credentials.h>

#define EEPROM_SIZE 1000

#define POWEROFF // power off instead of deep sleep


volatile bool enableSleep_ = false;
unsigned long entry;

// rename ttn_credentials.h.example to ttn_credentials.h and add you keys
static const u1_t PROGMEM APPEUI[8] = TTN_APPEUI;
static const u1_t PROGMEM DEVEUI[8] = TTN_DEVEUI;
static const u1_t PROGMEM APPKEY[16] = TTN_APPKEY;
void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t mydata[] = "Test";
static osjob_t sendjob;

// Schedule TX every this many seconds
// Respect Fair Access Policy and Maximum Duty Cycle!
// https://www.thethingsnetwork.org/docs/lorawan/duty-cycle.html
// https://www.loratools.nl/#/airtime
const unsigned TX_INTERVAL = 3600;

// Saves the LMIC structure during DeepSleep
RTC_DATA_ATTR lmic_t RTC_LMIC;

#define PIN_LMIC_NSS 18
#define PIN_LMIC_RST 14
#define PIN_LMIC_DIO0 26
#define PIN_LMIC_DIO1 33
#define PIN_LMIC_DIO2 32

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = PIN_LMIC_NSS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PIN_LMIC_RST,
  .dio = {PIN_LMIC_DIO0, PIN_LMIC_DIO1, PIN_LMIC_DIO2},
};

// opmode def
// https://github.com/mcci-catena/arduino-lmic/blob/89c28c5888338f8fc851851bb64968f2a493462f/src/lmic/lmic.h#L233
void LoraWANPrintLMICOpmode(void)
{
  Serial.print(F("LMIC.opmode: "));
  if (LMIC.opmode & OP_NONE)
  {
    Serial.print(F("OP_NONE "));
  }
  if (LMIC.opmode & OP_SCAN)
  {
    Serial.print(F("OP_SCAN "));
  }
  if (LMIC.opmode & OP_TRACK)
  {
    Serial.print(F("OP_TRACK "));
  }
  if (LMIC.opmode & OP_JOINING)
  {
    Serial.print(F("OP_JOINING "));
  }
  if (LMIC.opmode & OP_TXDATA)
  {
    Serial.print(F("OP_TXDATA "));
  }
  if (LMIC.opmode & OP_POLL)
  {
    Serial.print(F("OP_POLL "));
  }
  if (LMIC.opmode & OP_REJOIN)
  {
    Serial.print(F("OP_REJOIN "));
  }
  if (LMIC.opmode & OP_SHUTDOWN)
  {
    Serial.print(F("OP_SHUTDOWN "));
  }
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.print(F("OP_TXRXPEND "));
  }
  if (LMIC.opmode & OP_RNDTX)
  {
    Serial.print(F("OP_RNDTX "));
  }
  if (LMIC.opmode & OP_PINGINI)
  {
    Serial.print(F("OP_PINGINI "));
  }
  if (LMIC.opmode & OP_PINGABLE)
  {
    Serial.print(F("OP_PINGABLE "));
  }
  if (LMIC.opmode & OP_NEXTCHNL)
  {
    Serial.print(F("OP_NEXTCHNL "));
  }
  if (LMIC.opmode & OP_LINKDEAD)
  {
    Serial.print(F("OP_LINKDEAD "));
  }
  if (LMIC.opmode & OP_LINKDEAD)
  {
    Serial.print(F("OP_LINKDEAD "));
  }
  if (LMIC.opmode & OP_TESTMODE)
  {
    Serial.print(F("OP_TESTMODE "));
  }
  if (LMIC.opmode & OP_UNJOIN)
  {
    Serial.print(F("OP_UNJOIN "));
  }
}

void LoraWANDebug(lmic_t lmic_check)
{
  Serial.println("");
  Serial.println("");

  LoraWANPrintLMICOpmode();
  Serial.println("");

  Serial.print(F("LMIC.seqnoUp = "));
  Serial.println(lmic_check.seqnoUp);

  Serial.print(F("LMIC.globalDutyRate = "));
  Serial.print(lmic_check.globalDutyRate);
  Serial.print(F(" osTicks, "));
  Serial.print(osticks2ms(lmic_check.globalDutyRate) / 1000);
  Serial.println(F(" sec"));

  Serial.print(F("LMIC.globalDutyAvail = "));
  Serial.print(lmic_check.globalDutyAvail);
  Serial.print(F(" osTicks, "));
  Serial.print(osticks2ms(lmic_check.globalDutyAvail) / 1000);
  Serial.println(F(" sec"));

  Serial.print(F("LMICbandplan_nextTx = "));
  Serial.print(LMICbandplan_nextTx(os_getTime()));
  Serial.print(F(" osTicks, "));
  Serial.print(osticks2ms(LMICbandplan_nextTx(os_getTime())) / 1000);
  Serial.println(F(" sec"));

  Serial.print(F("os_getTime = "));
  Serial.print(os_getTime());
  Serial.print(F(" osTicks, "));
  Serial.print(osticks2ms(os_getTime()) / 1000);
  Serial.println(F(" sec"));

  Serial.print(F("LMIC.txend = "));
  Serial.println(lmic_check.txend);
  Serial.print(F("LMIC.txChnl = "));
  Serial.println(lmic_check.txChnl);

  Serial.println(F("Band \tavail \t\tavail_sec\tlastchnl \ttxcap"));
  for (u1_t bi = 0; bi < MAX_BANDS; bi++)
  {
    Serial.print(bi);
    Serial.print("\t");
    Serial.print(lmic_check.bands[bi].avail);
    Serial.print("\t\t");
    Serial.print(osticks2ms(lmic_check.bands[bi].avail) / 1000);
    Serial.print("\t\t");
    Serial.print(lmic_check.bands[bi].lastchnl);
    Serial.print("\t\t");
    Serial.println(lmic_check.bands[bi].txcap);
  }
  Serial.println("");
  Serial.println("");
}

void PrintRuntime()
{
  long seconds = millis() / 1000;
  Serial.print("Runtime: ");
  Serial.print(seconds);
  Serial.println(" seconds");
}

void PrintLMICVersion()
{
  Serial.print(F("LMIC: "));
  Serial.print(ARDUINO_LMIC_VERSION_GET_MAJOR(ARDUINO_LMIC_VERSION));
  Serial.print(F("."));
  Serial.print(ARDUINO_LMIC_VERSION_GET_MINOR(ARDUINO_LMIC_VERSION));
  Serial.print(F("."));
  Serial.print(ARDUINO_LMIC_VERSION_GET_PATCH(ARDUINO_LMIC_VERSION));
  Serial.print(F("."));
  Serial.println(ARDUINO_LMIC_VERSION_GET_LOCAL(ARDUINO_LMIC_VERSION));
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("artKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i)
        {
          Serial.print(artKey[i], HEX);
        }
        Serial.println("");
        Serial.print("nwkKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i)
        {
          Serial.print(nwkKey[i], HEX);
        }
        Serial.println("");
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      // Transmit completed, includes waiting for RX windows.
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      // setTxIndicatorsOn(false);
      // printEvent(timestamp, ev);
      // printFrameCounters();
      // Check if downlink was received
      if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
      {
        uint8_t fPort = 0;
        if (LMIC.txrxFlags & TXRX_PORT)
        {
          fPort = LMIC.frame[LMIC.dataBeg - 1];
        }
        // printDownlinkInfo();
        // processDownlink(timestamp, fPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      }
      enableSleep_ = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    int _adc = analogRead(BATTPIN);
    Serial.print("ADC ");
    Serial.println(_adc);
    float voltage = (float)_adc * BATTFACTOR;
    int _hi = (int)voltage;
    Serial.print("Voltage ");
    Serial.println(_hi/ 100.0);
    mydata[0] = _hi & 0xFF;
    mydata[1] = _hi >> 8 & 0xFF;
    LMIC_setTxData2(1, mydata, 2, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void SaveLMICToRTC(int deepsleep_sec)
{
  Serial.println(F("Save LMIC to RTC"));
  RTC_LMIC = LMIC;

  // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
  // Therefore reset DutyCyles

  unsigned long now = millis();

  // EU Like Bands
#if defined(CFG_LMIC_EU_like)
  Serial.println(F("Reset CFG_LMIC_EU_like band avail"));
  for (int i = 0; i < MAX_BANDS; i++)
  {
    ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (correctedAvail < 0)
    {
      correctedAvail = 0;
    }
    RTC_LMIC.bands[i].avail = correctedAvail;
  }

  RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
  if (RTC_LMIC.globalDutyAvail < 0)
  {
    RTC_LMIC.globalDutyAvail = 0;
  }
#else
  Serial.println(F("No DutyCycle recalculation function!"));
#endif
}

void LoadLMICFromRTC() {
  if (RTC_LMIC.seqnoUp != 0) {
    Serial.println(F("Load LMIC from RTC"));
    LMIC = RTC_LMIC;
  }
}

void GoDeepSleep()
{
  Serial.println(F("Go DeepSleep"));
  PrintRuntime();
  Serial.flush();
  esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
  esp_deep_sleep_start();
}

void switchOff() {
  Serial.println("---------POWER DOWN");
  digitalWrite(POWERPIN, LOW);  // Switch board off
  delay(TX_INTERVAL * 1000); // These lines are only for test. They are never reached
  // during normal operation
  ESP.restart();
}

void setup()
{
#ifdef POWEROFF
  pinMode(POWERPIN, OUTPUT);
  digitalWrite(POWERPIN, HIGH);
#endif
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  Serial.println(F("Starting DeepSleep test"));
  PrintLMICVersion();

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.  LMIC_reset();

#ifdef POWEROFF
  LoadLMICfromEEPROM();
#else
  LoadLMICfromRTC();
#endif

  LoraWANDebug(LMIC);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
  entry = millis();
}

void loop()
{
  static unsigned long lastPrintTime = 0;

  os_runloop_once();

  const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));
  if (!timeCriticalJobs && enableSleep_ == true && !(LMIC.opmode & OP_TXRXPEND))
  {
    Serial.print(F("Can go sleep "));
    LoraWANPrintLMICOpmode();
#ifdef POWEROFF
    SaveLMICtoEEPROM(TX_INTERVAL);
    switchOff();
#else
    SaveLMICtoRTC(TX_INTERVAL);
    GoDeepSleep();
#endif
  }
  else if (lastPrintTime + 2000 < millis())
  {
    Serial.print(F("Cannot sleep "));
    Serial.print(F("TimeCriticalJobs: "));
    Serial.print(timeCriticalJobs);
    Serial.print(" ");

    LoraWANPrintLMICOpmode();
    PrintRuntime();
    lastPrintTime = millis();
  }
  if (millis() - entry > 30000) {
    Serial.println("Reset LMIC");
    LMIC.seqnoUp = 0;
#ifdef POWEROFF
    SaveLMICtoEEPROM(TX_INTERVAL);
    switchOff();
#else
    SaveLMICtoRTC(TX_INTERVAL);
    GoDeepSleep();
#endif
  }
}
