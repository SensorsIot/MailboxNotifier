// This sketch demonstrates how to use the TPL5010 timer to wakeup the Atmgea 1284P from the most low power sleep for a transmissions to reduce power consumption

/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Copyright (c) 2018 Terry Moore, MCCI

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in
   arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include "PinChangeInterrupt.h" // https://github.com/NicoHood/PinChangeInterrupt // you can also use the attachInterrupt() function from Arduino, to be consistent through all examples this lib is here used too


// Create a file TTN_Credentials.h and add the following lines with your keys from TTN or fill in your keys below
#define FILLMEIN_APPEUI 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define FILLMEIN_DEVEUI 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define FILLMEIN_APPKEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//#include "TTN_Credentials.h"

enum wakeupReasons {
  unknown,
  door,
  lid
};

#define DEBUG

#ifdef DEBUG
#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTLN(x) Serial.println(x)
#else
#define DEBUGPRINT(x)
#define DEBUGPRINTLN(x)
#endif

bool next = false;
const int donePin = 3; // define done pin where TPL5010 is connected to, in our case pin 3
const int TPLWakePin = 2; // define WDT pin where TPL5010 is connected to, in our case pin 2
const int doorSwitch = 4; // define wakeup pin, in our case pin 4 which is pulled up to VCC with a 10K resistor on the board
const int lidSwitch = 17; // lid switch connected to SDA pin
uint8_t wakeupReason = 0; // set to 0 means startup, set to 1 means wakeup from TPL5010, set to 2 means wakeup from user pin, set to 3 means unknown wakeup reason
int batteryReadPin = A0;        // set the input pin for the battery measurement
int voltageDividerPin = 0;      // set the pin to enable the voltage divider
const float AREF = 1.0833;         // internal reference votlage, for better accuracy use the printed value on the sticker!
const float RGND = 20.051;          // voltage divider resistor from A0 to GND, for better accuracy use the printed value on the sticker! Value in K ohms
const float RSUP = 99.98;         // voltage divider resistor from A0 to VCC or UREG depending on solder jumper, for better accuracy use the printed value on the sticker! Value in K ohms

bool mailAvailable = false;

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
//# define COMPILE_REGRESSION_TEST //comment this out if you want to compile this sketch with 'FILLMEIN' values for APPEUI, DEVEUI and APPKEY
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { FILLMEIN_APPEUI };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { FILLMEIN_DEVEUI };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN_APPKEY };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// This value is only used to hack timer0, set it to the programmed time configuration of the TPL5010
const unsigned TX_INTERVAL = 120;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 14,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 13,
  .dio = {10, 11, 12},
};

void printHex2(unsigned v) {
  v &= 0xff;
#ifdef DEBUG
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
#endif
}

void onEvent (ev_t ev) {
  DEBUGPRINT(os_getTime());
  DEBUGPRINT(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      DEBUGPRINTLN(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      DEBUGPRINTLN(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      DEBUGPRINTLN(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      DEBUGPRINTLN(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      DEBUGPRINTLN(F("EV_JOINING"));
      break;
    case EV_JOINED:
      DEBUGPRINTLN(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
#ifdef DEBUG
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
#endif
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            DEBUGPRINT("-");
          printHex2(artKey[i]);
        }
        DEBUGPRINTLN("");
        DEBUGPRINT("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            DEBUGPRINT("-");
          printHex2(nwkKey[i]);
        }
        DEBUGPRINTLN();
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
      ||     DEBUGPRINTLN(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      DEBUGPRINTLN(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      DEBUGPRINTLN(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      DEBUGPRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        DEBUGPRINTLN(F("Received ack"));
      if (LMIC.dataLen) {
        DEBUGPRINT(F("Received "));
        DEBUGPRINT(LMIC.dataLen);
        DEBUGPRINTLN(F(" bytes of payload"));
      }
      // Schedule next transmission
      next = true;
      //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      DEBUGPRINTLN(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      DEBUGPRINTLN(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      DEBUGPRINTLN(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      DEBUGPRINTLN(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      DEBUGPRINTLN(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    DEBUGPRINTLN(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      DEBUGPRINTLN(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      DEBUGPRINTLN(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      DEBUGPRINTLN(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      DEBUGPRINT(F("Unknown event: "));
      DEBUGPRINTLN((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    DEBUGPRINTLN(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    int voltage = readBattVoltage();

    DEBUGPRINT("mailAvalable ");
    Serial.println(mailAvailable);

    DEBUGPRINT("reason ");
    Serial.println(wakeupReason);

    DEBUGPRINT("voltage ");
    DEBUGPRINTLN(voltage);

    //Prepare uplink data
    byte mydata[3];
    mydata[0] = wakeupReason | mailAvailable << 7;
    mydata[1] = voltage >> 8;
    mydata[2] = voltage;

    /*

      //TTNv3 Payload decoder
function decodeUplink(input) {
  var data = {};
  if (input.fPort == 1) {
    data.voltage = ((input.bytes[1] << 8) | input.bytes[2]) / 100.00;
    data.reason = (input.bytes[0])& 0x0F;
    data.mailAvailable = (input.bytes[0]>>7)
  }
  return {
    data: data,
    warnings: [],
    errors: []
  };
}

    */

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    DEBUGPRINTLN(F("Packet queued"));

    // Set wakeupReason value to 99, if we ever transmit the value '99', then the device woke up because of an unknown reason
    wakeupReason = 99;
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void wakeupFromTimer(void) {
  wakeupReason = 1;
  DEBUGPRINTLN("Timer Interrupt");
}

void wakeupFromLidSwitch(void) {
  if (digitalRead(lidSwitch) == HIGH) {
    wakeupReason = 2;
    mailAvailable = true;
  }
  DEBUGPRINTLN("Lid");
}

void wakeupFromDoorSwitch(void) {
  if (digitalRead(doorSwitch) == HIGH) {
    wakeupReason = 3;
    mailAvailable = true;
  }
  DEBUGPRINTLN("Door");
}

uint16_t analogOversample(int pin) {
  uint16_t reading = 0;
  int i = 0;
  while (i < 8)
  {
    i++;
    reading = reading + analogRead(pin);
  }
  return reading >> 3;
}

int readBattVoltage() {
  //Enable voltage divider, warm up ADC, read voltage, disable voltage divider
  digitalWrite(voltageDividerPin, HIGH);
  for (int i = 0; i < 16; i++) analogRead(batteryReadPin); // first readings are not accurate
  uint16_t ADCreading = analogOversample(batteryReadPin);
  digitalWrite(voltageDividerPin, LOW);

  //Convert analog ADC reading to millivolts
  int _voltage = (((ADCreading * AREF / 1024.0) * (RSUP + RGND) / RGND) * 100);
  return _voltage;
}
void do_sendmac(osjob_t* j) {
  // This function is called if we need to process a MAC message
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    DEBUGPRINTLN(F("OP_TXRXPEND, not sending"));
  }
  else {
    // Prepare upstream data transmission at the next possible time.
    byte mydata[1];
    LMIC_setTxData2(0, mydata, sizeof(mydata), 0);
    DEBUGPRINTLN(F("Packet queued (MAC command)"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  DEBUGPRINTLN("Starting");

  // Pin settings for TPL5010 usage
  pinMode(TPLWakePin, INPUT);
  pinMode(donePin, OUTPUT);

  // Pin settings for User interrupt pin
  pinMode(doorSwitch, INPUT);
  pinMode(lidSwitch, INPUT);

  // Attach TPL5010 interrupt
  attachPCINT(digitalPinToPCINT(TPLWakePin), wakeupFromTimer, CHANGE);

  // Attach user interrupt pin
  attachPCINT(digitalPinToPCINT(lidSwitch), wakeupFromLidSwitch, CHANGE);

  // Attach user interrupt pin
  attachPCINT(digitalPinToPCINT(doorSwitch), wakeupFromDoorSwitch, CHANGE);

  // Pin settings to read supply voltage
  analogReference(INTERNAL1V1);         // Set internal reference to 1.1V
  pinMode(batteryReadPin, INPUT);       // Set analog pin to input
  pinMode(voltageDividerPin, OUTPUT);   // Set pin which controls the voltage divider to output

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  extern volatile unsigned long timer0_overflow_count;
  if (next == false) {
    os_runloop_once();

    // If we've to wait for the RFM95 module to complete TX or wait for RX window can't powerDown the MCU,
    // as LMIC controls RX timing and LMIC uses micros(), powerDown would stop micros() from running.
    // Instead we can put the MCU into idle state with TIMER0_ON. Use here the lowest possible sleeptime (SLEEP_15MS),
    // as TIMER0 will overflow every 1ms, MCU will wake up every ms again. Also, MCU will be waken up after the
    // selected time, in this case 15ms. We can save about 60% of the power during waiting with this simple trick.

    // Check if any time critical jobs is due in the next 1ms
    if (os_queryTimeCriticalJobs(ms2osticks(1)) == 0) {
      // If no time critical job is due, enter idle mode for 1ms
      LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
    }
  }
  else {
    if (LMIC.pendMacLen > 0) {
      DEBUGPRINTLN("Pending MAC message");
      next = false;
      do_sendmac(&sendjob);
    }
    else {
      DEBUGPRINTLN("No Pending MAC message");
      Serial.flush(); // give the serial print chance to complete
      delay(20); // We need here some delay to prevent wakeup from WDT we used before. 20ms seems to be safe.

      // set DONE high for 100 micro seconds to tell the TPL5010 we're done
      // if DONE was not high between TPL5010 interrupts, TPL5010 will reset the MCU through the reset pin if connected, otherwise it will do nothing until 'DONE' was set high once
      digitalWrite(donePin, HIGH);
      delayMicroseconds(100);
      digitalWrite(donePin, LOW);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

      // wakeupReason 99 means that the interrupt was called on a falling edge. We only want to trigger the transmit on the rising edge
      if (wakeupReason < 99) {
        // LMIC uses micros() to keep track of the duty cycle, so
        // hack timer0_overflow for a rude adjustment:
        timer0_overflow_count += TX_INTERVAL * 64 * clockCyclesPerMicrosecond();
        next = false;
        do_send(&sendjob);
      }
    }
  }
}
