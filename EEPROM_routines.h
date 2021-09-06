lmic_t EEPROM_LMIC;
#define EEPROM_SIZE 1000


int writeEEPROM(const lmic_t & value) {
  const byte * p = (const byte*) &value;
  int i;
  for (i = 0; i < sizeof value; i++) {
    byte EE = (*p++);
    EEPROM.write(i, EE);
    Serial.print(EE);
  }
  Serial.println();
  Serial.println(i);
  EEPROM.commit();
  return i;
}

unsigned int readEEPROM(lmic_t & value) {
  byte * p = (byte*) &value;
  unsigned int i;
  byte _hi;
  for (i = 1; i < sizeof value; i++) {
    _hi = EEPROM.read(i);
    *p++ = _hi;
    Serial.print(_hi);
  }
  Serial.println();
  Serial.println(i);
  return i;
}

void SaveLMICtoEEPROM(int deepsleep_sec) {
  EEPROM_LMIC = LMIC;
  // EU Like Bands

  // System time is resetted after sleep. So we need to calculate the dutycycle
  // with a resetted system time
  unsigned long now = millis();

#if defined(CFG_LMIC_EU_like)
  for (int i = 0; i < MAX_BANDS; i++) {
    ostime_t correctedAvail =
      EEPROM_LMIC.bands[i].avail -
      ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (correctedAvail < 0) {
      correctedAvail = 0;
    }
    EEPROM_LMIC.bands[i].avail = correctedAvail;
  }

  EEPROM_LMIC.globalDutyAvail = EEPROM_LMIC.globalDutyAvail -
                                ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
  if (EEPROM_LMIC.globalDutyAvail < 0) {
    EEPROM_LMIC.globalDutyAvail = 0;
  }
#else
  Serial
  .println("No DutyCycle recalculation function!")
#endif
  EEPROM.put(0, EEPROM_LMIC);
  EEPROM.commit();
  Serial.println();
  Serial.println("EEPROM Saved");
}

// Load LMIC structure from EEPROM to avoid re-joining
void LoadLMICfromEEPROM() {
  Serial.print("Retrieving EEPROM ");
  EEPROM.get(0, EEPROM_LMIC);
  if (EEPROM_LMIC.seqnoUp != 0) {
    Serial.println("Retrieving LMIC");
    LMIC = EEPROM_LMIC;
  }
  else {
    LMIC.seqnoUp = 0;
    Serial.println("Sequence set to 0");
  }
}
