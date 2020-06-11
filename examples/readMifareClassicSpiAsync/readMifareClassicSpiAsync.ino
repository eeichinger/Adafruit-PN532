/*
   Demo for non-blocking wait for ID targets using SPI and IRQ
*/

#include <Adafruit_PN532.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SS   (10)
#define PN532_IRQ   (2)

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
Adafruit_PN532 nfc(PN532_SS);

// NFC interrupt handler
void handleInterrupt() {
  nfc.handleInterrupt();
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println("Init PN532");
  nfc.begin();
  delay(100);
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  } else {
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);
    // configure board to read RFID tags
    nfc.SAMConfig();
  }

  delay(10);       // Optional delay. Some board do need more time after init to be ready, see Readme

  // register interrupt handler
  attachInterrupt(digitalPinToInterrupt(PN532_IRQ), handleInterrupt, FALLING);
  // start async listening mode
  nfc.beginReadPassiveTargetID(PN532_MIFARE_ISO14443A);
}

void loop() {
  byte success = false;
  byte uid[] = "\0\0\0\0\0\0\0"; // Buffer to store the returned UID
  byte uidLength;                // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  unsigned long timeForConsumeResult = micros();

  // shouldn't be needed, but just to be safe disable interrupts while reading card id
  // note: this blocks interrupts for ~1.5ms causing issues if your timing requirements are more sensitive
  noInterrupts();
  // fetch result if IRQ line indicates that there is one
  if (nfc.isAsyncCommandResultAvailable()) {
    success = nfc.completeReadPassiveTargetID(uid, &uidLength);
    Serial.print("Interrupt, Success:"); Serial.println(success, HEX);
  }
  interrupts();

  if (success) {
    timeForConsumeResult = micros() - timeForConsumeResult;
    success = false;
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.println("  Time Taken: "); Serial.print(timeForConsumeResult);
    Serial.print("  UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");
    // put back into listening mode
    nfc.beginReadPassiveTargetID(PN532_MIFARE_ISO14443A);
  }
  Serial.flush();
  delay(100);
}
