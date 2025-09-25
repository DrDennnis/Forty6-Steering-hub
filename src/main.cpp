#include <Arduino.h>
#include "IbusSerial.h"
#include "E46_Codes.h"

IbusSerial ibus;
byte source, length, destination, databytes[36];

// =====================
// TYPES
// =====================
enum ButtonType {
  VOID,
  BTN_CRUISE_MINUS,
  BTN_CRUISE_PLUS,
  BTN_CRUISE_IO,
  BTN_CRUISE_RESET,
  BTN_VOL_UP,
  BTN_VOL_DOWN,
  BTN_NEXT,
  BTN_PREV,
  BTN_RT,
  BTN_MODE,
  BTN_FLASH_HIGHBEAM
};

struct ButtonAction {
  ButtonType type;
  const char* name;
  void (*handler)();
};

struct PinButtonMap {
  int pin;
  ButtonType type;
  ButtonType alternativeType;
  bool lastState;
};

// Pin mappings
// Depending on the mode, other dispatch function is used
PinButtonMap pinMapping[] = {
  { 0, BTN_CRUISE_MINUS, BTN_FLASH_HIGHBEAM, false },
  { 1, BTN_CRUISE_PLUS,  VOID,               false },
  { 2, BTN_CRUISE_IO,    VOID,               false },
  { 3, BTN_CRUISE_RESET, VOID,               false },
  { 5, BTN_VOL_UP,       VOID,               false },
  { 6, BTN_VOL_DOWN,     VOID,               false },
  { 7, BTN_NEXT,         VOID,               false },
  { 8, BTN_PREV,         VOID,               false },
};

const int modeSwitchPin = 10;
const int emulatePinOut = 9;
const int BIT_0_US = 200;
const int BIT_1_US = 400;
const int INTER_MSG_GAP_US = 10000;

void driveLow() {
  pinMode(emulatePinOut, OUTPUT);
  digitalWrite(emulatePinOut, LOW);
}

void releaseLine() {
  pinMode(emulatePinOut, INPUT);
}

void sendBit(bool bit) {
  driveLow();
  if (bit) {
    delayMicroseconds(BIT_1_US);
  } else {
    delayMicroseconds(BIT_0_US);
  }
  releaseLine();
  delayMicroseconds(50);
}

void sendByte(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    bool bit = (value >> i) & 0x01;
    sendBit(bit);
  }
}

void emulateButton(uint8_t code, int repeats = 5) {
  for (int i = 0; i < repeats; i++) {
    sendByte(code);
    delayMicroseconds(INTER_MSG_GAP_US);
  }
}

void ibusWrite(const byte message[], byte size)
{
  ibus.write(message, size);

  for (int i = 0; i < size; i++) {
    if (message[i] < 0x10)
      Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
}

// Media buttons (MFL module, source 0x68)
// https://curious.ninja/blog/arduino-bmw-i-bus-interface-messages/
const uint8_t NEXT_TRACK[7] PROGMEM = {0x50, 0x04, 0x68, 0x3B, 0x01, 0x06};
const uint8_t PREV_TRACK[7] PROGMEM = {0x50, 0x04, 0x68, 0x3B, 0x08, 0x0F};
const uint8_t FLASH_HIGHBEAM[5] PROGMEM = {0x00, 0x04, 0xbf, 0x08};
const uint8_t ibusMode[]     = {0x68, 0x04, 0x3B, 0x40};

void handleVoid()          {}
void handleCruiseMinus()   { Serial.print("Cruise Minus pressed");        emulateButton(252);                                }
void handleCruisePlus()    { Serial.print("Cruise Plus pressed");         emulateButton(182);                                }
void handleCruiseIO()      { Serial.print("Cruise IO pressed");           emulateButton(110);                                }
void handleCruiseReset()   { Serial.print("Cruise re-set pressed");       emulateButton(218);                                }
void handleVolUp()         { Serial.print("Volume + pressed");            ibusWrite(MFL_VOL_UP,     sizeof(MFL_VOL_UP));     }
void handleVolDown()       { Serial.print("Volume - pressed");            ibusWrite(MFL_VOL_DOWN,   sizeof(MFL_VOL_DOWN));   }
void handleNext()          { Serial.print("Next track pressed");          ibusWrite(NEXT_TRACK,     sizeof(NEXT_TRACK));     }
void handlePrev()          { Serial.print("Prev track pressed");          ibusWrite(PREV_TRACK,     sizeof(PREV_TRACK));     }
void handleRT()            { Serial.print("R/T pressed");                 ibusWrite(MFL_RT_PRESS,   sizeof(MFL_RT_PRESS));   }
void handleMode()          { Serial.print("Mode pressed");                ibusWrite(ibusMode,       sizeof(ibusMode));       }
void handleFlashHighbeam() { Serial.print("Flash Highbeam pressed");      ibusWrite(FLASH_HIGHBEAM, sizeof(FLASH_HIGHBEAM)); }

ButtonAction actions[] = {
  { VOID,               "Void",          handleVoid,          },
  { BTN_CRUISE_MINUS,   "Cruise Minus",  handleCruiseMinus,   },
  { BTN_CRUISE_PLUS,    "Cruise Plus",   handleCruisePlus,    },
  { BTN_CRUISE_IO,      "Cruise IO",     handleCruiseIO,      },
  { BTN_CRUISE_RESET,   "Cruise re-set", handleCruiseReset,   },
  { BTN_VOL_UP,         "Volume +",      handleVolUp,         },
  { BTN_VOL_DOWN,       "Volume -",      handleVolDown,       },
  { BTN_NEXT,           "Next Track",    handleNext,          },
  { BTN_PREV,           "Prev Track",    handlePrev,          },
  { BTN_RT,             "R/T",           handleRT,            },
  { BTN_MODE,           "Mode",          handleMode,          },
  { BTN_FLASH_HIGHBEAM, "Mode",          handleFlashHighbeam, }
};

void setup()
{
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8E1, RX, TX);
  ibus.setIbusSerial(Serial1);
  ibus.sleepEnable(60);

  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(emulatePinOut, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  for (auto &map : pinMapping) {
    pinMode(map.pin, INPUT_PULLUP);
  }

  releaseLine();

  Serial.println("Forty6 Steeringhub started");
}

void dispatchButton(ButtonType type) {
  for (auto& action : actions) {
    if (action.type == type) {
      action.handler();
      Serial.println();
    }
  }
}

void checkPins() {
  for (auto &map : pinMapping) {
    bool pressed = (digitalRead(map.pin) == LOW);
    if (pressed && !map.lastState) {
      if (map.alternativeType != VOID && digitalRead(modeSwitchPin) == LOW) {
        dispatchButton(map.alternativeType);
      } else {
        dispatchButton(map.type);
      }
    }

    map.lastState = pressed;
  }
}

void loop()
{
  ibus.run();
  checkPins();
}
