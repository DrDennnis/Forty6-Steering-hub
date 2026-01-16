#include <Arduino.h>
#include "driver/timer.h"
#include "E46_Codes.h"
#include <IbusTrx.h>

#define IBUS_RX_PIN 20
#define IBUS_TX_PIN 21

IbusTrx ibus;
byte source, length, destination, databytes[36];

const int modeSwitchPin = 10;
const int CRUISE_TX_PIN = 9;

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
  { 4, BTN_VOL_UP,       VOID,               false },
  { 5, BTN_VOL_DOWN,     VOID,               false },
  { 6, BTN_NEXT,         VOID,               false },
  { 7, BTN_PREV,         VOID,               false },
};

enum ButtonCommand : uint8_t {
  CMD_KEEPALIVE1 = 255, // 11111111
  CMD_KEEPALIVE2 = 254, // 11111110
  CMD_IO         = 219,
  CMD_RESET      = 111,
  CMD_PLUS       = 183,
  CMD_MINUS      = 253 
};

const uint64_t TIMER_INTERVAL_US = 100;
const int TICKS_FOR_GAP          = 100;

enum TxState {
  TX_IDLE,
  TX_DATA_BIT,
  TX_STOP_BIT,
  TX_GAP
};
volatile TxState txState = TX_IDLE;

volatile ButtonCommand heldCommand = CMD_KEEPALIVE1;
volatile bool keepAliveToggle = false;
volatile uint8_t currentByte = 0;
volatile int bitIndex = 8; 
volatile int tickCounter = 0;
volatile int lowPulseTicks = 0;

hw_timer_t * timer = NULL; 

void IRAM_ATTR onTimer();
void initTimer();

void scheduleFrame(ButtonCommand cmd) { 
  heldCommand = cmd; 
}

// Media buttons (MFL module, source 0x68)
// https://curious.ninja/blog/arduino-bmw-i-bus-interface-messages/
uint8_t NEXT_TRACK[7] PROGMEM = {0x50, 0x04, 0x68, 0x3B, 0x01, 0x06};
uint8_t PREV_TRACK[7] PROGMEM = {0x50, 0x04, 0x68, 0x3B, 0x08, 0x0F};
uint8_t FLASH_HIGHBEAM[5] PROGMEM = {0x00, 0x04, 0xbf, 0x08};
uint8_t IBUS_MODE[5] PROGMEM = {0x68, 0x04, 0x3B, 0x40};

void handleVoid()          {}
void handleCruiseMinus()   { Serial.print("Cruise Minus pressed");        scheduleFrame(CMD_MINUS);   }
void handleCruisePlus()    { Serial.print("Cruise Plus pressed");         scheduleFrame(CMD_PLUS);    }
void handleCruiseIO()      { Serial.print("Cruise IO pressed");           scheduleFrame(CMD_IO);      }
void handleCruiseReset()   { Serial.print("Cruise re-set pressed");       scheduleFrame(CMD_RESET);   }
void handleVolUp()         { Serial.print("Volume + pressed");            ibus.write(MFL_VOL_UP);     }
void handleVolDown()       { Serial.print("Volume - pressed");            ibus.write(MFL_VOL_DOWN);   }
void handleNext()          { Serial.print("Next track pressed");          ibus.write(NEXT_TRACK);     }
void handlePrev()          { Serial.print("Prev track pressed");          ibus.write(PREV_TRACK);     }
void handleRT()            { Serial.print("R/T pressed");                 ibus.write(MFL_RT_PRESS);   }
void handleMode()          { Serial.print("Mode pressed");                ibus.write(IBUS_MODE);      }
void handleFlashHighbeam() { Serial.print("Flash Highbeam pressed");      ibus.write(FLASH_HIGHBEAM); }

ButtonAction actions[] = {
  { VOID,               handleVoid,          },
  { BTN_CRUISE_MINUS,   handleCruiseMinus,   },
  { BTN_CRUISE_PLUS,    handleCruisePlus,    },
  { BTN_CRUISE_IO,      handleCruiseIO,      },
  { BTN_CRUISE_RESET,   handleCruiseReset,   },
  { BTN_VOL_UP,         handleVolUp,         },
  { BTN_VOL_DOWN,       handleVolDown,       },
  { BTN_NEXT,           handleNext,          },
  { BTN_PREV,           handlePrev,          },
  { BTN_RT,             handleRT,            },
  { BTN_MODE,           handleMode,          },
  { BTN_FLASH_HIGHBEAM, handleFlashHighbeam, }
};

void setup()
{
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8E1, IBUS_RX_PIN, IBUS_TX_PIN);
  ibus.begin(Serial1);

  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (auto &map : pinMapping) {
    pinMode(map.pin, INPUT_PULLUP);
  }

  pinMode(CRUISE_TX_PIN, OUTPUT);
  digitalWrite(CRUISE_TX_PIN, HIGH);
  initTimer();

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
  ibus.available();
  checkPins();
}

void initTimer() {
  // ESP32-C3 compatible timer initialization
  timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz), count up
  if (timer == NULL) {
    Serial.println("Timer initialization failed!");
    return;
  }
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERVAL_US, true);  // Trigger every 100us
  timerAlarmEnable(timer);
}

void IRAM_ATTR onTimer() {
  tickCounter++;

  switch (txState) {
    // --- Idle (bus HIGH) ---
    case TX_IDLE:
      digitalWrite(CRUISE_TX_PIN, HIGH); // Ensure idle is HIGH
      // Immediately start sending the next message
      tickCounter = 0;
      currentByte = (heldCommand == CMD_KEEPALIVE1 || heldCommand == CMD_KEEPALIVE2)
                    ? (keepAliveToggle ? CMD_KEEPALIVE2 : CMD_KEEPALIVE1)
                    : heldCommand;
      keepAliveToggle = !keepAliveToggle;
      bitIndex = 8; // start with MSB
      txState = TX_DATA_BIT; // first bit
      break;

    // --- Data Bit Slot ---
    case TX_DATA_BIT: {
      bool bit = (currentByte >> (bitIndex - 1)) & 1;

      int highTicks, lowTicks;
      if (bit) {
        // Bit 1: shorter LOW at the end
        highTicks = 4;  // adjust as needed
        lowTicks  = 2;
      } else {
        // Bit 0: longer LOW at the end
        highTicks = 2;  // adjust as needed
        lowTicks  = 4;
      }

      if (tickCounter <= highTicks) {
        digitalWrite(CRUISE_TX_PIN, HIGH);
      } else if (tickCounter <= (highTicks + lowTicks)) {
        digitalWrite(CRUISE_TX_PIN, LOW);
      } else {
        // End of slot
        tickCounter = 0;
        bitIndex--;
        if (bitIndex == 0) {
          // All 8 bits sent, send stop bit (LOW 1/2 ticks for example)
          lowPulseTicks = 3;
          digitalWrite(CRUISE_TX_PIN, LOW);
          txState = TX_STOP_BIT;
        } else {
          txState = TX_DATA_BIT; // next data bit
        }
      }
      break;
    }

    // --- Stop Bit ---
    case TX_STOP_BIT:
      if (tickCounter >= lowPulseTicks) {
        tickCounter = 0;
        digitalWrite(CRUISE_TX_PIN, HIGH); // return to idle
        txState = TX_GAP;
      }
      break;

    // --- Gap Between Messages ---
    case TX_GAP:
      digitalWrite(CRUISE_TX_PIN, HIGH);
      if (tickCounter >= TICKS_FOR_GAP) {
        tickCounter = 0;
        txState = TX_IDLE;
      }
      break;
  }
}
