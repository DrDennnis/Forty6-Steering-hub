#include <Arduino.h>
#include "driver/timer.h"
#include "E46_Codes.h"
#include <IbusTrx.h>

// Pin definitions
#define IBUS_RX_PIN       20
#define IBUS_TX_PIN       21
#define LED_PIN           8
#define MODE_SWITCH_PIN   10
#define CRUISE_TX_PIN     9

#define BTN_PIN_0         0
#define BTN_PIN_1         1
#define BTN_PIN_2         2
#define BTN_PIN_3         3
#define BTN_PIN_4         4
#define BTN_PIN_5         5
#define BTN_PIN_6         6
#define BTN_PIN_7         7

// PWM configuration
#define PWM_CHANNEL       0
#define PWM_FREQ_HZ       2000
#define PWM_RESOLUTION    8
#define GAMMA_CORRECTION  1.8f

// Serial configuration
#define SERIAL_BAUD_RATE  115200
#define IBUS_BAUD_RATE    9600

// IBUS addresses and commands
#define LCM_ADDR          0xD0
#define IBUS_CMD_DIMMER   0x5C

// Brightness settings (0-100 range)
#define DEFAULT_BRIGHTNESS    50
#define BRIGHTNESS_MIN        0
#define BRIGHTNESS_MAX        100
#define PWM_VALUE_MIN         0
#define PWM_VALUE_MAX         255

// Timer configuration for 1MHz tick rate (80MHz / 80 prescaler)
#define TIMER_INDEX           0
#define TIMER_PRESCALER       80
#define TIMER_INTERVAL_US     100
#define TICKS_FOR_GAP         100

// Cruise control bit timing in timer ticks
#define BIT_HIGH_TICKS_ONE    4
#define BIT_LOW_TICKS_ONE     2
#define BIT_HIGH_TICKS_ZERO   2
#define BIT_LOW_TICKS_ZERO    4
#define STOP_BIT_TICKS        3
#define BITS_PER_BYTE         8

IbusTrx ibus;

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
  uint8_t pin;
  ButtonType type;
  ButtonType alternativeType;
  bool lastState;
};

// Pin to button mappings with alternative actions when mode switch is active
PinButtonMap pinMapping[] = {
  { BTN_PIN_0, BTN_CRUISE_MINUS, BTN_FLASH_HIGHBEAM, false },
  { BTN_PIN_1, BTN_CRUISE_PLUS,  VOID,               false },
  { BTN_PIN_2, BTN_CRUISE_IO,    VOID,               false },
  { BTN_PIN_3, BTN_CRUISE_RESET, VOID,               false },
  { BTN_PIN_4, BTN_VOL_UP,       VOID,               false },
  { BTN_PIN_5, BTN_VOL_DOWN,     VOID,               false },
  { BTN_PIN_6, BTN_NEXT,         VOID,               false },
  { BTN_PIN_7, BTN_PREV,         VOID,               false },
};

// Cruise control command bytes
enum ButtonCommand : uint8_t {
  CMD_KEEPALIVE1 = 255,
  CMD_KEEPALIVE2 = 254,
  CMD_IO         = 219,
  CMD_RESET      = 111,
  CMD_PLUS       = 183,
  CMD_MINUS      = 253
};

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
volatile uint8_t bitIndex = BITS_PER_BYTE;
volatile uint16_t tickCounter = 0;
volatile uint8_t lowPulseTicks = 0;

hw_timer_t* timer = NULL; 

void IRAM_ATTR onTimer();
void initTimer();
void setBrightness(uint8_t linValue);

void scheduleFrame(ButtonCommand cmd) { 
  heldCommand = cmd; 
}

// IBUS media button messages for MFL module (source 0x68)
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
  Serial.begin(SERIAL_BAUD_RATE);

  Serial1.begin(IBUS_BAUD_RATE, SERIAL_8E1, IBUS_RX_PIN, IBUS_TX_PIN);
  ibus.begin(Serial1);

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (auto &mapping : pinMapping) {
    pinMode(mapping.pin, INPUT_PULLUP);
  }

  pinMode(CRUISE_TX_PIN, OUTPUT);
  digitalWrite(CRUISE_TX_PIN, HIGH);
  initTimer();

  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
  setBrightness(DEFAULT_BRIGHTNESS);

  Serial.println("Forty6 Steeringhub started");
}

uint8_t gammaCorrect(uint8_t value) {
  float normalized = value / (float)PWM_VALUE_MAX;
  float corrected = pow(normalized, GAMMA_CORRECTION);
  return (uint8_t)(corrected * PWM_VALUE_MAX + 0.5f);
}

void setBrightness(uint8_t linValue) {
  linValue = constrain(linValue, BRIGHTNESS_MIN, BRIGHTNESS_MAX);

  uint8_t pwm = map(linValue, BRIGHTNESS_MIN, BRIGHTNESS_MAX, PWM_VALUE_MIN, PWM_VALUE_MAX);
  pwm = gammaCorrect(pwm);

  ledcWrite(PWM_CHANNEL, pwm);
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
  for (auto &mapping : pinMapping) {
    bool pressed = (digitalRead(mapping.pin) == LOW);
    if (pressed && !mapping.lastState) {
      if (mapping.alternativeType != VOID && digitalRead(MODE_SWITCH_PIN) == LOW) {
        dispatchButton(mapping.alternativeType);
      } else {
        dispatchButton(mapping.type);
      }
    }

    mapping.lastState = pressed;
  }
}

void handleIbusMessage() {
  IbusMessage message = ibus.readMessage();
  uint8_t src = message.source();
  uint8_t len = message.length();

  // Handle dimmer brightness messages from Light Control Module
  if (src == LCM_ADDR && len >= 2) {
    uint8_t cmd = message.b(0);

    if (cmd == IBUS_CMD_DIMMER && len >= 2) {
      uint8_t brightness = message.b(1);
      uint8_t mappedBrightness = map(brightness, PWM_VALUE_MIN, PWM_VALUE_MAX, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
      setBrightness(mappedBrightness);

      Serial.print("IBUS Dimmer: ");
      Serial.print(brightness);
      Serial.print(" -> ");
      Serial.println(mappedBrightness);
    }
  }
}

void loop()
{
  if (ibus.available()) {
    handleIbusMessage();
  }

  checkPins();
}

// Initialize hardware timer for cruise control signal generation
void initTimer() {
  timer = timerBegin(TIMER_INDEX, TIMER_PRESCALER, true);
  if (timer == NULL) {
    Serial.println("Timer initialization failed!");
    return;
  }
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERVAL_US, true);
  timerAlarmEnable(timer);
}

void IRAM_ATTR onTimer() {
  tickCounter++;

  switch (txState) {
    case TX_IDLE:
      digitalWrite(CRUISE_TX_PIN, HIGH);
      tickCounter = 0;
      currentByte = (heldCommand == CMD_KEEPALIVE1 || heldCommand == CMD_KEEPALIVE2)
                    ? (keepAliveToggle ? CMD_KEEPALIVE2 : CMD_KEEPALIVE1)
                    : heldCommand;
      keepAliveToggle = !keepAliveToggle;
      bitIndex = BITS_PER_BYTE;
      txState = TX_DATA_BIT;
      break;

    case TX_DATA_BIT: {
      bool bit = (currentByte >> (bitIndex - 1)) & 1;

      uint8_t highTicks = 0;
      uint8_t lowTicks = 0;
      if (bit) {
        highTicks = BIT_HIGH_TICKS_ONE;
        lowTicks  = BIT_LOW_TICKS_ONE;
      } else {
        highTicks = BIT_HIGH_TICKS_ZERO;
        lowTicks  = BIT_LOW_TICKS_ZERO;
      }

      if (tickCounter <= highTicks) {
        digitalWrite(CRUISE_TX_PIN, HIGH);
      } else if (tickCounter <= (highTicks + lowTicks)) {
        digitalWrite(CRUISE_TX_PIN, LOW);
      } else {
        tickCounter = 0;
        bitIndex--;
        if (bitIndex == 0) {
          lowPulseTicks = STOP_BIT_TICKS;
          digitalWrite(CRUISE_TX_PIN, LOW);
          txState = TX_STOP_BIT;
        } else {
          txState = TX_DATA_BIT;
        }
      }
      break;
    }

    case TX_STOP_BIT:
      if (tickCounter >= lowPulseTicks) {
        tickCounter = 0;
        digitalWrite(CRUISE_TX_PIN, HIGH);
        txState = TX_GAP;
      }
      break;

    case TX_GAP:
      digitalWrite(CRUISE_TX_PIN, HIGH);
      if (tickCounter >= TICKS_FOR_GAP) {
        tickCounter = 0;
        txState = TX_IDLE;
      }
      break;
  }
}
