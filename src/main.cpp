#include <Arduino.h>
#include "driver/timer.h"
#include "E46_Codes.h"
#include <IbusTrx.h>

// Pin definitions
#define IBUS_RX_PIN       20
#define IBUS_TX_PIN       21
#define MODE_SWITCH_PIN   10
#define LED_PIN           6
#define CRUISE_TX_PIN     7

// 74HC165 shift register pins
#define SR_LOAD_PIN       0
#define SR_CLOCK_PIN      1
#define SR_DATA_PIN       2

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
#define PWM_OUTPUT_MIN        51   // 20%
#define PWM_OUTPUT_MAX        153  // 60%

// Timer configuration for 1MHz tick rate (80MHz / 80 prescaler)
#define TIMER_INDEX           0
#define TIMER_PRESCALER       80
#define TIMER_INTERVAL_US     100
#define GAP_BETWEEN_MESSAGES  100

// Cruise control bit timing in timer ticks
#define BIT_HIGH_TICKS_ONE    3
#define BIT_LOW_TICKS_ONE     4
#define BIT_HIGH_TICKS_ZERO   1
#define BIT_LOW_TICKS_ZERO    6
#define STOP_BIT_TICKS        1
#define START_PULSE_TICKS     3
#define BITS_PER_BYTE         8

// Repeat interval for held buttons (milliseconds)
#define BUTTON_REPEAT_DELAY_MS  200

IbusTrx ibus;

struct ButtonMap {
  void (*handler)();
  void (*alternativeHandler)();
  bool repeats;
  bool lastState;
  unsigned long lastActionTime;
};

enum TxState {
  TX_IDLE,
  TX_START_PULSE,
  TX_DATA_BIT,
  TX_STOP_BIT,
  TX_GAP
};

// Cruise control command bytes (base values with bit 0 = 0)
// Last bit is a toggle bit that alternates every frame for ALL commands
enum ButtonCommand : uint8_t {
  CMD_KEEPALIVE = 0b11111110, // idle (no button)
  CMD_IO        = 0b11011010, // cruise on/off
  CMD_RESET     = 0b01101110, // resume/set
  CMD_PLUS      = 0b10110110, // speed +
  CMD_MINUS     = 0b11111100  // speed -
};

volatile TxState txState = TX_IDLE;
volatile ButtonCommand heldCommand = CMD_KEEPALIVE;
volatile bool keepAliveToggle = false;
volatile uint8_t currentByte = 0;
volatile uint8_t bitIndex = BITS_PER_BYTE;
volatile uint16_t tickCounter = 0;
hw_timer_t* timer = NULL; 

void scheduleFrame(ButtonCommand cmd) { 
  heldCommand = cmd;
}

void handleCruiseMinus()   { Serial.print("Cruise Minus pressed");        scheduleFrame(CMD_MINUS);   }
void handleCruisePlus()    { Serial.print("Cruise Plus pressed");         scheduleFrame(CMD_PLUS);    }
void handleCruiseIO()      { Serial.print("Cruise IO pressed");           scheduleFrame(CMD_IO);      }
void handleCruiseReset()   { Serial.print("Cruise re-set pressed");       scheduleFrame(CMD_RESET);   }
void handleVolUp()         { Serial.print("Volume + pressed");            ibus.write(MFL_VOL_UP);     }
void handleVolDown()       { Serial.print("Volume - pressed");            ibus.write(MFL_VOL_DOWN);   }
void handleNext()          { Serial.print("Next track pressed");          ibus.write(NEXT_TRACK);     }
void handlePrev()          { Serial.print("Prev track pressed");          ibus.write(PREV_TRACK);     }
void handleFlashHighbeam() { Serial.print("Flash Highbeam pressed");      ibus.write(FLASH_HIGHBEAM); }

// Button mappings (index = shift register bit), with alternative actions when mode switch is active
ButtonMap buttonMapping[] = {
  { handleCruiseMinus, handleFlashHighbeam, true,  false, 0 },
  { handleCruisePlus,  nullptr,             true,  false, 0 },
  { handleCruiseIO,    nullptr,             false, false, 0 },  // Single press only
  { handleCruiseReset, nullptr,             false, false, 0 },  // Single press only
  { handleVolUp,       nullptr,             true,  false, 0 },
  { handleVolDown,     nullptr,             true,  false, 0 },
  { handleNext,        nullptr,             false, false, 0 },
  { handlePrev,        nullptr,             false, false, 0 },
};
constexpr uint8_t NUM_BUTTONS = sizeof(buttonMapping) / sizeof(buttonMapping[0]);

void IRAM_ATTR onTimer();
void initTimer();
void setBrightness(uint8_t linValue);
uint8_t readShiftRegister();

// Read 8 button states from 74HC165 shift register
uint8_t readShiftRegister() {
  digitalWrite(SR_LOAD_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SR_LOAD_PIN, HIGH);

  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    data |= digitalRead(SR_DATA_PIN) ? 1 : 0;
    digitalWrite(SR_CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(SR_CLOCK_PIN, LOW);
  }
  return data;
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  Serial1.begin(IBUS_BAUD_RATE, SERIAL_8E1, IBUS_RX_PIN, IBUS_TX_PIN);
  ibus.begin(Serial1);

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // 74HC165 shift register pins
  pinMode(SR_LOAD_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, INPUT);
  digitalWrite(SR_LOAD_PIN, HIGH);
  digitalWrite(SR_CLOCK_PIN, LOW);

  pinMode(CRUISE_TX_PIN, OUTPUT);
  digitalWrite(CRUISE_TX_PIN, LOW);  // Idle state (output HIGH)
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

  uint8_t pwm;
  if (linValue == 0) {
    pwm = 0;  // Turn off completely when lights are off
  } else {
    pwm = map(linValue, BRIGHTNESS_MIN, BRIGHTNESS_MAX, PWM_OUTPUT_MIN, PWM_OUTPUT_MAX);
    pwm = gammaCorrect(pwm);
  }

  ledcWrite(PWM_CHANNEL, pwm);
}

void checkButtons() {
  unsigned long now = millis();
  uint8_t buttonStates = readShiftRegister();
  bool modeSwitchActive = (digitalRead(MODE_SWITCH_PIN) == LOW);

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    ButtonMap &btn = buttonMapping[i];
    bool pressed = !(buttonStates & (1 << i));  // Active low
    bool shouldFire = false;

    if (pressed && !btn.lastState) {
      shouldFire = true;
    } else if (pressed && btn.repeats && (now - btn.lastActionTime >= BUTTON_REPEAT_DELAY_MS)) {
      shouldFire = true;
    }

    if (shouldFire) {
      if (btn.alternativeHandler && modeSwitchActive) {
        btn.alternativeHandler();
      } else if (btn.handler) {
        btn.handler();
      }
      Serial.println();
      btn.lastActionTime = now;
    }

    // Reset to keepalive on button release
    if (!pressed && btn.lastState) {
      heldCommand = CMD_KEEPALIVE;
    }

    btn.lastState = pressed;
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

  checkButtons();
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
      currentByte = heldCommand | keepAliveToggle;
      keepAliveToggle = !keepAliveToggle;
      bitIndex = BITS_PER_BYTE;
      tickCounter = 0;
      digitalWrite(CRUISE_TX_PIN, HIGH);  // Start pulse (output LOW) to mark message start
      txState = TX_START_PULSE;
      break;

    case TX_START_PULSE:
      if (tickCounter >= START_PULSE_TICKS) {
        tickCounter = 0;
        txState = TX_DATA_BIT;
      }
      break;

    case TX_DATA_BIT: {
      bool bit = (currentByte >> (bitIndex - 1)) & 1;
      uint8_t highTicks = bit ? BIT_HIGH_TICKS_ONE : BIT_HIGH_TICKS_ZERO;
      uint8_t totalTicks = highTicks + (bit ? BIT_LOW_TICKS_ONE : BIT_LOW_TICKS_ZERO);

      digitalWrite(CRUISE_TX_PIN, tickCounter <= highTicks ? LOW : HIGH);

      if (tickCounter >= totalTicks) {
        tickCounter = 0;
        if (--bitIndex == 0) {
          digitalWrite(CRUISE_TX_PIN, HIGH);
          txState = TX_STOP_BIT;
        }
      }
      break;
    }

    case TX_STOP_BIT:
      if (tickCounter >= STOP_BIT_TICKS) {
        digitalWrite(CRUISE_TX_PIN, LOW);  // Idle state (output HIGH)
        tickCounter = 0;
        txState = TX_GAP;
      }
      break;

    case TX_GAP:
      if (tickCounter >= GAP_BETWEEN_MESSAGES) {
        tickCounter = 0;
        txState = TX_IDLE;
      }
      break;
  }
}
