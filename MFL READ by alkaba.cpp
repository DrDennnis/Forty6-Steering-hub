// by discord, bmw, @alkaba
// i can use this to read MFL signals from E46 steering wheel and convert them to CAN bus messages for standalone ecus

// search this in discord
// ally had time to make e46 MFL conversion to CAN bus switches. When you want to use steering wheel switches with standalone ecu (emu Black etc) and have a working Cruise control or rolling antilag

#include <SPI.h>

const int interruptPin = 2;

boolean m_Possibles[12];
byte Events[12][11] = {{0, 0x86, 0x1E, 0x80, 0xF8, 0x06, 0x98, 0x98, 0xff, 0xff, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x18, 0x78, 0xE0, 0xF8, 0x80, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x18, 0x78, 0x60, 0xFE, 0xE0, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x80, 0xFE, 0x00, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x80, 0xFE, 0xE0, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x06, 0xFE, 0x86, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x00, 0x78, 0xff, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x60, 0x78, 0xE0, 0xff},
                       {0, 0x86, 0x60, 0x80, 0x9E, 0x9E, 0x78, 0x80, 0x60, 0x18, 0xff},
                       {0, 0x86, 0x60, 0x80, 0xF8, 0x9E, 0x78, 0x7E, 0xE0, 0xff, 0xff},
                       {0, 0x86, 0x60, 0x80, 0xF8, 0x9E, 0x78, 0x98, 0x7E, 0xF8, 0xff},
                       {0, 0x86, 0x60, 0x80, 0xF8, 0x9E, 0x78, 0x66, 0x00, 0xff, 0xff}};
int m_Pointer = 0;
long m_EndTime;
long m_Duration = 100;
int INTERRUPTPIN;
int m_CCReceivedBuffer;
volatile int m_CCReceived;
volatile bool m_CCNewData;

void PinInterrupt()
{
  delayMicroseconds(234);
  int bit = digitalRead(interruptPin);
  delayMicroseconds(234);
  if (digitalRead(interruptPin) == HIGH)
  {
    m_CCReceived = m_CCReceivedBuffer >> 1;
    m_CCReceivedBuffer = 0;
    m_CCNewData = true;
  }
  else
  {
    m_CCReceivedBuffer = (m_CCReceivedBuffer << 1) + bit;
  }
}
void setup()
{
  Serial.begin(115200);
  SPI.begin();

  delay(50);
  Serial.println("DEBUG START");
  for (int i = 0; i < 12; i++)
    m_Possibles[i] = true;
  m_Pointer = 0;
  m_EndTime = 0;
  m_CCReceived = 0;
  m_CCReceivedBuffer = 0;
  m_CCNewData = false;

  pinMode(interruptPin, INPUT);
  INTERRUPTPIN = digitalPinToInterrupt(interruptPin);
  attachInterrupt(INTERRUPTPIN, PinInterrupt, RISING);
}
void loop()
{
  readCC(); // function for reading CC signals from MFL
  // serialprint(); //DEBUG to see which button is pressed on serial output
}

void readCC()
{
  m_CCNewData = false;

  // i dont need this, but describes how we can use the data to reverse engineer the signal
  switch (m_CCReceived)
  {
  case 127:
    digitalWrite(PIN_CC, LOW);
    digitalWrite(PIN_IO, LOW);
    digitalWrite(PIN_SPUP, LOW);
    digitalWrite(PIN_SPDN, LOW);
    break;
  case 55:
    digitalWrite(PIN_CC, HIGH);
    break;
  case 109:
    digitalWrite(PIN_IO, HIGH);
    break;
  case 91:
    digitalWrite(PIN_SPUP, HIGH);
    break;
  case 126:
    digitalWrite(PIN_SPDN, HIGH);
    break;
  }
  m_CCReceived = 0;
}