// BMW Steering Wheel Buttons Universal Adapter
// It decodes the original signals from the buttons to on/off
// signals and steers a programable resistor for modern radio headers

//Target: Pro Micro
//
#include <MCP4151.h>
#include "IbusTrx.h"
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
  #define DEBUG_PRINTLN(x)
#endif

#define CS     10
#define MOSI   16
#define MISO   14
#define SCK    15

const int POT_VOLMINUS = 255 - 0;
const int POT_VOLPLUS = 255 - 1;
const int POT_CHNMINUS = 255 - 2;
const int POT_CHNPLUS = 255 - 3;
const int POT_PHONE = 255 - 4;
const int POT_SPMINUS = 255 - 6;
const int POT_SPPLUS = 255 - 9;
const int POT_IO = 255 - 13;
const int POT_CC = 255 - 19;
const int POT_RT = 255 - 26;
const int POT_MAX = 256 - 38;


const int INTERRUPTPIN = 2;
const int PIN_CHNPLUS = 3;
const int PIN_CHNMINUS = 4;
const int PIN_VOLPLUS = 5;
const int PIN_VOLMINUS = 6;
const int PIN_PHONE = 7;

const int PIN_CC = 8;
const int PIN_IO = 9;
const int PIN_SPPLUS = 20;
const int PIN_SPMINUS = 19;
const int PIN_RT = 18;

IbusTrx ibusTrx;
long m_EndTime;
long m_Duration;
int m_CCReceivedBuffer;
volatile int m_CCReceived;
volatile bool m_CCNewData;
bool m_SuppressRT;
bool m_WasCC;

MCP4151 pot(CS, MOSI, MISO, SCK);

void PinInterrupt(){
  delayMicroseconds(234);
  int bit = digitalRead(INTERRUPTPIN);
  delayMicroseconds(234);
  if(digitalRead(INTERRUPTPIN) == HIGH){
    m_CCReceived = m_CCReceivedBuffer>>1;
    m_CCReceivedBuffer = 0;
    m_CCNewData = true;
  }else{
    m_CCReceivedBuffer = (m_CCReceivedBuffer<<1) + bit;
  }
}

void setup() {

#ifdef DEBUG
  Serial.begin(9600);
  delay(600);
  Serial.println("Start");
#endif
  
  m_Duration = 100;
  m_EndTime = 0;
  m_CCReceived = 0;
  m_CCReceivedBuffer = 0;
  m_CCNewData = false;
  m_SuppressRT = false;
  m_WasCC = false;
  pinMode(PIN_CHNMINUS, OUTPUT);
  pinMode(PIN_CHNPLUS, OUTPUT);
  pinMode(PIN_VOLMINUS, OUTPUT);
  pinMode(PIN_VOLPLUS, OUTPUT);
  pinMode(PIN_PHONE, OUTPUT);
  pinMode(PIN_RT, OUTPUT);
  pinMode(PIN_CC, OUTPUT);
  pinMode(PIN_IO, OUTPUT);
  pinMode(PIN_SPMINUS, OUTPUT);
  pinMode(PIN_SPPLUS, OUTPUT);
  pinMode(INTERRUPTPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), PinInterrupt, RISING);
  ibusTrx.begin(Serial1);
  pot.writeValue(POT_MAX);
}

void loop() {
  if(m_EndTime != 0 && millis() > m_EndTime){
    digitalWrite(PIN_RT, LOW);
    digitalWrite(PIN_VOLPLUS, LOW);
    digitalWrite(PIN_VOLMINUS, LOW);
    pot.writeValue(POT_MAX);
    m_EndTime = 0;
    DEBUG_PRINTLN("EndTime!");
  } 

  if (m_CCNewData){
    m_CCNewData = false;
    ccEvent();
    m_CCReceived = 0;
  }
  
  if (ibusTrx.available()) radioEvent();
}

void radioEvent(){
  IbusMessage message = ibusTrx.readMessage();
  if(message.source() != 80)return;

  if(message.destination() == 176 || message.destination() == 208) m_SuppressRT = true;

  if(message.destination() != 104 && message.destination() != 200) return;

  if(message.b(0) == 1){
    if(m_SuppressRT){
      m_SuppressRT = false;
      return;
    }
    m_EndTime = millis() + m_Duration;
    pot.writeValue(POT_RT);
    digitalWrite(PIN_RT, HIGH);
    DEBUG_PRINTLN("RT");
    return;
  }

  switch(message.b(0)<<8 | message.b(1)){
    case  0x3211:
      m_EndTime = millis() + m_Duration;
      pot.writeValue(POT_VOLPLUS);
      digitalWrite(PIN_VOLPLUS, HIGH);
      DEBUG_PRINTLN("PIN_VOLPLUS_DOWN");
    break;

    case  0x3210:
      m_EndTime = millis() + m_Duration;
      pot.writeValue(POT_VOLMINUS);
      digitalWrite(PIN_VOLMINUS, HIGH);
      DEBUG_PRINTLN("PIN_VOLMINUS_DOWN");
    break;

    case  0x3B01:
      pot.writeValue(POT_CHNPLUS);
      digitalWrite(PIN_CHNPLUS, HIGH);
      DEBUG_PRINTLN("PIN_CHNPLUS_DOWN");
    break;

    case  0x3B21:
      pot.writeValue(POT_MAX);
      digitalWrite(PIN_CHNPLUS, LOW);
      DEBUG_PRINTLN("PIN_CHNPLUS_UP");
    break;

    case  0x3B08:
      pot.writeValue(POT_CHNMINUS);
      digitalWrite(PIN_CHNMINUS, HIGH);
      DEBUG_PRINTLN("PIN_CHNMINUS_DOWN");
    break;

    case  0x3B28:
      pot.writeValue(POT_MAX);
      digitalWrite(PIN_CHNMINUS, LOW);
      DEBUG_PRINTLN("PIN_CHNMINUS_UP");
    break;

    case  0x3B80:
      pot.writeValue(POT_PHONE);
      digitalWrite(PIN_PHONE, HIGH);
      DEBUG_PRINTLN("PIN_PHONE_DOWN");
    break;

    case  0x3BA0:
      pot.writeValue(POT_MAX);
      digitalWrite(PIN_PHONE, LOW);
      DEBUG_PRINTLN("PIN_PHONE_UP");
    break;
  }
}

void ccEvent(){
  switch(m_CCReceived){
    case 127:
      if(m_WasCC) pot.writeValue(POT_MAX);
      digitalWrite(PIN_CC, LOW);
      digitalWrite(PIN_IO, LOW);
      digitalWrite(PIN_SPPLUS, LOW);
      digitalWrite(PIN_SPMINUS, LOW);
      m_WasCC=false;
    break;

    case 55:
      pot.writeValue(POT_CC);
      digitalWrite(PIN_CC, HIGH);
      m_WasCC=true;
      DEBUG_PRINTLN("PIN_CC_DOWN");
    break;

    case 109:
      pot.writeValue(POT_IO);
      digitalWrite(PIN_IO, HIGH);
      DEBUG_PRINTLN("PIN_IO_DOWN");
      m_WasCC=true;
    break;

    case 91:
      pot.writeValue(POT_SPPLUS);
      digitalWrite(PIN_SPPLUS, HIGH);
      DEBUG_PRINTLN("PIN_SPPLUS_DOWN");
      m_WasCC=true;
    break;

    case 126:
      pot.writeValue(POT_SPMINUS);
      digitalWrite(PIN_SPMINUS, HIGH);
      DEBUG_PRINTLN("PIN_SPMINUS_DOWN");
      m_WasCC=true;
    break;
  }
}