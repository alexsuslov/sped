#include <Arduino.h>
#include <EncButton.h>
#include "GyverTM1637.h"
#include <EEPROM.h>
#include "GyverTimers.h"

# define pwmAddr 0
# define stepsAddr 1

// motor PWM
#define motorPin 7 

// Speed Encoder 
#define PIN_ENCODER_A 2 //8 
#define PIN_ENCODER_B 3 //9

// Control Encoder 
#define PIN_CONTROL_ENCODER_A 8// 2
#define PIN_CONTROL_ENCODER_B 9// 3
#define PIN_CONTROL_BTN 4

// Display 
#define CLK 6 
#define DIO 5 

GyverTM1637 disp(CLK, DIO);
volatile boolean speedDisplay=false;
byte welcome_banner[] = {_H, _E, _L, _L, _O};
byte error_banner[] = {_E, _r, _r,_o};

// volatile int mode=0; 

// Speed Control
EncButton<EB_TICK, PIN_CONTROL_ENCODER_A, PIN_CONTROL_ENCODER_B, 
  PIN_CONTROL_BTN> btn;

volatile int buttonClicks=0;

int speedControlMin=0;
int speedControlMax=255;

volatile int speedControl=0;

// Current Speed
volatile int steps=0;
volatile int lastSteps=0;
#define RATIO 100 // step/mm
volatile int stepsControl=0; //

// motor PWM
#define motorPin 7 

void risingA ( ) {
  steps += digitalRead(PIN_ENCODER_A) == digitalRead(PIN_ENCODER_B) ? -1 : 1;
}

void risingB ( ) {
  steps += digitalRead(PIN_ENCODER_A) != digitalRead(PIN_ENCODER_B) ? -1 : 1;
}

void Println(String msg, int val){
  Serial.print(msg);
  Serial.println(speedControl);
}

void SetPWM(int i){
  Println("PWM=",i);
  analogWrite(motorPin, i);
}

void setup() {
  // pins
  pinMode(motorPin, OUTPUT);
  pinMode(PIN_ENCODER_A , INPUT_PULLUP);
  pinMode(PIN_ENCODER_B , INPUT_PULLUP);

  // interaps
  attachInterrupt(0, risingA, RISING);
  attachInterrupt(1, risingB, RISING);

  //serial
  Serial.begin(9600);
  Serial.println("Starting...");

  // restore PWM
  speedControl = EEPROM.read(pwmAddr);
  Println("PWM=",speedControl);
  analogWrite(motorPin, speedControl);

  // restore Steps
  stepsControl = EEPROM.read(stepsAddr);
  Println("Steps=", stepsControl);

  // display
  disp.clear();
  disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
  disp.clear();
  disp.runningString(welcome_banner, sizeof(welcome_banner), 200);
  disp.displayInt(speedControl);
  
  // timer
  Timer1.setFrequency(1); 
  Timer1.enableISR(); 
}

void loop() {
  btn.tick();
  if (btn.click()){ 
    if(speedControl==0 ) {
      stepsControl=0;
      EEPROM.write(stepsAddr, stepsControl);
      Serial.print("reset");
    }else{
      stepsControl=lastSteps;
      EEPROM.write(stepsAddr, stepsControl);
      Println("eprom write steps control=", stepsControl);
    }
  }
  // speedControl mode
  if (stepsControl==0){  
    if (btn.right()){
      if (speedControl!=speedControlMax) {
        speedControl++;
      }
    }
    if (btn.left()) {
      if (speedControl!=speedControlMin) {
        speedControl--;   
      }
    }
    // store
    if (EEPROM.read(pwmAddr)!=speedControl){
      Println("set pwm ", speedControl);
      disp.displayInt(speedControl);
      analogWrite(motorPin, speedControl);
      EEPROM.write(pwmAddr,speedControl);
    }
  }
}

// таймер
ISR(TIMER1_A) {
  lastSteps=steps;
  if (speedControl==0){
    // setup
    Println("ratio steps counter=",lastSteps);
    disp.displayInt(steps);
  }else{
    // normal
    steps=0;
    if (stepsControl>0){
      // error
      if (lastSteps==0){
        speedControl=0;
        stepsControl=0;
        analogWrite(motorPin, 0);
        disp.displayByte(error_banner);
        Serial.println("rotation error");
        return;
      } else if (lastSteps>stepsControl){
        speedControl--;
        Println("speed down ", speedControl);
      } else if (lastSteps<stepsControl){
        speedControl++;
        Println("speed up ", speedControl);
      }
      disp.displayInt(lastSteps/RATIO); // current steps/sec
    }
  }  
}






