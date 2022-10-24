#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_INA219.h>


//CONTROLS TWO DOOR MOTORS VIA simple pins.. each click runs a motor in a set dir until overload on currnet, thens tops, saves new direction, and waits for next interupt.
//stack the shields, wire the motor through the INA shield on +side Vin- to motor and to M1 on motor shield
//4.7V + to a USB connector and to the motor shield
//4.7V - to motor shield ground and usb ground
//wire the relay to pin 2 and board ground, and pin 3 and board ground
//solder 2 and ground to proto area?


const int IDpin = 2; //feather has interrupt #0 here on ditigal pin 2, connect inside door relay
const int ODpin = 3; //feather has interrupt #1 here on ditigal pin 3, connect outside door relay
int maxcurrent = 250; //max current in milliamps--INPUT, DO NOT SET ABOVE 1200 DUE TO LIMIT OF AFMS
int motorspeed = 100;//0 is stopped, 255 is full speed--INPUT

Adafruit_INA219 ina219_IN(0x40);//inside door current monitor
Adafruit_INA219 ina219_OUT(0x41);//SOLDER THIS BOARD TO CHANGE ADDRESS--outside door current monitor
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *Idoormotor = AFMS.getMotor(1);
Adafruit_DCMotor *Odoormotor = AFMS.getMotor(2);

float IDcurrent_mA = 0;
float ODcurrent_mA = 0;
int Idirectionvar = 0;
int Odirectionvar = 0;
int IDtrigger = 0;
int ODtrigger = 0;

long ODlastDebounceTime = 0; // the last time the output pin was toggled
long ODdebounceDelay = 500;    // the debounce time; increase if the output flickers
long IDlastDebounceTime = 0; // the last time the output pin was toggled
long IDdebounceDelay = 500;    // the debounce time; increase if the output flickers



void setup() {
  Serial.begin(9600);
  uint32_t currentFrequency;
  ina219_IN.begin();
  ina219_OUT.begin();
  ina219_IN.setCalibration_16V_400mA(); //32V_1A and 32V_2A also possible_16V_400mA
  ina219_OUT.setCalibration_16V_400mA(); //32V_1A and 32V_2A also possible_16V_400mA
  AFMS.begin();
  Idoormotor->setSpeed(motorspeed);  
  Odoormotor->setSpeed(motorspeed); 
  pinMode(IDpin, INPUT_PULLUP); 
  digitalWrite(IDpin, HIGH);   
  pinMode(ODpin, INPUT_PULLUP);
  digitalWrite(ODpin, HIGH);  
}



void loop() {

if ( (millis() - IDlastDebounceTime) > IDdebounceDelay) {
if (IDtrigger == 0 && digitalRead(IDpin) == LOW){
  IDtrigger = 1; 
  IDlastDebounceTime = millis();
}

else if (IDtrigger == 1 && digitalRead(IDpin) == LOW){
  irelease();
  IDcycledir();
  IDtrigger = 0;
  IDlastDebounceTime = millis();
}
if (abs(IDcurrent_mA) >= maxcurrent){
  irelease();
  IDcycledir();
  IDtrigger = 0;
  IDlastDebounceTime = millis();
}  
}


if ( (millis() - ODlastDebounceTime) > ODdebounceDelay) {
if (ODtrigger == 0 && digitalRead(ODpin) == LOW){
  ODtrigger = 1;
  ODlastDebounceTime = millis();
 
}
else if (ODtrigger == 1 && digitalRead(ODpin) == LOW){
  orelease();
  ODcycledir();
  ODtrigger = 0;
  ODlastDebounceTime = millis();
}
if (abs(ODcurrent_mA) >= maxcurrent){
  orelease();
  ODcycledir();
  ODtrigger = 0;
  ODlastDebounceTime = millis();
}  
}



if (ODtrigger == 1 && Odirectionvar == 1 ){
  oforward();
  
}  
if (ODtrigger == 1 && Odirectionvar == 0 ){
  oreverse();

}  
if (IDtrigger == 1 && Idirectionvar == 1 ){
  iforward();
 
}  
if (IDtrigger == 1 && Idirectionvar == 0 ){
  ireverse();
  
}

IDcurrent_mA = ina219_IN.getCurrent_mA();
Serial.print("IDCurrent:       "); Serial.print(IDcurrent_mA); Serial.println(" mA");
ODcurrent_mA = ina219_OUT.getCurrent_mA();
Serial.print("ODCurrent:       "); Serial.print(ODcurrent_mA); Serial.println(" mA");
}






void IDcycledir (){
  if (Idirectionvar == 0){
    Idirectionvar = 1;
  }
else {
  Idirectionvar = 0;
}
}


void ODcycledir (){
  if (Odirectionvar == 0){
    Odirectionvar = 1;
  }
else {
  Odirectionvar = 0;
}
}


void iforward (){
  Idoormotor->run(FORWARD);
}


void ireverse (){
  Idoormotor->run(BACKWARD);
}


void irelease (){
  Idoormotor->run(RELEASE);
}


  void oforward (){
  Odoormotor->run(FORWARD);
}


void oreverse (){
  Odoormotor->run(BACKWARD);
}


void orelease (){
  Odoormotor->run(RELEASE);
}