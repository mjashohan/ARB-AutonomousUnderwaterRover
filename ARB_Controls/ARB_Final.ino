#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include "hidjoystickrptparser.h"

#include <AFMotor.h>
AF_DCMotor motorL(1);
AF_DCMotor motorR(2);
AF_DCMotor motorM(3);

static int speed1=250;
static int speed2=110;
static int speed3=80;

static uint8_t xCor;
static uint8_t yCor;
static uint8_t ac;
static uint8_t ad;
static uint8_t ae;
static uint8_t af;

//buttons
static int totalBtns=10;

static bool btns [] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  /// trur/false conditions of the buttons

static bool joystickAvailable=false;

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

void setup() {
        Serial.begin(115200);
pinMode(22, OUTPUT);
pinMode(24, OUTPUT);
pinMode(26, OUTPUT);
        
#if !defined(__MIPSEL__)
        while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
        Serial.println("Start");

        if (Usb.Init() == -1)
                Serial.println("OSC did not start.");

        delay(200);

        if (!Hid.SetReportParser(0, &Joy))
                ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
        Usb.Task();

//speed controll
     
//        if(joystickAvailable && btns[7]){
//        speed1 = 70;
//        }
//        if(joystickAvailable && btns[5]){
//        speed1 = 255;
//        }

        if(joystickAvailable && btns[2]){
        speed3 = 150;
        }
        
        




        if(joystickAvailable && !btns[5]){
          
          if ( yCor==127 || yCor==128 ){ 
                      
                      
                      if ( xCor==0 ) moveLeft();
                      
          
                      else if ( xCor > 200 ) moveRight();            

                      else moveStop();
            }
          
          if ( yCor==0 ) moveForward();
          
          if ( yCor > 200 ) moveBackward();

        }
        
if(joystickAvailable && btns[1]){
 
        MiddleMotorOn();
        delay(2);
        }

    
        
if(joystickAvailable && btns[3]){
        MiddleMotorOff();
        delay(2);
        }
        
if(joystickAvailable && btns[6]){
  if(speed3 >0 && speed3<251){
        speed3 = speed3 + 5;
        Serial.print("Middle : ");
        Serial.println(speed3);
        delay(100);
        }
        }
        
if(joystickAvailable && btns[8]){
  if(speed3 >0 && speed3<251){
         speed3 = speed3 - 5;
         Serial.print("Middle : ");
        Serial.println(speed3);
        delay(100);
        }                
          
          }

if(joystickAvailable && btns[5]){
  lightOn();
  }

  if(joystickAvailable && btns[7]){
  lightOff();
  }
}


void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
       xCor = (evt->X);
       yCor = (evt->Y);
       ac = (evt->Z1);
       ad = (evt->Z2);
       ae = (evt->Rz);
       
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
        Serial.print("Hat Switch: ");
      //  PrintHex<uint8_t > (hat, 0x80);
        Serial.println("");
        Serial.println("joystickAvailable = true");

        joystickAvailable = true;
}
void JoystickEvents::OnButtonUp(uint8_t but_id) {
        Serial.print("Up: ");
        Serial.println(but_id, DEC);   
         btns[(but_id)]=0;    
        
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
        Serial.print("Dn: ");
        Serial.println(but_id, DEC);
         btns[(but_id)]=1;
}


void moveForward()
{
    motorL.setSpeed(speed1);
    motorR.setSpeed(speed1);

    
    motorL.run(FORWARD);
    motorR.run(FORWARD);
  
}

void moveBackward()
{
    motorL.setSpeed(speed1);
    motorR.setSpeed(speed1);
    
    motorL.run(BACKWARD);
    motorR.run(BACKWARD);
   
}

void moveRight()
{
    motorL.setSpeed(speed2);
    motorR.setSpeed(speed2);
    
    motorL.run(FORWARD);
    motorR.run(BACKWARD);
  
}

void moveLeft()
{
    motorL.setSpeed(speed2);
    motorR.setSpeed(speed2);
    
    motorL.run(BACKWARD);
    motorR.run(FORWARD);
    
  
}

void moveStop()
{
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    
    motorL.run(BRAKE);
    motorR.run(BRAKE);
  
}

void MiddleMotorOn()
{
    motorM.setSpeed(speed3);
    
    
    motorM.run(FORWARD);
    
  
}

void MiddleMotorOff()
{
   motorM.setSpeed(0);
    
    
    motorM.run(BRAKE);
    
  
}

void lightOn()
{
   digitalWrite(22, HIGH); 
    digitalWrite(24, HIGH); 
     digitalWrite(26, HIGH);    
}

void lightOff()
{
   digitalWrite(22, LOW); 
    digitalWrite(24, LOW); 
     digitalWrite(26, LOW);    
}
