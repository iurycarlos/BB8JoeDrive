  // ===================================================================================================================================================================================================== 
  //                         Joe's Drive  - Main drive mechanism - Updated 5/2/18
  //
  //             ***         You are free to use, and share this code as long as it is not sold. There is no warranty, guarantee, or other tomfoolery. 
  //                         This entire project was masterminded by an average Joe, your mileage may vary. 
  // ===================================================================================================================================================================================================== 
  //                            Written by Joe Latiola - https://www.facebook.com/groups/JoesDrive/
  //
  //                            You will need libraries: EepromEX: https://github.com/thijse/Arduino-EEPROMEx
  //                                                     PIDLibrary: http://playground.arduino.cc/Code/PIDLibrary  
  //                                                     EasyTransfer: https://github.com/madsci1016/Arduino-EasyTransfer
  //                                                     VarSpeedServo: https://github.com/netlabtoolkit/VarSpeedServo
  //                                                     Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69  
  //                                                     
  // 
  // ===================================================================================================================================================================================================== 
  // =====================================================================================================================================================================================================
  // Update these as necessary to match your setup
  
  //*****************These only matter if you're NOT using Serial3 for sounds
  #define soundpin1 26          // Connected to sound pin 0
  #define soundpin2 28          // Connected to sound pin 1
  #define soundpin3 30          // Connected to sound pin 2
  #define soundpin4 32          // Connected to sound pin 3
  #define soundpin5 46          // Connected to sound pin 4
  #define soundpin6 44          // Connected to sound pin 5
  //#################################################################

  //*****************These only matter if you're using Serial3 for sounds
  #define numberOfVoice   50        // This is the number of 'voice' sound files NOT INCLUDING Music and such
  #define numberOfMusic   6         // This is the number of 'music' files
      // Below are used for the multipress button sounds. Pressing button 1 on the left or button 3 on the right once plays a speach track at random, pressing 2-6 times will play quickVoice1-5. 
  #define quickVoice1     6
  #define quickVoice2     8
  #define quickVoice3     20
  #define quickVoice4     22
  #define quickVoice5     1
      // Below are used for the multipress button sounds. Pressing button 2 on the left once plays a sound at random, pressing 2-6 times will play quickMusic1-5. 
  #define quickMusic1     33
  #define quickMusic2     34
  #define quickMusic3     35
  #define quickMusic4     36
  #define quickMusic5     38
  #define SFX_RST 37
  //#################################################################
  
  //************************ Dome Tilt MK2 Settings: *****************
  #define domeTiltPotPin A1     // Connected to Potentiometer on the dome tilt mast
  #define easeDomeTilt 1.2      // Lower number means more easing when moving forward and back a.k.a. slower
  #define domeTiltPWM1 5        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define domeTiltPWM2 6        // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define MaxDomeTiltAngle 17   // Maximum angle in which the dome will tilt. **  Max is 25  


  //#################################################################

  //************************ Dome Tilt MK3 Settings: *****************
  #define leftDomeTiltServo  4  //Signal pin for the left dome tilt servo 
  #define rightDomeTiltServo 5  //Signal pin for the right dome tilt servo
  #define MaxDomeTiltY  10      // Maximum angle to tilt the dome in the Y axis ** - MAX IS 20
  #define MaxDomeTiltX  12      // Maximum angle to tilt the dome in the X axis ** - MAX IS 18
  #define DomeYEase .4          // Spead of front to back dome movement, higher == faster
  #define DomeXEase .7          // Speed of side to side domemovement, higher == faster
  #define domeSpeed 60          // Speed that the servos will move
  

  //#################################################################

  
  #define domeServoModeAngle 85 // Angle in which dome will look left/right when in servo mode 
  #define S2SenablePin 29       // Pin that provides power to motor driver enable pins
  #define enablePin 31          // Pin that provides power to motor driver enable pins
  #define enablePinDome 33      // Pin that provides power to Dome motor driver enable pin
  #define S2SpotPin A0          // Pin connected to side tilt potentiometer 
  #define ACTpin 34            // Pin connected to ACT on soundboard
  #define fadePin A2            // Connected to + of one channel on sound board(use resistor to ground)
  #define easeDomeUp 23         // Lower number means more easing when spinning
  #define easeDomeDown 35       // Lower number means more easing when spinning
  #define domeSpinPot A4        // Pin used to monitor dome spin potentiometer
  #define battMonitor A3        // Pin used to monitor battery voltage
  #define outputVoltage 5.2     // This is the output voltage from the Buck Converter powering the arduino
  #define drivePWM1 12          // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define drivePWM2 13          // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define s2sPWM1 6             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define s2sPWM2 7             // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define maxS2STilt 20         // max tilt using the joystick; max is 25
  #define domeSpinPWM1 10       // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define domeSpinPWM2 11       // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed
  #define flywheelSpinPWM1 8    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed 
  #define flywheelSpinPWM2 9    // PWM Pin for movement, swap the pin numbers on this axis if axis is reversed 
  #define resistor1 151000      // Larger resisitor used on voltage divider to read battery level
  #define resistor2 82000       // Smaller resisitor used on voltage divider to read battery level
  #define flywheelEase 3        // Speed in which flywheel will increase/decrease during gradual movements
  #define S2SEase 2.5           // speed in which side to side moves. Higher number equates to faster movement
  #define resetPin 40
  #define drivespeed1 50        // Speed of "Slow"
  #define drivespeed2 70        // Speed of "Med"
  #define drivespeed3 110       // Speed of "Fast" DO NOT EXCEED 110
  #define dataDelay   0
  #define recDelay    10
  #define sendDelay   40


//**************************** Choose the options that match your setup ****************************
  //#define MK2_Dome
  #define MK3_Dome

  #define SerialSound           // ONLY ENABLE IF YOU HAVE THE SOUNDBOARD WIRED TO SERIAL3
  
  //#define disablePSIflash       // Uncomment to remove the PSI flash.    
  
  //#define TiltDomeForwardWhenDriving      // uncomment this if you want to tilt the dome forward when driving. 



//**************************** Use these to correct any reversals ****************************

  //Controller settings:
      //#define reverseDrive          // uncomment if your drive joystick up/down is reversed
      //#define reverseDomeTilt       // uncomment if your dome tilt joystick is reversed
      //#define reverseS2S            // uncomment if your side to side joystick is reversed
      #define reverseDomeSpin         // uncomment if your dome spin joystick is reversed
      #define reverseDomeSpinServo    // uncomment if your dome spin joystick is reversed in servo mode
      //#define reverseFlywheel       // uncomment if your flywheel joystick is reversed

  //IMU settings:
      //#define reversePitch          // reverse Pitch. Test this by moving the drive by hand; the weight/stabilization should move WITH you, NOT AGAINST. This means the weight should always stay down. 
      //#define reverseRoll           // reverse Roll. Test this by moving the drive by hand; the weight/stabilization should move WITH you, NOT AGAINST. This means the weight should always stay down.

  //Potentiometer settings:
      #define reverseDomeTiltPot      // uncomment to reverse this pot. MK2 ONLY This is needed if the dome tilt arm oscilates back/forth
      //#define reverseDomeSpinPot    // uncomment to reverse dome spin pot. If the dome spin acts strange when in servo mode, this is likely the culprit. 
      #define reverseS2SPot           // uncomment to reverse the side tilt pot. If your side to side is going apesh*t, try this one. 

  //PWM settings:
      // If your drive takes off by itself as soon as you enable it, try reversing the Drive PWM pins. 
      // If the side tilt goes all the way to one side as soon as you enable it, try reversing the S2S PWM pins. 
      // If the dome tilts all the way forward/backward when you enable the drive, you guessed it, try reversing the Dome Tilt PWM pins. 
  



//**************************** Debug  ****************************
  
           //#define printRemote              // Uncomment to see values passed from controller
           //#define debugS2S                 // Uncomment to see Side tilt variables, PID values, ETC.
           //#define debugDrive               // Uncomment to see main drive variables, PID values, ETC.
           //#define debugDomeTilt            // Uncomment to see Dome tilt variables, PID values, ETC.
           //#define debugdomeRotation        // Uncomment to see Dome rotation variables, PID values, ETC.
           //#define debugPSI                 // Uncomment to see PSI values.
           //#define printbodyBatt            // Uncomment to see battery level 
           //#define printYPR                 // Uncomment to see Yaw, Pitch, and Roll
           //#define printDome                // Uncomment to see the Dome's Yaw
           //#define printOffsets             // Uncomment to see the offsets
           //#define printOutputs
           //#define printSoundPins
           //#define debugFlywheelSpin
  
  
  
  
  
  // =====================================================================================================================================================================================================
  // =====================================================================================================================================================================================================
  // 
  
  
  #include <EEPROMex.h>   
  #include "Arduino.h"

  #ifdef SerialSound
  #include "Adafruit_Soundboard.h"  
  Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);
  #endif
  
  #include <SPI.h>
  #include <PID_v1.h>

  #ifdef SerialSound

  struct RECEIVE_DATA_STRUCTURE_REMOTE{
        int Joy1Y=256; //main drive
        int Joy1X=256; //tilt / steer
        int Joy2Y=256; //head tilt
        int Joy2X=256; //head spin
        int Joy3X=256; //spin Flywheel
        int Joy4X=256;
        byte lJoySelect; //Select on left joystick
        byte lBut1; //left 1
        byte lBut2; //left 2
        byte lBut3; //left3
        byte Fwd; //Select on right joystick = rJoySelect
        byte Speed;
        byte rBut2; //right 2
        byte rBut3; //right 3
        byte motorEnable=1; //toggle on top
        byte CalibID; 
        byte wireless=1;    
  }recFromRemote; 

  #else
  
  struct RECEIVE_DATA_STRUCTURE_REMOTE{
        int Joy1Y=256; //main drive
        int Joy1X=256; //tilt / steer
        int Joy2Y=256; //head tilt
        int Joy2X=256; //head spin
        int Joy3X=256; //spin Flywheel
        int Joy4X=256;
        byte lJoySelect=1; //Select on left joystick
        byte lBut1=1; //left 1
        byte lBut2=1; //left 2
        byte lBut3=1; //left3
        byte Fwd; //Select on right joystick = rJoySelect
        byte Speed;
        byte rBut2=1; //right 2
        byte rBut3=1; //right 3
        byte motorEnable=1; //toggle on top
        byte CalibID; 
        byte wireless;    
  }recFromRemote; 

  #endif

  
  //remote to body
  
  struct SEND_DATA_STRUCTURE_REMOTE{
        int PSI;
        byte lBut3;
        float bodyBatt;
        
  }sendTo;


  // EasyTransfer for IMU Setup =========================
  
  #include <EasyTransfer.h>
  EasyTransfer RecIMU; 
  EasyTransfer RecRemote;
  EasyTransfer SendBody;
  
  struct RECEIVE_DATA_STRUCTURE_IMU{
    float IMUloop;
    float pitch;
    float roll;
  }recIMUData;

  // End EasyTransfer Setup =========================


  #ifdef MK3_Dome
  #include <VarSpeedServo.h>
  VarSpeedServo leftServo;
  VarSpeedServo rightServo;
  int Joy2Ya, Joy2XLowOffset, Joy2XHighOffset, Joy2XLowOffsetA, Joy2XHighOffsetA, ServoLeft, ServoRight;
  double Joy2X, Joy2Y, LeftJoy2X, LeftJoy2Y, Joy2XEase, Joy2YEase,  Joy2XEaseMap;

  #endif 
  double Joy2YEaseMap;

  bool SendData;
  byte RightBut1Sound, RightBut2Sound;
  byte readPinState = 1; 
  byte playSound, soundState;
  byte randSoundPin;
  int soundPins[] = {soundpin1, soundpin2, soundpin3, soundpin4};
    
  byte bodyStatus = 0;
  byte autoDisableDoubleCheck, autoDisable, autoDisableState;
  byte IMUStatus, DataStatus, ControllerStatus;
  byte lastCalibID, sendID = 1;
  byte psiState, quitState;
  byte lastSpeed;
  
  int musicState, voiceNum, musicNum;
  int flywheelRotation;
  int ch4Servo;           
  int currentDomeSpeed;
  int domeRotation;
  int joystickS2S;
  int joystickDrive;
  int ch5PWM;
  int lastDirection;
  int speedDomeTilt = 0;
  int domeTiltPot;
  int domeSpinOffset;
  int S2Spot;
  int joystickDome;
  int driveSpeed = 55;
  int driveAccel;
  int potOffsetS2S;
  int domeTiltPotOffset;

  
  // the speedArray is used to create an S curve for the 'throttle' of bb8
  int speedArray[] = {0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,5,5,5,5,6,6,7,7,8,8,9,
  9,10,10,11,12,12,13,13,14,15,16,16,17,18,19,19,20,21,22,23,24,25,26,26,27,28,29,30,
  31,32,33,33,34,35,36,37,37,38,39,40,40,41,42,42,43,44,44,45,45,46,46,47,47,48,48,49,
  49,50,50,50,51,51,51,52,52,52,52,53,53,53,53,54,54,54,54,54,55,55,55,55,55};
  
  
  double Joy2YPitch, Joy2YDirection, Joy2XDirection;
  double domeTiltOffset;
  
  float pitch, roll;
  float pitchOffset, rollOffset;
  float R1 = resistor1; 
  float R2 = resistor2;
  float countdown;
  unsigned long lastSendMillis; //, rBut2Millis;
  
  unsigned long soundMillis; 
  unsigned long autoDisableMotorsMillis, autoDisableDoubleCheckMillis;
  unsigned long lastLoopMillis;
  unsigned long IMUMillis, lastRecMillis, moveDataMillis, lastReceivedMillis;
  unsigned long lastBatteryUpdate;
  
  

  
  
  //PID1 is for the side to side tilt
  double Pk1 = 14;  
  double Ik1 = 0;
  double Dk1 = 0.0;
  double Setpoint1, Input1, Output1, Output1a;    
  
  PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);   
   
  //PID2 is for side to side stability
  double Pk2 = .5; 
  double Ik2 = 0;
  double Dk2 = .01;
  double Setpoint2, Input2, Output2, Output2a;    
  
  PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - S2S stability   
  
  //PID3 is for the main drive
  double Pk3 = 5; 
  double Ik3 = 0;
  double Dk3 = 0;
  double Setpoint3, Input3, Output3, Output3a;    
  
  PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // Main drive motor
  
  //PID4 is for dome tilt fwd/back
  double Pk4 = 6;  
  double Ik4 = 0;
  double Dk4 = 0.05;
  double Setpoint4, Input4, Output4, Output4a;    
  
  PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);   
  
  double Setpoint5a;
  
  //PID5 is for the dome spin servo
  double Kp5=4, Ki5=0, Kd5=0;
  double Setpoint5, Input5, Output5, Output5a;
  
  PID PID5(&Input5, &Output5, &Setpoint5, Kp5, Ki5, Kd5, DIRECT);     
  

  

  
  
  // ================================================================
  // ===                      INITIAL SETUP                       ===
  // ================================================================
  
  void setup() {
  
      Serial.begin(115200);
      Serial1.begin(57600);
      Serial2.begin(115200);
      Serial3.begin(9600);
      
  
      #ifdef MK3_Dome
      leftServo.attach(leftDomeTiltServo);
      rightServo.attach(rightDomeTiltServo);
      leftServo.write(95,50, false); 
      rightServo.write(90, 50, false);
      #endif
      
      RecIMU.begin(details(recIMUData), &Serial2);
      RecRemote.begin(details(recFromRemote), &Serial1);
      SendBody.begin(details(sendTo), &Serial1);
  
      pinMode(enablePin, OUTPUT);  // enable pin
      pinMode(S2SenablePin, OUTPUT); //enable pin for S2S
      pinMode(enablePinDome, OUTPUT);  // enable pin for dome spin
      pinMode(13, OUTPUT);
      digitalWrite(13, LOW);
      pinMode(ACTpin, INPUT_PULLUP); // read stat of Act on Soundboard
      

      #ifdef serialSound
      digitalWrite(reset_pin, LOW);
      pinMode(reset_pin, OUTPUT);
      delay(10);
      pinMode(reset_pin, INPUT);
      delay(1000);
      #else
      pinMode(soundpin1, OUTPUT); // play sound from pin 0 on Soundboard
      pinMode(soundpin2, OUTPUT); // play sound from pin 1 on Soundboard
      pinMode(soundpin3, OUTPUT); // play sound from pin 2 on Soundboard
      pinMode(soundpin4, OUTPUT); // play sound from pin 3 on Soundboard
      pinMode(soundpin5, OUTPUT); // play sound from pin 4 on Soundboard
      pinMode(soundpin6, OUTPUT); // play sound from pin 4 on Soundboard
      digitalWrite(soundpin6, HIGH);
      digitalWrite(soundpin5, HIGH);
      digitalWrite(soundpin4, HIGH);
      digitalWrite(soundpin3, HIGH);
      digitalWrite(soundpin2, HIGH);
      digitalWrite(soundpin1, HIGH);
      #endif
      
      // *********** PID setup ***********
  
      PID1.SetMode(AUTOMATIC);              // PID Setup -  S2S SERVO     
      PID1.SetOutputLimits(-255, 255);
      PID1.SetSampleTime(15);
  
      PID2.SetMode(AUTOMATIC);              // PID Setup -  S2S Stability
      PID2.SetOutputLimits(-255, 255);
      PID2.SetSampleTime(15);
  
      PID3.SetMode(AUTOMATIC);              // PID Setup - main drive motor
      PID3.SetOutputLimits(-255, 255);
      PID3.SetSampleTime(15);
  
      PID4.SetMode(AUTOMATIC);              // PID Setup - dome tilt
      PID4.SetOutputLimits(-255, 255);
      PID4.SetSampleTime(15);
  
      PID5.SetMode(AUTOMATIC);
      PID5.SetOutputLimits(-255, 255);      // PID Setup - dome spin 'servo'
      PID5.SetSampleTime(15);
  
      // *********  Read offsets from EEPROM  **********
  
      pitchOffset = EEPROM.readFloat(0);
      rollOffset = EEPROM.readFloat(4);
      potOffsetS2S = EEPROM.readInt(8);
      domeTiltPotOffset = EEPROM.readInt(10);
      domeSpinOffset = EEPROM.readInt(12);
  
      if( isnan(pitchOffset)){
        pitchOffset = 0; 
      }
      if( isnan(rollOffset)){
        rollOffset = 0; 
      }    
      if( isnan(potOffsetS2S)){
        potOffsetS2S = 0; 
      }    
      if( isnan(domeTiltPotOffset)){
        domeTiltPotOffset = 0; 
      }    
      if( isnan(domeSpinOffset)){
        domeSpinOffset = 0; 
      }
      
    readVin();
  }
  
   //     =======================================================================================================================================================================================
   //     =======================================================================================================================================================================================
   //     =====                                                                                LOOP, bruh!                                                                                  =====
   //     =======================================================================================================================================================================================
   //     =======================================================================================================================================================================================
   
   void loop() {
    
     receiveRemote();
     
     if(millis() - lastRecMillis >= 10){
        lastRecMillis = millis();
        receiveIMUData();
        
     }
      
     if (millis() - lastLoopMillis >= 20){
        lastLoopMillis = millis();
        movement(); 
        readVin();
        setDriveSpeed();
        sounds();
        
     }

     sendDriveData();
  }
  


