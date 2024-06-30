// Ardumower Sunray 
//MrTree: Some things can be deleted here... cause its of no use....

#include "config.h"
#include "rcmodel.h"
#include "robot.h"
#include "motor.h"
//#include "PWM.hpp"
//#include <RunningMedian.h>

//RunningMedian CHAN_1x_med = RunningMedian(3);
//RunningMedian CHAN_2x_med = RunningMedian(3);
//RunningMedian CHAN_3x_med = RunningMedian(5);

//PWM ch3(pinRemoteMow);


bool bMow = false;
//int mowRPM_RC = 0;

float CHAN_1;             //Kanal 1 -255 .. 255 (Linear)
float CHAN_2;             //Kanal 2 -255 .. 255 (Angular)
float CHAN_3;             //Kanal 3 -255 .. 255 (Workaround kleiner -150 = Mähwerk aus/ größer 150 = Mähwerk an, bMow)
float CHAN_1x;            //Kanal 1 als Real für SetLinearAngular funktion (Linear)
float CHAN_2x;            //Kanal 2 als Real für SetLinearAngular funktion (Angular)
float CHAN_3x;            //Kanal 3 als Real für SetLinearAngular funktion (nicht verwendet da keine Mähmotorübergabe in Funktion)
  

int PWMLeft;
int PWMRight;

volatile unsigned long PPM_start_linear = 0;
volatile unsigned long PPM_end_linear = 0;                
volatile unsigned long PPM_start_angular = 0;
volatile unsigned long PPM_end_angular = 0 ;        
volatile unsigned long PPM_start_mow = 0;
volatile unsigned long PPM_end_mow = 0 ;      

void get_lin_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteMow)==HIGH) PPM_start_linear = micros();  
  else                                   PPM_end_linear = micros();    
}

void get_ang_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteSteer)==HIGH) PPM_start_angular = micros();  
  else                                   PPM_end_angular = micros();  
}

void get_mow_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinLift)==HIGH) PPM_start_mow = micros();  
  else                                   PPM_end_mow = micros();  
}



void RCModel::begin()
{                                   
  RC_Mode = false; 
  nextControlTime = 0;
  // R/C
  pinMode(pinRemoteMow, INPUT);
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinLift, INPUT);  
  pinMode(pinRemoteSwitch, OUTPUT); //Relaisboard IN2 for Powerup RC Receiver, K2 switching Receiver Ground On/Off
  digitalWrite(pinRemoteSwitch, HIGH); //RC Receiver ausschalten und auf RC_Mode warten
  //ch3.begin(true);
  
#ifdef RC_DEBUG
  nextOutputTime = millis() + 1000;
#endif
  if (RCMODEL_ENABLE)
  {
    //attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);// Interrupt aktivieren
    //attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);// Interrupt aktivieren 
  }
} 

void RCModel::run()
{
  unsigned long t = millis();
  if (!RCMODEL_ENABLE) return;
  if (t < nextControlTime) return;
  nextControlTime = t + 50;                                                           // save CPU resources by running at 20 Hz
  
  if (stateButton == 3){                                                              // 3 button beeps
      stateButton = 0;                                                                // reset button state
      RC_Mode = !RC_Mode;                                                             // R/C-Mode toggle
      if (RC_Mode)  {                                                                 // R/C-Mode ist aktiv
        CONSOLE.println("button mode 3 - RC Mode ON");
        buzzer.sound(SND_ERROR, true);                                                // 3x Piep für R/C aktiv        
        digitalWrite(pinRemoteSwitch, LOW);                                         // RC Receiver Powerup over Relaisboard K2
        attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);  // Interrupt aktivieren
        attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);  // Interrupt aktivieren
        attachInterrupt(digitalPinToInterrupt(pinLift), get_mow_PPM, CHANGE);         // Interrupt aktivieren
      }
      if (!RC_Mode) {                 
        CONSOLE.println("button mode 3 - RC Mode OFF");                               // R/C-Mode inaktiv
        buzzer.sound(SND_WARNING, true);                                              // 2x Piiiiiiiep für R/C aus
        motor.setLinearAngularSpeed(0, 0);
        motor.setMowState(false);
        motor.mowRPM_RC = 0;
        motor.mowPWM_RC = 0;
        digitalWrite(pinRemoteSwitch, HIGH);                                        // RC Receiver Powerdown over Relaisboard K2
        detachInterrupt(digitalPinToInterrupt(pinRemoteMow));                       // Interrupt deaktivieren
        detachInterrupt(digitalPinToInterrupt(pinRemoteSteer));                       // Interrupt deaktivieren
        detachInterrupt(digitalPinToInterrupt(pinLift));                              // Interrupt deaktivieren

      }    
  }
  
  if (RC_Mode)    
  { 
    battery.resetIdle();                                                        // dont turn mower off in rc mode cause of idletime... 
    if (stateButton == 2)                                                       // 2 button beeps
    {                                                                           // Workaround um Mähwerk im RC Modus zu aktivieren
      bMow = !bMow;
      stateButton = 0;  
    }
         
    //lin_PPM = 0;
    if (PPM_start_linear < PPM_end_linear) lin_PPM = PPM_end_linear - PPM_start_linear; 
    if (lin_PPM < 2001 && lin_PPM > 999)                                        // Wert innerhalb 1100 bis 1900µsec
    { 
      CHAN_1x = (lin_PPM - 1500) / 1500;                                        // PPM auf kompletten Bereich....
      CHAN_1 = (lin_PPM - 1500) / 2;                                            // halbieren...
      
      //CHAN_1x_med.add(CHAN_1x);                                                 
      //CHAN_1x = CHAN_1x_med.getMedian();
   
      if ((CHAN_1 > -10) && (CHAN_1 < 10)) CHAN_1 = 0;
      if ((CHAN_1x < 0.02) && (CHAN_1x > -0.02)) CHAN_1x = 0;                   // NullLage vergrössern         
      linearPPM = CHAN_1x*2;                                                    // Weitergabe
    }

    //ang_PPM = 0;
    if (PPM_start_angular < PPM_end_angular) ang_PPM = PPM_end_angular - PPM_start_angular; 
    if (ang_PPM < 2001 && ang_PPM > 999)                                        // Wert innerhalb 1100 bis 1900µsec
    {   
      CHAN_2x = (ang_PPM - 1500) / 1500;                                        // PPM auf kompletten Bereich....
      CHAN_2 = (ang_PPM - 1500) / 2;                                            // halbieren...

      //CHAN_2x_med.add(CHAN_2x);
      //CHAN_2x = CHAN_2x_med.getMedian();
     
      if ((CHAN_2 > -10) && (CHAN_2 < 10)) CHAN_2 = 0; 
      if ((CHAN_2x < 0.01) && (CHAN_2x > -0.01)) CHAN_2x = 0;                   // NullLage vergrössern         
      angularPPM = CHAN_2x*8;
    }
    
    //mow_PPM = 0;
    if (PPM_start_mow < PPM_end_mow) mow_PPM = PPM_end_mow - PPM_start_mow; 
    if (mow_PPM < 2001 && mow_PPM > 999)                                      // Wert innerhalb 1100 bis 1900µsec
    { 
      CHAN_3x = (mow_PPM - 1500) / 1500;                                      // PPM auf kompletten Bereich....
      CHAN_3 = (mow_PPM - 1500) / 2;                                          // halbieren...

      //CHAN_3x_med.add(CHAN_3x);
      //CHAN_3x = CHAN_3x_med.getMedian();
      
      if ((CHAN_3 > -10) && (CHAN_3 < 10)) CHAN_3 = 0;                        // Nullage PPM
      if ((CHAN_3x < 0.05) && (CHAN_3x > -0.05)) CHAN_3x = 0;                 // NullLage vergrössern         
      mowPPM = CHAN_3x;
      if ((CHAN_3 > 50) || (CHAN_3 < -50)) 
      { bMow = true; }else{ bMow = false;}    
      mowPWM_RC = CHAN_3;
      //mowRPM_RC = (4000/255)*abs(CHAN_3);
      if (USE_MOW_RPM_SET) motor.mowRPM_RC = (4000/255)*abs(CHAN_3);
      else motor.mowPWM_RC = CHAN_3;
      //CONSOLE.println(CHAN_3);
    }
    //CONSOLE.println(ch3.getValue());
    
    //Mix Channels to PWM Out -255..255
    PWMLeft = CHAN_1 - CHAN_2;
    PWMRight = CHAN_1 + CHAN_2;
    //Safetycheck
    if (PWMLeft > 255) PWMLeft = 255;
    if (PWMRight > 255) PWMRight = 255;
    if (PWMLeft < -255) PWMLeft = -255;
    if (PWMRight < -255) PWMRight = -255;
  
#ifdef RC_DEBUG
    if (t >= nextOutputTime) 
    {
      nextOutputTime = t + 1000;
      //CONSOLE.println(PWMLeft);
      //CONSOLE.println(PWMRight);
      //CONSOLE.print("RC: linearPPM= ");
      //CONSOLE.print(linearPPM);
      //CONSOLE.print("RC: linear_PPM= ");
      //CONSOLE.print(lin_PPM);   
      //CONSOLE.print("RC: angularPPM= ");
      //CONSOLE.print(angularPPM);
      //CONSOLE.print("RC: angular_PPM= ");
      //CONSOLE.print(ang_PPM);
      //CONSOLE.print("RC: mowPPM= ");
      //CONSOLE.print(mowPPM);
      //CONSOLE.print("RC: mow_PPM= ");
      //CONSOLE.print(mow_PPM);
    }
#endif
   motor.setLinearAngularSpeed(linearPPM, angularPPM, false);                     // R/C Signale an Motor leiten
   if (bMow != motor.switchedOn) motor.setMowState(bMow);                         // bMow vom Poti als Schwellwertschalter
   //motor.speedPWM(PWMLeft, PWMRight, 0);
   //motorDriver.setMotorPwm(PWMLeft, PWMRight, 0);
  }
}
