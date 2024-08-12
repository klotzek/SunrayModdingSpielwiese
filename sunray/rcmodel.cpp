// Ardumower Sunray

// this works with a 2,4ghz rc receiver low voltage directly connected from servo/driver pwm receiver output to digital interrupt"capable" 
// input pins of ardumower pcb (not too many... rc connector has 4, but only 2 are interrupt capable, that´s why here the tilt Pin is used for mow motor)
// low voltage means that rc receiver is powered directly from the pcb 5volt distributor array without anything else
// -- > calculate and map the pwm modulated channel inputs from the receiver to according values for linear and angular speeds, 
// also switch on rc mow flag and set rpm or pwm for mow driver in motor.cpp with .h include vars (could use globals?)

#include "config.h"
#include "rcmodel.h"
#include "robot.h"
#include "motor.h"
#include <RunningMedian.h>

RunningMedian<unsigned int, 5> CHAN_1_med;
RunningMedian<unsigned int, 5> CHAN_2_med;
RunningMedian<unsigned int, 5> CHAN_3_med;

bool bMow = false;

float CHAN_1x;            //Channel 1 RC --- float for SetLinearAngular function (linear)
float CHAN_2x;            //Channel 2 RC --- float for SetLinearAngular function (angular)
int CHAN_3;               //Channel 3 RC --- mapped to +-8Bits: -255 .. 255 

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

  pinMode(pinRemoteMow, INPUT);
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinLift, INPUT);  
  pinMode(pinRemoteSwitch, OUTPUT);     //Relaisboard IN2 for Powerup RC Receiver, K2 switching Receiver Ground On/Off
  digitalWrite(pinRemoteSwitch, HIGH);  //RC Receiver ausschalten und auf RC_Mode warten
  
#ifdef RC_DEBUG
  nextOutputTime = millis() + 1000;
#endif
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
    if (PPM_start_linear < PPM_end_linear) lin_PWM = PPM_end_linear - PPM_start_linear; 
    if (lin_PWM < 2001 && lin_PWM > 999)                                        // Wert innerhalb 1100 bis 1900µsec
    { 
      CHAN_1_med.add(lin_PWM);
      CHAN_1_med.getMedian(lin_PWM);
      CHAN_1x = map(lin_PWM, 1000, 2000, -600, 600);
      CHAN_1x /= 1000.0;
      if ((CHAN_1x < 0.02) && (CHAN_1x > -0.02)) CHAN_1x = 0;                   // NullLage vergrössern
      rc_linear = CHAN_1x;                                                    // Weitergabe
    }

    //ang_PPM = 0;
    if (PPM_start_angular < PPM_end_angular) ang_PWM = PPM_end_angular - PPM_start_angular; 
    if (ang_PWM < 2001 && ang_PWM > 999)                                        // Wert innerhalb 1100 bis 1900µsec
    {
      CHAN_2_med.add(ang_PWM);
      CHAN_2_med.getMedian(ang_PWM);   
      CHAN_2x = map(ang_PWM, 1000, 2000, -PI*1000, PI*1000);
      CHAN_2x /= 1000.0;
      if ((CHAN_2x < 0.01) && (CHAN_2x > -0.01)) CHAN_2x = 0;                   // NullLage vergrössern         
      rc_angular = CHAN_2x;
    }
    
    //mowmotor
    if (PPM_start_mow < PPM_end_mow) mow_PWM = PPM_end_mow - PPM_start_mow; 
    if (mow_PWM < 2101 && mow_PWM > 899)                                      // Wert innerhalb 1100 bis 1900µsec
    { 
      CHAN_3_med.add(mow_PWM);
      CHAN_3_med.getMedian(mow_PWM);
      CHAN_3 = (mow_PWM - 1500) / 2;                                          // halbieren... -255 | +255
      if (CHAN_3 > 50 || CHAN_3 < -50) bMow = true;
        else bMow = false;    
      rc_mowPWM = CHAN_3;
      rc_mowRPM= 4000/255 * abs(CHAN_3);

      if (USE_MOW_RPM_SET) motor.mowRPM_RC = rc_mowRPM;
      else motor.mowPWM_RC = rc_mowPWM;
    }
/*#ifdef RC_DEBUG
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
#endif*/
   motor.setLinearAngularSpeed(rc_linear, rc_angular, false);                     // R/C Signale an Motor leiten
   if (bMow != motor.switchedOn) motor.setMowState(bMow);                         // bMow vom Poti als Schwellwertschalter
  }
}
