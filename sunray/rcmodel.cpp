// Ardumower Sunray 


#include "config.h"
#include "rcmodel.h"
#include "robot.h"


volatile unsigned long PPM_start_lin = 0;
volatile unsigned long PPM_end_lin = 0;                
volatile unsigned long PPM_start_ang = 0;
volatile unsigned long PPM_end_ang = 0 ;        
// +----------
volatile unsigned long PPM_start_mow = 0;
volatile unsigned long PPM_end_mow = 0 ;      
// -----------

void get_lin_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteMow)==HIGH) PPM_start_lin = micros();  
  else                                   PPM_end_lin = micros();    
}

void get_ang_PPM()                                                        // Interrupt Service Routine
{
  if (digitalRead(pinRemoteSteer)==HIGH) PPM_start_ang = micros();  
  else                                   PPM_end_ang = micros();  
}

// +-----------------
void get_mow_PPM() // Interrupt Service Routine
{
if (digitalRead(pinTilt)==HIGH) PPM_start_mow = micros();
else PPM_end_mow = micros();
}
// -----------------

void RCModel::begin(){  
// +------------
  float mow_PPM = 0;
  float mowPPM = 0;
  bool bMow = false;
// -------------
  lin_PPM = 0;                                            
  linearPPM = 0;                                         
  ang_PPM = 0;                                            
  angularPPM = 0;                                         
  RC_Mode = false; 
  nextControlTime = 0;
  // R/C
  pinMode(pinRemoteSteer, INPUT_PULLUP);
  pinMode(pinRemoteMow, INPUT_PULLUP);
// +----------------
  pinMode(pinTilt, INPUT_PULLUP);
// -----------------

#ifdef RC_DEBUG
  nextOutputTime = millis() + 1000;
#endif
  if (RCMODEL_ENABLE){
    //attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);// Interrupt aktivieren
    //attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);// Interrupt aktivieren 
  }
} 

void RCModel::run(){
  unsigned long t = millis();
  if (!RCMODEL_ENABLE) return;
  if (t < nextControlTime) return;
  nextControlTime = t + 50;                                       // save CPU resources by running at 20 Hz
  // nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  
  if (stateButton == 3){                                           // 3 button beeps
      stateButton = 0;                                             // reset button state
      RC_Mode = !RC_Mode;                                                   // R/C-Mode toggle
      if (RC_Mode)  {                                                       // R/C-Mode ist aktiv
        CONSOLE.println("button mode 3 - RC Mode ON");
        buzzer.sound(SND_ERROR, true);                                      // 3x Piep für R/C aktiv        
        attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);// Interrupt aktivieren
        attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);// Interrupt aktivieren 
// +------------
        attachInterrupt(digitalPinToInterrupt(pinTilt), get_mow_PPM, CHANGE);// Interrupt aktivieren 
// -------------
      }
      if (!RC_Mode) {                 
        CONSOLE.println("button mode 3 - RC Mode OFF");                                      // R/C-Mode inaktiv
        buzzer.sound(SND_WARNING, true);                          // 2x Piiiiiiiep für R/C aus
        motor.setLinearAngularSpeed(0, 0);                                 
        motor.setMowState(false);
        detachInterrupt(digitalPinToInterrupt(pinRemoteMow));             // Interrupt deaktivieren
        detachInterrupt(digitalPinToInterrupt(pinRemoteSteer));             // Interrupt deaktivieren
// +----------
        detachInterrupt(digitalPinToInterrupt(pinTilt));             // Interrupt deaktivieren
// -----------
      }    
  }
  
  if (RC_Mode)    {       
    lin_PPM = 0;
    if (PPM_start_lin < PPM_end_lin) lin_PPM = PPM_end_lin - PPM_start_lin; 
    if (lin_PPM < 2000 && lin_PPM > 1000)   {                               // Wert innerhalb 1100 bis 1900µsec
      float value_l = (lin_PPM - 1500) / 1500;                                // PPM auf Bereich +0.30 bis -0.30
      if ((value_l < 0.02) && (value_l > -0.02)) value_l = 0;                 // NullLage vergrössern         
      // linearPPM = value_l;                                                    // Weitergabe an Debug
      linearPPM = value_l*1.8;                                                    // Weitergabe an Debug
    }

    ang_PPM = 0;
    if (PPM_start_ang < PPM_end_ang) ang_PPM = PPM_end_ang - PPM_start_ang; 
    if (ang_PPM < 2000 && ang_PPM > 1000)   {                               // Wert innerhalb 1100 bis 1900µsec
      // float value_a = (ang_PPM - 1500) / 950;                                 // PPM auf Bereich +0.50 bis -0.50
      float value_a = (ang_PPM - 1500) / 1500;                                 // PPM auf Bereich +0.50 bis -0.50
      if ((value_a < 0.02) && (value_a > -0.02)) value_a = 0;                 // NullLage vergrössern         
      // angularPPM = value_a;                                                   // Weitergabe an Debug
      angularPPM = value_a*4;                                                   // Weitergabe an Debug
    }

// bMow = false;
    mow_PPM = 0;
    if (PPM_start_mow < PPM_end_mow) mow_PPM = PPM_end_mow - PPM_start_mow;
    if (mow_PPM < 2000 && mow_PPM > 1000) { // Wert innerhalb 1100 bis 1900µsec
      float value_b = (mow_PPM - 1515) / 1515; // PPM auf Bereich +0.30 bis -0.30
      if ((value_b < 0.05) && (value_b > -0.05)) value_b = 0; // NullLage vergrössern
      mowPPM = value_b; // Weitergabe an Debug
      if (mow_PPM > 1600) bMow = true;
      if (mow_PPM < 1600) bMow = false;
    }


#ifdef RC_DEBUG
    if (t >= nextOutputTime) {
      nextOutputTime = t + 1000;

      CONSOLE.print("RC: linearPPM= ");
      CONSOLE.print(linearPPM);
      CONSOLE.print("  angularPPM= ");
      CONSOLE.print(angularPPM);
      CONSOLE.print("  mow_PPM= ");
      CONSOLE.print(mow_PPM);
      CONSOLE.print("  bMow= ");
      CONSOLE.println(bMow);
    }
#endif
    motor.setLinearAngularSpeed(linearPPM, angularPPM, false);                     // R/C Signale an Motor leiten
    motor.setMowState(bMow);
  }
}
