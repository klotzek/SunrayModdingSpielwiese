// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"
#include "rcmodel.h"


void Motor::begin() {
  speedUpTrig = false;
  linearCurrSet = 0;
  motorMowRpmCheck = false;//MrTree
  motorMowRPMTrigFlag = false;//MrTree
  escapeLawnTrigFlag = false;//Mrtree
  motorMowRpmError = false;//MrTree
  motorMowSpunUp = false;//MrTree
	pwmMax = MOTOR_PID_LIMIT;
  switchedOn = false;//MrTree
  #ifdef MOW_PWM
    if (MOW_PWM <= 255) {
      mowPwm = MOW_PWM;
    }
    else mowPwm = 255;
  #else 
    mowPwm = 255;
  #endif

  #ifdef MOW_RPM_NORMAL                //MrTree
    if (MOW_RPM_NORMAL <= 6500) {
      mowRpm = MOW_RPM_NORMAL;
    }
    else mowRpm = 6500;
  #else 
    mowRpm = 6500;
  #endif
  
  //ticksPerRevolution = 1060/2;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
  mowticksPerRevolution = MOTOR_MOW_TICKS_PER_REVOLUTION; //MrTree
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm         = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / 3.1415;    // computes encoder ticks per cm (do not change)  

  motorLeftPID.Kp       = MOTOR_PID_KP;  // 2.0;  
  motorLeftPID.Ki       = MOTOR_PID_KI;  // 0.03; 
  motorLeftPID.Kd       = MOTOR_PID_KD;  // 0.03;
  motorLeftPID.reset(); 
  motorRightPID.Kp       = motorLeftPID.Kp;
  motorRightPID.Ki       = motorLeftPID.Ki;
  motorRightPID.Kd       = motorLeftPID.Kd;
  motorRightPID.reset();

  motorLeftLpf.Tf = MOTOR_PID_LP;
  motorLeftLpf.reset();
  motorRightLpf.Tf = MOTOR_PID_LP;  
  motorRightLpf.reset();

  motorMowPID.Kp       = MOWMOTOR_PID_KP; //MrTree
  motorMowPID.Ki       = MOWMOTOR_PID_KI; //MrTree
  motorMowPID.Kd       = MOWMOTOR_PID_KD; //MrTree
  motorMowPID.reset();     	              //MrTree

  robotPitch = 0;
  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  
  // apply optional custom motor direction swapping 
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    motorLeftSwapDir = !motorLeftSwapDir;
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    motorRightSwapDir = !motorRightSwapDir;
  #endif

  motorError = false;
  recoverMotorFault = false;
  recoverMotorFaultCounter = 0;
  nextRecoverMotorFaultTime = 0;
  enableMowMotor = ENABLE_MOW_MOTOR; //Default: true
  tractionMotorsEnabled = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false; 
  
  odometryError = false;  
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;  
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;  
  motorsSenseLP = 0;

  activateLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  SpeedFactor = 1; //MrTree
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowRpmSet = 0;//MrTree
  mowPowerAct = 0; //MrTree
  speedcurr = 0; //MrTree
  motorMowPWMSet = 0;  
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lastControlTime = 0;
  nextSenseTime = 0;
  lastMowRPMCheckTime = 0; //MrTree
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorMowTicks = 0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;    
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  motorLeftPWMCurrLP = 0;
  motorRightPWMCurrLP=0;   
  motorMowPWMCurrLP = 0;
  
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  motorMowRpmCurr=0;  
  motorLeftRpmLast = 0;
  motorRightRpmLast = 0;
  motorLeftRpmCurrLP = 0;
  motorRightRpmCurrLP = 0;
  motorMowRpmCurrLP = 0;
  motorMowRpmCurrLPFast = 0;  //MrTree
  
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;

  mowRPM_RC = 0; //MrTree
  
  motorRecoveryState = false;

  retryslow = false; //MrTree
  keepslow = false;   //MrTree
  y_before = 100;    //MrTree
  keepslow_y = 100;  //MrTree
  
  motorMowRPMStall = false;    //MrTree
  motorMowRPMStallDuration = 0;//MrTree

  drvfixtimer = DRVFIXTIMER; //MrTree
  drvfixreset = false; //MrTree
  drvfixcounter = 0; //MrTree
}

void Motor::setMowPwm( int val ){
  mowPwm = val;
}

void Motor::speedPWM ( int pwmLeft, int pwmRight, int pwmMow )
{ 
  //Correct Motor Direction
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  // ensure pwm is lower than Max
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMax, max(-pwmMax, pwmMow)); 
  
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow);
}

// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){  
   if (millis() < motorMowSpinUpTime + MOWSPINUPTIME){      //searching for a bug.... when mower will move without waiting for mowmotor
    //CONSOLE.println("Motor::setLinearAngularSpeed trying to wait for mowmotor....");
    linear = 0;                                     //workaround
    angular = 0; 
   }
   if (angular == 0) resetAngularMotionMeasurement(); //MrTree
   if (linear == 0) resetLinearMotionMeasurement();  //MrTree
   setLinearAngularSpeedTimeout = millis() + 1000;
   setLinearAngularSpeedTimeoutActive = true;
   linearCurrSet = linear;//MrTree
   if ((ADAPTIVE_SPEED)&&(linear > 0)) {   
     linear = linear * adaptiveSpeed();
     if ((speedUpTrig)&&(linear >= linearCurrSet)) linear = 0.998 * linearSpeedSet + 0.002 * linear; //Speed up slow if triggered from adaptive speed function
   }   
   if ((activateLinearSpeedRamp) && (useLinearRamp)) {
     linearSpeedSet = 0.82 * linearSpeedSet + 0.18 * linear;                                                                        //MrTree reduce overshooting lp changed from 0.9 to 0.82 //svolos calculated ramp would be better, but must be used here as own function besides adaptivespeed
     if ((linear == 0) && (fabs(linearSpeedSet) < 0.1)) linearSpeedSet = 0;                                                         //MrTree further reduce overshooting with cutting ramp if ramp speed is lower than (value) and setspeed is 0 
   } else {
     linearSpeedSet = linear;
   }
   //Global linear Speedlimit                                                                                                       //MrTree this code is obsolete if there is no false behaviour of other code
   if (GLOBALSPEEDLIMIT) {                                                                                                          //MrTree
    if (linearSpeedSet >0){                                                                                                         //MrTree
      linearSpeedSet = max(linearSpeedSet,MOTOR_MIN_SPEED);    //If positive linear is less than MINSPEED, use MINSPEED as limit    //MrTree
      linearSpeedSet = min(linearSpeedSet,MOTOR_MAX_SPEED);    //If positive linear is more than MAXSPEED, use MAXSPEED as limit    //MrTree
    }                                                                                                                               //MrTree
    if (linearSpeedSet <0){                                                                                                         //MrTree
      linearSpeedSet = min(linearSpeedSet,-1*MOTOR_MIN_SPEED); //If negative linear is more than -MINSPEED, use -MINSPEED as limit  //MrTree
      linearSpeedSet = max(linearSpeedSet,-1*MOTOR_MAX_SPEED); //If negative linear is less than -MAXSPEED, use -MAXSPEED as limit  //MrTree
    }                                                                                                                               //MrTree
   }
   angularSpeedSet = angular;   
   float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 /2);          
   float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 /2);          
   // RPM = V / (2*PI*r) * 60
   if (fabs(linearSpeedSet) < MOTOR_MIN_SPEED) resetLinearMotionMeasurement(); //MrTree less than MOTOR_MIN_SPEED 
   if (fabs(angularSpeedSet) <= 0,02) resetAngularMotionMeasurement(); //MrTree less then 1deg/s

   motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
   motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
   /*
   CONSOLE.print("setLinearAngularSpeed ");
   CONSOLE.print(linear);
   CONSOLE.print(",");
   CONSOLE.print(angular); 
   CONSOLE.print(",");
   CONSOLE.print(lspeed);
   CONSOLE.print(",");
   CONSOLE.println(rspeed);
   */
}


void Motor::enableTractionMotors(bool enable){
  if (enable == tractionMotorsEnabled) return;
  if (enable)
    CONSOLE.println("traction motors enabled");
  else 
    CONSOLE.println("traction motors disabled");
  tractionMotorsEnabled = enable;
}


void Motor::setMowState(bool switchOn){

  if (!USE_MOW_RPM_SET){
    if ((enableMowMotor) && (switchOn)){       
      if (abs(motorMowPWMSet) > 0) {
        CONSOLE.print("Motor::setMowState PWM - ");
        CONSOLE.println("Motor already switched on!");
        return; // mowing motor already switch ON
      }
      switchedOn = true; 
      motorMowSpinUpTime = millis();
      if (toggleMowDir){
        // toggle mowing motor direction each mow motor start
        motorMowForwardSet = !motorMowForwardSet;
        if (motorMowForwardSet) motorMowPWMSet = mowPwm;  
          else motorMowPWMSet = -mowPwm;  
      } else {      
        motorMowPWMSet = mowPwm;  
      }
      CONSOLE.print("Motor::setMowState ");
      CONSOLE.print(switchOn);
      CONSOLE.print(" PWM: ");
      CONSOLE.println(motorMowPWMSet);
    } else {
      motorMowPWMSet = 0;
      switchedOn = false;
      CONSOLE.print("Motor::setMowState PWM OFF");  
      //motorMowPWMCurr = 0; MrTree when we switchoff, there is no need for mowdriver to go to full break. its no emergency, we can use the ramp 
    }
  }
   
  if (USE_MOW_RPM_SET){ 
    if ((enableMowMotor) && (switchOn)){   
      if (abs(motorMowRpmSet) > 0) {
        CONSOLE.print("Motor::setMowState RPM - ");
        CONSOLE.println("Motor already switched on!");
        return; // mowing motor already switch ON
      }
      switchedOn = true; 
      motorMowSpinUpTime = millis();
      if (toggleMowDir){
        // toggle mowing motor direction each mow motor start
        motorMowForwardSet = !motorMowForwardSet;
        if (motorMowForwardSet) motorMowRpmSet = mowRpm;  
          else motorMowRpmSet = -mowRpm;  
      } else {      
        motorMowRpmSet = mowRpm;  
      }
      CONSOLE.print("Motor::setMowState ");
      CONSOLE.print(switchOn);
      CONSOLE.print(" RPM: ");
      CONSOLE.println(motorMowRpmSet);
    } else {
      CONSOLE.println("Motor::setMowState RPM OFF");
      motorMowRpmSet = 0;
      switchedOn = false;  
    }
  } 
}


void Motor::stopImmediately(bool includeMowerMotor){
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;      
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0;   
  if (includeMowerMotor) {
    switchedOn = false;
    motorMowPWMSet = 0;
    //motorMowPWMCurr = 0;//MrTree
    motorMowRpmSet = 0;//MrTree
    //motorMowRpmCurr = 0;//MrTree    
  }
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
  // reset PID
  motorLeftPID.reset();
  motorRightPID.reset();
  motorLeftLpf.reset();
  motorRightLpf.reset();
  motorMowPID.reset();//MrTree
  // reset unread encoder ticks
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}


void Motor::run() {
  if (millis() < lastControlTime + 50) return;
  
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
    
  sense();
  checkmotorMowRPMStall();         //MrTree
  drvfix();
  //if ((RCModel.RC_Mode)&&(USE_MOW_RPM_SET)) {
  //  if (motorMowRpmSet < 0)motorMowRpmSet = -mowRPM_RC; //MrTree
  //  if (motorMowRpmSet > 0)motorMowRpmSet = -mowRPM_RC; //MrTree
  //}
  
  // if motor driver indicates a fault signal, try a recovery   
  // if motor driver uses too much current, try a recovery     
  // if there is some error (odometry, too low current, rpm fault), try a recovery 
  if (!recoverMotorFault) {
    bool someFault = ( (checkFault()) || (checkCurrentTooHighError()) || (checkMowRpmFault()) 
                         || (checkOdometryError()) || (checkCurrentTooLowError()) );
    if (someFault){
      stopImmediately(true);
      recoverMotorFault = true;
      nextRecoverMotorFaultTime = millis() + 1000;                  
      motorRecoveryState = true;
    } 
  } 

  // try to recover from a motor driver fault signal by resetting the motor driver fault
  // if it fails, indicate a motor error to the robot control (so it can try an obstacle avoidance)  
  if (nextRecoverMotorFaultTime != 0){
    if (millis() > nextRecoverMotorFaultTime){
      if (recoverMotorFault){
        nextRecoverMotorFaultTime = millis() + 10000;
        recoverMotorFaultCounter++;                                               
        CONSOLE.print("motor fault recover counter ");
        CONSOLE.println(recoverMotorFaultCounter);
        motorDriver.resetMotorFaults();
        recoverMotorFault = false;  
        if (recoverMotorFaultCounter >= 10){ // too many successive motor faults
          //stopImmediately();
          CONSOLE.println("ERROR: motor recovery failed");
          recoverMotorFaultCounter = 0;
          motorError = true;
        }
      } else {
        CONSOLE.println("resetting recoverMotorFaultCounter");
        recoverMotorFaultCounter = 0;
        nextRecoverMotorFaultTime = 0;
        motorRecoveryState = false;
      }        
    }
  }
  
  int ticksLeft;
  int ticksRight;
  int ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  
  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  if (motorMowPWMCurr < 0) ticksMow *= -1;
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  //CONSOLE.print("motorMowTicks = ");
  //CONSOLE.println(motorMowTicks);
  //CONSOLE.print("motorMowSpinUpTime = ");
  //CONSOLE.println(motorMowSpinUpTime);
  unsigned long currTime = millis();
  float deltaControlTimeSec =  ((float)(currTime - lastControlTime)) / 1000.0;
  lastControlTime = currTime;
  //CONSOLE.print("deltaControlTimeSec = ");
  //CONSOLE.println(deltaControlTimeSec);

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)mowticksPerRevolution) ) / deltaControlTimeSec; //MrTree, added define
  //CONSOLE.print("motorMowRpmCurr = ");
  //CONSOLE.println(motorMowRpmCurr);
  float lp = 0.9; // 0.995
  motorLeftRpmCurrLP = lp * motorLeftRpmCurrLP + (1.0-lp) * motorLeftRpmCurr;
  motorRightRpmCurrLP = lp * motorRightRpmCurrLP + (1.0-lp) * motorRightRpmCurr;
  motorMowRpmCurrLP = lp * motorMowRpmCurrLP + (1.0-lp) * motorMowRpmCurr;  
  motorMowRpmCurrLPFast = 0.75 * motorMowRpmCurrLPFast + (1.0-0.75) * motorMowRpmCurr;    //MrTree added LPFast for more dynamical readings
  //CONSOLE.print("motorMowRpmCurrLPFast = ");
  //CONSOLE.println(motorMowRpmCurrLPFast); 
  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 2) motorLeftRpmCurr = 0;
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 2) motorRightRpmCurr = 0;
  } else motorRightTicksZero = 0;

  
  // speed controller
  control();    
  motorLeftRpmLast = motorLeftRpmCurr;
  motorRightRpmLast = motorRightRpmCurr;
}  


// check if motor current too high
bool Motor::checkCurrentTooHighError(){
  bool motorLeftFault = (motorLeftSense > MOTOR_FAULT_CURRENT);
  bool motorRightFault = (motorRightSense > MOTOR_FAULT_CURRENT);
  bool motorMowFault = (motorMowSense > MOW_FAULT_CURRENT);     //MrTree evade Current Spikes of mowmotor
  if (motorLeftFault || motorRightFault || motorMowFault){
    CONSOLE.print("ERROR motor current too high: ");
    CONSOLE.print("  current=");
    CONSOLE.print(motorLeftSense);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSense);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSense);
    return true;
  } 
  return false; 
}


// check if motor current too low
bool Motor::checkCurrentTooLowError(){
  //CONSOLE.print(motorRightPWMCurr);
  //CONSOLE.print(",");
  //CONSOLE.println(motorRightSenseLP);
  if  (    ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (motorMowSenseLP < MOW_TOO_LOW_CURRENT)) 
        ||  ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))  ){        
    // at least one motor is not consuming current      
    // first try reovery, then indicate a motor error to the robot control (so it can try an obstacle avoidance)    
    CONSOLE.print("ERROR: motor current too low: pwm (left,right,mow)=");
    CONSOLE.print(motorLeftPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPWMCurr);
    CONSOLE.print("  average current amps (left,right,mow)=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  }
  return false;
}


// check motor driver (signal) faults
bool Motor::checkFault() {
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  if (ENABLE_FAULT_DETECTION){    
    motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  }
  if (leftFault) {
    CONSOLE.println("Error: motor driver left signaled fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor driver right signaled fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor driver mow signaled fault");
    fault = true;
  }
  return fault;
}


// check odometry errors
bool Motor::checkOdometryError() {
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if  (   ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (abs(motorLeftRpmCurrLP) < 0.001))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (abs(motorRightRpmCurrLP) < 0.001))  )
    {               
      // odometry error
      CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
      CONSOLE.print(motorLeftRpmCurrLP);
      CONSOLE.print(",");
      CONSOLE.println(motorRightRpmCurrLP);     
      return true;        
    }
  }
  return false;
}


// check motor overload
void Motor::checkOverload(){
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseLP > MOW_OVERLOAD_CURRENT);
  if (motorLeftOverload || motorRightOverload || motorMowOverload){                   //MrTree
    if (motorOverloadDuration == 0){
      CONSOLE.print("ERROR motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }
}


// Adaptive_Speed: ramp mowingspeed with mow motor rpm or current measuring            //**MrTree                                              //MrTree
float Motor::adaptiveSpeed(){                                                          //----->  
  if (ADAPTIVE_SPEED){
    //if ((millis() > linearMotionStartTime + BUMPER_DEADTIME) && (bumper.obstacle())) return 1;     // bumper interrupt behaviour workaround    
    if (!switchedOn){
      keepslow = false;
      retryslow = false;
      keepSlowTime = 0;
      retrySlowTime = 0;
      speedUpTrig = false;
      return 1;
    }
    //prepare variables
    float mowPowerMax = 0;
    float mowPowerMin = 0;
    //float mowPowerAct = 0; Changed to header
    float dx = 0;  
    float dy = 0;                                     
    float m = 0;
    float b = 0; 
    float x = 0;                                                    
    float y = 0;
    speedcurr = linearCurrSet; //changed to header

    if (MOWMOTORPOWER) {
      if (MOWPOWERMAX_AUTO) mowPowerMax = motorMowPowerMax;
      else mowPowerMax = MOWPOWERMAX;
      mowPowerMin = MOWPOWERMIN;
      dx = mowPowerMax - mowPowerMin;  
      dy = 100 - (MOTOR_MIN_SPEED/speedcurr*100);                                             
      m = -1*dy/dx;
      b = 100 - (m * mowPowerMin); 
      x = mowPowerAct;                                                    
      y = (m*x)+b;
      
    } else if (MOWMOTORRPM) {
      //build linear function: delta mow rpm(dx), delta mowing speed(dy), slope m, shift b
      //y = mx+b; x=RPM, y=SpeedFactor
      dx = abs(motorMowRpmSet) - ((abs(motorMowRpmSet) * MOW_RPMtr_SLOW/100));  //+constant: we want to go to min speed before possible rpm stall or keep slow triggers
      dy = 100 - (MOTOR_MIN_SPEED/speedcurr*100);                               //use CurrSpeed or linearSpeedSet insted of setSpeed???                
      m = dy/dx;
      b = 100 - (m * abs(motorMowRpmSet)); 
      x = abs(motorMowRpmCurrLPFast);                                               //rpm of mowmotor defines x in linear ramp
      y = (m*x)+b;                                                                  //calced mower speed due to rpm (m/s) as speedfactor
    } else return 1;
    if (keepslow && retryslow) keepslow = false;                                    //reset keepslow because retryslow is prior

    if (abs(motorMowRpmCurrLPFast) < (abs(MOW_RPM_NORMAL) * MOW_RPMtr_SLOW/100)){   //trigger and set timer once, trigger by rpm percentage
      if ((!keepslow) && (!retryslow)){                                             //only if not already trigged
        CONSOLE.println("Adaptive_Speed: Keeping slow!");
        keepslow = true;                                                            //enable keepslow state
        if (abs(motorMowRpmSet) != abs(MOW_RPM_SLOW)){                              //set the keepslow rpm
          CONSOLE.println("Adaptive_Speed: Using MOW_RPM_SLOW");
          if (motorMowRpmSet > 0) motorMowRpmSet = MOW_RPM_SLOW;                    //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?
          if (motorMowRpmSet < 0) motorMowRpmSet = -MOW_RPM_SLOW;                   //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?                                                       
        }
      }                                                                             //step out of trigger condition
      if (keepslow) keepSlowTime = millis()+KEEPSLOWTIME;                           //set or refresh keepslowtimer
      if (retryslow) retrySlowTime = millis()+RETRYSLOWTIME;                        //if we already are in retryslow condition, we refresh the timer of retry also                               
    }
                                              
    if (retryslow){                                                    //retryslow is triggered by the end of escapelawnop                       
      if (abs(motorMowRpmSet) != abs(MOW_RPM_RETRY)) {
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_RETRY");
        if (motorMowRpmSet > 0) motorMowRpmSet = MOW_RPM_RETRY; 
        if (motorMowRpmSet < 0) motorMowRpmSet = -MOW_RPM_RETRY;                      
      }                   
      if (millis() > retrySlowTime) {
        CONSOLE.println("Adaptive_Speed: Retryslow done! Going to Keepslow!");
        //CONSOLE.println("Adaptive_Speed: Speeding up!");
        retryslow = false;
        keepslow = true;
        keepSlowTime = millis()+KEEPSLOWTIME;
        //speedUpTrig = true; 
      }
    }

    if (keepslow){                                                    //keepslow is triggered in this function, retryslow in escapelawnOp   
      if (millis() > keepSlowTime) {
        CONSOLE.println("Adaptive_Speed: Keepslow done!");
        CONSOLE.println("Adaptive_Speed: Speeding up!");
        keepslow = false;                                             //reset keepslow if no rpm stall      
        speedUpTrig = true;
      }
    }

    if (speedUpTrig){
      if ((millis() > keepSlowTime + KEEPSLOWTIME)){                  // || (millis() > retrySlowTime + RETRYSLOWTIME)){
        CONSOLE.println("Adaptive_Speed: Speeding up done!");
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_NORMAL");
        if (motorMowRpmSet > 0) motorMowRpmSet = MOW_RPM_NORMAL;
        if (motorMowRpmSet < 0) motorMowRpmSet = -MOW_RPM_NORMAL;
        speedUpTrig = false;
      }
    }                             

    y = constrain(y, MOTOR_MIN_SPEED/speedcurr*100, 100);//limit val
    y_before = y;
    SpeedFactor = y/100;                                //used for linear modifier as: MOTOR_MIN_SPEED/setSpeed*100 < Speedfactor <= 1

    //Logging
    //if (TUNING_LOG){
      //CONSOLE.println("motor.cpp: adaptive_speed() [deltaRPM, deltaSpeed, slope m, shift b, Mow RPM, motorMowRpmSet, motorMowRPM, mowPowerAct, SpeedFactor: ");
      //CONSOLE.print(dx);
      //CONSOLE.print(", ");
      //CONSOLE.print(dy);
      //CONSOLE.print(", ");
      //CONSOLE.print(m);
      //CONSOLE.print(", ");
      //CONSOLE.print(b);
      //CONSOLE.print(", ");
      //CONSOLE.print(x);
      //CONSOLE.print(", ");
      //CONSOLE.print(motorMowRpmSet);
      //CONSOLE.print(", ");
      //CONSOLE.print(motorMowSense);
      //ONSOLE.print(", ");
      //CONSOLE.print(mowPowerAct);
      //CONSOLE.print(", ");
      //CONSOLE.println(SpeedFactor);
    //}
    //if (speedcurr == 0) return 1; //speedcurr = 0.49;

    return SpeedFactor;
  } else { //MrTree adaptive speed is not activated
    if (TUNING_LOG){
    CONSOLE.println("motor.cpp: adaptive_speed() disabled or conditions not met... RETURNING 1"); 
    }
    return 1;
  }
}  


// check mow motor RPM stalls                                                          
void Motor::checkmotorMowRPMStall(){ 
  if ((ENABLE_RPM_FAULT_DETECTION)&&(switchedOn)) {//&&(motorMowRpmSet != 0)&&((millis() > linearMotionStartTime + BUMPER_DEADTIME) && (!bumper.obstacle()) )){    
    if (((motorMowSpinUpTime + MOWSPINUPTIME) > millis())){//||(!switchedOn)){  
      //CONSOLE.println("Motor::checkmotorMowRPMStall returning mowmotor not ready....");
      motorMowRPMStallDuration = 0;
      motorMowRPMStall = false;
      motorMowSpunUp = false;
      //motorMowRpmError = false;
      //motorMowRpmCheck = false;     
      return;
    }      
    if (!motorMowSpunUp){
      if ((abs(motorMowRpmCurrLPFast) < abs(motorMowRpmSet)-150) || (abs(motorMowRpmCurrLPFast) > abs(motorMowRpmSet)+150)){
        motorMowSpunUp = true;    
        CONSOLE.println("checkmotorMowRPMStall: WARNING mow motor did not spun up to RPM set.");
        CONSOLE.println("Increase SPINUPTIME or consider mow motor not reaching or overspinning specific RPM set by more than +- 150 RPM.");
        CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet, RPM difference: ");
        CONSOLE.print(MOWSPINUPTIME);
        CONSOLE.print(", ");
        CONSOLE.print(abs(motorMowPWMCurr));
        CONSOLE.print(", ");
        CONSOLE.print(abs(motorMowRpmCurrLPFast));
        CONSOLE.print(", ");
        CONSOLE.print(abs(motorMowRpmSet));
        CONSOLE.print(", ");
        CONSOLE.println(abs(motorMowRpmSet)-abs(motorMowRpmCurrLPFast));
      } else {     
          motorMowSpunUp = true;
          CONSOLE.println("checkmotorMowRPMStall: Mow motor Spun up!");
          CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet: ");
          CONSOLE.print(MOWSPINUPTIME);
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowPWMCurr));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmCurrLPFast));
          CONSOLE.print(", ");
          CONSOLE.println(abs(motorMowRpmSet));
      }
    }   
    //CONSOLE.print(abs(motorMowRpmCurr));
    //CONSOLE.print(", "); 
    //CONSOLE.println(abs(motorMowRpmCurrLPFast));
    unsigned long deltaControlTimeSec = (millis() - lastMowRPMCheckTime);
    if ((millis() < linearMotionStartTime + BUMPER_DEADTIME) || (bumper.obstacle())) return;
    motorMowRPMStall = (abs(motorMowRpmCurrLPFast) < (abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));  //LPFast? // changed motorMowRpmSet to mowRpm   
    if ((motorMowRPMStall) ){ 
      if (motorMowRPMStallDuration == 0)CONSOLE.println("motorMowRPM stalled!");   
      motorMowRPMTrigFlag = true;
      motorMowRPMStallDuration += deltaControlTimeSec;     
    } else {
      if (motorMowRPMStallDuration != 0){
        motorMowRPMTrigFlag = false;         
        CONSOLE.print("checkmotorMowRPMStall: WARNING mow motor RPM stalled for duration(ms) >= ");
        CONSOLE.println(motorMowRPMStallDuration);
        CONSOLE.print("Data: Trigger RPM, Mow RPM: ");
        CONSOLE.print((abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));
        CONSOLE.print(", ");
        CONSOLE.println(abs(motorMowRpmSet));      
      }      
      motorMowRPMStallDuration = 0;
    }  
    lastMowRPMCheckTime = millis();      
    return;       
  }                                                                                                         //<-----  
  motorMowRPMStall = false;
  return;                                                                                           //**MrTree
}

void Motor::drvfix(){
  if (DRV8308_FIX) {
    if (drvfixreset) {
      drvfixcounter++;
      if (drvfixcounter >= DRVFIXITERATIONS) {
        speedPWM(0, 0, 0);
        drvfixreset = false;
        drvfixcounter = 0;
      }
    }
    if ((millis() >= drvfixtimer) && (battery.chargerConnected()) && (motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0) && (motorMowPWMCurr == 0)){
      drvfixtimer = millis() + DRVFIXTIMER;
      speedPWM(PWM_GEAR, PWM_GEAR, PWM_MOW);
      drvfixreset = true;
    }  
    return;
  } else {
    return;
  }
}

// check mow rpm fault
bool Motor::checkMowRpmFault(){
  //CONSOLE.print(motorMowPWMCurr);
  //CONSOLE.print(",");
  //CONSOLE.print(motorMowPWMCurrLP);  
  //CONSOLE.print(",");
  //CONSOLE.println(motorMowRpmCurrLP);
  if (ENABLE_RPM_FAULT_DETECTION){
    if  ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (abs(motorMowRpmCurrLP) < 10.0)) {        
      CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
      CONSOLE.print(motorMowPWMCurr);
      CONSOLE.print("  pwmLP=");      
      CONSOLE.print(motorMowPWMCurrLP);      
      CONSOLE.print("  rpmLP=");
      CONSOLE.print(motorMowRpmCurrLP);
      CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
      return true;
    }
  }  
  return false;
}


// measure motor currents
void Motor::sense(){
  if (millis() < nextSenseTime) return;
  nextSenseTime = millis() + 20;
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);
  float lp = 0.88; //0.90, 0.995
  motorRightSenseLP = lp * motorRightSenseLP + (1.0-lp) * motorRightSense;
  motorLeftSenseLP = lp * motorLeftSenseLP + (1.0-lp) * motorLeftSense;
  lp = 0.9;
  motorMowSenseLP = lp * motorMowSenseLP + (1.0-lp) * motorMowSense; 
  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;
  lp = 0.995;
  motorRightPWMCurrLP = lp * motorRightPWMCurrLP + (1.0-lp) * ((float)motorRightPWMCurr);
  motorLeftPWMCurrLP = lp * motorLeftPWMCurrLP + (1.0-lp) * ((float)motorLeftPWMCurr);  
  motorMowPWMCurrLP = lp * motorMowPWMCurrLP + (1.0-lp) * ((float)motorMowPWMCurr); 
 
  // compute normalized current (normalized to 1g gravity)
  //float leftAcc = (motorLeftRpmCurr - motorLeftRpmLast) / deltaControlTimeSec;
  //float rightAcc = (motorRightRpmCurr - motorRightRpmLast) / deltaControlTimeSec;
  float cosPitch = cos(robotPitch); 
	float pitchfactor;
  float robotMass = 1.0;
	// left wheel friction
	if (  ((motorLeftPWMCurr >= 0) && (robotPitch <= 0)) || ((motorLeftPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch; // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch;  // increase by angle
	motorLeftSenseLPNorm = abs(motorLeftSenseLP) * robotMass * pitchfactor;  
	// right wheel friction
	if (  ((motorRightPWMCurr >= 0) && (robotPitch <= 0)) || ((motorRightPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch;  // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch; // increase by angle
  motorRightSenseLPNorm = abs(motorRightSenseLP) * robotMass * pitchfactor; 

  //////////////////////////////// MrTree need more data
  mowPowerAct = battery.batteryVoltage * motorMowSense;               //actual mow motor Power
  motorRightPowerAct = battery.batteryVoltage * motorRightSense;      //actual right motor power
  motorLeftPowerAct = battery.batteryVoltage * motorLeftSense;        //actual left motor power
  motorMowPowerMax = max(motorMowPowerMax, mowPowerAct);              //save mow motor max power during mower activated
  motorRightPowerMax = max(motorRightPowerMax, motorRightPowerAct);   //save right motor max power during mower activated
  motorLeftPowerMax = max(motorLeftPowerMax, motorLeftPowerAct);      //save left motor max power during mower activated
  ///////////////////////////////

  checkOverload();  
}


void Motor::control(){  
    
  //########################  Calculate PWM for left driving motor ############################

  motorLeftPID.TaMax = 0.20;
  motorLeftPID.x = motorLeftLpf(motorLeftRpmCurr);
  motorLeftPID.w  = motorLeftRpmSet;
  motorLeftPID.y_min = -pwmMax;
  motorLeftPID.y_max = pwmMax;
  motorLeftPID.max_output = pwmMax;
  motorLeftPID.output_ramp = MOTOR_PID_RAMP;
  motorLeftPID.compute();
  motorLeftPWMCurr = motorLeftPWMCurr + motorLeftPID.y;
  if (motorLeftRpmSet > 0) motorLeftPWMCurr = min( max(0, (int)motorLeftPWMCurr), pwmMax); // 0.. pwmMax  //MrTree Bugfix deleted =
  if (motorLeftRpmSet < 0) motorLeftPWMCurr = max(-pwmMax, min(0, (int)motorLeftPWMCurr));  // -pwmMax..0

  //########################  Calculate PWM for right driving motor ############################
  
  motorRightPID.TaMax = 0.20;
  motorRightPID.x = motorRightLpf(motorRightRpmCurr);
  motorRightPID.w = motorRightRpmSet;
  motorRightPID.y_min = -pwmMax;
  motorRightPID.y_max = pwmMax;
  motorRightPID.max_output = pwmMax;
  motorRightPID.output_ramp = MOTOR_PID_RAMP;
  motorRightPID.compute();
  motorRightPWMCurr = motorRightPWMCurr + motorRightPID.y;
  if (motorRightRpmSet > 0) motorRightPWMCurr = min( max(0, (int)motorRightPWMCurr), pwmMax);  // 0.. pwmMax //MrTree Bugfix deleted =
  if (motorRightRpmSet < 0) motorRightPWMCurr = max(-pwmMax, min(0, (int)motorRightPWMCurr));   // -pwmMax..0  

  //if ((abs(motorLeftRpmSet) < 0.01) && (motorLeftPWMCurr < 30)) motorLeftPWMCurr = 0;
  //if ((abs(motorRightRpmSet) < 0.01) && (motorRightPWMCurr < 30)) motorRightPWMCurr = 0;


  //########################  Calculate PWM for mow motor ############################                          //MrTree
  //if ((USE_MOW_RPM_SET) && (motorMowRpmSet != 0)){                                                               //**------>
    if ((mowRPM_RC != 0)&&(RC_Mode)) {
      if (motorMowRpmSet < 0)motorMowRpmSet = -mowRPM_RC; //MrTree
      if (motorMowRpmSet > 0)motorMowRpmSet = mowRPM_RC; //MrTree
    }
    
    float comp = MOWMOTOR_RPM_OFFSET;
    if (motorMowRpmSet < 0) comp = -comp;
    
    motorMowPID.TaMax = 0.20;
    motorMowPID.x = motorMowRpmCurrLPFast-comp; //LPFast;
    motorMowPID.w = motorMowRpmSet;
    motorMowPID.y_min = -pwmMax;
    motorMowPID.y_max = pwmMax;
    motorMowPID.max_output = pwmMax;
    motorMowPID.compute();
    motorMowPWMCurr = motorMowPWMCurr + motorMowPID.y;
    if (motorMowRpmSet > 0) motorMowPWMCurr = min( max(0, (int)motorMowPWMCurr), pwmMax);  // 0.. pwmMax
    if (motorMowRpmSet < 0) motorMowPWMCurr = max(-pwmMax, min(0, (int)motorMowPWMCurr));   // -pwmMax..0  

    //we use simple low pass when stopping mow motor
    if ((USE_MOW_RPM_SET) && (motorMowRpmSet == 0)){
      motorMowPWMCurr = 0.96 * motorMowPWMCurr + 0.04 * motorMowPWMSet;
      if (abs(motorMowRpmCurrLPFast) < 400) motorMowPWMCurr = 0;    
    }                                                                                                           //<------**
  //}                                                                                                             //MrTree
  //mow motor ramp
  if (!USE_MOW_RPM_SET) motorMowPWMCurr = 0.93 * motorMowPWMCurr + 0.07 * motorMowPWMSet;     //MrTree slightly increased spinuprate (0.99|0.01) before) 
  
  //set PWM for all motors
  if (!tractionMotorsEnabled){
    motorLeftPWMCurr = motorRightPWMCurr = 0;
  }
  //motorMowPWMCurr = 100;
  //CONSOLE.print("Controller motorMowPWMCurr, motorMowRpmSet, motorMowRpmCurrLPFast: ");
  //CONSOLE.print(motorMowPWMCurr);
  //CONSOLE.print(", ");
  //CONSOLE.print(motorMowRpmSet);
  //CONSOLE.print(", ");
  //CONSOLE.println(motorMowRpmCurrLPFast);
  //if (motorMowRpmSet == 0) motorMowSpinUpTime = millis(); //MrTree test to force waiting for mowmotor......
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr); 
}


void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}


void Motor::test(){
  CONSOLE.println("motor test - 10 revolutions");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  int pwmLeft = 200;
  int pwmRight = 200; 
  bool slowdown = true;
  unsigned long stopTicks = ticksPerRevolution * 10;
  unsigned long nextControlTime = 0;
  while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks){
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20;
      if ((slowdown) && ((motorLeftTicks + ticksPerRevolution  > stopTicks)||(motorRightTicks + ticksPerRevolution > stopTicks))){  //Letzte halbe drehung verlangsamen
        pwmLeft = pwmRight = 20;
        slowdown = false;
      }    
      if (millis() > nextInfoTime){      
        nextInfoTime = millis() + 1000;            
        dumpOdoTicks(seconds);
        seconds++;      
      }    
      if(motorLeftTicks >= stopTicks)
      {
        pwmLeft = 0;
      }  
      if(motorRightTicks >= stopTicks)
      {
        pwmRight = 0;      
      }
      
      speedPWM(pwmLeft, pwmRight, 0);
      sense();
      //delay(50);         
      watchdogReset();     
      robotDriver.run();
    }
  }  
  speedPWM(0, 0, 0);
  CONSOLE.println("motor test done - please ignore any IMU/GPS errors");
}


void Motor::plot(){
  CONSOLE.println("motor plot (left,right,mow) - NOTE: Start Arduino IDE Tools->Serial Plotter (CTRL+SHIFT+L)");
  delay(5000);
  CONSOLE.println("pwmLeft,pwmRight,pwmMow,ticksLeft,ticksRight,ticksMow");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  motorMowTicks = 0;
  int pwmLeft = 0;
  int pwmRight = 0; 
  int pwmMow = 0;
  int cycles = 0;
  int acceleration = 1;
  bool forward = true;
  unsigned long nextPlotTime = 0;
  unsigned long stopTime = millis() + 1 * 60 * 1000;
  unsigned long nextControlTime = 0;

  while (millis() < stopTime){   // 60 seconds...
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20; 

      int ticksLeft=0;
      int ticksRight=0;
      int ticksMow=0;
      motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
      motorLeftTicks += ticksLeft;
      motorRightTicks += ticksRight;
      motorMowTicks += ticksMow;

      if (millis() > nextPlotTime){ 
        nextPlotTime = millis() + 100;
        CONSOLE.print(300+pwmLeft);
        CONSOLE.print(",");  
        CONSOLE.print(300+pwmRight);
        CONSOLE.print(",");
        CONSOLE.print(pwmMow);
        CONSOLE.print(",");        
        CONSOLE.print(300+motorLeftTicks);    
        CONSOLE.print(",");
        CONSOLE.print(300+motorRightTicks);
        CONSOLE.print(",");
        CONSOLE.print(motorMowTicks);        
        CONSOLE.println();
        motorLeftTicks = 0;
        motorRightTicks = 0;
        motorMowTicks = 0;      
      }

      speedPWM(pwmLeft, pwmRight, pwmMow);
      if (pwmLeft >= 255){
        forward = false;
        cycles++; 
      }      
      if (pwmLeft <= -255){
        forward = true;
        cycles++;               
      } 
      if ((cycles == 2) && (pwmLeft >= 0)) {
        if (acceleration == 1) acceleration = 20;
          else acceleration = 1;
        cycles = 0;
      }         
      if (forward){
        pwmLeft += acceleration;
        pwmRight += acceleration;
        pwmMow += acceleration;
      } else {
        pwmLeft -= acceleration;
        pwmRight -= acceleration;
        pwmMow -= acceleration;
      }
      pwmLeft = min(255, max(-255, pwmLeft));
      pwmRight = min(255, max(-255, pwmRight));          
      pwmMow = min(255, max(-255, pwmMow));                
    }  
    //sense();
    //delay(10);
    watchdogReset();     
    robotDriver.run(); 
  }
  speedPWM(0, 0, 0);
  CONSOLE.println("motor plot done - please ignore any IMU/GPS errors");
}
