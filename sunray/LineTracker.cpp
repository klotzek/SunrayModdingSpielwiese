// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"
#include "Stats.h"


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
//Polygon circle(8);

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

float targetDist = 0;     //MrTree
float lastTargetDist = 0; //MrTree

float setSpeed = 0.1; //externally controlled (app) linear speed (m/s)
float CurrSpeed = 0;  //actual used speed from motor.linearSpeedSet
float linear = 0;
float angular = 0;

float x_old = 0;
float y_old = 0;
float x_new = 0;
float y_new = 0;
Point lastPoint;

//Point last_rotation_target;
bool mow = false;
bool trackslow_allowed = false;
bool straight = false;
bool shouldRotate = false;                      //MrTree
bool shouldRotatel = false;                     //MrTree
bool angleToTargetFits = false;
bool angleToTargetPrecise = true;
bool langleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
float distToPath = 0;
bool stateKidnapped = false;
unsigned long reachedPointBeforeDockTime = 0;   //MrTree
bool dockTimer = false;                         //MrTree
bool oneTrigger = false;                        //MrTree
int dockGpsRebootState;                     // Svol0: status for gps-reboot at specified docking point by undocking action
bool blockKidnapByUndocking;                // Svol0: kidnap detection is blocked by undocking without gps
unsigned long dockGpsRebootTime;            // Svol0: retry timer for gps-fix after gps-reboot
unsigned long dockGpsRebootFixCounter;      // Svol0: waitingtime for fix after gps-reboot
unsigned long dockGpsRebootFeedbackTimer;   // Svol0: timer to generate acustic feedback
bool dockGpsRebootDistGpsTrg = false;       // Svol0: trigger to check solid gps-fix position (no jump)
bool allowDockLastPointWithoutGPS = false;  // Svol0: allow go on docking by loosing gps fix
bool allowDockRotation = true;              //MrTree: disable rotation on last dockingpoint
bool warnDockWithoutGpsTrg = false;         // Svol0: Trigger for warnmessage
float stateX_1 = 0;                         // Svol0
float stateY_1 = 0;                         // Svol0
float stateX_2 = 0;                         // Svol0
float stateY_2 = 0;                         // Svol0
float stateX_3 = 0;                         // Svol0
float stateY_3 = 0;                         // Svol0
int counterCheckPos = 0;                    // check if gps position is reliable

bool printmotoroverload = false;
bool trackerDiffDelta_positive = false;

bool AngleToTargetFits() {
  // allow rotations only near last or next waypoint or if too far away from path
  if (targetDist < 0.3 || lastTargetDist < 0.3 || fabs(distToPath) > 1.0 ) {
    angleToTargetFits = (fabs(trackerDiffDelta) / PI * 180.0 < 20);                                       //MrTree we have more than 20deg difference to point
  } else {
	// while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
  }

  if ((!angleToTargetFits || !angleToTargetPrecise) && !dockTimer) angleToTargetFits = false;   //MrTree added !dockTimer to prevent the jumping gps point to cause linear=0 because of !angleToTargetFits, added !angleToTargetPrecise 

  return angleToTargetFits;  
}

void rotateToTarget() {
  // angular control (if angle to far away, rotate to next waypoint)
    if (!angleToTargetFits) angleToTargetPrecise = false;
    linear = 0; //MrTree while turning from >= 20/45 deg difference, linear is set to 0... still decelerating or accelerating on stepin/out
    if ((maps.isDocking() || maps.isUndocking()) && maps.trackSlow && trackslow_allowed){
      angular = DOCKANGULARSPEED / 180.0 * PI;  //MrTree use DOCKANGULARSPEED in config.h, added trackslowallowed  : RTT=29deg/s=0.5 rad/s;  
    } else {
      if (ROTATION_RAMP) {
        angular =  1.8 * fabs(trackerDiffDelta);
        angular = constrain(angular, ROTATION_RAMP_MIN / 180.0 * PI, ROTATION_RAMP_MAX / 180.0 * PI);
      } else {
        if (fabs(trackerDiffDelta)/PI*180.0 >= ANGLEDIFF1) angular = ROTATETOTARGETSPEED1 / 180.0 * PI;   //MrTree set angular to fast defined in config.h
        if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEDIFF1) angular = ROTATETOTARGETSPEED2  / 180.0 * PI;    //MrTree slow down turning when near desired angle     
        if (fabs(trackerDiffDelta)/PI*180.0 <= ANGLEDIFF2) angular = ROTATETOTARGETSPEED3 / 180.0 * PI;    //MrTree slow down turning even more when almost at desired angle     
      }
    }
    if (trackerDiffDelta < 0) {     //MrTree set rotation direction and do not keep it :)
      angular *= -1;
    }                     
    if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEPRECISE){
      angleToTargetPrecise = true;                          //MrTree Step out of everything when angle is precise...
      angular = 0;
    } 
    if (fabs(CurrSpeed) >= 0.1) angular = 0;                //MrTree reset angular if current speed is over given value (still deccelerating)
}

void stanleyTracker() {
   
  //Stanley parameters
  static float k;
  static float p;

  if (MAP_STANLEY_CONTROL) {
    //Mapping of Stanley Control Parameters in relation to actual Setpoint value of speed
    //Values need to be multiplied, because map() function does not work well with small range decimals
    // linarSpeedSet needed as absolut value for mapping
    k = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowK * 1000, stanleyTrackingNormalK * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    p = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowP * 1000, stanleyTrackingNormalP * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    k = k / 1000;
    p = p / 1000;
    k = max(stanleyTrackingSlowK, min(stanleyTrackingNormalK, k));  // limitation for value if out of range
    p = max(stanleyTrackingSlowP, min(stanleyTrackingNormalP, p));  // limitation for value if out of range
  } else {
    k = stanleyTrackingNormalK;                                     // STANLEY_CONTROL_K_NORMAL;
    p = stanleyTrackingNormalP;                                     // STANLEY_CONTROL_P_NORMAL;
    if (maps.trackSlow && trackslow_allowed) {
      k = stanleyTrackingSlowK;                                     //STANLEY_CONTROL_K_SLOW;
      p = stanleyTrackingSlowP;                                     //STANLEY_CONTROL_P_SLOW;
    }
  }
                                                                                                                                //MrTree
  angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(CurrSpeed)));       //MrTree, use actual speed correct for path errors
  // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
  angular = max(-PI/6, min(PI/6, angular)); //MrTree still used here because of gpsfix jumps that would lead to an extreme rotation speed

}

void linearSpeedState(){
  const int aLen = 8;                                           //array length of linearSpeed[]
  const String linearSpeedNames[aLen] = {                       //strings for message output accordingly to state
                                    "FLOATSPEED",
                                    "NEARWAYPOINTSPEED",
                                    "SONARSPEED",
                                    "OVERLOADSPEED",
                                    "KEEPSLOWSPEED",
                                    "RETRYSLOWSPEED",
                                    "TRACKSLOWSPEED",
                                    "DOCK_NO_ROTATION_SPEED"
                                  };
  const float linearSpeed[aLen] = {
                                    FLOATSPEED,
                                    NEARWAYPOINTSPEED,
                                    SONARSPEED,
                                    OVERLOADSPEED,
                                    KEEPSLOWSPEED,
                                    RETRYSLOWSPEED,
                                    TRACKSLOWSPEED,
                                    DOCK_NO_ROTATION_SPEED
                                  };
  static bool linearBool[aLen];     //helper array to choose lowest value from sensor/mower states
  static int chosenIndex;           //helper to know what speed is used
  static int chosenIndexl;          //helper to compare a index change and trigger a meassage in console
  int speedIndex = 0;               //used for linearBool and linearSpeed array index 
  trackslow_allowed = true;

  // in case of docking or undocking - check if trackslow is allowed
  if ( maps.isUndocking() || maps.isDocking() ) {
    static float dockX = 0;
    static float dockY = 0;
    static float dockDelta = 0;
    maps.getDockingPos(dockX, dockY, dockDelta);
    // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
    if (distance(dockX, dockY, stateX, stateY) > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
      trackslow_allowed = false;
    }
  }

  linear = setSpeed;                //always compare speeds against desired setSpeed 
  
  //which states apply?
  linearBool[0] = (gps.solution == SOL_FLOAT);                                                      // [0] FLOATSPEED
  linearBool[1] = (targetDist < NEARWAYPOINTDISTANCE || lastTargetDist < NEARWAYPOINTDISTANCE);     // [1] NEARWAYPOINTSPEED
  linearBool[2] = (sonar.nearObstacle());                                                           // [2] SONARSPEED
  linearBool[3] = (motor.motorLeftOverload || motor.motorRightOverload || motor.motorMowOverload);  // [3] OVERLOADSPEED
  linearBool[4] = (motor.keepslow);                                                                 // [4] KEEPSLOWSPEED
  linearBool[5] = (motor.retryslow);                                                                // [5] RETRYSLOWSPEED
  linearBool[6] = (maps.trackSlow && trackslow_allowed);                                            // [6] TRACKSLOWSPEED
  linearBool[7] = (dockTimer);                                                                      // [7] DOCK_NO_ROTATION_SPEED

  //disable near way point speed if we use the distance ramp
  if (DISTANCE_RAMP) linearBool[1] = false;

  //choose the lowest speed of the states
  for(speedIndex = 0; speedIndex < aLen; speedIndex ++){
    if (linearBool[speedIndex] == true){
      if (linearSpeed[speedIndex] < linear){
        linear = linearSpeed[speedIndex];
        chosenIndex = speedIndex;
      }
    }
  }

  //trigger a message if speed changes
  if (chosenIndex != chosenIndexl){
    CONSOLE.print("Linetracker.cpp - linearSpeedState(): ");
    CONSOLE.print(linearSpeedNames[chosenIndex]);
    CONSOLE.print(" = ");
    CONSOLE.print(linearSpeed[chosenIndex]);
    CONSOLE.println(" m/s");
  }

  //consider the distance ramp wih the chosen speed if we are approaching or leaving a waypoint
  if (DISTANCE_RAMP) {
    if (targetDist < 2 * NEARWAYPOINTDISTANCE || lastTargetDist < 2 * NEARWAYPOINTDISTANCE) {
      linear = distanceRamp(linear);
    }
  }

  chosenIndexl = chosenIndex;

  if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
}

float distanceRamp(float linear){
    float maxSpeed = linear*1000;
    float minSpeed = 2*MOTOR_MIN_SPEED * 1000;
    float maxDist = (linear * NEARWAYPOINTDISTANCE /setSpeed) * 1000;     //if we are going slow for example because of float, the ramp will kick in when mower is nearer to point
    float minDist = 0;                                                    //TARGET_REACHED_TOLERANCE*1000;
    float actDist = 0;
    float rampSpeed = 0;
    static bool wasStraight;

    if (targetDist <= lastTargetDist) {                                  //need to decide what ramp, leaving or aproaching? --> approaching
      maxDist += maxSpeed;                                               //add an speed dependent offset to target distance when approaching, because mower comes with high speed that causes a timing issue
      actDist = targetDist;
      if (straight) minSpeed = maxSpeed * 0.5;                           //if we don´t need to rotate, do not decellarate too much
      wasStraight = straight;
    } else {
      if (wasStraight) minSpeed = maxSpeed * 0.5;
      actDist = lastTargetDist;
      //actDist += 0.05; //add an offset      
    }

    actDist *= 1000;

    if (targetDist + lastTargetDist < maxDist) { //points are not far away from each other
      actDist *= 2;                              //multiply the actDist to trick the map function (hurryup because we wont reach full speed anyway)
    }

    rampSpeed = map(actDist, minDist, maxDist, minSpeed, maxSpeed);
    rampSpeed = constrain(rampSpeed, minSpeed, maxSpeed);
    rampSpeed /= 1000;
    //CONSOLE.print(straight); CONSOLE.print(" "); CONSOLE.println(rampSpeed);
    return rampSpeed;
}

void gpsRebootDock() {
  // reboot gps by undocking at a specified docking point (please see "DOCK_POINT_GPS_REBOOT" in config.h) //SOew
  if (maps.wayMode == WAY_MOW) dockGpsRebootState = 0; //MrTree searching for bug....
  if (dockGpsRebootState == 0){  // status dockGpsReboot: 0= off, 1= reset gps, 2= wait for gps-fix, 3= check for stable gps-fix
    counterCheckPos = 0;
  } else if (maps.wayMode == WAY_DOCK){ //MrTree only step in if waymode is dock ....searching for bug
    switch (dockGpsRebootState) {

      case 1:
        // reboot gps to get new GPS fix
        CONSOLE.println("LineTracker.cpp  dockGpsRebootState - start gps-reboot or waiting for FIX if GPS_REBOOT = false");
        if (GPS_REBOOT){
          gps.reboot();   // reboot gps to get new GPS fix
          dockGpsRebootTime = millis() + GPSWAITREBOOT; // load check timer for gps-fix with 10sec //MrTree moved to config.h
        } else {
          dockGpsRebootTime = millis()+2000;
        }
        
        dockGpsRebootFixCounter = 0;
        dockGpsRebootState = 2;
        //if ((maps.wayMode == WAY_FREE) || (maps.wayMode == WAY_MOW))dockGpsRebootState = 0; //MrTree changed to force finish step if waymode is free or mow
        break;

      case 2:
        // wait for gps-fix solution
        if (dockGpsRebootTime <= millis()) {
          if (gps.solution == SOL_FIXED) {
            //     maps.setLastTargetPoint(stateX, stateY);  // Manipulate last target point to avoid "KIDNAP DETECT"
            dockGpsRebootState = 3;
            dockGpsRebootFeedbackTimer  = millis();
            dockGpsRebootTime = millis(); // load check timer for stable gps-fix
            dockGpsRebootDistGpsTrg = false; // reset trigger
            CONSOLE.print("LineTracker.cpp  dockGpsRebootState - got gps-fix after ");
            CONSOLE.print(dockGpsRebootFixCounter);
            CONSOLE.println(" sec");
          }
          else {
            dockGpsRebootTime += 10000; // load check timer for gps-fix with 10sec
            dockGpsRebootFixCounter += 10;  // add 10 seconds
            if (!buzzer.isPlaying()) buzzer.sound(SND_TILT, true);
            CONSOLE.print("LineTracker.cpp  dockGpsRebootState - still no gps-fix after ");
            CONSOLE.print(dockGpsRebootFixCounter);
            CONSOLE.println(" sec");
          }
        }
        break;

      case 3:
        // wait if gps-fix position stays stable for at least GPS_STABLETIME
        if ((gps.solution == SOL_FIXED) && (millis() - dockGpsRebootTime > GPS_STABLETIME)) { //MrTree changed to variable Time
          // zur Sicherheit wird mind. 2 mal die Position kontrolliert
          if (counterCheckPos < 3) {
            if (counterCheckPos == 0) {
              stateX_1 = stateX;
              stateY_1 = stateY;
              counterCheckPos += 1;
              //dockGpsRebootState = 1; // erneuten reboot einleiten
              CONSOLE.print("LineTracker.cpp  1. gps-fix pos  x = ");
              CONSOLE.print(stateX_1);
              CONSOLE.print(" y = ");
              CONSOLE.println(stateY_1);
            } else if (counterCheckPos == 1) {
              stateX_2 = stateX;
              stateY_2 = stateY;
              counterCheckPos += 1;
              if ((fabs(stateX_1 - stateX_2) <= 0.05) && (fabs(stateY_1 - stateY_2) <= 0.05)) {
                dockGpsRebootState = 4; // sicherer Fix
              } else {
                //dockGpsRebootState = 1; // erneuten reboot einleiten
              }
              CONSOLE.print("LineTracker.cpp  2. gps-fix pos  x = ");
              CONSOLE.print(stateX_2);
              CONSOLE.print(" y = ");
              CONSOLE.println(stateY_2);
            } else if (counterCheckPos == 2) {
              stateX_3 = stateX;
              stateY_3 = stateY;
              counterCheckPos += 1;
              if (((fabs(stateX_1 - stateX_3) <= 0.05) && (fabs(stateY_1 - stateY_3) <= 0.05)) ||
                  ((fabs(stateX_2 - stateX_3) <= 0.05) && (fabs(stateY_2 - stateY_3) <= 0.05))) {
                dockGpsRebootState = 4; // sicherer Fix
              } else {
                counterCheckPos = 0;  // reset counter
                //dockGpsRebootState = 1; // erneuten reboot einleiten
              }
              CONSOLE.print("LineTracker.cpp  3. gps-fix pos  x = ");
              CONSOLE.print(stateX_3);
              CONSOLE.print(" y = ");
              CONSOLE.println(stateY_3);
            }
          }
        }
        if (gps.solution != SOL_FIXED) dockGpsRebootState = 2; // wait for gps-fix again
        if (dockGpsRebootDistGpsTrg == true) { // gps position is changing to much
          dockGpsRebootDistGpsTrg = false; // reset trigger
          dockGpsRebootTime = millis();
          CONSOLE.print("LineTracker.cpp  dockGpsRebootState - gps-pos is moving; timereset after");
          CONSOLE.print((millis() - dockGpsRebootTime));
          CONSOLE.println("msec");
          if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);
        }
        if (dockGpsRebootFeedbackTimer <= millis()) {
          dockGpsRebootFeedbackTimer = millis() + 5000;
          if (!buzzer.isPlaying()) buzzer.sound(SND_READY, true);
        }
        break;

      case 4:
        dockGpsRebootState      = 0; // finished
        CONSOLE.println("LineTracker.cpp  dockGpsRebootState - gps-pos is stable; continue undocking/docking;");
        break;

      case 10:
        // Wenn ohne GPS fix oder float das undocking gestartet wird, muss bei erreichen von fix oder float die "linearMotionStartTime" resetet werden, um "gps no speed => obstacle!" zu vermeiden
        //resetLinearMotionMeasurement();
        break;

    } // switch (dockGpsRebootState)
    if (dockGpsRebootState < 10) {
      // stop mower
      linear = 0;
      angular = 0;
      mow = false;
    }
  }
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl) {
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  CurrSpeed = motor.linearSpeedSet;           //MrTree take the real speed from motor.linearSpeedSet
  linear = 0;                                 //MrTree Changed from 1.0
  angular = 0;
  mow = false;                                //MrTree changed to false

  if (MOW_START_AT_WAYMOW &! oneTrigger) {                                                             
    if (maps.wayMode == WAY_MOW) {            //MrTree do not activate mow until there is a first waymow 
      mow = true;                             //MrTree this will only work directly after undocking and way free, the first time it is in waymow, mow will be true forever like before     
      oneTrigger = true;
    }                                              
  } else {
    mow = true;                               //MrTree --> original condition, mow will be true here and is maybe changed by a condition later in linetracker
  }

  if (stateOp == OP_DOCK || maps.shouldDock == true) {
    mow = false;
    oneTrigger = false;
  }
  
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
   else targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  targetDist = maps.distanceToTargetPoint(stateX, stateY);
  lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);
  targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  
  if (!AngleToTargetFits()) { 
    rotateToTarget();
  } else {
    linearSpeedState();  //compares the linear Speed to use according to configured mower state
    stanleyTracker();
  }
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0) {
    if (millis() > lastFixTime + fixTimeout * 1000.0) {
      activeOp->onGpsFixTimeout();
    }
  }

  if (gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT){
    warnDockWithoutGpsTrg = false;    // Svol0: reset warnmessage trigger
    if (fabs(CurrSpeed) >= MOTOR_MIN_SPEED) {                                        //MrTree
      if ((millis() > linearMotionStartTime + GPS_NO_SPEED_TIME) && (stateGroundSpeed < (MOTOR_MIN_SPEED * 0.5))) { 
        // if in linear motion and not enough ground speed => obstacle
        if (GPS_SPEED_DETECTION && !allowDockLastPointWithoutGPS) { 
          CONSOLE.println("gps no speed => obstacle!");
          statMowGPSNoSpeedCounter++;
          triggerObstacle();
          return;
        }
      }
    }
  } else {
    // no gps solution
    if (REQUIRE_VALID_GPS) {
      //MrTree: using Svol0´s solution
      // Svol0: continue docking if gps solution gets lost by driving to the last point (normal if dockingstation is under a roof)
      if (allowDockLastPointWithoutGPS == true) {
        if (!warnDockWithoutGpsTrg) {
          CONSOLE.println("LineTracker.cpp WARN: Continue docking with no gps solution!");
          warnDockWithoutGpsTrg = true;
        }
      } else {
        if (!warnDockWithoutGpsTrg) {
          CONSOLE.println("LineTracker.cpp WARN: no gps solution!");
          warnDockWithoutGpsTrg = true;
        }
        if (dockGpsRebootState == 0) activeOp->onGpsNoSignal(); //MrTree workaround for undock issue when svolo´s code will be cought in a loop resetting the gps reveiver
      }
    }
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT) {
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;
    if ( maps.isUndocking() || maps.isDocking() ) {
      float dockX = 0;
      float dockY = 0;
      float dockDelta = 0;
      maps.getDockingPos(dockX, dockY, dockDelta);
      float dist = distance(dockX, dockY, stateX, stateY);
      // check if current distance to docking station is below
      // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
      if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
        allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
      }
    }// MrTree integrated with keeping new sunray code Svol0: changed for GPS-Reboot at a
    if ((fabs(distToPath) > allowedPathTolerance) && (!blockKidnapByUndocking)) { // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped) {
        stateKidnapped = true;
        activeOp->onKidnapped(stateKidnapped);
      }
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        activeOp->onKidnapped(stateKidnapped);
      }
    }
  }


  gpsRebootDock();

  if (!allowDockRotation && !maps.isUndocking()){        //MrTree step in algorithm if allowDockRotation (computed in maps.cpp) is false and mower is not undocking
    if (!dockTimer){                                                  //set helper bool to start a timer and print info once
      reachedPointBeforeDockTime = millis();                          //start a timer when going to last dockpoint
      dockTimer = true;                                               //enables following code
      CONSOLE.println("allowDockRotation = false, timer to successfully dock startet. angular = 0, turning not allowed");
    }
    if (dockTimer){
      resetLinearMotionMeasurement();                                         //need to test if this is still neccessary
      if (targetDist < DOCK_NO_ROTATION_DISTANCE) angular = 0;                  //testing easier approach for DOCK_NO_ROTATION setup
      if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){      //check the time until mower has to reach the charger and triger obstacle if not reached
        CONSOLE.println("allowDockRotation = false, not docked in given time, triggering maps.retryDocking!");
        triggerObstacle();
        dockTimer = false;     
      } 
    }
  } else {
      dockTimer = false;     
  }

  if (runControl) {

    if (angleToTargetFits != langleToTargetFits && DEBUG_LOG) {
      CONSOLE.print("Linetracker.cpp angular: ");
      CONSOLE.println(angular*180.0/PI);
      //CONSOLE.print("angleToTargetFits: ");
      //CONSOLE.print(angleToTargetFits);
      //CONSOLE.print(" trackerDiffDelta: ");
      //CONSOLE.println(trackerDiffDelta);
      langleToTargetFits = angleToTargetFits;
    }

    shouldRotate = robotShouldRotate();
    if (shouldRotate != shouldRotatel && DEBUG_LOG){
      CONSOLE.print("Linetracker.cpp ShouldRotate = ");
      CONSOLE.println(shouldRotate);
      shouldRotatel = shouldRotate;
    }

    if (detectLift()){ // in any case, turn off mower motor if lifted  
      mow = false;  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
      linear = 0;
      angular = 0; 
    }

    if (mow != motor.switchedOn && motor.enableMowMotor){
      if (DEBUG_LOG) {
        CONSOLE.print("Linetracker.cpp changes mow status: ");
        CONSOLE.println(mow);
      }
      motor.setMowState(mow); 
    }

    motor.setLinearAngularSpeed(linear, angular, true);    
  }

  x_new = target.x();
  y_new = target.y();

  if ((x_old != x_new || y_old != y_new) && DEBUG_LOG){
    CONSOLE.print("LineTracker.cpp targetPoint  x = ");
    CONSOLE.print(x_new);
    CONSOLE.print(" y = ");
    CONSOLE.println(y_new);
    x_old = x_new;
    y_old = y_new;
  }
  
  if (targetReached) {
    activeOp->onTargetReached();
    straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false, stateX, stateY)) {
      // finish
      activeOp->onNoFurtherWaypoints();
    }
  }
}
