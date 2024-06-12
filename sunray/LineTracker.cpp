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


//PID pidLine(0.2, 0.01, 0); // not used
//PID pidAngle(2, 0.1, 0);  // not used
Polygon circle(8);

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

float targetDist = 0;     //MrTree
float lastTargetDist = 0; //MrTree

float setSpeed = 0.1; // linear speed (m/s)
float lastSpeed = 0; //MrTree

float x_old = 0;
float y_old = 0;
float x_new = 0;
float y_new = 0;
Point lastPoint;

Point last_rotation_target;
bool rotateLeft = false;
bool rotateRight = false;
bool shouldRotate = false;              //MrTree
bool shouldRotatel = false;             //MrTree
bool angleToTargetFits = false;
bool angleToTargetPrecise = true;
bool langleToTargetFits = false;
bool targetReached = false;
float trackerDiffDelta = 0;
bool stateKidnapped = false;
unsigned long reachedPointBeforeDockTime = 0;               //MrTree
bool dockTimer = false;                                     //MrTree
int dockGpsRebootState;                   // Svol0: status for gps-reboot at specified docking point by undocking action
bool blockKidnapByUndocking;              // Svol0: kidnap detection is blocked by undocking without gps
unsigned long dockGpsRebootTime;          // Svol0: retry timer for gps-fix after gps-reboot
unsigned long dockGpsRebootFixCounter;    // Svol0: waitingtime for fix after gps-reboot
unsigned long dockGpsRebootFeedbackTimer; // Svol0: timer to generate acustic feedback
bool dockGpsRebootDistGpsTrg = false;     // Svol0: trigger to check solid gps-fix position (no jump)
bool allowDockLastPointWithoutGPS = false;  // Svol0: allow go on docking by loosing gps fix
bool allowDockRotation = true;               //MrTree: disable rotation on last dockingpoint
bool warnDockWithoutGpsTrg = false;            // Svol0: Trigger for warnmessage
float stateX_1 = 0;                       // Svol0
float stateY_1 = 0;                       // Svol0
float stateX_2 = 0;                       // Svol0
float stateY_2 = 0;                       // Svol0
float stateX_3 = 0;                       // Svol0
float stateY_3 = 0;                       // Svol0
int counterCheckPos = 0;  // check if gps position is reliable

bool printmotoroverload = false;
bool trackerDiffDelta_positive = false;

int get_turn_direction_preference() {
  Point target = maps.targetPoint;
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  float center_x = stateX;
  float center_y = stateY;
  float r = (MOWER_SIZE / 100);
  float cur_angle = stateDelta;

  if (FREEWHEEL_IS_AT_BACKSIDE) {
    cur_angle = scalePI(stateDelta + PI);
    targetDelta = scalePI(targetDelta + PI);
  }

  // create circle / octagon around center angle 0 - "360"
  circle.points[0].setXY(center_x + cos(deg2rad(0)) * r, center_y + sin(deg2rad(0)) * r);
  circle.points[1].setXY(center_x + cos(deg2rad(45)) * r, center_y + sin(deg2rad(45)) * r);
  circle.points[2].setXY(center_x + cos(deg2rad(90)) * r, center_y + sin(deg2rad(90)) * r);
  circle.points[3].setXY(center_x + cos(deg2rad(135)) * r, center_y + sin(deg2rad(135)) * r);
  circle.points[4].setXY(center_x + cos(deg2rad(180)) * r, center_y + sin(deg2rad(180)) * r);
  circle.points[5].setXY(center_x + cos(deg2rad(225)) * r, center_y + sin(deg2rad(225)) * r);
  circle.points[6].setXY(center_x + cos(deg2rad(270)) * r, center_y + sin(deg2rad(270)) * r);
  circle.points[7].setXY(center_x + cos(deg2rad(315)) * r, center_y + sin(deg2rad(315)) * r);

  // CONSOLE.print("get_turn_direction_preference: ");
  // CONSOLE.print(" pos: ");
  // CONSOLE.print(stateX);
  // CONSOLE.print("/");
  // CONSOLE.print(stateY);
  // CONSOLE.print(" stateDelta: ");
  // CONSOLE.print(cur_angle);
  // CONSOLE.print(" targetDelta: ");
  // CONSOLE.println(targetDelta);
  int right = 0;
  int left = 0;
  for (int i = 0; i < circle.numPoints; ++i) {
    float angle = pointsAngle(stateX, stateY, circle.points[i].x(), circle.points[i].y());
    // CONSOLE.print(angle);
    // CONSOLE.print(" ");
    // CONSOLE.print(i);
    // CONSOLE.print(": ");
    // CONSOLE.print(circle.points[i].x());
    // CONSOLE.print("/");
    // CONSOLE.println(circle.points[i].y());
    if (maps.checkpoint(circle.points[i].x(), circle.points[i].y())) {

      // skip points in front of us
      if (fabs(angle - cur_angle) < 0.05) {
        continue;
      }

      if (cur_angle < targetDelta) {
        if (angle >= cur_angle && angle <= targetDelta) {
          left++;
        } else {
          right++;
        }
      } else {
        if (angle <= cur_angle && angle >= targetDelta) {
          right++;
        } else {
          left++;
        }
      }
    }
  }
  // CONSOLE.print("left/right: ");
  // CONSOLE.print(left);
  // CONSOLE.print("/");
  // CONSOLE.println(right);

  if (right == left) {
    return 0;
  }

  if (right < left) {
    return 1;
  }

  return -1;
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl) {
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float CurrSpeed = motor.linearSpeedSet;       //MrTree
  float linear = 0;                           //MrTree Changed from 1.0
  bool mow = true;
  if ((stateOp == OP_DOCK) || (maps.shouldDock == true)) mow = false;
  bool trackslow_allowed = false; //MrTree: definition moved up in code
  float angular = 0;
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  //float targetDelta = pointsAngle(lastTarget.x(), lastTarget.y(), target.x(), target.y()); //bugfix
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);

  // see what speed we have and reset linearmotion timer
  if (fabs(CurrSpeed) < MOTOR_MIN_SPEED)resetLinearMotionMeasurement(); //MrTree
  
  // limitation for setSpeed //SOew                           //MrTree (svol0)
  if (setSpeed > MOTOR_MAX_SPEED) setSpeed = MOTOR_MAX_SPEED; //MrTree

  lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);   //MrTree (changed varscope)
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);
  else
    targetReached = (targetDist < TARGET_REACHED_TOLERANCE);

  if ( (last_rotation_target.x() != target.x() || last_rotation_target.y() != target.y()) &&
       (rotateLeft || rotateRight ) ) {
    // CONSOLE.println("reset left / right rot (target point changed)");
    rotateLeft = false;
    rotateRight = false;
  }

  // allow rotations only near last or next waypoint or if too far away from path
  // it might race between rotating mower and targetDist check below
  // if we race we still have rotateLeft or rotateRight true
  if ( (targetDist < 0.3) || (lastTargetDist < 0.3) || (fabs(distToPath) > 1.0) ||
       rotateLeft || rotateRight ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta) / PI * 180.0 < 120);
    else
      angleToTargetFits = (fabs(trackerDiffDelta) / PI * 180.0 < 20);  //MrTree we have more than 20deg difference to point
  } else {
	// while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
  //  angleToTargetFits = true;
  }

  if ((!angleToTargetFits || !angleToTargetPrecise) && !dockTimer) {   //MrTree added !dockTimer, !angleToTargetPrecise
    // angular control (if angle to far away, rotate to next waypoint)
    if (!angleToTargetFits) angleToTargetPrecise = false;
    resetLinearMotionMeasurement();                                     //MrTree added function call from svol0
    linear = 0;                                                       //MrTree while turning from >= 20/45 deg difference, linear is 0... still decelerating or accelerating on stepin/out
    if (((maps.isDocking()) || (maps.isUndocking())) && 
         ((maps.trackSlow) && (trackslow_allowed))) {
      angular = DOCKANGULARSPEED / 180.0 * PI;  //MrTree use DOCKANGULARSPEED in config.h, added trackslowallowed  : RTT=29deg/s=0.5 rad/s;  
    } else {
        if (fabs(trackerDiffDelta)/PI*180.0 >= ANGLEDIFF1) angular = ROTATETOTARGETSPEED1 / 180.0 * PI;  //MrTree set angular to fast defined in config.h
        if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEDIFF1) angular = ROTATETOTARGETSPEED2  / 180.0 * PI;    //MrTree slow down turning when near desired angle     
        if (fabs(trackerDiffDelta)/PI*180.0 <= ANGLEDIFF2) angular = ROTATETOTARGETSPEED3 / 180.0 * PI;    //MrTree slow down turning even more when almost at desired angle     
    }
    if (trackerDiffDelta < 0) {     //MrTree set rotation direction and do not keep it :)
      rotateLeft = true;
      rotateRight = false;
    } else { 
      rotateRight = true;
      rotateLeft = false;
    }                   
    if (rotateLeft) angular *= -1;  
    if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEPRECISE){          
      rotateLeft = false;  // reset rotate direction
      rotateRight = false;
      angleToTargetPrecise = true;                    //MrTree Step out of everything when angle is precise...
      angular = 0;
    } 
    if (fabs(CurrSpeed) >= 0.1) angular = 0;       //MrTree reset angular if current speed is over given value (still deccelerating)
  } 
  else {
    // line control (stanley)
    bool straight = maps.nextPointIsStraight();
    //bool trackslow_allowed = true; //MrTree: definition moved up in code
    trackslow_allowed = true;

    rotateLeft = false;
    rotateRight = false;

    // in case of docking or undocking - check if trackslow is allowed
    if ( maps.isUndocking() || maps.isDocking() ) {
      float dockX = 0;
      float dockY = 0;
      float dockDelta = 0;
      maps.getDockingPos(dockX, dockY, dockDelta);
      float dist_dock = distance(dockX, dockY, stateX, stateY);
      // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
      if (dist_dock > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
        trackslow_allowed = false;
      }
    }

    if (maps.trackSlow && trackslow_allowed) {
      // planner forces slow tracking (e.g. docking etc)
      if (dockTimer) linear = DOCK_NO_ROTATION_SPEED;
      else linear = TRACKSLOWSPEED;                                                                                                    //MrTree, use TRACKSLOW in config.h
    } else if //(     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < NEARWAYPOINTDISTANCE) && (!straight))      //MrTree approaching/leaving NEARWAYPOINTDISTANCE in config.h
              //      || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + NEARWAYPOINTDISTANCE / NEARWAYPOINTSPEED * 1000)) //MrTree leaving Waypoint with Speedset to reach the NEARWAYPOINTDISTANCE ... besser lasttargetdistance verwenden!?
              //)
              ((targetDist < NEARWAYPOINTDISTANCE) || (lastTargetDist < NEARWAYPOINTDISTANCE))
    {
      linear = NEARWAYPOINTSPEED; //MrTree reduce speed when approaching/leaving waypoints with NEARWAYPOINTSPEED in config.h
    }
    else {
      if (gps.solution == SOL_FLOAT)
        linear = min(setSpeed, FLOATSPEED); //MrTree reduce speed for float solution with config.h FLOATSPEED
      else
        linear = setSpeed;         // desired speed
      if (sonar.nearObstacle()) linear = SONARSPEED;  //MrTree slow down near obstacles with config.h SONARSPEED
      if (motor.keepslow) linear = KEEPSLOWSPEED;     //MrTree slow down because of high grass with config.h KEEPSLOWSPEED
      if (motor.retryslow) linear = RETRYSLOWSPEED;   //MrTree slow down because mowmotor stalled with config.h RETRYSLOWSPEED
    }  
    // slow down speed in case of overload and overwrite all prior speed
    if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ) {
      if (!printmotoroverload) {
        CONSOLE.println("motor overload detected: reduce linear speed to OVERLOADSPEED");
      }
      printmotoroverload = true;
      linear = OVERLOADSPEED;  //MrTree continue with config.h OVERLOADSPEED
    } else {
      printmotoroverload = false;
    }

    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors
    //Stanley parameters
    float k = 0;
    float p = 0;

    if (MAP_STANLEY_CONTROL == true) {
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
      k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
      p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;
      if (maps.trackSlow && trackslow_allowed) {
        k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;
        p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;
      }
    }                                                                                                                             //MrTree
    angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(CurrSpeed)));       //MrTree, use actual speed correct for path errors
    /*pidLine.w = 0;
      pidLine.x = lateralError;
      pidLine.max_output = PI;
      pidLine.y_min = -PI;
      pidLine.y_max = PI;
      pidLine.compute();
      angular = -pidLine.y;   */
    //CONSOLE.print(lateralError);
    //CONSOLE.print(",");
    //CONSOLE.println(angular/PI*180.0);
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
    if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); //MrTree still used here because of gpsfix jumps
  }
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0) {
    if (millis() > lastFixTime + fixTimeout * 1000.0) {
      activeOp->onGpsFixTimeout();
    }
  }

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT))  {                     //MrTree Bugged, slow movements will entirely be ignored....
    warnDockWithoutGpsTrg = false;    // Svol0: reset warnmessage trigger
    if (fabs(CurrSpeed) >= MOTOR_MIN_SPEED) {                                                        //MrTree (changed: linear to CurrSpeed <= MinSpeedVal (works without compiler warning)) //origin: does not consider Mow Spinuptime. There will be an GPS Obstacle on the Start if long Spinup is done, linear is set before actually moving??? Need to change to actual input/output of function motor.setlinearangularspeed
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < (MOTOR_MIN_SPEED * 0.5))) { //Stateoverground < to half of Currspeed
        // if in linear motion and not enough ground speed => obstacle
        //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) {
        if (GPS_SPEED_DETECTION && !allowDockLastPointWithoutGPS) { 
          CONSOLE.println("gps no speed => obstacle!");
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

  // reboot gps by undocking at a specified docking point (please see "DOCK_POINT_GPS_REBOOT" in config.h) //SOew
  if (maps.wayMode == WAY_MOW)dockGpsRebootState = 0; //MrTree searching for bug....
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
        //     blockKidnapByUndocking  = false;  // enable Kidnap detection
        //    maps.setLastTargetPoint(stateX, stateY);  // Manipulate last target point to avoid "KIDNAP DETECT"
        CONSOLE.println("LineTracker.cpp  dockGpsRebootState - gps-pos is stable; continue undocking/docking;");

        break;

      case 10:
        // Wenn ohne GPS fix oder float das undocking gestartet wird, muss bei erreichen von fix oder float die "linearMotionStartTime" resetet werden, um "gps no speed => obstacle!" zu vermeiden
        resetLinearMotionMeasurement();
        break;

    } // switch (dockGpsRebootState)
    if (dockGpsRebootState < 10) {
      // stop mower
      linear = 0;
      angular = 0;
      mow = false;
    }
  } //if (dockGpsRebootState > 0)

  if ((allowDockRotation == false) && (!maps.isUndocking())) {       //MrTree
    if (!dockTimer){
      reachedPointBeforeDockTime = millis(); //start a timer when going to last dockpoint
      dockTimer = true;
      CONSOLE.println("allowDockRotation = false, timer to successfully dock startet. angular = 0, turning not allowed");
    }
    if (dockTimer){
      resetLinearMotionMeasurement(); //keep 0
      if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_DELAY) {
        angular = 0; //stop angular after last point passed
      }
      if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){
        CONSOLE.println("allowDockRotation = false, not docked in given time, triggering maps.retryDocking!");
        triggerObstacle();
        //maps.retryDocking(stateX, stateY);  //give 10 secs time to reach dock, or retry- Note: if !angletotargetfits robot will just stop and sit there, which is good, becouse it wouldnt have made it anyways
        dockTimer = false;     
      }
      
    }
    //if ((targetReached) && (!battery.chargerConnected()))maps.retryDocking(stateX, stateY);
    
    //else if target not reached and standing still....... after a certain time, trigger retrydock
  } else {
      dockTimer = false;     
  }

  if (runControl) {

    if (angleToTargetFits != langleToTargetFits) {
      CONSOLE.print("Linetracker.cpp angular: ");
      CONSOLE.println(angular*180.0/PI);
      //CONSOLE.print("angleToTargetFits: ");
      //CONSOLE.print(angleToTargetFits);
      //CONSOLE.print(" trackerDiffDelta: ");
      //CONSOLE.println(trackerDiffDelta);
      langleToTargetFits = angleToTargetFits;
    }
    if (linear != lastSpeed){ 
        CONSOLE.print("Linetracker.cpp linear: ");
        CONSOLE.println(linear);        
    }
    lastSpeed = linear;
    
    shouldRotate = robotShouldRotate();
    if (shouldRotate != shouldRotatel){
      CONSOLE.print("Linetracker.cpp ShouldRotate = ");
      CONSOLE.println(shouldRotate);
      shouldRotatel = shouldRotate;
    }
    

    if ((mow != motor.switchedOn) && (motor.enableMowMotor)){
      CONSOLE.print("Linetracker.cpp changes mow status: ");
      CONSOLE.println(mow);
      motor.setMowState(mow); 
    }
    // in any case, turn off mower motor if lifted 
    // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
    if (detectLift()) {
      mow = false;
      linear = 0;
      angular = 0; 
    }
    if (mow)  {      
      if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME){
       // wait until mowing motor is running
       if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
       //CONSOLE.println("linetracker.cpp trying to wait for mowmotor....");
        linear = 0;
        angular = 0; 
      }
    }

    motor.setLinearAngularSpeed(linear, angular);    
  }

  x_new = target.x();
  y_new = target.y();

  if ((x_old != x_new) || (y_old != y_new)){
  CONSOLE.print("LineTracker.cpp targetPoint  x = ");
  CONSOLE.print(x_new);
  CONSOLE.print(" y = ");
  CONSOLE.println(y_new);
  }
  x_old = x_new;
  y_old = y_new;
  
  if (targetReached) {
    
    rotateLeft = false;
    rotateRight = false;
    activeOp->onTargetReached();
    bool straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false, stateX, stateY)) {
      // finish
      activeOp->onNoFurtherWaypoints();
    } else {
      // next waypoint
      //if (!straight) angleToTargetFits = false;
    }
  }
}
