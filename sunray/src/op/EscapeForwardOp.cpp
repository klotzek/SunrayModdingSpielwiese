// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../config.h" //MrTree

String EscapeForwardOp::name(){
    return "EscapeForward";
}

void EscapeForwardOp::begin(){
    // rotate stuck avoidance
    driveForwardStopTime = millis() + (OBSTACLEAVOIDANCEWAY/OBSTACLEAVOIDANCESPEED*1000); 												//MrTree just add a constant time?
	//if ((!MOWMOTORSTOPONOBSTACLE) && (previousOp == &mowOp)) {
	//  if (!motor.switchedOn) {
	//	  CONSOLE.println("EscapeForwardOp:: Overriding mowmotor stop! (MOWMOTORSTOPONOBSTACLE = false)");
	//	  motor.setMowState(true);	
	//	}
	//} else {																		//MrTree
	//  if (motor.switchedOn) {
	//	CONSOLE.println("EscapeForwardOp:: Switch Off mow");
	//	motor.setMowState(false);  																	//MrTree  	
	//  }														//MrTree              				
	//}	
}


void EscapeForwardOp::end(){
}


void EscapeForwardOp::run(){
    battery.resetIdle();
	//if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME){
       // wait until mowing motor is running
    //   if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
    //    motor.setLinearAngularSpeed(0,0,false);
	//	driveForwardStopTime = millis() + (OBSTACLEAVOIDANCEWAY/OBSTACLEAVOIDANCESPEED*1000);
    //  }
	//else {
    motor.setLinearAngularSpeed(OBSTACLEAVOIDANCESPEED,0,true);	//MrTree
 	//}						
    if ((DISABLE_MOW_MOTOR_AT_OBSTACLE) && (motor.switchedOn)) {	//MrTree
      CONSOLE.println("EscapeForwardOp:: Switch Off mow");			//MrTree
	  motor.setMowState(false);  																	  	
	}  																							
											
    if (millis() > driveForwardStopTime){
        CONSOLE.println("driveForwardStopTime");
        motor.setLinearAngularSpeed(0,0,true);  //MrTree changed
		//motor.stopImmediately(false);  
        driveForwardStopTime = 0;
        /*maps.addObstacle(stateX, stateY);
        Point pt;
        if (!maps.findObstacleSafeMowPoint(pt)){
        setOperation(OP_DOCK, true); // dock if no more (valid) mowing points
        } else*/ 
        changeOp(*nextOp);    // continue current operation              
    }
}

void EscapeForwardOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeForwardOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}
