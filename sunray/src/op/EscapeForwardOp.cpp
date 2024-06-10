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
    resetAngularMotionMeasurement();                                                        // we couldnt´t rotate, so we reset the measurement
    if (escapeForwardCounter == 0)	escapeForwardStartTime = millis();   					// set triggered time on entrance if escapeForwardCounter successfully resets	
	escapeForwardCounter++;                                                                 //MrTree iterate counter
    if (((escapeForwardStartTime + 10000) < millis()) && (escapeForwardCounter <= 3)) escapeForwardCounter = 0;		//reset counter if escapeForward succeded without too many triggers in given time
    driveForwardStopTime = millis() + (OBSTACLEAVOIDANCEWAY/OBSTACLEAVOIDANCESPEED*1000); 	//MrTree just add a constant time to compensate offsets?
	//if ((!MOWMOTORSTOPONOBSTACLE) && (previousOp == &mowOp)) {
	//  if (!motor.switchedOn) {
	//	  CONSOLE.println("EscapeForwardOp:: Overriding mowmotor stop! (MOWMOTORSTOPONOBSTACLE = false)");
	//	  motor.setMowState(true);	
	//	}
	//} else {																		        //MrTree
	//  if (motor.switchedOn) {
	//	CONSOLE.println("EscapeForwardOp:: Switch Off mow");
	//	motor.setMowState(false);  															//MrTree  	
	//  }														                            //MrTree              				
	//}	
}


void EscapeForwardOp::end(){
}


void EscapeForwardOp::run(){
    battery.resetIdle();
	/*
    if (escapeForwardCounter == 3) {
	    CONSOLE.println("EscapeForwardOp:: too many retries for escapeForward op, assuming obstacle in front: changeOp-->escapeReverseOp");	      
        resetAngularMotionMeasurement();
        //escapeForwardCounter++;
        //CONSOLE.println(escapeForwardCounter);
        changeOp(escapeReverseOp, false);
        //changeOp(*nextOp, false);
		//escapeForwardCounter = 0;
		
	} 
    if (escapeForwardCounter > 3) {
		CONSOLE.println("EscapeForwardOp:: Error: too many retries in configured time (ESCAPELAWNTIMER)");
		stateSensor = SENS_OBSTACLE;            
        changeOp(errorOp);
		escapeForwardCounter = 0;
		return;
	}
	*/
    motor.setLinearAngularSpeed(OBSTACLEAVOIDANCESPEED,0,true);	    //MrTree						
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
