// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"
#include "../../config.h"


String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin(){
    // obstacle avoidance
    driveReverseStopTime = millis() + (OBSTACLEAVOIDANCEWAY/OBSTACLEAVOIDANCESPEED*1000); //+500); 	//MrTree just take a deadtime of 500ms to compensate deadtimes
	//if ((DISABLE_MOW_MOTOR_AT_OBSTACLE) && (previousOp == &mowOp)) {
	//	if (!motor.switchedOn) {
	//	  CONSOLE.println("EscapeReverseOp:: Overriding mowmotor stop! (MOWMOTORSTOPONOBSTACLE = false)");
	//	  motor.setMowState(true);	
	//	}
	//} else {																						//MrTree
	//	if (motor.switchedOn) {
	//	CONSOLE.println("EscapeReverseOp:: Switch Off mow");
	//	motor.setMowState(false);  																	//MrTree  	
	//	}
	//}
}


void EscapeReverseOp::end(){
}


void EscapeReverseOp::run(){
    battery.resetIdle();
	if (!detectObstacle()){ 			//Mr.Tree
        detectObstacleRotation();       //Mr.Tree                       
    }
	//if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME){
       // wait until mowing motor is running
    //   if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
    //    motor.setLinearAngularSpeed(0,0,false);
	//	driveReverseStopTime = millis() + (OBSTACLEAVOIDANCEWAY/OBSTACLEAVOIDANCESPEED*1000);
    //  }
	//else {
       motor.setLinearAngularSpeed(-OBSTACLEAVOIDANCESPEED,0,false);				
	//}																			
	if ((DISABLE_MOW_MOTOR_AT_OBSTACLE) && (motor.switchedOn)) {	//MrTree
      CONSOLE.println("EscapeReverseOp:: switch OFF mowmotor");			//MrTree
	  motor.setMowState(false);  																	  	
	}  																                                   																					
    if (millis() > driveReverseStopTime){
        CONSOLE.println("driveReverseStopTime");
        motor.stopImmediately(false); 
        driveReverseStopTime = 0;
        if (detectLift()) {
            CONSOLE.println("error: lift sensor!");
            stateSensor = SENS_LIFT;
            changeOp(errorOp);
            return;
        }
        if (maps.isDocking()){
            CONSOLE.println("continue docking");
            // continue without obstacles
            changeOp(*nextOp, false);    // continue current operation
        } else {
            CONSOLE.println("continue operation with virtual obstacle");
            maps.addObstacle(stateX, stateY);              
            //Point pt;
            //if (!maps.findObstacleSafeMowPoint(pt)){
            //    changeOp(dockOp); // dock if no more (valid) mowing points
            //} else changeOp(*nextOp);    // continue current operation
            changeOp(*nextOp, false);    // continue current operation
        }
    }
}



void EscapeReverseOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeReverseOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


