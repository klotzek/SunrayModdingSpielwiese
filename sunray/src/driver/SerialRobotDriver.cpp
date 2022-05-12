// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SerialRobotDriver.h"
#include "../../config.h"
#include "../../ioboard.h"

#define COMM  ROBOT



void SerialRobotDriver::begin(){
  CONSOLE.println("using robot driver: SerialRobotDriver");
  COMM.begin(ROBOT_BAUDRATE);
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  chargeVoltage = 0;
  chargeCurrent = 0;  
  batteryVoltage = 0;
  mowCurr = 0;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  resetMotorTicks = true;
  batteryTemp = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredRain = false;
  triggeredStopButton = false;
  triggeredLift = false;
  motorFault = false;
  ngpCommunicationLost = true;
  nextSummaryTime = 0;
  nextConsoleTime = 0;
  nextMotorTime = 0;
  cmdMotorResponseCounter = 0;
  cmdSummaryResponseCounter = 0;
  cmdMotorCounter = 0;
  cmdSummaryCounter = 0;
  requestLeftPwm = requestRightPwm = requestMowPwm = 0;

  #ifdef __linux__    
    // IMU power-on code (Alfred-PCB-specific) 
    // switch-on IMU via port-expander PCA9555     
    ioExpanderOut(EX1_I2C_ADDR, EX1_IMU_POWER_PORT, EX1_IMU_POWER_PIN, true);

    // select IMU via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_IMU_MPU, true);  // Alfred dev PCB with buzzer
    ioI2cMux(MUX_I2C_ADDR, SLAVE_BUS0, true); // Alfred dev PCB without buzzer    

    // select EEPROM via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_EEPROM, true);  

    // select ADC via multiplexer TCA9548A 
    ioI2cMux(MUX_I2C_ADDR, SLAVE_ADC, true);
    
    // buzzer test
    if (false){
      ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
      delay(500);
      ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);    
    }

    // start ADC
    ioAdcStart(ADC_I2C_ADDR);

    // ADC test    
    if (false){    
      for (int idx=1; idx < 9; idx++){
        ioAdcMux(idx);            
        delay(50);
        ioAdcTrigger(ADC_I2C_ADDR);
        delay(500);
        float v = ioAdc(ADC_I2C_ADDR);
        CONSOLE.print("S");
        CONSOLE.print(idx);
        CONSOLE.print("=");
        CONSOLE.println(v);   
      }
    }    

    // EEPROM test
    if (false){
      ioEepromWriteByte( EEPROM_I2C_ADDR, 0, 42);
      delay(50);
      int v = ioEepromReadByte( EEPROM_I2C_ADDR, 0);
      CONSOLE.print("EEPROM=");
      CONSOLE.println(v);
    }
    
  #endif
}

void SerialRobotDriver::sendRequest(String s){
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);  
  s += F("\r\n");             
  //CONSOLE.print(s);  
  //cmdResponse = s;
  COMM.print(s);  
}


void SerialRobotDriver::requestSummary(){
  String req;
  req += "AT+S";  
  sendRequest(req);
  cmdSummaryCounter++;
}

void SerialRobotDriver::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  String req;
  req += "AT+M,";
  req += rightPwm;      
  req += ",";
  req += leftPwm;    
  req += ",";  
  req += mowPwm;
  //if (abs(mowPwm) > 0)
  //  req += "1";
  //else
  //  req += "0";  
  sendRequest(req);
  cmdMotorCounter++;
}

void SerialRobotDriver::motorResponse(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();      
      if (counter == 1){                            
        encoderTicksRight = intValue;  // ag
      } else if (counter == 2){
        encoderTicksLeft = intValue;   // ag
      } else if (counter == 3){
        encoderTicksMow = intValue;
      } else if (counter == 4){
        chargeVoltage = floatValue;
      } else if (counter == 5){
        triggeredLeftBumper = (intValue != 0);
      } else if (counter == 6){
        triggeredLift = (intValue != 0);
      } else if (counter == 7){
        triggeredStopButton = (intValue != 0);
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }
  if (triggeredStopButton){
    CONSOLE.println("STOPBUTTON");
  }
  cmdMotorResponseCounter++;
  ngpCommunicationLost=false;
}


void SerialRobotDriver::summaryResponse(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();      
      float floatValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toFloat();      
      if (counter == 1){                            
        batteryVoltage = floatValue;
      } else if (counter == 2){
        chargeVoltage = floatValue;
      } else if (counter == 3){
        chargeCurrent = floatValue;
      } else if (counter == 4){
        triggeredLift = (intValue != 0);
      } else if (counter == 5){
        triggeredLeftBumper = (intValue != 0);
      } else if (counter == 6){
        triggeredRain = (intValue != 0);
      } else if (counter == 7){
        motorFault = (intValue != 0);
      } else if (counter == 8){
        //CONSOLE.println(cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1));
        mowCurr = floatValue;
      } else if (counter == 9){
        motorLeftCurr = floatValue;
      } else if (counter == 10){
        motorRightCurr = floatValue;
      } else if (counter == 11){
        batteryTemp = floatValue;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }
  cmdSummaryResponseCounter++;
  /*CONSOLE.print("motor currents=");
  CONSOLE.print(mowCurr);
  CONSOLE.print(",");
  CONSOLE.print(motorLeftCurr);
  CONSOLE.print(",");
  CONSOLE.println(motorRightCurr);*/
  //CONSOLE.print("batteryTemp=");
  //CONSOLE.println(batteryTemp);
}

// process response
void SerialRobotDriver::processResponse(bool checkCrc){
  cmdResponse = "";      
  if (cmd.length() < 4) return;
  byte expectedCrc = 0;
  int idx = cmd.lastIndexOf(',');
  if (idx < 1){
    if (checkCrc){
      CONSOLE.println("SerialRobot: CRC ERROR");
      return;
    }
  } else {
    for (int i=0; i < idx; i++) expectedCrc += cmd[i];  
    String s = cmd.substring(idx+1, idx+5);
    int crc = strtol(s.c_str(), NULL, 16);  
    if (expectedCrc != crc){
      if (checkCrc){
        CONSOLE.print("SerialRobot: CRC ERROR");
        CONSOLE.print(crc,HEX);
        CONSOLE.print(",");
        CONSOLE.print(expectedCrc,HEX);
        CONSOLE.println();
        return;  
      }      
    } else {
      // remove CRC
      cmd = cmd.substring(0, idx);
      //CONSOLE.print("SerialRobot resp:");
      //CONSOLE.println(cmd);
    }    
  }     
  if (cmd[0] == 'M') motorResponse();
  if (cmd[0] == 'S') summaryResponse();
}


// process console input
void SerialRobotDriver::processComm(){
  char ch;      
  if (COMM.available()){
    //battery.resetIdle();  
    while ( COMM.available() ){               
      ch = COMM.read();          
      if ((ch == '\r') || (ch == '\n')) {        
        //CONSOLE.println(cmd);
        processResponse(true);              
        //CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}


void SerialRobotDriver::run(){
  processComm();
  if (millis() > nextMotorTime){
    nextMotorTime = millis() + 20; // 50 hz
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
  }
  if (millis() > nextSummaryTime){
    nextSummaryTime = millis() + 500; // 2 hz
    requestSummary();
  }
  if (millis() > nextConsoleTime){
    nextConsoleTime = millis() + 1000;
    if (cmdMotorResponseCounter == 0){
      CONSOLE.println("WARN: resetting motor ticks");
      resetMotorTicks = true;
      ngpCommunicationLost = true;
    }    
    if ( (cmdMotorResponseCounter < 30) || (cmdSummaryResponseCounter == 0) ){
      CONSOLE.print("WARN: SerialRobot unmet communication frequency: motorFreq=");
      CONSOLE.print(cmdMotorCounter);
      CONSOLE.print("/");
      CONSOLE.print(cmdMotorResponseCounter);
      CONSOLE.print("  summaryFreq=");
      CONSOLE.print(cmdSummaryCounter);
      CONSOLE.print("/");
      CONSOLE.println(cmdSummaryResponseCounter);
      if (cmdMotorResponseCounter == 0){
        // FIXME: maybe reset motor PID controls here?
      }
    }   
    cmdMotorCounter=cmdMotorResponseCounter=cmdSummaryCounter=cmdSummaryResponseCounter=0;
  }
}


// ------------------------------------------------------------------------------------

SerialMotorDriver::SerialMotorDriver(SerialRobotDriver &sr): serialRobot(sr){
} 

void SerialMotorDriver::begin(){
  lastEncoderTicksLeft=0;
  lastEncoderTicksRight=0;         
}

void SerialMotorDriver::run(){
}

void SerialMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  //serialRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
  serialRobot.requestLeftPwm = leftPwm;
  serialRobot.requestRightPwm = rightPwm;
  serialRobot.requestMowPwm = mowPwm;
}

void SerialMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = serialRobot.motorFault;
  rightFault = serialRobot.motorFault;
  if (serialRobot.motorFault){
    CONSOLE.print("serialRobot: motorFault (lefCurr=");
    CONSOLE.print(serialRobot.motorLeftCurr);
    CONSOLE.print(" rightCurr=");
    CONSOLE.print(serialRobot.motorRightCurr);
    CONSOLE.print(" mowCurr=");
    CONSOLE.println(serialRobot.mowCurr);
  }
  mowFault = false;
}

void SerialMotorDriver::resetMotorFaults(){
  CONSOLE.println("serialRobot: resetting motor fault");
  //serialRobot.requestMotorPwm(1, 1, 0);
  //delay(1);
  //serialRobot.requestMotorPwm(0, 0, 0);
}

void SerialMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  //leftCurrent = 0.5;
  //rightCurrent = 0.5;
  //mowCurrent = 0.8;
  leftCurrent = serialRobot.motorLeftCurr;
  rightCurrent = serialRobot.motorRightCurr;
  mowCurrent = serialRobot.mowCurr;
}

void SerialMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  if (serialRobot.ngpCommunicationLost) {
    //CONSOLE.println("getMotorEncoderTicks: no ticks!");    
    leftTicks = rightTicks = 0; mowTicks = 0;
    return;
  }
  if (serialRobot.resetMotorTicks){
    serialRobot.resetMotorTicks = false;
    //CONSOLE.println("getMotorEncoderTicks: resetMotorTicks");
    lastEncoderTicksLeft = serialRobot.encoderTicksLeft;
    lastEncoderTicksRight = serialRobot.encoderTicksRight; 
  }
  leftTicks = serialRobot.encoderTicksLeft - lastEncoderTicksLeft;
  rightTicks = serialRobot.encoderTicksRight - lastEncoderTicksRight;
  if (leftTicks > 1000){
    leftTicks = 0;
  }
  if (rightTicks > 1000){
    rightTicks = 0;
  } 
  lastEncoderTicksLeft = serialRobot.encoderTicksLeft;
  lastEncoderTicksRight = serialRobot.encoderTicksRight;
  mowTicks = 0;
}


// ------------------------------------------------------------------------------------

SerialBatteryDriver::SerialBatteryDriver(SerialRobotDriver &sr) : serialRobot(sr){
  ngpBoardPoweredOn = true;
  nextADCTime = 0;
  linuxShutdownTime = 0;
}

void SerialBatteryDriver::begin(){
}

void SerialBatteryDriver::run(){
}    

float SerialBatteryDriver::getBatteryVoltage(){
  #ifdef __linux__
    if (serialRobot.ngpCommunicationLost){      
      // detect if ngp PCB is switch-off
      if (nextADCTime == 0){    
        // trigger ADC measurement (ngpPWR)
        ioAdcMux(ADC_NGP_PWR);
        ioAdcTrigger(ADC_I2C_ADDR);    
        nextADCTime = millis() + 1000;    
      } 
      if (millis() > nextADCTime){
        nextADCTime = 0;
        float v = ioAdc(ADC_I2C_ADDR);
        ngpBoardPoweredOn = true;
        if (v < 0){
          CONSOLE.println("ERROR reading ADC channel ngpPWR!");
        } else {
          if ((v >0) && (v < 0.4)){
            // no ngpPWR, ngp PCB is probably switched off
            CONSOLE.print("ngpPWR=");
            CONSOLE.println(v);      
            CONSOLE.println("NGP PCB powered OFF!");
            ngpBoardPoweredOn = false;        
          }
        }
      }
      if (!ngpBoardPoweredOn) return 0; // return zero volt if ngp PCB is switched-off (so we will be later requested to shutdown)
    }
  #endif         
  return serialRobot.batteryVoltage;
}

float SerialBatteryDriver::getChargeVoltage(){
  return serialRobot.chargeVoltage;
}
    
float SerialBatteryDriver::getChargeCurrent(){
  return serialRobot.chargeCurrent;
} 

void SerialBatteryDriver::enableCharging(bool flag){
}


void SerialBatteryDriver::keepPowerOn(bool flag){
  #ifdef __linux__
    if (flag){
      // keep power on
      linuxShutdownTime = 0;
    } else {
      // shutdown linux - request could be for two reasons:
      // 1. battery voltage sent by ngp-pcb seem to be too low 
      // 2. ngp-pcb is powered-off 
      if (linuxShutdownTime == 0){
        linuxShutdownTime = millis() + 5000; // some timeout 
      }
      if (millis() > linuxShutdownTime){
        linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
        CONSOLE.println("LINUX will SHUTDOWN!");
        Process p;
        p.runShellCommand("shutdown now");
      }
    }   
  #endif  
}


// ------------------------------------------------------------------------------------

SerialBumperDriver::SerialBumperDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialBumperDriver::begin(){
}

void SerialBumperDriver::run(){

}

bool SerialBumperDriver::obstacle(){
  return (serialRobot.triggeredLeftBumper || serialRobot.triggeredRightBumper); 
}

void SerialBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = serialRobot.triggeredLeftBumper;
  rightBumper = serialRobot.triggeredRightBumper;
}  	  		    


// ------------------------------------------------------------------------------------


SerialStopButtonDriver::SerialStopButtonDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialStopButtonDriver::begin(){
}

void SerialStopButtonDriver::run(){

}

bool SerialStopButtonDriver::triggered(){
  return (serialRobot.triggeredStopButton); 
}

// ------------------------------------------------------------------------------------


SerialRainSensorDriver::SerialRainSensorDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialRainSensorDriver::begin(){
}

void SerialRainSensorDriver::run(){

}

bool SerialRainSensorDriver::triggered(){
  return (serialRobot.triggeredRain); 
}

// ------------------------------------------------------------------------------------

SerialLiftSensorDriver::SerialLiftSensorDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialLiftSensorDriver::begin(){
}

void SerialLiftSensorDriver::run(){
}

bool SerialLiftSensorDriver::triggered(){
  return (serialRobot.triggeredLift);
}


// ------------------------------------------------------------------------------------

SerialBuzzerDriver::SerialBuzzerDriver(SerialRobotDriver &sr): serialRobot(sr){
}

void SerialBuzzerDriver::begin(){
}

void SerialBuzzerDriver::run(){
}

void SerialBuzzerDriver::noTone(){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);
}

void SerialBuzzerDriver::tone(int freq){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
}



