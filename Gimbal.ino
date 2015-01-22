
#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION "v50"
#define REVISION "r202"
#define VERSION_EEPROM 15 // change this number when eeprom data structure has changed


/*************************/
/* Include Header Files  */
/*************************/
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "definitions.h"
#include "MPU6050.h"
#include "SerialCommand.h"/
#include "EEPROMAnything.h"
#include "PinChangeInt.h"
#include "Timer1.h"
#include "Trace.h"
#include "variables.h"
MPU6050 mpu;            // Create MPU object
SerialCommand sCmd;     // Create SerialCommand object

#include "fastMathRoutines.h"     // fast Math functions required by orientationRoutines.h
#include "orientationRoutines.h"  // get Orientation from ACC
#include "RCdecode.h"             // RC Decoder to move camera by servo signal input
#include "BLcontroller.h"         // Motor Movement Functions and Timer Config
#include "SerialCom.h"            // Serial Protocol for Configuration and Communication

int prev_pitchAngleSet = 0;
int val = 0;
int intread = 0;
boolean reset = 1;


/**********************************************/
/* Initialization                             */
/**********************************************/
void setup() 
{

  // just for debugging
#ifdef STACKHEAPCHECK_ENABLE
  stackCheck();
  heapCheck();
#endif

  LEDPIN_PINMODE
  
  CH2_PINMODE
  CH3_PINMODE

  // Start Serial Port
  Serial.begin(115200);


  printMessage(MSG_INFO, F("BruGi ready"));
  printMessage(MSG_VERSION, F(""));


  setSerialProtocol();


  initBlController();

  initMotorStuff();
  
  // switch off PWM Power
  motorPowerOff();
    
  // Read Config, initialize if version does not match or CRC fails
  readEEPROM();
  if (config.versEEPROM != VERSION_EEPROM)
  {
    printMessage(MSG_WARNING, F("EEPROM version mismatch, initialized to default"));
    setDefaultParameters();
    writeEEPROM();
  }
  
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 
  // Initialize MPU 
  initResolutionDevider();
    
  // init I2C and MPU6050
  if (initI2C()) {  
    // Init IMU variables
    initIMU();
    // Gyro Offset calibration
    if (config.gyroCal) {
      gyroCalibrateCmd();
    }
  } else {
    gimState = GIM_ERROR;
  }
  
  // set sensor orientation
  initSensorOrientation();
  
  // Init PIDs parameters
  initPIDs();

  // init RC variables
  initRC();

  // Init RC-Input
  initRCPins();

  LEDPIN_OFF
  CH2_OFF
  CH3_OFF

}


int32_t ComputePID(int32_t DTms, int32_t DTinv, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd)
{
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTinv;
  *errorOld = error;

  out = out / 4096 / 8;
  
  return out;
  
}


/**********************************************/
/* Main Loop                                  */
/**********************************************/
void loop() 
{ 
  

  val = 0;
  
  while (Serial.available() > 0)
  {
    
    if (Serial.available() > 7)
      {
          Serial.read();
      }
          
      else
      {
        intread = Serial.read();
        Serial.println(intread);
        
        if (intread < 48 || intread > 57)
            reset = 1;
                        
        else 
        {
          if (reset == 1){
            val = 0;
            reset = 0;}
          
          val *= 10;
          val += intread - '0';
        }
        
        
      }
  }  
     
  
     
     pitchAngleSet = prev_pitchAngleSet;  
     
  if (val != 0)
  {
      pitchAngleSet = map(val,1000,2000,20,-100);
      pitchAngleSet = constrain(pitchAngleSet,-100,20);
      prev_pitchAngleSet = pitchAngleSet;
  }
      
    
  
//  Serial.print("val: ");
//  Serial.println(val);
//  Serial.print("pitchAngleSet: ");
//  Serial.println(pitchAngleSet);
//  Serial.print("Serial.available: ");
//  Serial.println(Serial.available());
//  Serial.println(" ");

  
  
  
  
  
  
  
  
  
  
  
//  Serial.println(pitchAngleSet);
//  config.maxPWMmotorPitch = 100; //map(analogRead(A1),0,1023, 0,255);
//  config.maxPWMmotorRoll = 100; //map(analogRead(A1),0,1023, 0,255);

//  pitchPIDpar.Ki = 5; //map(analogRead(A1),0,1023, 000, 10);
//  pitchPIDpar.Kd = 7; //map(analogRead(A1),0,1023, 0, 20);
//  pitchPIDpar.Kp = 300;;//map(analogRead(A1),0,1023, 000, 300);
//  
//   x++;    
//   if (x > 4000)
//  {
//    Serial.print("Setpoint = ");
//    Serial.print(pitchAngleSet);
//    Serial.print(" Input = ");
//    Serial.println(angle[PITCH]/1000);
//    Serial.print("Kp,Ki,Kd,PWM = ");
//    Serial.print(pitchPIDpar.Kp);
//    Serial.print(",");
//    Serial.print(pitchPIDpar.Ki);
//    Serial.print(",");
//    Serial.print(pitchPIDpar.Kd); 
//    Serial.print(",");
//    Serial.print(config.maxPWMmotorPitch);
//    Serial.print("\n");
//    x = 0;
//  }
    
//  rollPIDpar.Ki = 5; //map(analogRead(A1),0,1023, 000, 10);
//  rollPIDpar.Kd = 7; //map(analogRead(A1),0,1023, 0, 20);
//  rollPIDpar.Kp = 500; //map(analogRead(A1),0,1023, 100, 500);
//   x++;    
//   if (x > 10000)
//  {
//    Serial.print("Setpoint = ");
//    Serial.print(rollAngleSet);
//    Serial.print(" Input = ");
//    Serial.println(angle[ROLL]/1000);
//    Serial.print("Kp,Ki,Kd,PWM = ");
//    Serial.print(rollPIDpar.Kp);
//    Serial.print(",");
//    Serial.print(rollPIDpar.Ki);
//    Serial.print(",");
//    Serial.print(rollPIDpar.Kd); 
//    Serial.print(",");
//    Serial.print(config.maxPWMmotorRoll);
//    Serial.print("\n");
//    x = 0;
//  }
  
  
  
  
  int32_t pitchPIDVal;
  int32_t rollPIDVal;
  
  static char pOutCnt = 0;
  static char tOutCnt = 0;
  static char tOutCntSub = 0;
  static int stateCount = 0;
  static uint8_t ledBlinkCnt = 0;
  static uint8_t ledBlinkOnTime = 10;
  static uint8_t ledBlinkPeriod = 20;

  if (motorUpdate) // loop runs with motor ISR update rate (500 Hz)
  {
   
    motorUpdate = false;

    CH2_ON
    
    // loop period
    //     2.053/2.035 ms max/min, error = +5/-13 us (w/o rc)
    //     2.098/2.003 ms max/min, error = +50/-45 us (1 x PPM16 1 x PWM)
    
    // update IMU data            
    readGyros();   // td = 330us

    if (config.enableGyro) updateGyroAttitude(); // td = 176 us
    if (config.enableACC) updateACCAttitude(); // td = 21 us
    getAttiduteAngles(); // td = 372 us
   

    //****************************
    // pitch PID
    //****************************
    if (fpvModeFreezePitch==false) {
      // td = 92 us
      pitchPIDVal = ComputePID(DT_INT_MS, DT_INT_INV, angle[PITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
      // motor control
      pitchMotorDrive = pitchPIDVal * config.dirMotorPitch;
    }
 
    //****************************
    // roll PID
    //****************************
    if (fpvModeFreezeRoll==false) {
      // td = 92 us
      rollPIDVal = ComputePID(DT_INT_MS, DT_INT_INV, angle[ROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);
      // motor control
      rollMotorDrive = rollPIDVal * config.dirMotorRoll;
    }
    
    // motor update t=6us (*)
    if (enableMotorUpdates)
    {
      // set pitch motor pwm
      MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, maxPWMmotorPitchScaled); 
      // set roll motor pwm
      MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, maxPWMmotorRollScaled);
    }

    // Evaluate RC-Signals, td = 120 us
    if (fpvModePitch==true) {
      pitchAngleSet = utilLP3_float(qLPPitch, PitchPhiSet, rcLPFPitchFpv_tc);
    } else if(config.rcAbsolutePitch==1) {
      pitchAngleSet = utilLP3_float(qLPPitch, PitchPhiSet, rcLPFPitch_tc); // 63us
    } else {
      utilLP_float(&pitchAngleSet, PitchPhiSet, 0.01);
    }
    if (fpvModeRoll==true) {
      rollAngleSet = utilLP3_float(qLPRoll, RollPhiSet, rcLPFRollFpv_tc);
    } else if(config.rcAbsoluteRoll==1) {
      rollAngleSet = utilLP3_float(qLPRoll, RollPhiSet, rcLPFRoll_tc);
    } else {
      utilLP_float(&rollAngleSet, RollPhiSet, 0.01);
    }
    
    // tElapsed = 1.250 ms

    //****************************
    // slow rate actions
    //****************************
    switch (count) {
    case 1:
      readACCs(); // td = 330us
      break;
    case 2:
      updateACC(); // td = 120us
      break;
    case 3:
      // td = 210us, total
      voltageCompensation();
      break;
    case 4:
      
      // gimbal state transitions, td=56us
      switch (gimState)
      {
        case GIM_IDLE :
          // wait 2 sec to settle ACC, before PID controlerbecomes active 
          stateCount++;
          if (stateCount >= LOOPUPDATE_FREQ/10*1) // 1 sec 
          {
            gimState = GIM_UNLOCKED;
            stateCount = 0;
          }
          break;
        case GIM_UNLOCKED :
          // allow PID controller to settle on ACC position
          stateCount++;
          if (stateCount >= LOOPUPDATE_FREQ/10*LOCK_TIME_SEC) 
          {
            gimState = GIM_LOCKED;
            stateCount = 0;
          }
          break;
        case GIM_LOCKED :
          // normal operation
          break;
        case GIM_ERROR :
          // error state
          break;        
      }
      // gimbal state actions 
      switch (gimState) {
        case GIM_IDLE : // allow settling IMU
          enableMotorUpdates = false;
          setACCtc(0.2);
          disableAccGtest = true;
          break;
        case GIM_UNLOCKED : // fast settling of desired position
          enableMotorUpdates = true;
          disableAccGtest = true;
          setACCtc(2.0);
          disableAccGtest = true;
          break;
        case GIM_LOCKED : // normal operation
          enableMotorUpdates = true;
          disableAccGtest = false;
          if (altModeAccTime) { // alternate time constant mode switch
            setACCtc(config.accTimeConstant2);
          } else {
            setACCtc(config.accTimeConstant);
          }
          break;
        case GIM_ERROR :
          // error state
          // switch off motors
          enableMotorUpdates = false;
          // switch off PWM Power
          motorPowerOff();
          break;        
      }
      // handle mode switches
      decodeModeSwitches();  // td = 4 us
      
      // lpf avoids jerking during offset config
      updateLPFangleOffset(); // td = 65 us
           
      break;
    case 5:
      // RC Pitch function
      evaluateRCPitch();
      // td = 26 us
      getSetpoint(&PitchPhiSet, RC_DATA_PITCH, RC_DATA_FPV_PITCH, fpvModePitch, config.rcAbsolutePitch, config.maxRCPitch, config.minRCPitch);
      break;
    case 6:
      // RC roll function
      evaluateRCRoll();
      // td = 26us
      getSetpoint(&RollPhiSet, RC_DATA_ROLL, RC_DATA_FPV_ROLL, fpvModeRoll, config.rcAbsoluteRoll, config.maxRCRoll, config.minRCRoll);
      break;
    case 7:
      // evaluate RC-Signals. td = 90 us
      evaluateRCAux();
      
      // check RC channel timeouts
      checkRcTimeouts();  // td = 15 us

      break;
    case 8:
      // read RC Anlog inputs
      readRCAnalog(); // td = 354 us (if all 3 enabled, 118 us each analog channel)
      break;
    case 9:
      //   regular i2c test
      mpu.testConnection();
    break;
    case 10:    
      // regular ACC output
      pOutCnt++;
      if (pOutCnt == (LOOPUPDATE_FREQ/10/POUT_FREQ))
      {
        if (config.fTrace != TRC_OFF) {
          printTrace(config.fTrace);
        }
        pOutCnt = 0;
      }
      
      // print regular trace output
      tOutCnt++;
      if (tOutCnt == (LOOPUPDATE_FREQ/10/TRACE_OUT_FREQ))
      {
        tOutCntSub++;
        if (tOutCntSub > STRACE_IDX) {
          tOutCntSub = 1;
        }

        if (config.sTrace == TRC_ALL) {
            // cycle all trace types
            printTrace((traceModeType)tOutCntSub);      
        } else if (config.sTrace != TRC_OFF) {
            // use specific trace type
            printTrace(config.sTrace);
        }
        
        tOutCnt = 0;
      }

      ledBlinkCnt++;
      if (ledBlinkCnt <= ledBlinkOnTime) {
          LEDPIN_ON
      } else if (ledBlinkCnt <= ledBlinkPeriod) {
          LEDPIN_OFF
      } else {
        ledBlinkCnt = 0;
      }
        
#ifdef STACKHEAPCHECK_ENABLE
      stackHeapEval(false);
#endif
      count=0;
      break;
    default:
      break;
    }
    count++;
       
    //****************************
    // Evaluate Serial inputs 
    //****************************
//    sCmd.readSerial();


    // worst-case finalize after
    //    1.67 ms (w/o RC)
    //    1.76 ms (with 1 x PPM)
    //    1.9 ms (with 2 RC channels + 1 x PPM )

    CH2_OFF
  }

}


