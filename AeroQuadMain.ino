
//AeroQuad v3.0.1 

#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

#if defined(UseGPSNMEA) || defined(UseGPSUBLOX) || defined(UseGPSMTK) || defined(UseGPS406)
#define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
#error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(AutoLanding) && (!defined(AltitudeHoldBaro) || !defined(AltitudeHoldRangeFinder))
#error "AutoLanding NEED AltitudeHoldBaro and AltitudeHoldRangeFinder defined"
#endif

#if defined(ReceiverSBUS) && defined(SlowTelemetry)
#error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
#error "CameraTXControl need to have CameraControl defined"
#endif 

#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <PID_v1.h>
#include <AQMath.h>
#include <FourtOrderFilter.h>
#ifdef BattMonitor
#include <BatteryMonitorTypes.h>
#endif

//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************

//-----Enter WayPoints Below--------------------------------------------------------------------------------------------------------------------------

double wayGrid [2][2] =    //CHECK MATRIX NUMBERS

//{26.12340, -80.12310}; // Simulation

//{26.25235,-80.27787,
// 26.25245,-80.27797}; // House Parking Lot

//{26.25264,-80.27932,
//26.25287,-80.27957}; // Park

{26.24407,-80.26535, 
26.24427,-80.26518};  // Soccer Fields


//-----Enter Height & Speed For Automated Flight--------------------------------------------------------------------------------------------------------------

float Flight_Height = 3.00;
float Landing_Height = 0;
int Flight_Speed = 0;
float Static_Alt = 0;
//-----Enter Epsilons---------------------------------------------------------------------------------------------------------------------------------

double way_epsilon = 0.00004;      // Change waypoint when within this range
double follow_epsilon = 0.00004;   // Break when within this range
double filter_epsilon = 0.00003;   // Range to generate new incoming coordinate
double distance_epsilon = 0.00095; // Allowable distant between Waypoints or Follow Me
double range_epsilon = 0.10000;    // Allowable Travel Range when using Remote

//----------------------------------------------------------------------------------------------------------------------------------------------------

double tiny[3] = {0,0,0};
double Incoming[5] = {0,0,1000,0,0};
double waypoints[2] = {0,0};   
double Home_Position[2] = {0,0};
double prev_Incoming[2] = {0,0};
double next[2] = {0,0};
double prev_next[2] = {0,0};
int move_commands[2] = {0,0};
int wGc = 0;

int SIZE = sizeof(wayGrid)/sizeof(double)/2-1;
double x_dist = 0;
double y_dist = 0;
int passage[1] = {1};
int Emergency[1] = {0};
int flagps[1] = {0};
int no_remote_signal[1] = {0};
int no_receiver_signal[1] = {0};
int no_gps_signal[1] = {0};
unsigned long CycleTime = 0;
int key[4] = {0};
int latBig;
int latMed;
int longBig;
int longMed;

int actualCenter = 1160;
int desiredCenter = 1160;
int desiredAngle = 74;
int intsend = 1300;
String intmsg = "0";
String stopsign = "!";

float KpL=0.2;              
float KiL=0;               
float KdL=0.05;    
float KpW=0.25;              
float KiW=0;               
float KdW=0.05;  
float setGridAngle = 0;
double lengthSetpoint, widthSetpoint, length_dist, width_dist, lengthOutput, widthOutput; 
PIDD lengthPID(&length_dist, &lengthOutput, &lengthSetpoint, KpL, KiL, KdL, DIRECT); 
PIDD widthPID(&width_dist, &widthOutput, &widthSetpoint, KpW, KiW, KdW, DIRECT);
const int sampleRate = 250; 
double lower = -100, upper = 100;

#ifdef AeroQuadMega_v21
#define LED_Green 13
#define LED_Red 4
#define LED_Yellow 31

#include <Device_I2C.h>

// Gyroscope declaration
#define ITG3200_ADDRESS_ALTERNATE
#include <Gyroscope_ITG3200_9DOF.h>

// Accelerometer declaration
#include <Accelerometer_ADXL345_9DOF.h>

// Receiver Declaration
#define RECEIVER_MEGA

// Motor declaration
#define MOTOR_PWM_Timer

// heading mag hold declaration
#ifdef HeadingMagHold
#include <Compass.h>
#define SPARKFUN_9DOF_5883L
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
#define BMP085
#endif
#ifdef AltitudeHoldRangeFinder
#define XLMAXSONAR 
#endif


// Battery Monitor declaration
#ifdef BattMonitor
#ifdef POWERED_BY_VIN
#define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
#else
#define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
#endif
#else
#undef BattMonitorAutoDescent
#undef POWERED_BY_VIN        
#endif

#ifdef OSD
#define MAX7456_OSD
#endif  

#ifndef UseGPS
#undef UseGPSNavigator
#endif


#include <TinyGPS.h>
TinyGPS gps;

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN    48
#define CSN_PIN  49
const uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(CE_PIN,CSN_PIN);


void initPlatform() {

  pinMode(LED_Red, OUTPUT);
  digitalWrite(LED_Red, LOW);
  pinMode(LED_Yellow, OUTPUT);
  digitalWrite(LED_Yellow, LOW);

  // pins set to INPUT for camera stabilization so won't interfere with new camera class
  pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
  pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
  pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
  pinMode(43, OUTPUT); // LED 1
  pinMode(44, OUTPUT); // LED 2
  pinMode(45, OUTPUT); // LED 3
  pinMode(46, OUTPUT); // LED 4
  digitalWrite(43, HIGH); // LED 1 on
  digitalWrite(44, HIGH); // LED 2 on
  digitalWrite(45, HIGH); // LED 3 on
  digitalWrite(46, HIGH); // LED 4 on
  pinMode(10,OUTPUT);
  Wire.begin();
  TWBR = 12;
}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() {
  // Kenny default value, a real accel calibration is strongly recommended
  accelScaleFactor[XAXIS] = 0.0365570020;
  accelScaleFactor[YAXIS] = 0.0363000011;
  accelScaleFactor[ZAXIS] = -0.0384629964;
#ifdef HeadingMagHold
  magBias[XAXIS]  = 1.500000;
  magBias[YAXIS]  = 205.500000;
  magBias[ZAXIS]  = -33.000000;
#endif
}

/**
 * Measure critical sensors
 */
void measureCriticalSensors() {
  measureGyroSum();
  measureAccelSum();
}
#endif


//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************


// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
// CHR6DM have it's own kinematics, so, initialize in it's scope
#else
#include "Kinematics_ARG.h"
#endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(ReceiverHWPPM)
#include <Receiver_HWPPM.h>
#elif defined(ReceiverPPM)
#include <Receiver_PPM.h>
#elif defined(AeroQuad_Mini) && (defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config))
#include <Receiver_PPM.h>

#elif defined(RemotePCReceiver)
#include <Receiver_RemotePC.h>

#elif defined(ReceiverSBUS)
#include <Receiver_SBUS.h>
#elif defined(RECEIVER_328P)
#include <Receiver_328p.h>

#elif defined(RECEIVER_MEGA)
#include <Receiver_MEGA.h>

#elif defined(RECEIVER_APM)
#include <Receiver_APM.h>
#elif defined(RECEIVER_STM32PPM)
#include <Receiver_STM32PPM.h>  
#elif defined(RECEIVER_STM32)
#include <Receiver_STM32.h>  
#endif

#if defined(UseAnalogRSSIReader) 
#include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
#include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
#include <SBUSRSSIReader.h>
#endif


//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
#if defined (MOTOR_STM32)
#define MOTORS_STM32_TRI
#include <Motors_STM32.h>    
#else
#include <Motors_Tri.h>
#endif
#elif defined(MOTOR_PWM)
#include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
#include <Motors_PWM_Timer.h>
#elif defined(MOTOR_APM)
#include <Motors_APM.h>
#elif defined(MOTOR_I2C)
#include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
#include <Motors_STM32.h>    
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
#include <HeadingFusionProcessorMARG.h>
#include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
#include <HeadingFusionProcessorMARG.h>
#include <Magnetometer_HMC5883L.h>
#elif defined(COMPASS_CHR6DM)
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
#include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
#include <BarometricSensor_MS5611.h>
#endif
#if defined(XLMAXSONAR)
#include <MaxSonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
#include <BatteryMonitor.h>
#ifndef BattCustomConfig
#define BattCustomConfig BattDefaultConfig
#endif
struct BatteryData batteryData[] = {
  BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
#include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
#include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
#include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#if defined(quadXConfig)
#include "FlightControlQuadX.h"
#endif

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)
#if !defined(HeadingMagHold)
#error We need the magnetometer to use the GPS
#endif 
#include <GpsAdapter.h>
#include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
#include <Device_SPI.h>
#include "OSDDisplayController.h"
#include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
#include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
#if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
#error "Menu cannot be used without OSD or LCD"
#endif
#include "OSDMenu.h"
#endif


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(WirelessTelemetry) 
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SERIAL_PORT Serial3
#else    // force 328p to use the normal port
#define SERIAL_PORT Serial
#endif
#else  
#if defined(SERIAL_USES_USB)   // STM32 Maple
#define SERIAL_PORT SerialUSB
#undef BAUD
#define BAUD
#else
#define SERIAL_PORT Serial
#endif
#endif  

#ifdef SlowTelemetry
#include <AQ_RSCode.h>
#endif

#ifdef SoftModem
#include <AQ_SoftModem.h>
#endif


// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
#include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
#include "MavLink.h"
#else
#include "SerialCom.h"
#endif

/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/

void setup() {
  Serial.begin(115200);
  pinMode(30, OUTPUT);
  digitalWrite(30, LOW);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);

  Serial1.begin(4800);
  initCommunication();

  Serial2.begin(115200);

  readEEPROM(); // defined in DataStorage.h
  boolean firstTimeBoot = false;
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    initializeEEPROM();
    writeEEPROM();
    firstTimeBoot = true;
  }

  initPlatform();

#if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
  initializeMotors(FOUR_Motors);

#endif

  initializeReceiver(LASTCHANNEL);
  initReceiverFromEEPROM();

  //-----************Receiver************-----

  //MISO 50
  //MOSI 51
  //SCK  52
  //SS   53
  //CE   48
  //CSN  49

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipe);
  radio.startListening();

  //-----************Receiver************-----

  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process


  initializeAccel(); // defined in Accel.h
  if (firstTimeBoot) {
    computeAccelBias();
    writeEEPROM();
  }
  setupFourthOrder();
  initSensorsZeroFromEEPROM();

  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;

  // Flight angle estimation
  initializeKinematics();

#ifdef HeadingMagHold
  vehicleState |= HEADINGHOLD_ENABLED;
  initializeMagnetometer();
  initializeHeadingFusion();
#endif

  // Optional Sensors
#ifdef AltitudeHoldBaro
  initializeBaro();
  vehicleState |= ALTITUDEHOLD_ENABLED;
#endif
#ifdef AltitudeHoldRangeFinder
  inititalizeRangeFinders();
  vehicleState |= RANGE_ENABLED;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = PID[BARO_ALTITUDE_HOLD_PID_IDX].P*2;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = PID[BARO_ALTITUDE_HOLD_PID_IDX].I;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = PID[BARO_ALTITUDE_HOLD_PID_IDX].D;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard;
#endif

#ifdef BattMonitor
  initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
  vehicleState |= BATTMONITOR_ENABLED;
#endif

#if defined(CameraControl)
  initializeCameraStabilization();
  vehicleState |= CAMERASTABLE_ENABLED;
#endif

#if defined(MAX7456_OSD)
  initializeSPI();
  initializeOSD();
#endif

#if defined(SERIAL_LCD)
  InitSerialLCD();
#endif

#if defined(BinaryWrite) || defined(BinaryWritePID)
#ifdef OpenlogBinaryWrite
  binaryPort = &Serial1;
  binaryPort->begin(115200);
  delay(1000);
#else
  binaryPort = &Serial;
#endif
#endif

  for (int GCounter = 1; GCounter<= 25; GCounter++)
  {
    GETGPS(tiny);
    delay(5);
  }

#if defined(UseGPS)
  initializeGps();
#endif 

#ifdef SlowTelemetry
  initSlowTelemetry();
#endif

  previousTime = micros();
  safetyCheck = 0;


  lengthPID.SetMode(AUTOMATIC);  //Turn on the PID loop
  lengthPID.SetSampleTime(sampleRate); //Sets the sample rate
  lengthPID.SetOutputLimits(lower,upper);
  widthPID.SetMode(AUTOMATIC);  //Turn on the PID loop
  widthPID.SetSampleTime(sampleRate); //Sets the sample rate
  widthPID.SetOutputLimits(lower,upper);


  mCameraPitch = 506;    
  mCameraRoll = -360;    
  mCameraYaw = 318.3;
  servoCenterPitch = 1100;
  servoCenterRoll = 1596;
  servoCenterYaw = 1500;
  servoMinPitch = 875;
  servoMinRoll = 1412;
  servoMinYaw = 1000;
  servoMaxPitch = 1690;
  servoMaxRoll = 1780;
  servoMaxYaw = 2000;

}


/*******************************************************************
 * 100Hz task
 ******************************************************************/

void GETGPS(double x[]){

  bool newData = false;

  while (Serial1.available())
  {
    char c = Serial1.read();
    if (gps.encode(c)) 
      newData = true;
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon);

    x[0] = flat;
    x[1] = flon;
    x[2] = 1;
  }

  else 
    x[2] = 0;

}


void CameraStabilizerServoTX()
{ 

  if (receiverCommand[AUX2] < 1250)
    servoCenterPitch = actualCenter;

  else
  {
    desiredCenter = map(receiverCommand[AUX3],1000,2000,servoMinPitch,servoMaxPitch);

    if (abs(servoCenterPitch - desiredCenter) > 7)
    {
      if (servoCenterPitch < desiredCenter)
        servoCenterPitch += 5;

      else 
        servoCenterPitch -= 5;     
    }
  }

  //   Serial.println(receiverCommand[AUX3]);
  //   Serial.println(servoCenterPitch);

}


void CameraStabilizerBrushlessTX()
{

  if (receiverCommand[AUX2] < 1250)
  {
    desiredAngle = -16;
    intsend = map(desiredAngle,20,-100,1000,2000);
  }

  else
  {

    desiredAngle = receiverCommand[AUX3];

    if (abs(intsend - desiredAngle) > 25)
    {
      if (intsend < desiredAngle)
        intsend += 7;

      else 
        intsend -= 7;  
    }
  }


  intmsg = String(intsend + stopsign);

  Serial2.println(intmsg);

}


void process100HzTask() {

  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;

  evaluateGyroRate();
  evaluateMetersPerSec();

  for (int axis = XAXIS; axis <= ZAXIS; axis++) {
    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
  }

  calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;
  if (!runtimaZBiasInitialized) {
    runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
    runtimaZBiasInitialized = true;
  }
  estimatedZVelocity += zVelocity;
  estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);
#endif    

#if defined(AltitudeHoldBaro)
  measureBaroSum(); 
  if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
    evaluateBaroAltitude();
  }
#endif

  processFlightControl(alpha, beta);


#if defined(BinaryWrite)
  if (fastTransfer == ON) {
    // write out fastTelemetry to Configurator or openLog
    fastTelemetry();
  }
#endif      

#ifdef SlowTelemetry
  updateSlowTelemetry100Hz();
#endif 


#if defined(UseGPS)
  updateGps();
#endif      

#if defined(CameraControl)
  moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
#if defined CameraTXControl
  processCameraTXControl();
#endif
#endif 


}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
void process50HzTask() {
  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;

  GETGPS(tiny);
  if (tiny[2] == 1)
    flagps[0] = 1;

  ReceiveIncoming(Incoming);

  // Reads external pilot commands and performs functions based on stick configuration
  readPilotCommands(beta, lengthOutput, widthOutput, hotel, delta, Remote_Val, Prev_Remote_Val, Landing, Emergency, no_remote_signal); 

#if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
  readRSSI();
#endif

#ifdef AltitudeHoldRangeFinder
  updateRangeFinders();
#endif

#if defined(UseGPS)
  if (haveAGpsLock() && !isHomeBaseInitialized()) {
    initHomeBase();
  }
#endif      

  CameraStabilizerServoTX();
  CameraStabilizerBrushlessTX();

}

/*******************************************************************
 * 10Hz task
 ******************************************************************/
void process10HzTask1() {

#if defined(HeadingMagHold)

  G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
  tenHZpreviousTime = currentTime;

  measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);

  calculateHeading();





#endif
}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
void process10HzTask2() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;

#if defined(BattMonitor)
  measureBatteryVoltage(G_Dt*1000.0);
#endif

  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();
}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
void process10HzTask3() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
  lowPriorityTenHZpreviousTime2 = currentTime;

#ifdef OSD_SYSTEM_MENU
  updateOSDMenu();
#endif

#ifdef MAX7456_OSD
  updateOSD();
#endif

#if defined(UseGPS) || defined(BattMonitor)
  processLedStatus();
#endif

#ifdef SlowTelemetry
  updateSlowTelemetry10Hz();
#endif
}

/*******************************************************************
 * 1Hz task 
 ******************************************************************/


void ReceiveIncoming(double Incoming[]){

  unsigned int x_in[1];


  if (radio.available())
  {
    bool done = false;
    while (!done)
      done = radio.read(x_in, sizeof(x_in));  


    if (x_in[0] == 0)
    {
      Incoming[2] = 1000;
      Incoming[0] = 0;
      Incoming[1] = 0;
    }

    else if (x_in[0]/10000 == 1){
      latBig = x_in[0] % 10000;
      key[0] = 1;
    }

    else if (x_in[0]/10000 == 2){
      latMed = x_in[0] % 20000;
      key[1] = 1;
    }

    else if (x_in[0]/10000 == 3)
    {
      longBig = x_in[0] % 30000;
      Incoming[3] = longBig - longBig/10*10 + 3;
      longBig /= 10;
      key[2] = 1;
    }

    else if (x_in[0]/10000 == 4){
      longMed = x_in[0] % 40000;
      key[3] = 1;
    }

    else if (x_in[0]/1000 == 50)
    {
      Incoming[2] = 2000; 
      Incoming[0] = 0;
      Incoming[1] = 0;
      Incoming[3] = x_in[0] % 50000 + 3;
    }

    else if (x_in[0]/1000 == 55)
    {
      Incoming[2] = 2000; 
      Incoming[0] = 123;
      Incoming[1] = 123;
    }

    for (int ijk = 0; ijk<=3; ijk++)
    {
      if (key[ijk] == 0)
        break;
      if (ijk == 3 & key[ijk] == 1)
      {
        Incoming[2] = 2000; 
        Incoming[0] = ((double)latBig*10000+latMed)/100000;
        Incoming[1] = ((double)longBig*10000+longMed)/100000*-1;
        for (int iijk = 0; iijk<=3; iijk++)
          key[iijk] = 0;
      }
    }

    Incoming[4] = 1;   
  }
  else 
    Incoming[4] = 0;  


  //    Serial.println(x_in[0]);
  //    Serial.println(Incoming[0],5);
  //    Serial.println(Incoming[1],5);
  //    Serial.println(Incoming[2]);
  //    Serial.println(Incoming[3]);
  //    Serial.println(Incoming[4]);
  //    Serial.println(" ");     

}


void Waypoint_Procedure(double next_coord[], double tiny[], int move_commands[], double Distance_Calc[], double stop_epsilon){

  next[1] = next_coord[1];
  next[0] = next_coord[0];
  x_dist = next_coord[1] - tiny[1];
  y_dist = next_coord[0] - tiny[0];       
  Distance_Calc[0] = sqrt(x_dist*x_dist + y_dist*y_dist);
  int move_forward;

  if (x_dist == 0)
    x_dist = 0.00001;

  if (y_dist == 0)
    y_dist = 0.00001;


  if (Distance_Calc[0] > stop_epsilon)
    move_forward = 1;
  else
    move_forward = 0;


  int theta = (atan(x_dist/y_dist) * 180/PI);

  if (y_dist < 0)
    alpha = 180 + theta;

  else if (x_dist<0 && y_dist>0)
    alpha = 360 + theta;

  else 
    alpha = 0 + theta;  

  move_commands[0] = alpha;
  move_commands[1] = move_forward; 

}

void setFlightEmergency(int beta, double Incoming[], int Emergency[], int passage[]){

  if (beta <= 1750 && Landing[0] == 0)
    Emergency[0] = 1;


  else if (beta > 1750 && Incoming[0] == 0 && Landing[0] == 0){
    Emergency[0] = 2;
    passage[0] = 1;
  }



  else if (beta > 1750 && Incoming[0] != 0 && Landing[0] == 0){
    Emergency[0] = 3;
    passage[0] = 1;
  }

}

void checkSignals(int flagps[0], int no_gps_signal[], double Incoming[], int no_receiver_signal[]){

  //---Remote---

  //Remote is checked in Receiver.h


  //---GPS---

  if (flagps[0] == 0)
    no_gps_signal[0]++;

  else{
    no_gps_signal[0] = 0;
    digitalWrite(30,LOW);
  }

  flagps[0] = 0;


  //---Receiver---

  if (Incoming[4] == 0)
    no_receiver_signal[0]++;
  else{
    no_receiver_signal[0] = 0;
    digitalWrite(28,LOW);
  }

}

void staticWayPoint(double wayGrid[][2], double waypoints[], double tiny[], int Emergency[], int Landing[], int move_commands[], double Distance_Calc[], double way_epsilon, int* wGc, int SIZE){

  //   Serial.println("Entered Static");

  if (Emergency[0] == 2)
  {

    //      Serial.println("Entered WayPoint Procedure");

    if (*wGc <= SIZE) // Else fly to next WayPoint
    {
      //        Serial.println("Entered Next WayPoint");
      waypoints[0] = wayGrid[*wGc][0];
      waypoints[1] = wayGrid[*wGc][1];
      Waypoint_Procedure(waypoints, tiny, move_commands, Distance_Calc, way_epsilon);
      if (move_commands[1] == 0)
      {
        (*wGc) = (*wGc) + 1;
        if (*wGc <= SIZE) // Recalculate and fly to next WayPoint
        {
          waypoints[0] = wayGrid[*wGc][0];
          waypoints[1] = wayGrid[*wGc][1];
          Waypoint_Procedure(waypoints, tiny, move_commands, Distance_Calc, way_epsilon);
        } 
      }
    }
  }
}


void dynamicWayPoint(double tiny[], double Incoming[], double prev_Incoming[], double Distance_Calc[], int move_commands[], double filter_epsilon, double way_epsilon){

  if (Emergency[0] == 3)
  {  

    Waypoint_Procedure(Incoming, prev_Incoming, move_commands, Distance_Calc, filter_epsilon); // This is used to determine distance between new incoming coordinates and get rid of noise 

    if (move_commands[1] == 0)
    {
      Incoming[0] = prev_Incoming[0];
      Incoming[1] = prev_Incoming[1];
    }
    else 
    {
      prev_Incoming[0] = Incoming[0];
      prev_Incoming[1] = Incoming[1];
    }

    Waypoint_Procedure(Incoming, tiny, move_commands, Distance_Calc, way_epsilon);
    //     Serial.println("Entered Follow Me");
  }  
}


void setHP(double tiny[], double Home_Position[], double Distance_Calc[], int Emergency[], int passage[], int Landing[], boolean motorArmed, int wGc){

  if (motorArmed == OFF)
    passage[0] = 1;

  if (tiny[0] != 0 && passage[0] == 1 && Landing[0] == 0 && motorArmed == ON)
  {        

    if (Emergency[0] == 3)
    {
      Home_Position[0] = tiny[0];
      Home_Position[1] = tiny[1];
    }

    else if (Emergency[0] == 2)
    {
      if (wGc <= SIZE){
        Home_Position[0] = wayGrid[wGc][0]; 
        Home_Position[1] = wayGrid[wGc][1];
      } 
    }
    else
    {
      Home_Position[0] = tiny[0];
      Home_Position[1] = tiny[1];
      passage[0] = 0;
    }
  }
}


void ExecuteFlightCommands(double Home_Position[], double tiny[] , double Distance_Calc[], int move_commands[],int no_remote_signal[], int no_gps_signal[], int no_receiver_signal[], int Emergency[], int Landing[], double way_epsilon, double distance_epsilon, double range_epsilon, int wGc, int SIZE){

  //--Emergency 1: Remote Signal--

  if (Emergency[0] == 1)
  {

    Waypoint_Procedure(Home_Position, tiny, move_commands, Distance_Calc, way_epsilon);

    if (no_remote_signal[0] >= 10 || Distance_Calc[0] > range_epsilon){
      //          Serial.println("No Remote or Virual Fence");  
      Landing[0] = 2;
    }

    else 
      Landing[0] = 0;
  }

  //--Emergency 2: Lost GPS Signal--

  else if (Emergency[0] == 2)
  {
    // ---Reset Home Position if Necessary---
    if (Distance_Calc[0] >= distance_epsilon)
    {
      //            Serial.println("Distance is Great");
      if (wGc == 0){
        Home_Position[0] = tiny[0]; 
        Home_Position[1] = tiny[1];
      }

      else{
        Home_Position[0] = wayGrid[wGc-1][0]; 
        Home_Position[1] = wayGrid[wGc-1][1];
      }

      Landing[0] = 2;
    }    

    else if (wGc > SIZE)
      Landing[0] = 2; 

    else
      Landing[0] = 0;          
  }

  // --Emergency 3: Lost Receiver-- 

  else if (Emergency[0] == 3) 
  {
    if (no_receiver_signal[0] >= 7 || Distance_Calc[0] >= distance_epsilon || Incoming[0] == 123 ){
      //            Serial.println("No Receiver or Too Far to Dynamic WayPoint");  
      Landing[0] = 2;
    }

    else
      Landing[0] = 0;      
  }

  if (no_gps_signal[0] >= 7 || Home_Position[0] == 0){
    //          Serial.println("No Signal GPS");
    Landing[0] = 1;
  }

  // Set Lights ON if No Signal for all Flight Modes

  if (no_gps_signal[0] >= 7)
    digitalWrite(30,HIGH);

  if (no_receiver_signal[0] >= 7)
    digitalWrite(28,HIGH);    
}


void landingProcedure(double Home_Position[], double tiny[] , double Distance_Calc[], int move_commands[], int Landing[], int Emergency[], int* beta, float* hotel, double way_epsilon, float Landing_Height){

  if (Landing[0] == 2 || Landing[0] == 1)    // Return Home if Landing 2
  {     

    Waypoint_Procedure(Home_Position, tiny, move_commands, Distance_Calc, way_epsilon); 
    *beta = 2000;  

    if (Emergency[0] == 1)
      *hotel = Static_Alt;
    else
      *hotel = Flight_Height;

    if (move_commands[1] == 0){
      //          Serial.println("Move Commands");
      Landing[0] = 1;
    }
  }   


  if (Landing[0] == 1)    // Stop moving and come low to the ground
  {
    *beta = 2000;             // Automation turns on and Alitude Hold at Auto Height activates
    *hotel = Landing_Height;  // Sets the landing height 
    move_commands[1] = 0;     // Turns off move forward and allows for one Stop Procedure
  }   
}

void positionPIDcalc(){


  if (Emergency[0] == 1 && Landing[0] == 0){
    prev_next[0] = 0;
    prev_next[1] = 0;
  }

  if (next[0] == 0){
    lengthOutput = 0;
    widthOutput = 0;
  }

  else{

    if (next[0] != prev_next[0] || next[1] != prev_next[1])
    {
      setGridAngle = alpha; 
      prev_next[0] = next[0];
      prev_next[1] = next[1];   
    }


    Serial.println(Distance_Calc[0]);

    if (Distance_Calc[0] == 0) // This applied to Follow Me when losing signal won't yaw to 45 degrees. Now, it will just stay at its current heading
    {
      setGridAngle = setHeading;
      alpha = setHeading;
    }


    //First use alpha to measure the necessary flight corrections

    length_dist = -Distance_Calc[0]*cos((setGridAngle - alpha)*PI/180);
    length_dist *= 1000000;
    lengthSetpoint = 0;   //-0.00000* 1000000; // Output easier in non decimal
    lengthPID.Compute();  

    width_dist = Distance_Calc[0]*sin((setGridAngle - alpha)*PI/180);   
    width_dist *= 1000000;
    widthSetpoint = 0;
    widthPID.Compute(); 

    //Then set alpha to be equal to setGridAngle so flight path is parallel to original orientation 
    alpha = setGridAngle;

    //      Serial.print("next[0]: ");
    //      Serial.println(next[0],5);
    //      Serial.print("next[1]: ");
    //      Serial.println(next[1],5);
    //      Serial.print("Distance_Calc[0]: ");
    //      Serial.println(Distance_Calc[0],5);
    //      Serial.print("setGridAngle: ");
    //      Serial.println(setGridAngle);
    //      Serial.print("alpha:");
    //      Serial.println(alpha);
    //      Serial.print("length_dist = ");
    //      Serial.println(length_dist/1000000,5);
    //      Serial.print("width_dist= ");
    //      Serial.println(width_dist/1000000,5);
    //      Serial.print("lengthOutput: ");
    //      Serial.println(lengthOutput);
    //      Serial.print("widthOutput: ");
    //      Serial.println(widthOutput);
    //      Serial.println(" ");

  }
}

void process1HzTask() {
  G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
  oneHZpreviousTime = currentTime;


#ifdef MavLink
  sendSerialHeartbeat();   
#endif
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {

  currentTime = micros();
  deltaTime = currentTime - previousTime;

  if (frameCounter == 0){
    CycleTime = micros();
  }

  measureCriticalSensors();

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {

    frameCounter++;

    process100HzTask();

    // ================================================================
    // 50Hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }

    // ================================================================
    // 10Hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
      process10HzTask1();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
      process10HzTask2();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
      process10HzTask3();
    }

    // ================================================================
    // 2Hz task loop
    // ================================================================
    if (frameCounter % 50 == 0) {  //   2 Hz tasks
      process1HzTask();

      if (receiverCommand[MODE] > 1500){

        //        //    ---Simulation---
        //
        //        tiny[0] = 26.12300;
        //        tiny[1] = -80.12300;
        //        
        //        float tortuse = map(receiverCommand[THROTTLE],1000,2000,-500,500);
        //        
        //        float turtle = map(receiverCommand[XAXIS],1000,2000,-200,200);
        //        
        //        tiny[0] += tortuse/1000000;
        //        tiny[1] += turtle/1000000;
        //        
        //  
        //        //    ---Simulation---

        beta = Incoming[2];   // Gets passed to AltitudeControlProcessor.h, HeadingHoldProcessor.h and Receiver.h to turn on automation
        Flight_Height = Incoming[3]; 
        hotel = Flight_Height;

        setFlightEmergency(beta, Incoming, Emergency, passage);

        checkSignals(flagps, no_gps_signal, Incoming, no_receiver_signal);

        staticWayPoint(wayGrid, waypoints, tiny, Emergency, Landing, move_commands, Distance_Calc, way_epsilon, &wGc, SIZE);   

        dynamicWayPoint(tiny, Incoming, prev_Incoming, Distance_Calc, move_commands, filter_epsilon, way_epsilon); 

        setHP(tiny, Home_Position, Distance_Calc, Emergency, passage, Landing, motorArmed, wGc);

        ExecuteFlightCommands(Home_Position, tiny, Distance_Calc, move_commands, no_remote_signal, no_gps_signal, no_receiver_signal, Emergency, Landing, way_epsilon, distance_epsilon, range_epsilon, wGc, SIZE);

        landingProcedure(Home_Position, tiny, Distance_Calc, move_commands, Landing, Emergency, &beta, &hotel, way_epsilon, Landing_Height);

        positionPIDcalc();


//        Serial.print("millis(): "); 
//        Serial.println(millis());
//        Serial.print("Emergency[0]: ");
//        Serial.println(Emergency[0]); 
//        Serial.print("tiny[0]: ");
//        Serial.println(tiny[0],5);  
//        Serial.print("tiny[1]: ");
//        Serial.println(tiny[1],5); 
//        Serial.print("Incoming[1]: ");
//        Serial.println(Incoming[1],5);
//        Serial.print("Home_Position[0]: ");
//        Serial.println(Home_Position[0],5);    
//        Serial.print("Landing[0]: ");
//        Serial.println(Landing[0]);      
//        Serial.print("waypoints[0]: ");
//        Serial.println(waypoints[0],5);      
//        Serial.print("waypoints[1]: ");
//        Serial.println(waypoints[1],5);
//        Serial.print("relativeHeading: ");
//        Serial.println(relativeHeading);
//        Serial.print("alpha: ");
//        Serial.println(alpha);     
//        Serial.print("setHeading: ");
//        Serial.println(setHeading); 
//        Serial.print("beta: ");
//        Serial.println(beta);     
//        Serial.print("delta: ");
//        Serial.println(delta); 
//        Serial.print("BUMP: "); 
//        Serial.println(BUMP);    
//        Serial.print("hotel: ");
//        Serial.println(hotel,2);          
//        Serial.print("wGc: ");
//        Serial.println(wGc);      
//        Serial.print("Distance_Calc[0]: ");
//        Serial.println(Distance_Calc[0],5);
//        Serial.println(" ");

      }

      else
      {
        beta = 0;
        hotel = Static_Alt;
      }  
    } 

    previousTime = currentTime;
  }

  if (frameCounter >= 100) {
    //unsigned long DifferentialTimeX = micros() - CycleTime; 
    //Serial.println(DifferentialTimeX*2);
    frameCounter = 0;
  }
}















