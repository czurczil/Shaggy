#include <iostream>
#include <cstdio>

#include <wiringPi.h>
#include <softPwm.h>

#include "robot.h"

int echo_sensors_pins[4][2]={{21,22},{26,23},{27,24},{28,29}} ;


using namespace std;


Robot::Robot()
{
    //ctor
    m_LeftWheel = 0;
    m_RightWheel = 0;
    m_CycleMillis = 0;
    m_DistLeft = 0;
    m_DistRight = 0;
    m_DistFront = 0;
    m_DistBack = 0;
    m_AccelX = 0;
    m_AccelY = 0;
    m_Angle = 0;
    m_SilenceInit=true;
    m_CycleNumber=0;
}

Robot::~Robot()
{
    //dtor
}

void Robot::CompassCalibrationSetOffset(int16_t xo, int16_t yo)
{
    compass.setOffset(xo,yo);
}

Vector Robot::CompassCalibrationCycle(int l, int r)
{
    SetMove(l, r,0);

    compass.startSingleMeasure(); // start measurement
//unsigned int anglems=micros();
//while(!compass.checkRDYFromStatus());
while(digitalRead(GY80_M_DRDY) > 0); // wait for measurement
while(digitalRead(GY80_M_DRDY) == 0); // wait for write output registers

//printf(" Angle Measurement time: %u",micros()-anglems);
//m_Angle = compass.getAngle();

    return compass.readRaw();
}

int Robot::Cycle()
{
int start_ms = millis();
m_CycleNumber++;
// Set Movement
SetMove(m_LeftWheel, m_RightWheel,0);
// Measure Distance
m_DistFront = UsonicReadCM(echo_sensors_pins[0][0],echo_sensors_pins[0][1] ) + 7.7;
//m_DistBack  = UsonicReadCM(echo_sensors_pins[1][0],echo_sensors_pins[1][1] ) + 7.7;
m_DistLeft  = UsonicReadCM(echo_sensors_pins[2][0],echo_sensors_pins[2][1] ) + 5.75;
//m_DistRight = UsonicReadCM(echo_sensors_pins[3][0],echo_sensors_pins[3][1] ) + 5.75;
m_DistBack  = UsonicReadCM(echo_sensors_pins[1][0],echo_sensors_pins[1][1] )+7.7;
//m_DistLeft  = 0;//UsonicReadCM(echo_sensors_pins[2][0],echo_sensors_pins[2][1] )+5.75;
m_DistRight = UsonicReadCM(echo_sensors_pins[3][0],echo_sensors_pins[3][1] )+5.75;

// Measure Angle

compass.startSingleMeasure(); // start measurement
//unsigned int anglems=micros();
//while(!compass.checkRDYFromStatus());
while(digitalRead(GY80_M_DRDY) > 0); // wait for measurement
while(digitalRead(GY80_M_DRDY) == 0); // wait for write output registers

//printf(" Angle Measurement time: %u",micros()-anglems);
m_Angle = compass.getAngle();
//if(m_Angle < 0) m_Angle += 360;



// Measure Acceleration with 10 ms Cycle
//unsigned int aaa = micros();

Activites act;
do
{
    act = accelerometer.readActivites(false);
}

while(!act.isDataReady);
//
//while(digitalRead(GY80_AINT_1)==0);
//printf(" Accel Measurement time: %u",micros()-aaa);
    Vector norm = accelerometer.readNormalize();
     m_AccelX = norm.XAxis;
     m_AccelY = norm.YAxis;
     m_AccelZ = norm.ZAxis;
     Log();
     m_CycleMillis = millis()-start_ms;
return m_CycleMillis;
}


int CheckTimeout(unsigned int start_us, unsigned int timeout_us)
{
    int res=0;
    unsigned int curr_us=micros();
    if(curr_us <= start_us){
        res = ((curr_us + (0xFFFFFFFF - start_us) >= timeout_us) ?1:0);
    }
    else {
        res = ((curr_us - start_us) >= timeout_us ?1:0);
    }
    return res;
}

float Robot::UsonicReadCM(int trig, int echo) {
        //Send trig pulse
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        delayMicroseconds(240); //send signal
        unsigned int timeout = micros();
        int tim=0;
        //Wait for echo start
        while(digitalRead(echo) == LOW && !tim ) tim=CheckTimeout(timeout,3000);
        if(tim) {
            delayMicroseconds((100));
        return 0;}
       // while(digitalRead(echo) == LOW);
        //Wait for echo end
        long startTime = micros();
        tim=0;
        delayMicroseconds(60); // 2 cm
        while(digitalRead(echo) == HIGH && !tim) tim=CheckTimeout(startTime,7000);
        if(tim)
        {
            delayMicroseconds((10));
            return 120;
        }
    //    while(digitalRead(echo) == HIGH);
        long travelTime = micros() - startTime;

        //Get distance in cm
        float distance = travelTime / 58.0;
         delayMicroseconds(10);
//        delayMicroseconds(3200);
//          delayMicroseconds(1000);
        return distance;
}






//int abs(int _xx)
//{
//	if(_xx>=0) return _xx;
//	return -_xx;
//}

void Robot::SetMove(int l, int r, int save_direction)
{

	softPwmWrite(AENBL,abs(l));
    softPwmWrite(BENBL,abs(r));
    if(!save_direction)
    {
		int lphase = (l>=0 ? 0:1);
		int rphase = (r>=0 ? 0:1);

		digitalWrite(APHASE, lphase);
		digitalWrite(BPHASE, rphase);
	}
}


int Robot::Init_RPI()
{
    return wiringPiSetup();

}

void Robot::Init_Motor()
{
    pinMode(BIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(MOTOR_MODE, OUTPUT);
    digitalWrite(MOTOR_MODE, HIGH);

    softPwmCreate(AENBL,0,100);
    softPwmCreate(BENBL,0,100);
    softPwmWrite(AENBL,0);
    softPwmWrite(BENBL,0);
}
void Robot::Init_Usonic()
{
    for(int i=0;i<4;i++)
    {
        pinMode(echo_sensors_pins[i][0], OUTPUT);
        pinMode(echo_sensors_pins[i][1], INPUT);
        digitalWrite(echo_sensors_pins[i][0], LOW);
        delay(30);
    }
        //TRIG pin must start LOW
}

void Robot::Init_IMU()
{
	pinMode(GY80_AINT_1, INPUT);
    pinMode(GY80_M_DRDY, INPUT);

}
void Robot::Init_Accel()
{
   // Initialize ADXL345
  if(!m_SilenceInit) printf("Initialize ADXL345");
  if (!accelerometer.begin())
  {
    printf("Could not find a valid ADXL345 sensor, check wiring!");
    delay(200);
  }

  // Set measurement range
  // +/-  2G: ADXL345_RANGE_2G
  // +/-  4G: ADXL345_RANGE_4G
  // +/-  8G: ADXL345_RANGE_8G
  // +/- 16G: ADXL345_RANGE_16G
  accelerometer.setRange(ADXL345_RANGE_16G);
  accelerometer.setDataRate( ADXL345_DATARATE_100HZ);
  accelerometer.useInterrupt( ADXL345_INT1);
  //accelerometer.invertInterrupt();

  // Show current setting
  if(!m_SilenceInit)
  {
    accelerometer.showRange();
    accelerometer.showDataRate();
   }
    fflush(stdout);
  delay(100);
}
void Robot::Init_Compas()
{
    //  Serial.begin(9600);

  // Initialize HMC5883L
  if(!m_SilenceInit) printf("Initialize HMC5883L");
  while (!compass.begin())
  {
    printf("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA
  // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
  // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
  // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
  // +/- 4.00 Ga: HMC5883L_RANGE_4GA
  // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
  // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
  // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  // Idle mode:              HMC5883L_IDLE
  // Single-Measurement:     HMC5883L_SINGLE
  // Continuous-Measurement: HMC5883L_CONTINOUS (default)
  //compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setMeasurementMode(HMC5883L_SINGLE);

  // Set data rate
  //  0.75Hz: HMC5883L_DATARATE_0_75HZ
  //  1.50Hz: HMC5883L_DATARATE_1_5HZ
  //  3.00Hz: HMC5883L_DATARATE_3HZ
  //  7.50Hz: HMC5883L_DATARATE_7_50HZ
  // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
  // 30.00Hz: HMC5883L_DATARATE_30HZ
  // 75.00Hz: HMC5883L_DATARATE_75HZ
  compass.setDataRate(HMC5883L_DATARATE_7_5HZ);

  // Set number of samples averaged
  // 1 sample:  HMC5883L_SAMPLES_1 (default)
  // 2 samples: HMC5883L_SAMPLES_2
  // 4 samples: HMC5883L_SAMPLES_4
  // 8 samples: HMC5883L_SAMPLES_8
  //compass.setSamples(HMC5883L_SAMPLES_4);
  compass.setSamples(HMC5883L_SAMPLES_1);
  compass.setOffset(87,25);
  // Check settings
  if(!m_SilenceInit) compass.checkSettings();
  fflush(stdout);
  delay(100);
}

int Robot::Init()
{
int result = Init_RPI();
Init_Motor();
Init_Usonic();
Init_Accel();
Init_Compas();

return result;
}


void Robot::StartLog(std::string filename)
{
if(m_LogStream.is_open()) m_LogStream.close();
m_LogStream.open(filename.c_str());
if(m_LogStream.is_open())
    {
        m_LogStream << "t;";
        m_LogStream << "ax;";
        m_LogStream << "ay;";
        m_LogStream << "az;";
        m_LogStream << "distF;";
        m_LogStream << "distB;";
        m_LogStream << "distL;";
        m_LogStream << "distR;";
        m_LogStream << "alpha;";
        m_LogStream << "powerL;";
        m_LogStream << "powerR;";
        //m_LogStream << ";";
        m_LogStream << endl;

    }

}
void Robot::Log()
{
    if(m_LogStream != NULL &&  m_LogStream.is_open())
    {
        m_LogStream<<millis()<<";";
        m_LogStream<<GetAccelX()<<";";
        m_LogStream<<GetAccelY()<<";";
        m_LogStream<<GetAccelZ()<<";";
        m_LogStream<<GetDistFront()<<";";
        m_LogStream<<GetDistBack()<<";";
        m_LogStream<<GetDistLeft()<<";";
        m_LogStream<<GetDistRight()<<";";
        m_LogStream<<GetAngle()<<";";
        m_LogStream<<GetLeftWheel()<<";";
        m_LogStream<<GetRightWheel()<<";";

        m_LogStream<<endl;

    }

}

void Robot::StopLog()
{
m_LogStream.close();
}
