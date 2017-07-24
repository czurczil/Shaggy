#include <cstdlib>
#include <cmath>
#include <string>
#include <fstream>

#include "ADXL345.h"
#include "HMC5883L.h"


#ifndef ROBOT_H
#define ROBOT_H


#define BIN2	0
#define BIN1	1
#define AIN2	3
#define AIN1	4
#define MOTOR_MODE	2

#define BENBL  BIN2
#define BPHASE BIN1
#define AENBL  AIN2
#define APHASE AIN1

#define GY80_AINT_1 5
#define GY80_M_DRDY 7

typedef struct _RobotState
{
    float ax,ay,az; // acceletations in 3 dimensions xyz [m/s2]
    float angle; // flat orientation angle 0 - 360 deg [degreese]
    float distFront,distBack,distLeft,distRight; // distances from Ultra Sonic sensors, [cm]
    int wheelRightPWM, wheelLeftPWM; // PWM power set to wheels, 0 - 100 [%]
    int cycleMillis; // real cycle time [ms]
    unsigned long cycleNumber; // cycle number
    unsigned long microsTimestamp; // timestamp in microseconds, approximately wrap after 71 minutes [us]
    unsigned long millisTimestamp; // timestamp in milliseconds, wrap after 49 days [ms]
    unsigned long sysTimestamp; // system timestamp in seconds [s]

}RobotState;

class Robot
{
    public:
        Robot();
        virtual ~Robot();
        void StartLog(std::string filename);
        void StopLog();
        int GetCycleMillis() { return m_CycleMillis; }
        int GetLeftWheel() { return m_LeftWheel; }
        void SetLeftWheel(int val) { m_LeftWheel = val; }
        int GetRightWheel() { return m_RightWheel; }
        void SetRightWheel(int val) { m_RightWheel = val; }
        void SetWheel(int left, int right) {  m_RightWheel = right; m_LeftWheel=left; }
        void SetWheelPreserveDirection(int left, int right)
        {
            m_RightWheel = ( m_RightWheel > 0? abs(right) : -abs(right));
            m_LeftWheel =  ( m_LeftWheel>0? abs(left) : -abs(left));
        }
        void GetState(RobotState *state)
        {
            state->ax = m_AccelX;
            state->ay = m_AccelY;
            state->az = m_AccelZ;

            state->angle = m_Angle;

            state->distFront = m_DistFront;
            state->distBack = m_DistBack;
            state->distLeft= m_DistLeft;
            state->distRight = m_DistRight;

            state->cycleMillis = m_CycleMillis;
            state->wheelLeftPWM = m_LeftWheel;
            state->wheelRightPWM = m_RightWheel;
            state->cycleNumber=m_CycleNumber;

            state->microsTimestamp = micros();
            state->millisTimestamp = millis();
            state->sysTimestamp = time(NULL);
        }
        float GetDistLeft() { return m_DistLeft; }
        float GetDistRight() { return m_DistRight; }
        float GetDistFront() { return m_DistFront; }
        float GetDistBack() { return m_DistBack; }
        float GetAccelX() { return m_AccelX; }
        float GetAccelY() { return m_AccelY; }
        float GetAccelZ() { return m_AccelZ; }
        float GetAngle() { return m_Angle; }
        int Init();
        int Cycle();
        Vector CompassCalibrationCycle(int l, int r);
        void CompassCalibrationSetOffset(int16_t xo, int16_t yo);
        int16_t CompassGetOffsetX() {return compass.getOffsetX();}
        int16_t CompassGetOffsetY() {return compass.getOffsetY();}
        int16_t CompassGetOffsetZ() {return compass.getOffsetZ();}
    protected:
    private:
        std::string m_LogFilename;
        std::ofstream m_LogStream;
        int m_LeftWheel;
        int m_RightWheel;
        int m_CycleMillis;
        float m_DistLeft;
        float m_DistRight;
        float m_DistFront;
        float m_DistBack;
        float m_AccelX;
        float m_AccelY;
        float m_AccelZ;
        float m_Angle;
        bool m_SilenceInit;
        unsigned long m_CycleNumber;
        HMC5883L compass;
        ADXL345 accelerometer;

        float UsonicReadCM(int trig, int echo);
        void  SetMove(int l, int r, int save_direction=0);

        int Init_RPI();
        void Init_Motor();
        void Init_Usonic();
        void Init_Accel();
        void Init_Compas();
        void Check_Accel_Settings();
        void Check_Compas_Settings();
        void Log();


        void Init_IMU();
};

#endif // ROBOT_H
