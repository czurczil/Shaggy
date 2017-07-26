#include <iostream>
#include <string>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <limits>
#include <sstream>

#include <wiringPi.h>
#include <wiringSerial.h>


#include "utils.h"
#include "terminal_functions.h"
#include "robot.h"
#include "DB.h"

using namespace std;


void robot_info(void)
{
    printf("\r\nRobot Kudlaty (2016) oparty o podzespoly\r\n");
    printf("- podwozie uniwersalne trzykolowe z dwoma silnikami DC\r\n");
    printf("- komputer RaspberryPi 2\r\n");
    printf("- modul inercyjny GY-80: ADXL245B, HMC5883L, L3G4200D, BMP085\r\n");
    printf("- ultradzwiekowe czujniki odleglosci HC-SR04\r\n");
    printf("- dwukanalowy kontroler silnikow DC: DRV8835\r\n");
    printf("- czterokanalowy konwerter poziomow logicznych Sparkfun\r\n");
    printf("- przetwornica StepUpDown zasilajaca RPI\r\n");
    printf("- zasilanie akumulatorowe: 4 x AA NiMh\r\n");
}


void print_main_menu(void)
{
    printf("Wybierz:\r\n");
    printf("m - sterowanie reczne\r\n");
    printf("t - wczytaj trajektorie z pliku\r\n");
    printf("e - tryb ucieczki od przeszkody\r\n");
    printf("k - kalibracja magnetometru (kompasu)\r\n");
    printf("w - regulacja kata\r\n");
    printf("o - regulacja odleglosci\r\n");
    printf("d - kat i odleglosc\r\n");
    printf("? - informacje o robocie\r\n");

}

static int mtmState=0;
int mtmCount=0;
void MakeTerrainMap(Robot *robot, RobotState *state, DB *db)
{
    switch(mtmState)
    {
    case 0: // do nothing

        break;
    case 1:
        db->ClearTerrainMap();
        mtmState=2;
        break;
    case 2:
        robot->SetWheel(-33,33);
        mtmCount++;
        if(mtmCount > 400)
        {
            mtmState=0;
            mtmCount=0;
            robot->SetWheel(0,0);
        }
        db->WriteStateAsMap(state);
        // cout<<mtmCount<<endl;
        break;
    default:
        mtmState=0;
    }
}

void ProcessCommand(string cmd, Robot *robot, RobotState *state, DB *db)
{
    if(cmd == "mm")
    {
        mtmState=1;
    }
    else if(cmd == "cm")
    {
        db->ClearTerrainMap();
    }
    MakeTerrainMap(robot, state, db);

}

/*
int CheckRightTurnTimestamp(Robot robot, RobotState robotState)
{
    robot.SetWheel(0,0);
    int stopTimestamp = robotState.microsTimestamp;
    int rightTimestamp = 0;
    robot.SetWheel(40,-40);
    if(robotState.distLeft > (15.0 + 5.0))
    {
        robot.SetWheel(0,0);
        rightTimestamp = robotState.microsTimestamp - stopTimestamp;
        int returnTimestamp = robotState.microsTimestamp;
        robot.SetWheel(-40,40);
        if(robotState.microsTimestamp == returnTimestamp + rightTimestamp)
        {
            robot.SetWheel(0,0);
        }
    }
    return rightTimestamp;
}
*/
int main()
{

    changemode(1); // zmiana trybu terminala na nieblokujacy
    int priority=70;

    char kb=0;
    char lastKb = 0;
    printf("Edukacyjny robot \"Kudlaty\" wersja 1.1 2017 \r\n");
    int res =  piHiPri(priority);
    printf("Ustawienie priorytetu na %i: %s\r\n",priority, (res == 0?"OK":"ERROR"));
    //print_main_menu();
    //DB db;
    //db.Open();
    /*Inicjalizacja robota*/
    RobotState robotState;
    Robot robot;
    robot.Init();
    robot.SetWheel(0,0); // stop
    robot.Cycle(); //first cycle

    stringstream fnamestream;
    fnamestream <<"log_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);

    bool checkTimespans = false;
    unsigned long stopTimestamp = 0;
    unsigned long leftTimestamp = 0;
    unsigned long returnLeftTimestamp = 0;
    unsigned long rightTimestamp = 0;
    unsigned long returnRightTimestamp = 0;
    unsigned long rightSum = 0;
    /*Main loop*/
    while(kb!= 'q')
    {
        /* Input */
        kb=0;
        if(kbhit())
        {
            cin>>kb;
            lastKb = kb;
            switch(kb)
            {
            case '?':
                robot_info();
                print_main_menu();
                break;
            case 'w':
                robot.SetWheel(40,40);
                break;
            case 's':
                robot.SetWheel(-40,-40);
                break;
            case 'a':
                robot.SetWheel(-40,0);
                break;
            case 'd':
                robot.SetWheel(0,-40);
                break;
            case 'e':
                robot.SetWheel(0,0);
                break;
            }
            usleep(200);

        }
        robot.GetState(&robotState);
        // TODO log
        printf("%4.1f | %c | %4.1f | %4.1f | %lu | %lu \n", robotState.distFront, lastKb, robotState.distLeft, robotState.distRight, robotState.microsTimestamp, rightSum);
        printf("%lu | %lu \n", leftTimestamp, rightTimestamp);
        /* MongoDB*/
        //db.WriteState(&robotState);

        /* Read command from db */
        //string command = db.ReadCommand();

        /* Process */
        //ProcessCommand(command,&robot, &robotState, &db);


        if(robotState.distFront < 15.0 && lastKb == 'w' && stopTimestamp == 0 && rightSum == 0)
        {
            checkTimespans = true;
            robot.SetWheel(0,0);
            stopTimestamp = robotState.microsTimestamp;
            printf("Stop: %lu \n", stopTimestamp);

            if(rightTimestamp == 0)
            {
                robot.SetWheel(40,-40);
            }
            else if(leftTimestamp == 0)
            {
                robot.SetWheel(-40,40);
            }
            if(leftTimestamp != 0 && rightTimestamp != 0)
            {
                printf("LeftTimestamp: %lu | RightTimestamp: %lu", leftTimestamp, rightTimestamp);
                break;
            }
        }
        if(robotState.distLeft < 25.0 && rightTimestamp == 0 && checkTimespans == true)
        {
            robot.SetWheel(0,0);

            returnRightTimestamp = robotState.microsTimestamp;

            rightTimestamp = returnRightTimestamp - stopTimestamp;
            rightSum = returnRightTimestamp + rightTimestamp;
            printf("Right: %lu \n", rightTimestamp);
            robot.SetWheel(-40,40);
            stopTimestamp = 0;
        }
        if(robotState.distRight < 25.0 && leftTimestamp == 0 && rightTimestamp != 0 && checkTimespans == true)
        {
            robot.SetWheel(0,0);
            returnLeftTimestamp = robotState.microsTimestamp;
            leftTimestamp = returnLeftTimestamp - stopTimestamp;
            printf("Left: %lu \n", leftTimestamp);
            robot.SetWheel(40,-40);
            stopTimestamp = 0;
        }
        if((rightSum) < robotState.microsTimestamp && rightSum != 0)
        {
            robot.SetWheel(0,0);
            printf("righttimestampstop");
            rightSum = 0;
        }
        if(rightSum > robotState.microsTimestamp )
        {
            printf("righttimestamp");
            robot.Cycle();
        }
        robot.Cycle();
        /* Output */
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
    //db.Close();
    changemode(0);
    return 0;
}
