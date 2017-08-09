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
    fnamestream << "log_" << time(NULL) <<"_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);

    unsigned long stopTimestamp = 0;
    unsigned long leftTimestamp = 0;
    unsigned long returnLeftTimestamp = 0;
    unsigned long rightTimestamp = 0;
    unsigned long returnRightTimestamp = 0;
    unsigned long rightSum = 0;
    unsigned long leftSum = 0;
    unsigned long roundTimestamp = 0;
    unsigned long roundTime = 100000;
    float stopAngle = 0.0;
    float bypassingAngle = 0.0;
    int state = 0;
    bool left = false;
    bool right = false;
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
                robot.SetWheel(50,50);
                break;
            case 's':
                robot.SetWheel(-50,-50);
                break;
            case 'a':
                robot.SetWheel(-50,0);
                break;
            case 'd':
                robot.SetWheel(0,-50);
                break;
            case 'e':
                robot.SetWheel(0,0);
                break;
            }
            usleep(200);

        }
        robot.GetState(&robotState);
        // TODO log
        printf("%4.1f | %c | %4.1f | %4.1f | state=%i \n", robotState.distFront, lastKb, robotState.distLeft, robotState.distRight, state);
        printf("%lu| %lu \n", robotState.microsTimestamp, roundTimestamp);
        //printf("%f | %f \n", stopAngle, bypassingAngle);
        //printf("%lu | %lu \n", leftTimestamp, rightTimestamp);
        /* MongoDB*/
        //db.WriteState(&robotState);

        /* Read command from db */
        //string command = db.ReadCommand();

        /* Process */
        //ProcessCommand(command,&robot, &robotState, &db);

        if(robotState.distFront < 25.0 && lastKb == 'w' && state == 0)
        {
            robot.SetWheel(30,30);
            state = 1;
        }

        switch(state)
        {
        case 1://napotkanie przeszkody i skret w lewo
            if(robotState.distFront < 15.0 )
            {
                stopAngle = robot.GetAngle();
                robot.SetWheel(0,0);
                stopTimestamp = robotState.microsTimestamp;
                printf("Stop: %lu \n", stopTimestamp);

                if(rightTimestamp == 0)
                {
                    robot.SetWheel(50,-50);
                    state = 2;
                }
            }
            break;
        case 2://policzenie czasu do napotkania krawedzi z lewego czujnika i odkrecanie w lewo
            if(robotState.distLeft < 25.0)
            {
                robot.SetWheel(0,0);

                returnRightTimestamp = robotState.microsTimestamp;

                rightTimestamp = returnRightTimestamp - stopTimestamp;
                rightSum = returnRightTimestamp + rightTimestamp + 100000;
                printf("Right: %lu \n", rightTimestamp);
                robot.SetWheel(-50,50);
                state = 4;
            }
            break;
        case 3://policzenie czasu do napotkania krawedzi z prawego czujnika i odkrecanie w prawo
            if(robotState.distRight < 25.0)
            {
                robot.SetWheel(0,0);
                returnLeftTimestamp = robotState.microsTimestamp;
                leftTimestamp = returnLeftTimestamp - stopTimestamp;
                leftSum = returnLeftTimestamp + leftTimestamp + 100000;
                printf("Left: %lu \n", leftTimestamp);
                robot.SetWheel(50,-50);
                state = 5;
            }
            break;
        case 4://powrot do wczesniejszej pozycji (na podstawie czasu skretu w prawo) i odkrecanie w lewo
            if(rightSum > robotState.microsTimestamp )
            {
                printf("righttimestamp");
                robot.Cycle();
            }
            if(rightSum < robotState.microsTimestamp)
            {
                robot.SetWheel(0,0);
                stopTimestamp = robotState.microsTimestamp;
                printf("righttimestampstop: %lu \n", stopTimestamp);
                robot.SetWheel(-50,50);
                state = 3;
            }
            break;
        case 5://po powrocie do pierwszej pozycji zatrzymanie i wypisanie czasow skretu w lewo i prawo
            if(leftSum > robotState.microsTimestamp )
            {
                printf("lefttimestamp");
                robot.Cycle();
            }
            else if(leftSum < robotState.microsTimestamp)
            {
                robot.SetWheel(0,0);
                printf("LeftTimestamp: %lu | RightTimestamp: %lu \n", leftTimestamp, rightTimestamp);
                state = 6;
            }
            break;
        case 6://obliczenie kierunku objazdu przeszkody
            if(rightTimestamp > leftTimestamp)
            {
                state = 8;
            }
            else if(leftTimestamp > rightTimestamp)
            {
                state = 8;
            }
            break;
        case 7://ominiecie przeszkody z prawej strony
            right = true;
            robot.SetWheel(40,-40);
            printf("obrot w prawo");
            stopTimestamp = robotState.microsTimestamp;
            roundTimestamp = stopTimestamp + 1000000;
            state = 9;
            break;
        case 8://ominiecie przeszkody z lewej strony
            left = true;
            robot.SetWheel(-40,40);
            stopTimestamp = robotState.microsTimestamp;
            roundTimestamp = stopTimestamp + 1000000;
            state = 9;
            break;
        case 9://ruch do przodu
            if(robotState.distFront > 50.0)
            {
                printf("do przodu \n");
                robot.SetWheel(40,40);
                state = 10;
            }
        case 10:
            if(right && robotState.microsTimestamp > roundTimestamp)
            {
                //stopTimestamp = robotState.microsTimestamp;
                //roundTimestamp = stopTimestamp + roundTime;
                state = 11;
            }
            else if(left && robotState.microsTimestamp > roundTimestamp)
            {
                //stopTimestamp = robotState.microsTimestamp;
                //roundTimestamp = stopTimestamp + roundTime;
                state = 12;
            }
            break;
        case 11://obrkazanie z prawej
            if(robotState.distLeft > 30.0)
            {
                printf("w lewo \n");
                robot.SetWheel(-40,40);
            }
            else if(robotState.distLeft < 30.0)
            {
                stopTimestamp = robotState.microsTimestamp;
                roundTimestamp = stopTimestamp + roundTime;
                state = 9;
            }
            else
            {
                printf("nie znalazl \n");
                robot.SetWheel(0,0);
            }
            break;
        case 12://obkrazanie z lewej
            if(robotState.distRight > 30.0)
            {
                printf("w prawo \n");
                robot.SetWheel(40,-40);
            }
            else if(robotState.distRight < 30.0)
            {
                stopTimestamp = robotState.microsTimestamp;
                roundTimestamp = stopTimestamp + roundTime;
                state = 9;
            }
            else
            {
                printf("nie znalazl \n");
                robot.SetWheel(0,0);
            }
            break;
        default:
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
