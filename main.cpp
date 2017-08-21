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

void robot_regulacja_odleglosci_prawy(float distRight, float stopAngle)
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci prawy czujnik"<<endl;

    float temp_stop;
    /*ostringstream fnamestream;
    fnamestream <<"log_"<<time(NULL)<<".csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);*/
    float currentAngle = 0.0;

    float w=distRight,e, u=0,uP,y, e_=0, Ti=1, Tc=0.02, uI, uI_=0 ,y_=0;
    float kp=0.5;
    int next_loop = 0;
    float speed=0, speed_template=40;

    if(stopAngle + 90 > 360) temp_stop = stopAngle - 360;
	else temp_stop = stopAngle;

    while(true)
    {
        currentAngle = robot.GetAngle();

        if(currentAngle >= temp_stop + 90 && currentAngle < temp_stop + 90 + 10)
        {
            break;
        }

        float ym=robot.GetDistRight();
        if(next_loop == 0)
        {
            y_=ym;    //pierwszy raz nie startuje od zera tylko od wartosci ym
            next_loop = 1;
        }
        float a=0.25;//parametr filtru zakresu 0-1 okresla waznosc poprzednich wartosci (przy 0.25 liczy srednia z jakichs 4 cykli poprzednich)
        y=(1-a)*y_+a*ym;//filter (filtr inercyjny pierwszego rzedu)

        speed=speed_template;
        float limit=0.5, ew;
        ew=w-y;
        /* Strefa nieczulosci bledu regulacji*/
        if(ew>0)
        {
            if(ew>limit) e=ew-limit;
            else e=0;
        }
        else
        {
            if(ew < -limit) e=ew+limit;
            else e=0;
        }
        uP=kp*e;//wsp kp (szybkosc filtra) * blad
        uI=uI_+kp*e_*Tc/Ti;//czesc calkujaca inercja poprzednia + kp*blad poprzedni * wsp. czas cykly/czas inercji
        u=uP+uI;
        uI=trim(uI,-100,100);
        uP=trim(uP,-100,100);
        u=trim(u,-100,100);

        printf("angle=%f, stopAngle=%f \n", currentAngle, stopAngle);
        printf("w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f, %4.0f",w,e,u,y,speed);

        e_=e;
        uI_=uI;
        y_=y;
        if(u>100) u=100;
        if(u<-100) u=-100;


        robot.SetWheel((int)(-u+speed),(int)(u+speed));

        robot.Cycle();


        cout<<endl;
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    //robot.StopLog();
}

void robot_regulacja_odleglosci_lewy(float distLeft, float stopAngle)
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci lewy czujnik"<<endl;

    float temp_stop;
    /*ostringstream fnamestream;
    fnamestream <<"log_"<<time(NULL)<<".csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);*/
    float currentAngle = 0.0;

    float w=distLeft,e, u=0,uP,y, e_=0, Ti=1.0, Tc=0.02, uI, uI_=0 ,y_=0;//Ti=1.0
    float kp=0.5;//0.5
    int next_loop = 0;
    float speed=0, speed_template=40;

    if(stopAngle - 90 < 0) temp_stop = stopAngle + 360;
	else temp_stop = stopAngle;

    while(true)
    {
        currentAngle = robot.GetAngle();

        if(currentAngle <= temp_stop - 90 && currentAngle > temp_stop - 90 - 10)
        {
            break;
        }

        float ym=robot.GetDistLeft();
        if(next_loop == 0)
        {
            y_=ym;    //pierwszy raz nie startuje od zera tylko od wartosci ym
            next_loop = 1;
        }
        float a=0.25;//parametr filtru zakresu 0-1 okresla waznosc poprzednich wartosci (przy 0.25 liczy srednia z jakichs 4 cykli poprzednich)
        y=(1-a)*y_+a*ym;//filter (filtr inercyjny pierwszego rzedu)

        speed=speed_template;
        float limit=0.5, ew;
        ew=w-y;
        /* Strefa nieczulosci bledu regulacji */
        if(ew>0)
        {
            if(ew>limit) e=ew-limit;
            else e=0;
        }
        else
        {
            if(ew < -limit) e=ew+limit;
            else e=0;
        }
        uP=kp*e;//wsp kp (szybkosc filtra) * blad
        uI=uI_+kp*e_*Tc/Ti;//czesc calkujaca inercja poprzednia + kp*blad poprzedni * wsp. czas cykly/czas inercji
        u=uP+uI;
        uI=trim(uI,-100,100);
        uP=trim(uP,-100,100);
        u=trim(u,-100,100);

        printf("angle=%f, stopAngle=%f \n", currentAngle, stopAngle);
        printf("w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f, %4.0f",w,e,u,y,speed);

        e_=e;
        uI_=uI;
        y_=y;
        if(u>100) u=100;
        if(u<-100) u=-100;


        robot.SetWheel((int)(u+speed),(int)(-u+speed));

        robot.Cycle();


        cout<<endl;
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    //robot.StopLog();
}

void test_prawy()
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci prawy czujnik"<<endl;

    char c=0;
      /*ostringstream fnamestream;
                        fnamestream <<"log_"<<time(NULL)<<".csv";
                        string fname= fnamestream.str();
                        robot.StartLog(fname);*/
    char auto_mode=0, stan=0;
    float w=20,e,um=0, u=0,uP,y, e_=0, Ti=1, Tc=0.02, uI, uI_=0 ,y_=0;
//    float w_=0;
    float kp=0.5;
    int next_loop = 0;
    float speed=0, speed_template=35;

    while(c != 'q')
    {
        c = 0;
        while(kbhit())
        {
            cin>>c;
//            last_c=c;
//          time_c_prev=time_c;
//            time_c=millis();
            switch (c)
            {
                case 'a':
                        if(auto_mode == 1) auto_mode=0;
                        else auto_mode=1;
                        cout<< "Auto: "<<auto_mode<<endl;
                    break;
                case '+':
                        w+=1;
                        if(w>100) w=100;
                    break;
                case '-':
                        w-=1;
                        if(w<0) w=0;
                    break;
                case '[':

                        speed_template ++;
                    break;
                case ']':
                        speed_template--;
                    break;
                case '?':
                       //robot_regulacja_odleglosci_menu_prawy();
                    break;
            }
           // cout<<"speed_hi="<<speed_hi<<" speed_low="<<speed_low<<" left_wheel="<<robot.GetLeftWheel()<<" right_wheel="<<robot.GetRightWheel()<<" delta="<<time_c-time_c_prev<<endl;
           // c0count=0;
        }
        float ym=robot.GetDistRight();
        if(next_loop == 0) { y_=ym; next_loop = 1;}//pierwszy raz nie startuje od zera tylko od wartosci ym
        float a=0.25;//parametr filtru zakresu 0-1 okresla waznosc poprzednich wartosci (przy 0.25 liczy srednia z jakichs 4 cykli poprzednich)
        y=(1-a)*y_+a*ym;//filter (filtr inercyjny pierwszego rzedu)
        if(auto_mode)
        {
            //e=y-w;
            speed=speed_template;
            float limit=0.5, ew;
            ew=w-y;
            /* Strefa nieczulosci bledu regulacji*/
            if(ew>0)
            {
                if(ew>limit) e=ew-limit;
                else e=0;
            }
            else
            {
                if(ew < -limit) e=ew+limit;
                else e=0;
            }
            uP=kp*e;//wsp kp (szybkosc filtra) * blad
            uI=uI_+kp*e_*Tc/Ti;//czesc calkujaca inercja poprzednia + kp*blad poprzedni * wsp. czas cykly/czas inercji
            u=uP+uI;
            uI=trim(uI,-100,100);
            uP=trim(uP,-100,100);
            u=trim(u,-100,100);
            //if(abs(e)<5) {u=0;uI=0;uP=0;}
            um=u;
            printf("w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f, %2i, %4.0f",w,e,u,y,stan,speed);

        }
        else
        {
            speed=0;
            u=um;
            w=y;
            e=0.0;
            uI=u;
            printf("w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f",w,e,u,y);
        }
//        w_=w;
        e_=e;
        uI_=uI;
        y_=y;
        if(u>100) u=100;
        if(u<-100) u=-100;
        float delta_u=25, un=u;

       // if(abs(u)>2.5) un=u+ (u>=0?1:-1)*delta_u;
        //else un=u;
        robot.SetWheel((int)(-un+speed),(int)(un+speed));
        //robot.SetWheel((int)u,(int)-u);
        robot.Cycle();


    cout<<endl;
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    //robot.StopLog();
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
    //Inicjalizacja robota

    RobotState robotState;
    Robot robot;
    robot.Init();
    robot.SetWheel(0,0); // stop
    robot.Cycle(); //first cycle

    /*stringstream fnamestream;
    fnamestream << "log_" << time(NULL) <<"_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);*/
    unsigned long stopTimestamp = 0;
    unsigned long leftTimestamp = 0;
    unsigned long returnLeftTimestamp = 0;
    unsigned long rightTimestamp = 0;
    unsigned long returnRightTimestamp = 0;
    unsigned long rightSum = 0;
    unsigned long leftSum = 0;
    float stopAngle = 0.0;
    int state = 0;
    //Main loop
    while(kb!= 'q')
    {
        //Input
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
        //printf("%lu| %lu \n", robotState.microsTimestamp, roundTimestamp);
        printf("current=%f | stop=%f\n ", robotState.angle, stopAngle);
        //printf("%lu | %lu \n", leftTimestamp, rightTimestamp);
        //MongoDB
        //db.WriteState(&robotState);

        //Read command from db
        //string command = db.ReadCommand();

        //Process
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
                stopAngle = robotState.angle;
                stopTimestamp = robotState.microsTimestamp;
                printf("Stop: %lu \n", stopTimestamp);

                //robot.SetWheel(0,0);
                robot.SetWheel(50,-50);
                state = 2;
            }
            break;
        case 2://policzenie czasu do napotkania krawedzi z lewego czujnika i odkrecanie w lewo
            if(robotState.distLeft < 25.0)
            {
                //robot.SetWheel(0,0);
                returnLeftTimestamp = robotState.microsTimestamp;
                leftTimestamp = returnLeftTimestamp - stopTimestamp;
                //rightSum = returnRightTimestamp + rightTimestamp + 90000;
                printf("Right: %lu \n", leftTimestamp);
                robot.SetWheel(-50,50);
                state = 4;
            }
            break;
        case 3://policzenie czasu do napotkania krawedzi z prawego czujnika i odkrecanie w prawo
            if(robotState.distRight < 25.0)
            {
                //robot.SetWheel(0,0);
                returnRightTimestamp = robotState.microsTimestamp;
                rightTimestamp = returnRightTimestamp - stopTimestamp;
                //leftSum = returnLeftTimestamp + leftTimestamp + 90000;
                printf("Left: %lu \n", rightTimestamp);
                robot.SetWheel(50,-50);
                state = 5;
            }
            break;
        case 4://powrot do wczesniejszej pozycji (na podstawie czasu skretu w prawo) i odkrecanie w lewo
            if(robotState.angle >= stopAngle && robotState.angle < stopAngle + 5)
            {
                //robot.SetWheel(0,0);
                stopTimestamp = robotState.microsTimestamp;
                robot.SetWheel(-50,50);
                state = 3;
            }
            /*if(rightSum > robotState.microsTimestamp )
            {
                printf("righttimestamp");
                robot.Cycle();
            }
            if(rightSum < robotState.microsTimestamp)
            {
                robot.SetWheel(0,0);
                stopTimestamp = robotState.microsTimestamp;
                printf("righttimestampstop: %lu \n", stopTimestamp);
                //robot.SetWheel(-50,50);
                //state = 3;
            }*/
            break;
        case 5://po powrocie do pierwszej pozycji zatrzymanie i wypisanie czasow skretu w lewo i prawo
            if(robotState.angle <= stopAngle && robotState.angle > stopAngle - 5)
            {
                robot.SetWheel(0,0);
                printf("LeftTimestamp: %lu | RightTimestamp: %lu \n", leftTimestamp, rightTimestamp);
                state = 6;
            }
            /*if(leftSum > robotState.microsTimestamp )
            {
                printf("lefttimestamp");
                robot.Cycle();
            }
            else if(leftSum < robotState.microsTimestamp)
            {
                robot.SetWheel(0,0);
                printf("LeftTimestamp: %lu | RightTimestamp: %lu \n", leftTimestamp, rightTimestamp);
                state = 6;
            }*/
            break;
        case 6://obliczenie kierunku objazdu przeszkody
            if(rightTimestamp > leftTimestamp)
            {
                state = 7;
            }
            else if(leftTimestamp > rightTimestamp)
            {
                state = 8;
            }
            break;
        case 7://ominiecie przeszkody z prawej strony
            robot.SetWheel(40,-40);
            state = 9;
            break;
        case 8://ominiecie przeszkody z lewej strony
            robot.SetWheel(-40,40);
            state = 9;
            break;
        case 9://okrazanie przeszkody z lewej lub prawej strony
            if(robotState.distRight < 25.0)
            {
                    robot_regulacja_odleglosci_prawy(robotState.distRight, stopAngle);
                    robot.SetWheel(0,0);
                    state = 10;
            }
            if(robotState.distLeft < 25.0)
            {
                    robot_regulacja_odleglosci_lewy(robotState.distLeft, stopAngle);
                    robot.SetWheel(0,0);
                    state = 10;
            }
            break;
        case 10:
            robot.SetWheel(50,-50);
            state = 12;
            break;
        case 11:
            robot.SetWheel(-50,50);
            state = 12;
            break;
        case 12:
            float currentAngle;
            currentAngle = robot.GetAngle();
            if(currentAngle > stopAngle && currentAngle < stopAngle + 10)
            {
                robot.SetWheel(0,0);
                state = 13;
            }
            break;
        case 13:
            robot.SetWheel(50,50);
            state = 14;
            break;
        case 14:
            if(robotState.distFront < 15.0)
            {
                robot.SetWheel(0,0);
                break;
            }
            break;
        default:
            robot.Cycle();
        }

        //Output
        robot.Cycle();
    }
    //test_prawy();
    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
    changemode(0);

    return 0;
}
