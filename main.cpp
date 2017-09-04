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

void test_prawy();
void test_lewy();

void eksperyment1()
{
    printf("Roznica mocy na kolach = 0");

    char kb=0;
    RobotState robotState;
    Robot robot;
    robot.Init();
    robot.SetWheel(40,40);
    robot.Cycle();

    stringstream fnamestream;
    fnamestream << "log_eksperyment1_" << time(NULL) <<"_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);

    while(kb!= 'q')
    {
        kb=0;
        if(kbhit())
        {
            cin>>kb;
            usleep(200);
        }
        robot.GetState(&robotState);

        robot.Cycle();
    }

    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
}

void eksperyment2()
{
    printf("Roznica mocy na kolach = 10");

    char kb=0;
    RobotState robotState;
    Robot robot;
    robot.Init();
    robot.SetWheel(40,40);
    robot.Cycle();

    stringstream fnamestream;
    fnamestream << "log_eksperyment2_" << time(NULL) <<"_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);

    while(kb!= 'q')
    {
        int skip = 0;
        kb=0;
        if(kbhit())
        {
            cin>>kb;
            usleep(200);
        }
        robot.GetState(&robotState);

        if(robotState.distLeft > 30 && skip == 0)
        {
            robot.SetWheel(40,50);
            skip = 1;
        }
        robot.Cycle();
    }

    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
}

void eksperyment3()
{
    printf("Roznica mocy na kolach = 30");

    char kb=0;
    RobotState robotState;
    Robot robot;
    robot.Init();
    robot.SetWheel(40,40);
    robot.Cycle();

    stringstream fnamestream;
    fnamestream << "log_eksperyment3_" << time(NULL) <<"_robot.csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);

    while(kb!= 'q')
    {
        int skip = 0;
        kb=0;
        if(kbhit())
        {
            cin>>kb;
            usleep(200);
        }
        robot.GetState(&robotState);

        if(robotState.distLeft > 30 && skip == 0)
        {
            robot.SetWheel(30,60);
            skip = 1;
        }
        robot.Cycle();
    }

    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
}

void robot_regulacja_odleglosci_prawy(float distRight, float stopAngle)
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci prawy czujnik"<<endl;

    float temp_stop;
    ostringstream fnamestream;
    fnamestream <<"log_regulacja_odleglosci_prawy"<<time(NULL)<<".csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);
    float currentAngle = 0.0;

    float w=distRight,e, um = 0, u=0,uP,y, e_=0, Ti=4.0, Tc=0.02, uI, uI_=0 ,y_=0;
    float kp=1.0;
    int next_loop = 0;
    int cycle_count = 0;
    bool state = 0;
    float speed=0, speed_template=35;

    if(stopAngle + 90 > 360) temp_stop = stopAngle - 360;
    else temp_stop = stopAngle;

    while(true)
    {

        currentAngle = robot.GetAngle();

        if(currentAngle >= temp_stop + 90 && currentAngle < temp_stop + 90 + 5)
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
    robot.StopLog();
}

void robot_regulacja_odleglosci_lewy(float distLeft, float stopAngle)
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci lewy czujnik"<<endl;

    float temp_stop;
    ostringstream fnamestream;
    fnamestream <<"log_regulacja_odleglosci_lewy_"<<time(NULL)<<".csv";
    string fname= fnamestream.str();
    robot.StartLog(fname);
    float currentAngle = 0.0;
    float w=distLeft,e,um=0, u=0,uP,y, e_=0, Ti=4.0, Tc=0.02, uI, uI_=0 ,y_=0;//Ti=1.0
    float kp=1.0;//0.5
    int next_loop = 0;
    float speed=0, speed_template=35;

    if(stopAngle - 90 < 0) temp_stop = stopAngle + 360;
    else temp_stop = stopAngle;

    while(true)
    {
        currentAngle = robot.GetAngle();

        if(currentAngle <= temp_stop - 90 && currentAngle > temp_stop - 90 - 5)
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
    robot.StopLog();
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

    //Inicjalizacja robota

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
    float stopAngle = 0.0;
    float temp_angle = 0.0;
    int state = 0;
    int last_state = 0;
    //Main loop
    while(kb!= 'q')
    {
        //Input
        kb=0;
        if (kbhit())
        {
            cin >> kb;
            lastKb = kb;
            switch (kb)
            {
            case 'w':
                robot.SetWheel(50, 50);
                break;
            case 's':
                robot.SetWheel(-50, -50);
                break;
            case 'a':
                robot.SetWheel(-50, 0);
                break;
            case 'd':
                robot.SetWheel(0, -50);
                break;
            case 'e':
                robot.SetWheel(0, 0);
                break;
            }
            usleep(200);
        }
        robot.GetState(&robotState);

        printf("%4.1f | %c | %4.1f | %4.1f | state=%i \n", robotState.distFront, lastKb, robotState.distLeft, robotState.distRight, state);

        //printf("current=%f | stop=%f\n ", robotState.angle, stopAngle);

        //printf("stopTimestamp=%lu | current=%lu | left=%lu | right=%lu \n", stopTimestamp, robotState.microsTimestamp, leftTimestamp, rightTimestamp);

        if(robotState.distFront < 30.0 && lastKb == 'w' && state == 0)
        {
            robot.SetWheel(30,30);
            state = 1;
        }

        switch(state)
        {
         case 1://napotkanie przeszkody i skret w prawo
            if(robotState.distFront < 15.0 )
            {
                robot.SetWheel(0,0);
                stopAngle = robotState.angle;
                stopTimestamp = robotState.microsTimestamp;
                printf("Stop: %lu \n", stopTimestamp);

                robot.SetWheel(40,-40);
                state = 2;
            }
            break;
        case 2://policzenie czasu do zgubienia robota i odkrecanie w lewo
            if(robotState.distFront > 50.0)
            {
                robot.SetWheel(0,0);
                returnLeftTimestamp = robotState.microsTimestamp;
                leftTimestamp = robotState.microsTimestamp - stopTimestamp;
                leftSum = returnLeftTimestamp + leftTimestamp + 100000;
                printf("Left: %lu \n", leftTimestamp);
                robot.SetWheel(-40,40);
                state = 4;
            }
            break;
        case 3://policzenie czasu do zgubienia robota i odkrecanie w prawo
            if(robotState.distFront > 50.0)
            {
                robot.SetWheel(0,0);
                returnRightTimestamp = robotState.microsTimestamp;
                rightTimestamp = returnRightTimestamp - stopTimestamp;
                rightSum = returnRightTimestamp + rightTimestamp + 100000;
                printf("Right: %lu \n", rightTimestamp);
                robot.SetWheel(40,-40);
                state = 5;
            }
            break;
        case 4://powrot do wczesniejszej pozycji (na podstawie czasu skretu w prawo) i odkrecanie w lewo
            if(leftSum < robotState.microsTimestamp || (robotState.angle >= stopAngle && robotState.angle < stopAngle + 2))
            {
                robot.SetWheel(0,0);
                stopTimestamp = robotState.microsTimestamp;
                printf("lefttimestampstop: %lu \n", stopTimestamp);
                robot.SetWheel(-40,40);
                state = 3;
            }
            if (leftSum > robotState.microsTimestamp)
            {
                robot.Cycle();
            }
            break;
        case 5://po powrocie do pierwszej pozycji zatrzymanie i wypisanie czasow skretu w lewo i prawo
            if(rightSum < robotState.microsTimestamp || (robotState.angle <= stopAngle && robotState.angle > stopAngle - 2))
            {
                robot.SetWheel(0,0);
                printf("LeftTimestamp: %lu | RightTimestamp: %lu \n", leftTimestamp, rightTimestamp);
                state = 6;
            }
            if (rightSum > robotState.microsTimestamp)
            {
                robot.Cycle();
            }
            break;
        case 6://obliczenie kierunku objazdu przeszkody
            if(rightTimestamp < leftTimestamp)
            {
                state = 7;
            }
            else if(leftTimestamp < rightTimestamp)
            {
                state = 8;
            }
            temp_angle = robotState.angle;
            break;
        case 7://ominiecie przeszkody z prawej strony
            temp_angle += 35;
            robot.SetWheel(30,-30);
            last_state = state;
            state = 9;
            break;
        case 8://ominiecie przeszkody z lewej strony
            temp_angle -= 35;
            robot.SetWheel(-30,30);
            last_state = state;
            state = 9;
            break;
        case 9:
            if(robotState.angle > temp_angle && robotState.angle < temp_angle + 5)
            {
                robot.SetWheel(35,35);
                state = 10;
            }
            break;
        case 10:
            if(last_state == 7 && robotState.distLeft < 13.0 && robotState.distLeft > 11.0)
            {
                robot_regulacja_odleglosci_lewy(robotState.distLeft, stopAngle);
                robot.SetWheel(0,0);
                state = 11;
            }
            else if(last_state == 8 && robotState.distRight < 13.0 && robotState.distRight > 11.0)
            {
                robot_regulacja_odleglosci_prawy(robotState.distRight, stopAngle);
                robot.SetWheel(0,0);
                state = 12;
            }
            break;
        case 11:
            robot.SetWheel(-40,40);
            state = 13;
            break;
        case 12:
            robot.SetWheel(40, -40);
            state = 13;
            break;
        case 13:
            if(robotState.angle > stopAngle && robotState.angle < stopAngle + 5)
            {
                robot.SetWheel(0,0);
                state = 14;
            }
            break;
        case 14:
            robot.SetWheel(50,50);
            state = 15;
            break;
        case 15:
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
    //test_lewy();
    //eksperyment1();
    //eksperyment2();
    //eksperyment3();

    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
    changemode(0);

    return 0;
}
void test_lewy()
{
    Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci lewy czujnik"<<endl;

    char c=0;
    /*ostringstream fnamestream;
                      fnamestream <<"log_"<<time(NULL)<<".csv";
                      string fname= fnamestream.str();
                      robot.StartLog(fname);*/
    char auto_mode=0, stan=0;
    float w=180,e,um=0, u=0,uP,y, e_=0, Ti=2.0, Tc=0.02, uI, uI_=0 ,y_=0;//Ti=1.0
    float kp=1.0;//0.5
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
        float ym=robot.GetDistLeft();
        if(next_loop == 0)
        {
            y_=ym;    //pierwszy raz nie startuje od zera tylko od wartosci ym
            next_loop = 1;
        }
        float a=0.25;//0.25 parametr filtru zakresu 0-1 okresla waznosc poprzednich wartosci (przy 0.25 liczy srednia z jakichs 4 cykli poprzednich)
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
        //w_=w;
        e_=e;
        uI_=uI;
        y_=y;
        if(u>100) u=100;
        if(u<-100) u=-100;
        //float delta_u=25, un=u;

        //if(abs(u)>2.5) un=u+ (u>=0?1:-1)*delta_u;
        //else un=u;
        robot.SetWheel((int)(u-5+speed),(int)(-u+5+speed));
        //robot.SetWheel((int)u,(int)-u);
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
    float w=15,e,um=0, u=0,uP,y, e_=0, Ti=4.0, Tc=0.02, uI, uI_=0 ,y_=0;//Ti=1.0
    float kp=1.0;//0.5
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
        if(next_loop == 0)
        {
            y_=ym;    //pierwszy raz nie startuje od zera tylko od wartosci ym
            next_loop = 1;
        }
        float a=0.25;//0.25 parametr filtru zakresu 0-1 okresla waznosc poprzednich wartosci (przy 0.25 liczy srednia z jakichs 4 cykli poprzednich)
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
        //w_=w;
        e_=e;
        uI_=uI;
        y_=y;
        if(u>100) u=100;
        if(u<-100) u=-100;
        //float delta_u=25, un=u;

        //if(abs(u)>2.5) un=u+ (u>=0?1:-1)*delta_u;
        //else un=u;
        robot.SetWheel((int)(-u+speed),(int)(u+speed));
        //robot.SetWheel((int)u,(int)-u);
        robot.Cycle();


        cout<<endl;
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    //robot.StopLog();
}
