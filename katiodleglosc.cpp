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


#include <wiringPi.h>
#include <wiringSerial.h>

#include "utils.h"
#include "terminal_functions.h"
#include "robot.h"
#include "escape.h"
#include "prog_parser.h"
#include "katiodleglosc.h"


using namespace std;



void katiodleglosc_menu()
{
     printf("Regulacja odleglosci i kata menu:\r\n");
    printf("a - auto/manual\r\n");
    printf("+/- -wartosc zadana odleglosci w\r\n");
}

void katiodleglosc_work()
{

    Robot robot;
    robot.Init();
    cout<<"Regulacja kata i odleglosci"<<endl;

    char c=0;
//    int time_c=0;
//    int time_delta_c=851;
//    int time_c_prev=millis();

 //   struct timeval tv;
  //  gettimeofday(&tv,NULL);
      ostringstream fnamestream;
                        fnamestream <<"log_"<<time(NULL)<<".csv";
                        string fname= fnamestream.str();
                        robot.StartLog(fname);
    char auto_mode=0, stan=0;
    float w=180,e,um=0, u=0,uP,y, e_=0, Ti=1, Tc=0.01, uI, uI_=0 ,y_=0, kp=3.8;
    float wk=18,ek,umk=0, uk=0,uPk,yk, e_k=0, Tik=1, Tck=0.01, uIk, uI_k=0 ,y_k=0, kpk=0.5;
//    float w_=0;
    int next_loop, next_loop_k = 0;

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
                        w+=5;
                        if(w>359) w=w-359;
                    break;
                case '-':
                        w-=5;
                        if(w<0) w=w+359;
                    break;
                case '[':
                        wk+=1;
                    break;
                case ']':
                        wk-=1;
                    break;
                case '?':
                       katiodleglosc_menu();
                    break;
            }
           // cout<<"speed_hi="<<speed_hi<<" speed_low="<<speed_low<<" left_wheel="<<robot.GetLeftWheel()<<" right_wheel="<<robot.GetRightWheel()<<" delta="<<time_c-time_c_prev<<endl;
           // c0count=0;
        }
        //////// Distance
        float ym=robot.GetDistFront();
        if(next_loop == 0) { y_=ym; next_loop = 1;}
        float a=0.25;
        y=(1-a)*y_+a*ym;
        if(auto_mode)
        {

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
            uP=kp*e;
            uI=uI_+kp*e_*Tc/Ti;
            u=uP+uI;
            uI=trim(uI,-100,100);
            uP=trim(uP,-100,100);
            u=trim(u,-100,100);
            //if(abs(e)<5) {u=0;uI=0;uP=0;}
            um=u;
            printf("Dist: w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f",w,e,u,y);

        }
        else
        {
            u=um;
            w=y;
            e=0.0;
            uI=u;
            printf("Dist: w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f",w,e,u,y);
        }
        e_=e;
        uI_=uI;
        y_=y;


         //////// Angle
        float ymk=robot.GetAngle();
        if(next_loop_k == 0) { y_k=ymk; next_loop_k = 1;}
        float ak=0.25;
        yk=(1-ak)*y_k+ak*ymk;
        if(auto_mode)
        {

            float limit=0.5, ew;
            ew=wk-yk;
            /* Strefa nieczulosci bledu regulacji*/
            if(ew>0)
            {
                if(ew>limit) ek=ew-limit;
                else ek=0;
            }
            else
            {
                if(ew < -limit) ek=ew+limit;
                else ek=0;
            }
            uPk=kpk*ek;
            uIk=uI_k+kpk*e_k*Tck/Tik;
            uk=uPk+uIk;
            uIk=trim(uIk,-100,100);
            uPk=trim(uPk,-100,100);
            uk=trim(uk,-100,100);
            //if(abs(e)<5) {u=0;uI=0;uP=0;}
            umk=uk;
            printf(" Angle: w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f",wk,ek,uk,yk);

        }
        else
        {
            uk=umk;
            wk=yk;
            ek=0.0;
            uIk=uk;
            printf(" Angle: w=%3.0f, e=%4.1f, u=%3.0f, y=%5.1f",wk,ek,uk,yk);
        }
        e_k=ek;
        uI_k=uIk;
        y_k=yk;



        if(u>100) u=100;
        if(u<-100) u=-100;
        float delta_u=25, unl, unp;

        float ul=-u+uk;
        float up=-u-uk;

        if(abs(ul)>2.5) unl=ul+ (ul>=0?1:-1)*delta_u;
        else unl=ul;
        if(abs(up)>2.5) unp=up+ (up>=0?1:-1)*delta_u;
        else unp=u;
        robot.SetWheel((int)(unl),(int)(unp));
        //robot.SetWheel((int)u,(int)-u);
        robot.Cycle();
    //cout<<robot.GetDistFront()<<";"<<robot.GetDistBack()<<";"<<robot.GetDistLeft()<<";"<<robot.GetDistRight()<<";"<<robot.GetAngle()<<";";
    //cout<<robot.GetAngle();
    //cout<<robot.GetDistFront()<<";"<<robot.GetAngle()<<";";

    //cout<<""<<robot.GetAccelX()<<";" <<robot.GetAccelY()<<";"<<robot.GetAccelZ();

    cout<<endl;
    }
    robot.SetWheel(0,0);
    robot.Cycle();
    robot.StopLog();
}
