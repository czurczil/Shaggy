#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "terminal_functions.h"
#include "robot.h"

using namespace std;

enum ESC_STATE {ESC_IDLE, ESC_BACK, ESC_FRONT, ESC_TURNING_LEFT, ESC_TURNING_RIGHT};
enum ESC_DIR {DIR_FRONT=0, DIR_BACK=1, DIR_LEFT=2, DIR_RIGHT=3};

#define AVG_SIZE 3
float avg_dists[4][AVG_SIZE];

void clear_avg_dists()
{

for (int i=0;i<4;i++)
    for(int j=0;j<AVG_SIZE;j++) avg_dists[i][j]=100.0;
}

float get_dist(int dir)
{
    float avg=0.0;
     for(int j=0;j<AVG_SIZE;j++) avg+=avg_dists[dir][j];
return avg/AVG_SIZE;
}

void put_dist(int dir,float val)
{
    for(int j=0;j<AVG_SIZE-1;j++) avg_dists[dir][j] = avg_dists[dir][j+1];

    avg_dists[dir][AVG_SIZE-1] = val;
}

void measure_dist(Robot *r)
{
put_dist(DIR_FRONT,r->GetDistFront());
put_dist(DIR_BACK,r->GetDistBack());
put_dist(DIR_LEFT,r->GetDistLeft());
put_dist(DIR_RIGHT,r->GetDistRight());

}

typedef struct _dist
{
int dir;
float val;
} Distance;

Distance get_nearest_dist(Distance *dist)
{
    Distance d=dist[0];
    for(int i=1;i<4;i++)
    {
        if(d.val > dist[i].val) d=dist[i];
    }
    return d;
}

Distance get_farthest_dist(Distance *dist)
{
    Distance d=dist[0];
    for(int i=1;i<4;i++)
    {
        if(d.val < dist[i].val) d=dist[i];
    }
    return d;
}

void robot_escape_mode()
{
     Robot robot;
    robot.Init();
    cout<<"Tryb ucieczki Kudlatego"<<endl;
    char c=0;
    int time_c=0;
    int time_delta_c=851;
   // int time_c_prev=millis();
    int speed_hi=80;
    int speed_low=25;

    float dist_safe=75.0;
    float dist_escape=15.0;
   // float dist_f=0, dist_b=0, dist_l=0, dist_r=0;
    int angle =0;

    Distance dist[4]; for (int i=0;i<4;i++) {dist[i].dir=i; dist[i].val=0.0;}
    dist[DIR_FRONT].dir = DIR_FRONT;
    dist[DIR_BACK].dir = DIR_BACK;
    dist[DIR_LEFT].dir = DIR_LEFT;
    dist[DIR_RIGHT].dir = DIR_RIGHT;
    ESC_STATE state = ESC_IDLE;;
    const int esc_cyclec = 15;

    while(c != 'q')
    {
        c = 0;
        while(kbhit())
        {
            cin>>c;
            time_c=millis();

            switch (c)
            {
                case 'w':
                        robot.SetWheel(speed_hi,speed_hi);
                    break;
                case 's':
                        robot.SetWheel(-speed_hi,-speed_hi);
                    break;
                case 'a':
                        robot.SetWheelPreserveDirection(speed_low,speed_hi);
                    break;
                case 'd':
                        robot.SetWheelPreserveDirection(speed_hi,speed_low);
                    break;
                case 'z':
                        robot.SetWheel(-speed_hi,speed_hi);
                    break;
                case 'x':
                        robot.SetWheel(speed_hi,-speed_hi);
                    break;
                case 'c':
                        robot.SetWheel(0,speed_hi);
                      break;
                case 'v':
                        robot.SetWheel(speed_hi,0);
                    break;
                case '.': // stop
                        robot.SetWheel(0,0);
                    break;
                case '+':
                        if(speed_hi<100) speed_hi++;
                    break;
                case '-':
                        if(speed_hi > 0) speed_hi--;
                        if(speed_low>speed_hi) speed_low=speed_hi;
                    break;
                case '[':
                        if(speed_low < speed_hi) speed_low ++;
                    break;
                case ']':
                        if(speed_low > 0) speed_low--;
                    break;
                case '?':
                        cout<<"Komendy:"<<endl;
                        cout<<"w - przod"<<endl;
                        cout<<"s - tyl"<<endl;
                        cout<<"a - lewo"<<endl;
                        cout<<"d - prawo"<<endl;
                        cout<<". - stop"<<endl;
                        cout<<"z - szybki obrot w lewo"<<endl;
                        cout<<"x - szybko obrot w prawo"<<endl;
                        cout<<"c - obrot w lewo"<<endl;
                        cout<<"v - obrot w prawo"<<endl;
                        cout<<"+ - zwiekszenie predkosci"<<endl;
                        cout<<"- - zmniejszenie predkosci"<<endl;
                        cout<<"[ - zwiekszenie predkosci wolnego kola"<<endl;
                        cout<<"] - zmniejszenie predkosci wolnego kola"<<endl;
                        cout<<"? - podpowiedz"<<endl;
                        cout<<"q - wyjscie"<<endl;
                    break;
            }
        }
        if(c==0)
        {
            int curr_time=millis();
            if(curr_time>(time_c+time_delta_c))
                robot.SetWheel(0,0);
        }
    //// Escape mode
        measure_dist(&robot);
        dist[DIR_FRONT].val = get_dist(DIR_FRONT);
        dist[DIR_BACK].val = get_dist(DIR_BACK);
        dist[DIR_LEFT].val = get_dist(DIR_LEFT);
        dist[DIR_RIGHT].val = get_dist(DIR_RIGHT);
        angle = (int)robot.GetAngle();
        Distance nearest = get_nearest_dist(dist);
        Distance farthest = get_farthest_dist(dist);

        if(nearest.val > dist_safe) state = ESC_IDLE;
        else if(nearest.val < dist_escape)
        {
            switch(nearest.dir)
            {
                case DIR_FRONT:
                    state=ESC_BACK;
                break;
                case DIR_BACK:
                    state=ESC_FRONT;
                break;
                case DIR_LEFT:
                    state = ESC_TURNING_LEFT;
                break;
                case DIR_RIGHT:
                    state = ESC_TURNING_RIGHT;
                break;
                default:
                    state = ESC_IDLE;
                break;
            }

        }

        switch(state)
        {
            case ESC_IDLE:
                    robot.SetWheel(0,0);
                break;
            case ESC_FRONT:
                    robot.SetWheel(speed_hi,speed_hi);
                break;
            case ESC_BACK:
                    robot.SetWheel(-speed_hi,-speed_hi);
                break;
            case ESC_TURNING_LEFT:
                     robot.SetWheel(speed_hi,-speed_hi);
                break;
            case ESC_TURNING_RIGHT:
                     robot.SetWheel(-speed_hi,speed_hi);
                break;
            default:
                state = ESC_IDLE;
        }

for (int i=0;i<4;i++) cout<<" "<<dist[i].dir<<" "<<dist[i].val<<" ";
cout<<" state="<<state<<" near d="<<nearest.dir<< " near val="<<nearest.val;
cout<<endl;





        //float dist_min = min()


        robot.Cycle();
    }
    robot.SetWheel(0,0);
    robot.Cycle();

}
