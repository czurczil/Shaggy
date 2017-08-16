void robot_regulacja_odleglosci_menu_prawy()
{
    printf("Regulacja odleglosci prawy menu:\r\n");
    printf("a - auto/manual\r\n");
    printf("+/- -wartosc zadana odleglosci w\r\n");
   // printf("[] - wartosc zadana um:\r\n");
    //printf("Regulacja kata menu:\r\n");

}



void robot_regulacja_odleglosci_prawy()
{
     Robot robot;
    robot.Init();
    cout<<"Regulacja odleglosci prawy czujnik"<<endl;

    char c=0;
      ostringstream fnamestream;
                        fnamestream <<"log_"<<time(NULL)<<".csv";
                        string fname= fnamestream.str();
                        robot.StartLog(fname);
    char auto_mode=0, stan=0;
    float w=180,e,um=0, u=0,uP,y, e_=0, Ti=1, Tc=0.02, uI, uI_=0 ,y_=0;
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
                       robot_regulacja_odleglosci_menu_prawy();
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
    robot.StopLog();
}
