#include <includs.h>


//unsigned int servo_widths[ 10 ]={1000,2250,2250,2250,2250,2250,2250,2250,2250,8000};
int servo_widths[ 10 ] = {1247,2931,2898,2823,2872,2805,2938,2805,2805,9973};//syc 2011 10.31
//2250 1.5ms 
         //servo_widths[1]-8 // roll pitch throttle yaw coll auto_switch 
/*int pwm_roll_mid = 2805,
    pwm_pitch_mid = 2805,
    pwm_yaw_mid = 2805,
    pwm_coll_mid = 2805;*/

//------------��ֱͨ�������ã�ע����2014/5/28-----------------
/*int pwm_roll_mid = 2830,
    pwm_pitch_mid = 1860,
    pwm_coll_mid = 800;
int temp_pwm_coll_mid = 800;*/
//------------------------------------------------------------

//------------Сֱ�������ã��޸���2015/4/28-------------------
int pwm_roll_mid = 2910,
    pwm_pitch_mid = 1921,
    pwm_yaw_mid,
    pwm_coll_mid;
int temp_pwm_coll_mid = 1365;

double desired_x_rate=0;
double desired_y_rate=0;
double desired_d_rate=0;
//------------------------------------------------------------

int amplify = 60;//default=30

int manual_servo_widths[3] = {0};
char last_control_mode = 0,
     control_mode = 0;

extern double LQR_pitchGain[3];
extern double LQR_rollGain[3];

double	servoAmplify[4] = {927.06, 955.52, 0.0, 218.0}; //roll, pitch,
//unsigned char pcm_err_char[PCM_NUM] ;
//char pcm_err_flag = 0;
//int pcm_err_pos = 0;
char pcm_rece_flag=0; //�ɹ����ܵ�27���ַ�
char pcm_update_flag =0;  //27���ַ���״̬  
unsigned char  pcm_line[PCM_NUM-4]="";
int man2auto = 100;
bool target_receive_flag;
bool receive_index;

int last_pwm[4]={2250,2250,2250,2250};
int servo_widths_mid[4]={0};
//unsigned int servo_widths_temp[10]={1000,2250,2250,2250,2250,2250,2250,2250,2250,8000};
int servo_widths_temp[10]={1247,2931,2898,2823,2872,2805,2938,2805,2805,9973};//2011 10 31
int last_servo_widths_temp[7]={1247,2931,2898,2823,2872,2805,2938};
int lim_servo_widths_temp[7]={1247,2931,2898,2823,2872,2805,2938};
int remote_median[2]={2839,2839};
char auto_roll_control_mode,
     auto_pitch_control_mode,
     auto_yaw_control_mode,
     auto_coll_control_mode;
//double target[UPDATE_TARGET_SIZE][4];   //�������ͣ��Ļ�������;
void init_translator(void)
{
  AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA ) ;
  AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOB ) ;
  
  
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, AT91C_PIO_PA0|AT91C_PIO_PA1);
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOB, AT91C_PIO_PB20|AT91C_PIO_PB21|
                       AT91C_PIO_PB22|AT91C_PIO_PB23) ; //change from PC0,1,2,3 by chen
                         
  AT91F_PIO_SetOutput( AT91C_BASE_PIOB,AT91C_PIO_PB20);
  /*PC0ʹtranslator���ϰ�ƬΪ3.3V��5V������*/
  
  /*PC2ʹtranslator���°�ƬΪ5V��3.3V�����䣬ͬʱͨ��OEʹ��translator*/
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOB, AT91C_PIO_PB21|AT91C_PIO_PB22|AT91C_PIO_PB23);  
 
  CDSET;/*ע�⣺����CD4017��RESET�źţ���ʹCD4017��'0'��ʼ����PWM�źš�����λ���д�����*/
}

//*---------------------------------------------------------------------------
//timer.c  timer2  is used to produce Servo  
//*---------------------------------------------------------------------------
//
//------------------------------Сֱ��������������------------------------------- 
void servo_update_auto(void)
{
    unsigned char * pcmline=NULL;
    long int pcm_temp[9]={0};
    int ppm_temp[9]={0};//11.01
    unsigned int decode_flag=0;
    double temp[4]={0.0}; 
  
    static int remote_missing = 0;
    
    
    man2auto++;
    if (man2auto > 10000)
          man2auto = 100; 
    
     if(pcm_rece_flag == 1)
    {
      pcm_rece_flag = 0; 
      remote_missing = 0;
      if ( (AVR_PCM[0] != '$') || (AVR_PCM[1] != '$') || AVR_PCM[PCM_NUM-2] != '\r' || AVR_PCM[PCM_NUM-1] != '\n')
      {
          pcm_update_flag = 3;
      }
      else
      {
          pcm_update_flag = 1;
          uncharcpy(pcm_line, AVR_PCM+2,24 );
      }
    } 
    else
    {
    remote_missing++;
    if  (remote_missing>15)
      remote_missing = 15;    
    }
    
    
    pcmline=pcm_line;
    decode_flag=0;
    
    for(char i=1;i<9;i++)
    {
        pcm_temp[i]=strntol((char*)pcmline,3,(char**)&pcmline,16);
        if(pcm_temp[i]==0)
        {
            decode_flag++;
        }
        ppm_temp[i] = (int)(0.8304* pcm_temp[i]-385.2857);
    }
/*-------------------------------���Կ�λ���---------------------------------*/
//2015-11-06byMei
    last_control_mode = control_mode;
    if(pcm_temp[7] < 0x400)
        control_mode=1; //0x028 is mode 1 0x200 is mode 2 �ֿ� 
    else
        control_mode=3;  //0x3d8 is mode 3;�Կ�
          
        if( decode_flag == 0 )
       {		
            for(char i=1;i<9;i++)
           {
            	servo_widths_temp[i]=( int)((ppm_temp[i]*1.1718+920)*3/2);
                servo_widths_temp[i]=servo_widths_temp[i]*91/73;
           }                                        
       }// end of decode_flag == 0
      else 
       {
        	pcm_update_flag=2;
       }   
 
          
/////////////////////////////////////////////////////////////////����������Ԥ����/////////////////////////     
   
//�ǶȻ�������ȡ
/**************************************�ǶȻ�ȡ,ûȡ������*********************************************/     
        static double roll_angle = 0.0,pitch_angle = 0.0,yaw_angle = 0.0;
        if(servo_widths_temp[3]>2200)
        {
          roll_angle = 0.015*raw_ahrs_theta[0] + 0.985*(roll_angle + raw_ahrs_pqr[0]*0.037);
          pitch_angle = 0.015*raw_ahrs_theta[1] + 0.985*(pitch_angle + raw_ahrs_pqr[1]*0.037);  
          yaw_angle = (yaw_angle + raw_ahrs_pqr[2]*0.042);
          if (yaw_angle > PI) yaw_angle -= 2*PI; if (yaw_angle <= -PI) yaw_angle += 2*PI;
          if (PI < (yaw_angle - raw_ahrs_theta[2]))  yaw_angle = 0.03*(raw_ahrs_theta[2]+2*PI) + 0.97*(yaw_angle); 
          else if ( (yaw_angle - raw_ahrs_theta[2]) < -PI) yaw_angle = 0.03*(raw_ahrs_theta[2]) + 0.97*(yaw_angle+2*PI); 
          else yaw_angle = 0.03*raw_ahrs_theta[2] + 0.97*yaw_angle;
          if (yaw_angle > PI)
          yaw_angle -= 2*PI; 
          if (yaw_angle <= -PI)
          yaw_angle += 2*PI;
        }
        
        else
        {
          roll_angle = 0.05*raw_ahrs_theta[0] + 0.95*(roll_angle + raw_ahrs_pqr[0]*0.037);//�ɻ���½ײ����̬��������ƫ��,������Լӿ�����
          pitch_angle = 0.05*raw_ahrs_theta[1] + 0.95*(pitch_angle + raw_ahrs_pqr[1]*0.037);//�ɻ���½ײ����̬��������ƫ��,������Լӿ�����
          yaw_angle = (yaw_angle + raw_ahrs_pqr[2]*0.042);
          if (yaw_angle > PI) yaw_angle -= 2*PI; if (yaw_angle <= -PI) yaw_angle += 2*PI;
          if (PI < (yaw_angle - raw_ahrs_theta[2]))  yaw_angle = 0.05*(raw_ahrs_theta[2]+2*PI) + 0.95*(yaw_angle); 
          else if ( (yaw_angle - raw_ahrs_theta[2]) < -PI) yaw_angle = 0.05*(raw_ahrs_theta[2]) + 0.95*(yaw_angle+2*PI); 
          else yaw_angle = 0.05*raw_ahrs_theta[2] + 0.95*yaw_angle;
          if (yaw_angle > PI)
          yaw_angle -= 2*PI; 
          if (yaw_angle <= -PI)
          yaw_angle += 2*PI; 
        }
        input_yaw = yaw_angle;
        ahrs_theta[0] = roll_angle;//��ɾ,����������
        ahrs_theta[1] = pitch_angle;//��ɾ,����������
        
        double cr,sr,cp,sp,cy,sy,srsp,crsp;
        cr = cos(roll_angle);sr = sin(roll_angle);
        cp = cos(pitch_angle);sp = sin(pitch_angle);
        cy = cos(yaw_angle);sy = sin(yaw_angle);
        srsp = sr*sp ; crsp = cr*sp;
        
        double acc_x,acc_y,acc_n,acc_e,acc_d;
        static double g = 0.0;
        static int stage = 0; 
        if (stage < 200)
        {
            stage++; 
            g += accel_g;
            acc_x = -(imu_accel[0]*cp   + imu_accel[1]*srsp     + imu_accel[2]*crsp);
            acc_y = -(                    imu_accel[1]*cr       - imu_accel[2]*sr);
            acc_n =   acc_x*cy - acc_y*sy; //acc_n = -(imu_accel[0]*cp*cy + imu_accel[1]*(cy*srsp-sy*cr) + imu_accel[2]*(crsp*cy+sr*sy));
            acc_e =   acc_x*sy + acc_y*cy;//acc_e = -(imu_accel[0]*cp*sy + imu_accel[1]*(sy*srsp+cy*cr) + imu_accel[2]*(crsp*sy-sr*cy));
            acc_d = -(imu_accel[0]*-sp  + imu_accel[1]*(cp*sr)  + imu_accel[2]*(cp*cr)   -accel_g);
        }
        else if (stage == 200)
        {
            stage++; 
            g  /= 200.0;
            acc_x = -(imu_accel[0]*cp   + imu_accel[1]*srsp     + imu_accel[2]*crsp);
            acc_y = -(                    imu_accel[1]*cr       - imu_accel[2]*sr);
            acc_n =   acc_x*cy - acc_y*sy; //acc_n = -(imu_accel[0]*cp*cy + imu_accel[1]*(cy*srsp-sy*cr) + imu_accel[2]*(crsp*cy+sr*sy));
            acc_e =   acc_x*sy + acc_y*cy;//acc_e = -(imu_accel[0]*cp*sy + imu_accel[1]*(sy*srsp+cy*cr) + imu_accel[2]*(crsp*sy-sr*cy));
            acc_d = -(imu_accel[0]*-sp  + imu_accel[1]*(cp*sr)  + imu_accel[2]*(cp*cr)   - g);
        }
        else
        {
            stage = 201;
            acc_x = -(imu_accel[0]*cp   + imu_accel[1]*srsp     + imu_accel[2]*crsp);
            acc_y = -(                    imu_accel[1]*cr       - imu_accel[2]*sr);
            acc_n =   acc_x*cy - acc_y*sy; //acc_n = -(imu_accel[0]*cp*cy + imu_accel[1]*(cy*srsp-sy*cr) + imu_accel[2]*(crsp*cy+sr*sy));
            acc_e =   acc_x*sy + acc_y*cy;//acc_e = -(imu_accel[0]*cp*sy + imu_accel[1]*(sy*srsp+cy*cr) + imu_accel[2]*(crsp*sy-sr*cy));
            acc_d = -(imu_accel[0]*-sp  + imu_accel[1]*(cp*sr)  + imu_accel[2]*(cp*cr)   - g);
        }    
              
        static double N=0,dN=0,E=0,dE=0,D=0,dD=0;

        dN = (dN+acc_n*0.0333333)*0.9 + position.raw_current_uvw[0]*0.1;
        N = (N+dN*0.0333333)*0.9 + position.raw_current_xyz[0]*0.1;
        
        dE = (dE+acc_e*0.0333333)*0.9 + (position.raw_current_uvw[1])*0.1;
        E = (E+dE*0.0333333)*0.9 + position.raw_current_xyz[1]*0.1;
     
        
 //��ѹ�����ݵ���С���˷��˲�
#define array_length 30
#define a_l 30.0      
#define sumx 465.0 //sumx=0;sumx2=0;for i = 1:array_length;sumx=sumx+i;sumx2 = sumx2+i*i;end;sumx,sumx2%matlab����
#define sumx2 9455.0
        static double d_array[array_length]={0};
        static int num = 0;
        static double sumy = 0;
        static double sumxy = 0;
        static double sumy_cal = 0;
        static double sumxy_cal = 0;

        double d,dd;
        sumxy = (sumxy - sumy) + a_l*bmp_height;      
        sumy = sumy - d_array[num] + bmp_height;
        d_array[num] = bmp_height;
        num++;
        sumy_cal += bmp_height;
        sumxy_cal += num*bmp_height;
        if (num == array_length) 
        {
          num = 0;
          sumy = sumy_cal;//�������Բ���У׼sumy��sumxy,��������,Ȼ��������ֲ���;
          sumxy = sumxy_cal;
          sumy_cal = 0;
          sumxy_cal = 0;
        }
        dd =  (sumxy*a_l- sumx*sumy)/(sumx2*a_l-sumx*sumx);       //��С���˷���ϵ�б��
        d = (sumy - dd*sumx)/a_l + dd*a_l;                        //��С���˷���ϵĹ��Ƶ�
        dd = dd*30.0;
        
        static double previous_d = 0.0;
        double delta_d;
        delta_d = d - previous_d;
        previous_d = d;
        
 //lidarlite    
#define healthy 0                                         //��lidarlite����״̬Ϊ0,������ʹ�������ݵ�״̬
        static int lidar_state = 0;                       //���ڼ�¼lidarlite״̬,0Ϊ����,������Ϊ��ȷ������,,,�������
        static int lidar_miss = 0;                        //���ڿ�lidarlite�Ƿ��и���,,,������
        static double previous_lidar_lite_distance;       
        int previous_lidar_state;                         //���ڼ�¼��һ��lidarlite��״̬
        previous_lidar_state = lidar_state;
        if (abs(previous_lidar_lite_distance - lidar_lite_distance)>1)
        {
            lidar_state = 15;                             //�����������,����15
        } 
        else 
        {
            if(lidar_state > healthy) lidar_state--;      //���û����,��lidar_state������СΪ0,������״̬
        }
        if (previous_lidar_lite_distance == lidar_lite_distance)  
        {
        lidar_miss++;
        if (lidar_miss>15)  lidar_miss = 15;              //������ݲ���,�ﵽ15����,����Ϊ�Ƕ�ʧlidarlite����,����û�н�lidarlite
        }
        else 
        {
        lidar_miss = 0;                                   //һ�������б�,��ô֤��lidarlite���ڸ���
        }
        previous_lidar_lite_distance = lidar_lite_distance;
        
        double distance;
        distance = lidar_lite_distance*cp*cr;             //lidarlite��������ֱ������ͶӰ
        
//lidarlite����С���˷��˲�
#define array_length_ld 10
#define a_l_ld 10.0      
#define sumx_ld 55.0 //sumx=0;sumx2=0;for i = 1:array_length;sumx=sumx+i;sumx2 = sumx2+i*i;end;sumx,sumx2%matlab����
#define sumx2_ld 385.0
        static double d_array_ld[array_length_ld]={0};
        static int num_ld = 0;
        static double sumy_ld = 0;
        static double sumxy_ld = 0;
        static double sumy_cal_ld = 0;
        static double sumxy_cal_ld = 0;

        double d_ld,dd_ld;
        sumxy_ld = (sumxy_ld - sumy_ld) + a_l_ld*distance;
        sumy_ld = sumy_ld - d_array_ld[num_ld] + distance;
        d_array_ld[num_ld] = distance;
        num_ld++;
        sumy_cal_ld += distance;
        sumxy_cal_ld += num_ld*distance;
        if (num_ld == array_length_ld) 
        {
          num_ld = 0;
          sumy_ld = sumy_cal_ld;//�������Բ���У׼sumy��sumxy,��������,Ȼ��������ֲ���;
          sumxy_ld = sumxy_cal_ld;
          sumy_cal_ld = 0;
          sumxy_cal_ld = 0;
        }
        dd_ld =  (sumxy_ld*a_l_ld- sumx_ld*sumy_ld)/(sumx2_ld*a_l_ld-sumx_ld*sumx_ld);
        d_ld = (sumy_ld - dd_ld*sumx_ld)/a_l_ld + dd_ld*a_l_ld;
        dd_ld = dd_ld*30.0;        
   
        static double raw_D = 0.0;
        double raw_dD;
        if ((lidar_state == healthy)&&(lidar_miss < 15)&&(lidar_lite_distance > -15.0))//lidar_lite
        {
          if (previous_lidar_state == healthy)
          {
           raw_D =  d_ld;
           raw_dD = dd_ld;
          }
          else              //����������
          {
           double bias;
           bias = d_ld - raw_D;
           raw_D = d_ld;
           D += bias;
           position.current_target[2] += bias;
           raw_dD = dd_ld;
          }
        }
        else
        {
           raw_D = raw_D + delta_d;
           raw_dD = dd;
        }
          
        dD += acc_d*0.0333333;
        dD = dD*0.9 + raw_dD*0.1;//0.6=0.02/0.0333333      
        D = (D+dD*0.0333333)*0.9 + raw_D*0.1;
        
//        double old_point;       
//        old_point = N; 
//        dN += acc_n*0.0333333;
//        N += dN*0.0333333;
//        N = N*0.8 + position.raw_current_xyz[0]*0.2;
//        dN = dN*0.9 + (N-old_point)*3.0;//3=0.1/0.0333333
//        
//        old_point = E; 
//        dE += acc_e*0.0333333;
//        E += dE*0.0333333;
//        E = E*0.8 + position.raw_current_xyz[1]*0.2;
//        dE = dE*0.9 + (E-old_point)*3.0;//3=0.1/0.0333333
//     
//        old_point = D; 
//        D = (D + (dD + acc_d*0.0333333)*0.0333333)*0.95 + bmp_height*0.05;
//        dD = dD*0.95 + (D-old_point)*1.5;//1.5=0.05/0.0333333
        
        input_xyz[0] = N;
        input_xyz[1] = E;
        input_xyz[2] = D;
        input_uvw[0] = dN;
        input_uvw[1] = dE;
        input_uvw[2] = dD;
        if(servo_widths_temp[3]<=2200)
        {
        position.current_target[0] = N;
        position.current_target[1] = E;
        position.current_target[2] = D;
        }
        
        double X,dX,Y,dY;
        X = cy*(N-position.current_target[0]) + sy*(E-position.current_target[1]);
        dX = cy*dN + sy*dE;
        Y = -sy*(N-position.current_target[0]) + cy*(E-position.current_target[1]);
        dY = -sy*dN + cy*dE; 
/////////////////////////////////////////////////////////////////������/////////////////////////    
static int coll_mid = 3185;//h
static int throttle = 2200;//h
if( remote_missing > 15) 
{
    control_mode=3;
    servo_widths_temp[8] = 2899;
    servo_widths_temp[2] = remote_median[1];
    servo_widths_temp[1] = remote_median[0];
    servo_widths_temp[0] = coll_mid;
}     
 //�ǶȻ�ƽ����ȡ
/***************************************ң�����ɵ�**********************************************/     
static double even_point[2] = {0,0};//ƽ���,��Ӧƽ���Ƕ�
if ((servo_widths_temp[7] > 2900) && (servo_widths_temp[8] > 2900))
{
     even_point[0] = (servo_widths_temp[7] - 3349)/20.0*C_DEG2RAD;
     even_point[1] = (servo_widths_temp[8] - 3349)/20.0*C_DEG2RAD;
}        
//�ǶȻ��ڴ�ֵ
double desired_roll_angle,desired_pitch_angle;        

if (servo_widths_temp[8] > 2900)
{
/**********************************�ֿ�ֵ--->�Ƕ�ֵ,�Ƕ���ҡ��ƫ��������*********************************************/ 
    desired_roll_angle =  (servo_widths_temp[1] - remote_median[0])/20.0*C_DEG2RAD;//ң��������ֵ�仯+-100��Ӧ����ֵ�仯+-5��
    desired_pitch_angle = -(servo_widths_temp[2] - remote_median[1])/20.0*C_DEG2RAD;//ң��������ֵ�仯+-100��Ӧ����ֵ�仯+-5��        
    position.current_target[0] = N;
    position.current_target[1] = E;
    position.current_target[2] = D;
    desired_x_rate = dX;
    desired_y_rate = dY;
    desired_d_rate = dD;
    coll_mid = servo_widths_temp[6];//h
    throttle = servo_widths_temp[3];
    temp[2] = 0;
}
else
{
//�ٶȻ��ڴ�ֵ
        double desired_final_x_velocity,desired_final_y_velocity;
        double desired_x_acceleration,desired_y_acceleration;
        static double desired_next_x_velocity = 0.0;
        static double desired_next_y_velocity = 0.0;
//λ�û��ڴ�ֵ�仯��
        double desired_d_x,desired_d_y;
/**********************************�ֿ�ֵ--->�仯��λ��ֵ,�����ٶ���ҡ��ƫ��������*********************************************/    
        
        desired_final_x_velocity  = (servo_widths_temp[2] - remote_median[1]);
        if(desired_final_x_velocity > 23.0) desired_final_x_velocity -= 23.0;//������,��Ʈ
        else if(desired_final_x_velocity < -23.0) desired_final_x_velocity += 23.0;
        else desired_final_x_velocity = 0.0;
        desired_final_x_velocity /= 200.0;//ң��������ֵ�仯+-100��Ӧ0.5m/s
        
        desired_final_y_velocity  = (servo_widths_temp[1] - remote_median[0]);
        if(desired_final_y_velocity > 23.0) desired_final_y_velocity -=  23.0;//������,��Ʈ 
        else if(desired_final_y_velocity < -23.0) desired_final_y_velocity += 23.0;
        else desired_final_y_velocity = 0.0;
        desired_final_y_velocity /= 200.0;//ң��������ֵ�仯+-100��Ӧ0.5m/s
        
        limit( desired_final_x_velocity , -1.5 , 1.5); //�ٶ��޷�
        limit( desired_final_y_velocity , -1.5 , 1.5); //�ٶ��޷�   
        
        if ((desired_final_x_velocity-desired_next_x_velocity)>0.4)//�����ٶȽ���,�Լ��ڴ����ٶȸ���
          {desired_next_x_velocity += 0.05;desired_x_acceleration = 1.5;}
        else if ((desired_final_x_velocity-desired_next_x_velocity)<-0.4)
          {desired_next_x_velocity -= 0.05;desired_x_acceleration = -1.5;}
        else
          {desired_next_x_velocity = desired_final_x_velocity;desired_x_acceleration = 0.0;}
        
        if ((desired_final_y_velocity-desired_next_y_velocity)>0.4)//�����ٶȽ���,�Լ��ڴ����ٶȸ���
          {desired_next_y_velocity += 0.05;desired_y_acceleration = 1.5;}
        else if ((desired_final_y_velocity-desired_next_y_velocity)<-0.4)
          {desired_next_y_velocity -= 0.05;desired_y_acceleration = -1.5;}
        else
          {desired_next_y_velocity = desired_final_y_velocity;desired_y_acceleration = 0.0;}
            
          
        desired_x_rate = desired_next_x_velocity;
        desired_y_rate = desired_next_y_velocity;       
        desired_d_x = desired_next_x_velocity/30.0;
        desired_d_y = desired_next_y_velocity/30.0;    
        position.current_target[0] += cy*desired_d_x - sy*desired_d_y;//����תned  
        position.current_target[1] += sy*desired_d_x + cy*desired_d_y;//����תned
/**********************************�ٶ�ֵ--->�ǶȻ�,���ٶȻ��������*********************************************/ 
        desired_pitch_angle = -(0 - X)*out_loop_X_PID[0];//�ٶ����1m/s��Ӧ�����Ƕȱ仯+-5��  
        desired_roll_angle = (0 - Y)*out_loop_Y_PID[0];//�ٶ����1m/s��Ӧ�����Ƕȱ仯+-5��
        desired_pitch_angle = limit( desired_pitch_angle , -10.0 , 10.0); //�Ƕ��޷�,�Ƕ���,�������޷�,����ͣʱ,�����Ʒɻ�����Ŀ��������ٶȵ�����
        desired_roll_angle = limit( desired_roll_angle , -10.0 , 10.0); //�Ƕ��޷�,�Ƕ���       
        desired_pitch_angle += -(desired_next_x_velocity - dX)*out_loop_X_PID[2];
        desired_roll_angle += (desired_next_y_velocity - dY)*out_loop_Y_PID[2];
        desired_pitch_angle = limit( desired_pitch_angle , -18.0 , 18.0); //�Ƕ��޷�,�Ƕ���
        desired_roll_angle = limit( desired_roll_angle , -18.0 , 18.0); //�Ƕ��޷�,�Ƕ���  
        desired_pitch_angle += -(desired_x_acceleration - acc_x)*8.5;                         
        desired_roll_angle += (desired_y_acceleration - acc_y)*8.5;
        desired_pitch_angle = limit( desired_pitch_angle , -18.0 , 18.0); //�Ƕ��޷�,�Ƕ���
        desired_roll_angle = limit( desired_roll_angle , -18.0 , 18.0); //�Ƕ��޷�,�Ƕ���  
        
        desired_pitch_angle *= C_DEG2RAD;
        desired_roll_angle *= C_DEG2RAD;
//        if (abs(ahrs_pqr[2])>0.8)
//        {
//        desired_pitch_angle = 0.0;
//        desired_roll_angle = 0.0;
//        }
            
                
        double desired_d_velocity;//h
        desired_d_velocity = -(servo_widths_temp[6] - coll_mid);
        if(desired_d_velocity > 23.0) desired_d_velocity -=  23.0;//������,��Ʈ 
        else if(desired_d_velocity < -23.0) desired_d_velocity += 23.0;
        else desired_d_velocity = 0.0;
        desired_d_velocity /= 100.0;//ң��������ֵ�仯+-100��Ӧ1m/s
        limit( desired_d_velocity , -0.5 , 0.5); //�ٶ��޷�//h
        desired_d_rate = desired_d_velocity;
        position.current_target[2] += desired_d_velocity/30.0;//h
        temp[2] = (position.current_target[2] - D)*out_loop_D_PID[0];//h
        //temp[2] = limit( temp[2] , -300.0 , 300.0); //h
        temp[2] += (desired_d_velocity - dD)*out_loop_D_PID[2];//h
        //temp[2] = limit( temp[2] , -300.0 , 300.0);  //h
        if (servo_widths_temp[3]>2200)  
        {
            servo_widths_temp[3] = throttle;
        }
        else
        {
            throttle = servo_widths_temp[3];
        }
        
}
        roll_commend[0]  = even_point[0] + desired_roll_angle; 
        pitch_commend[0] = even_point[1] + desired_pitch_angle;  
        

/*�ڻ�PD������*************���������ô��ڷ�,��ʽ:$,1100,20,900,13,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,**********************/       
        
/*���ĵ�ͨ�˲�*/  
static double roll_old_error = 0.0;     
static double pitch_old_error = 0.0; 
double error,d_error;
#define roll_alpha 0.4
#define pitch_alpha 0.4
    error = roll_commend[0] - roll_angle;
    d_error = (error - roll_old_error)*roll_alpha;
    error = roll_old_error + d_error;
    d_error = d_error/0.033333;
    roll_old_error = error;
    temp[0] = in_loop_roll_PID[0]*(error) + in_loop_roll_PID[2]*(d_error); //roll
//    temp[0] = in_loop_roll_PID[0] * error + (25.0) * d_error; //roll
    
    error = pitch_commend[0] - pitch_angle;
    d_error = (error - pitch_old_error)*pitch_alpha;
    error = pitch_old_error + d_error;
    d_error = d_error/0.033333;
    pitch_old_error = error; 
    temp[1] = in_loop_pitch_PID[0]*(error) + in_loop_pitch_PID[2]*(d_error); //pitch 
//    temp[1] = in_loop_pitch_PID[0] * error + (-30.0) * d_error; //pitch 
//      
//    temp[0] += abs(in_loop_roll_PID[2] * filter_pqr[2]);
//    temp[1] += in_loop_pitch_PID[2] * filter_pqr[2];
    
/*�Կ���ת����coll��pitch��ҡ����(�Կ�״̬��,���Կ�ʱҡ����ֱ����ң��������,Ҳ����˵�������Ƿ���)******************/                
        if(control_mode == 3 && last_control_mode == 3)//�Կ�
          {	
           servo_widths_temp[1] = remote_median[0] + temp[0];//roll
           servo_widths_temp[2] = remote_median[1] + temp[1];//pitch
           servo_widths_temp[6] = coll_mid + temp[2];//coll//h
          }
/*ҡ�������*******************************************************************************************************/
        servo_widths_mid[0] = (servo_widths_temp[1]-2838) *13/25;//roll
        servo_widths_mid[1] = (servo_widths_temp[2]-2838) *3/5;//pitch
        servo_widths_mid[2] = (servo_widths_temp[6]-2838) *2/5;//coll 
//--------------------------------�޷�-----------------------------------------------//        
            servo_widths_mid[0] = limit( servo_widths_mid[0] , -480 , 480);
            servo_widths_mid[1] = limit( servo_widths_mid[1] , -550 , 550); 
            servo_widths_mid[2] = limit( servo_widths_mid[2] , -500 , 500);//h
//------------------------------�Կ�PWM�л�����---------------------------------------//
            if (control_mode==3 && last_control_mode==1)
              {
                  man2auto = 0; 
                  remote_median[0] = servo_widths_temp[1];//rollҡ����λֵ
                  remote_median[1] = servo_widths_temp[2];//pitchҡ����λֵ
              }                 
            if(man2auto < 15 )
              {
                  if (man2auto == 0)
                  {
                      last_pwm[0] = servo_widths_mid[0];		//roll
                      last_pwm[1] = servo_widths_mid[1];		//pitch
                  } 
                  servo_widths_mid[0] = last_pwm[0] + (servo_widths_mid[0]-last_pwm[0]) / (15-man2auto) ;//roll            	                              
                  last_pwm[0] = servo_widths_mid[0];                  
                  servo_widths_mid[1] = last_pwm[1] + (servo_widths_mid[1]-last_pwm[1]) / (15-man2auto);//pitch
                  last_pwm[1] = servo_widths_mid[1];    
             }       
//--------------------------3ͨ���ֿ�ֵ����----------------------------------------------//
        servo_widths[1] = servo_widths_mid[0]   +  servo_widths_mid[1]/2  - servo_widths_mid[2] + 2838;//roll 
        servo_widths[2] =                          servo_widths_mid[1]    + servo_widths_mid[2] + 2838;//pitch
        servo_widths[6] = servo_widths_mid[0]   -  servo_widths_mid[1]/2  + servo_widths_mid[2] + 2838;//coll   
              
              
        servo_widths[1] = limit_int(servo_widths[1],1621,3927);//roll
        servo_widths[2] = limit_int(servo_widths[2],1621,3927);//pitch
        servo_widths[6] = limit_int(servo_widths[6],1683,3927);//coll   	
   
       //yaw(ԭ����gain,����ͨ�����ƫ��ͨ��ҡ��ֵ,���Խ�gy401)     
/********************************��β�㷨**********************************************/   
        servo_widths[7] = limit_int(servo_widths_temp[4],1621,3865);;//limit_int(servo_widths_temp[4],1621,3865);//����������401��yaw          
        avcs(servo_widths_temp+4);  //2015-6-29 byM , ��βavcs 
        servo_widths[4] = limit_int(servo_widths_temp[4],1621,3865);//yaw     
        servo_widths[5] = limit_int(servo_widths_temp[5],1621,3865);//gain        
        servo_widths[3] = limit_int(servo_widths_temp[3],1621,3865);//throttle
        
        servo_widths[8] = 2838;//limit_int(servo_widths_temp[8],1621,3865);           

        int pwm_sum = 0;
        for(char i=1;i<9;i++)
        pwm_sum+=servo_widths_temp[i];
        servo_widths[9]=srevo_period-servo_widths[0]-pwm_sum;  //keep 18ms period        
}


void deal_groundtoarm_date_auto(void)
{
  char save_line[UPLOAD_NUM],*line=NULL;//11.01 ��ָ�� ������
  strncpy( save_line, groundtoarm,UPLOAD_NUM );
  line=save_line;
  line++;// skip ,
  /*----PD kp KD-----------------------------roll----------------------*/ 
  in_loop_roll_PID[0]=strtod(line,&line);
  line++;// skip ,
  in_loop_roll_PID[2]=strtod(line,&line);
  line++;// skip ,
  /*----PD kp KD-------------------------pitch-----------------*/ 
  in_loop_pitch_PID[0]=-strtod(line,&line);
  line++;// skip ,
  in_loop_pitch_PID[2]=-strtod(line,&line);
  line++;// skip ,
  out_loop_X_PID[0]=strtod(line,&line);
  line++;// skip ,
  out_loop_X_PID[2]=strtod(line,&line);
  line++;// skip ,
  out_loop_Y_PID[0]=strtod(line,&line);
  line++;// skip ,
  out_loop_Y_PID[2]=strtod(line,&line);
  line++;// skip ,
  out_loop_D_PID[0]=-strtod(line,&line);
  line++;// skip ,
  out_loop_D_PID[2]=-strtod(line,&line);
}


//void servo_update(void)
//{
//    unsigned char * pcmline=NULL;
//    long int pcm_temp[9]={0};
//    int ppm_temp[9]={0};//11.01
//    unsigned int decode_flag=0;
//    double widths_temp;
//	int outputWidths[3] = {0};
//	int inputpwmMid[3]  = {0};
//	int outputpwmMid[3] = {0};
//	double temp[4]={0.0};   
//    man2auto++;
//    if (man2auto > 10000)
//          man2auto = 100; 
////---------------------------�޷�-------------------------------------------------------//
///*
//       auto_servo_widths[0] = limit_int(auto_servo_widths[0],1683,3927);//1350 is 0.9ms
//       auto_servo_widths[1] = limit_int(auto_servo_widths[1],1683,3927);//3150 is 2.1ms
//       auto_servo_widths[2] = limit_int(auto_servo_widths[2],1683,3927);
//       auto_servo_widths[3] = limit_int(auto_servo_widths[3],1683,3927); 
//*/             
//     if(pcm_rece_flag == 1)
//    {
//     	pcm_rece_flag=0;
//        pcm_update_flag = 1;
//              
//        if ( (AVR_PCM[0] != '$') || (AVR_PCM[1] != '$') || AVR_PCM[PCM_NUM-2] != '\r' || AVR_PCM[PCM_NUM-1] != '\n')
//       {
//        	pcm_update_flag = 3;
//       }
//        else
//       {
//		uncharcpy(pcm_line, AVR_PCM+2,24 );
// 	}
//    } 
//
//	 pcmline=pcm_line;
//	 
//     	for(char i=1;i<9;i++)
//       {
//        	pcm_temp[i]=strntol((char*)pcmline,3,(char**)&pcmline,16);
//            	if(pcm_temp[i]==0)
//               {
//                	decode_flag++;
//               }
//            ppm_temp[i] = (int)(0.8304* pcm_temp[i]-385.2857);
//       }
//        
//	    if(pcm_temp[7] < 0x400)//���Կ�ֵ��ֵ��
//        	ppm_temp[7] = 0x028;
//        else
//            ppm_temp[7] = 0x3D8;
//        if(pcm_temp[8] < 0x400)//���������ض�ֵ��
//            ppm_temp[8] = 0x028;
//        else
//            ppm_temp[8] = 0x3D8;
//                
//        if( decode_flag == 0 )
//       {
//        	int pwm_sum=0;
//			
//            for(char i=1;i<9;i++)
//           {
//            	servo_widths_temp[i]=( int)((ppm_temp[i]*1.1718+920)*3/2);
//                servo_widths_temp[i]=servo_widths_temp[i]*91/73;
//                pwm_sum+=servo_widths_temp[i];
//           }
//	   
// 		   
//           servo_widths_temp[9]=srevo_period-servo_widths_temp[0]-pwm_sum;  //keep 18ms period
//
////--------------------------------�����ֿ�ֵ����syc------------------------------------//
//			for(char i=1;i<7;i++)
//	   	   { 
//				widths_temp=last_servo_widths_temp[i]-servo_widths_temp[i];
//			
//				if(widths_temp < -1400 || widths_temp > 1400)
//		       {
//					lim_servo_widths_temp[i] = last_servo_widths_temp[i];
//					if(i!=5)
//						pcm_update_flag = 3;
//		       }
//				else
//		   	   {
//					lim_servo_widths_temp[i] = servo_widths_temp[i];
//		       } 
//					last_servo_widths_temp[i] = lim_servo_widths_temp[i];
//		    }		
////-------------------------------���Կ�λ���--------------------------------------------//
//         	last_control_mode=control_mode;
//         	if(servo_widths_temp[7]<1877)
//		        control_mode=1; //0x028 is mode 1 0x200 is mode 2
//            if(servo_widths_temp[7]>3649)
//		        control_mode=3;  //0x3d8 is mode 3;		
////------------------------------�Կ�PWM�л�����---------------------------------------//
//          	if (control_mode==3 && last_control_mode==1)
//          		man2auto = 0;
//          	if (man2auto == 0)
//           {
//          	last_pwm[0] = servo_widths_temp[1];		//roll
//            	last_pwm[1] = servo_widths_temp[2];		//pitch
//            	last_pwm[2] = servo_widths_temp[4];		//yaw
//            	last_pwm[3] = servo_widths_temp[6];		//coll
//           }
//          	if(man2auto < 20 )
//           {
//          		if (auto_servo_widths[0] > last_pwm[0])//roll
//            		auto_servo_widths[0] = last_pwm[0] + (auto_servo_widths[0] - last_pwm[0]) / 20 ;
//            	else 
//                	auto_servo_widths[0] = last_pwm[0] - (last_pwm[0] - auto_servo_widths[0]) / 20 ;
//                	last_pwm[0] = auto_servo_widths[0];
//                    
//            	if (auto_servo_widths[1] > last_pwm[1])//pitch
//             		auto_servo_widths[1] = last_pwm[1] + (auto_servo_widths[1] - last_pwm[1]) / 20;
//            	else 
//                	auto_servo_widths[1] = last_pwm[1] - ( last_pwm[1] - auto_servo_widths[1]) / 20 ;
//                	last_pwm[1] = auto_servo_widths[1];
//                
//            	if (auto_servo_widths[2] >  last_pwm[2])//yaw
//                	auto_servo_widths[2] =  last_pwm[2] + (auto_servo_widths[2] - last_pwm[2]) / 20 ;
//            	else 
//                	auto_servo_widths[2] = last_pwm[2] - (  last_pwm[2] - auto_servo_widths[2]) / 20 ;
//                	last_pwm[2] = auto_servo_widths[2];
//                
//            	if (auto_servo_widths[3] > last_pwm[3])//coll
//                	auto_servo_widths[3] = last_pwm[3] + (auto_servo_widths[3] - last_pwm[3]) / 20 ;
//            	else 
//                	auto_servo_widths[3] =last_pwm[3] - ( last_pwm[3] - auto_servo_widths[3]) / 20 ;
//                	last_pwm[3] = auto_servo_widths[3];
//           }
//       }//end of decode_flag == 0
//	else 
//         {
//                  pcm_update_flag=2;
//         }     
//                    
//            decode_flag=0;
//	   
////------------------------�Կ���--------------------------------------------//	   
//	   
//    	temp[0] = limit(outputs[1]*servoAmplify[0],-561,561);	//LQR roll
//    	temp[1] = limit(outputs[2]*servoAmplify[1],-748,748);	//LQR pitch
//    	temp[2] = limit(outputs[3]*amplify*C_RAD2DEG,-1122,1122);//yaw
//    	temp[3] = limit(outputs[0]*servoAmplify[3],-561,561);//coll    
//		
////------------------------3ͨ���ֿ�ֵ����-----------------------------------//
//		
//		inputpwmMid[0] = servo_widths_temp[6];//pitch
//		inputpwmMid[1] = servo_widths_temp[1];//roll
//		inputpwmMid[2] = servo_widths_temp[2];//coll
//		
//		DecoupingMatrix(outputpwmMid, inputpwmMid);
//		
//		manual_servo_widths[0] = outputpwmMid[1];//roll 
//		manual_servo_widths[1] = outputpwmMid[2];//coll  
//		manual_servo_widths[2] = outputpwmMid[0];//pitch 
//	   
////--------------------------3ͨ���Կ�ֵ����----------------------------------------------//
////pitch	   
//	if(auto_pitch_control_mode == '1' && control_mode == 3)//pitch��ͨ���Կ�
//        {
//            auto_servo_widths_mid[0] = pwm_pitch_mid + (int)temp[1];
//        }
//        else
//        {
//            auto_servo_widths_mid[0] = manual_servo_widths[2];
//        }
//
//	    //auto_servo_widths_mid[0] = manual_servo_widths[2];
//	    //auto_servo_widths_mid[0] = manual_servo_widths[2] + (int)temp[1];
//	    //auto_servo_widths_mid[0] = pwm_pitch_mid + (int)temp[1];
////roll
//	if(auto_roll_control_mode == '1' && control_mode == 3)//roll��ͨ���Կ�
//        {
//            auto_servo_widths_mid[1] = pwm_roll_mid + (int)temp[0];
//        }
//        else
//        {
//            auto_servo_widths_mid[1] = manual_servo_widths[0];
//        }
//
//		//auto_servo_widths_mid[1] = manual_servo_widths[0];	   
//	    //auto_servo_widths_mid[1] = manual_servo_widths[0] + (int)temp[0];
//		//auto_servo_widths_mid[1] = pwm_roll_mid + (int)temp[0];
////coll
//	if(auto_coll_control_mode == '1' && control_mode == 3)//coll��ͨ���Կ�
//        {
//            auto_servo_widths_mid[2] = pwm_coll_mid + (int)temp[3];
//        }
//        else
//        {
//            auto_servo_widths_mid[2] = manual_servo_widths[1];
//        }
//
//	    //auto_servo_widths_mid[2] = manual_servo_widths[1];
//		//auto_servo_widths_mid[2] = manual_servo_widths[1] + (int)temp[3];
//		//auto_servo_widths_mid[2] = pwm_coll_mid + (int)temp[3];
////yaw
//
//		auto_servo_widths_mid[3] = pwm_yaw_mid + (int)temp[2];
//		
//		CoupingMatrix(outputWidths, auto_servo_widths_mid);
//	   
//	    auto_servo_widths[0] = outputWidths[1];//roll
//	    auto_servo_widths[1] = outputWidths[2];//coll
//	    auto_servo_widths[2] = pwm_yaw_mid + (int)temp[2];
//	    auto_servo_widths[3] = outputWidths[0];//pitch   
//
////---------�ֿ��Կ�ֵѡ�������ʮ������H-1���HR3 coll��pitch����λ��--------------//
///*
////roll
//          if(auto_roll_control_mode == '1' && control_mode == 3)
//          	servo_widths[1] = auto_servo_widths[0];
//          else servo_widths[1]=limit(lim_servo_widths_temp[1],2119,3865);
////coll
//          if(auto_pitch_control_mode == '1' && control_mode == 3)
//          	servo_widths[2] = auto_servo_widths[1] ;
//          else servo_widths[2] = limit(lim_servo_widths_temp[2],2244,3865);
////yaw
//          if(auto_yaw_control_mode == '1' && control_mode == 3)  
//          	servo_widths[4] = auto_servo_widths[2] ;
//          else servo_widths[4] = limit(lim_servo_widths_temp[4],2119,3865);
////pitch
//          if(auto_coll_control_mode=='1'&&control_mode==3)
//          	servo_widths[6] = auto_servo_widths[3] ;
//          else servo_widths[6]=limit(lim_servo_widths_temp[6],2119,3865);
//*/
//  	if(control_mode == 3)
//	{	
//		servo_widths[1] = limit_int(auto_servo_widths[0],1683,3927);
//		servo_widths[2] = limit_int(auto_servo_widths[1],1683,3927);
//		servo_widths[4] = limit_int(auto_servo_widths[2],1683,3927);
//		servo_widths[6] = limit_int(auto_servo_widths[3],1683,3927);
//	}
//	else
//	{
//		servo_widths[1] = limit(lim_servo_widths_temp[1],1621,3865);
//		servo_widths[2] = limit(lim_servo_widths_temp[2],1621,3865);
//		servo_widths[4] = limit(lim_servo_widths_temp[4],1621,3865);
//		servo_widths[6] = limit(lim_servo_widths_temp[6],1621,3865);	
//	}
//		  
////throttle
//		servo_widths[3] = limit(lim_servo_widths_temp[3],1621,3865);
////lock tail
//		servo_widths[5] = limit(servo_widths_temp[5],1621,3865);
//                servo_widths[7] = limit(servo_widths_temp[7],1621,3865);
//                servo_widths[8] = limit(servo_widths_temp[8],1621,3865);
//                servo_widths[9] = limit(servo_widths_temp[9],2244,3865);
//}



//void deal_groundtoarm_date(void)
//{
//  char save_line[UPLOAD_NUM],*line=NULL;//11.01 ��ָ�� ������
//  strncpy( save_line, groundtoarm,UPLOAD_NUM );
//  line=save_line;
//  if(line[0]!='$'||line[UPLOAD_NUM-1]!='\n')
//  {
//    receive_index = false;
//     AT91F_US_DisableIt(COM0,AT91C_US_RXBUFF  );
//
//     COM0->US_RCR = 0; 
//     AT91F_US_EnableIt(COM0,AT91C_US_RXBUFF  );   
//    return;
//  }
//
//  
//   if (!strncmp(line,"$TARG",5))
//  {
//     target_receive_flag = true;
//     receive_index=true;
//     line=line+6;// skip $PARA  target ��һ������ڳ�ʼ��ͣ���λ��
//     target_command.get_command(line);
//  }
//
//  if (!strncmp(line,"$PARA",5))
//  {
//   receive_index=true;
//     line=line+6;// skip $PARA,
//  
//  auto_roll_control_mode=line[0];
//  line++;
//  auto_pitch_control_mode=line[0];
//  line++;
//  auto_yaw_control_mode=line[0];
//  line++;
//  auto_coll_control_mode=line[0];
//  line++;
//  line++;
//  
//  amplify = strntol(line,2,&line,10);
//  //radius=strntol(line,2,&line,10);//��Բ��
//  line++;
//  
//  //PID middle
//  pwm_roll_mid=strntol(line,3,&line,16);
//  pwm_pitch_mid=strntol(line,3,&line,16);
//  pwm_yaw_mid=strntol(line,3,&line,16);
//  //pwm_coll_mid=strntol(line,3,&line,16);
//  line+=3;//syc 4.16  ���Լ�ȷ��coll���ֵ ����
//  line++;
//  
//  /*----PID X kp ki KD-----------------------------X"-"-------------------------*/                                  
//  out_loop_X_PID[0]=-strtod(line,&line);
//  line++;
//  out_loop_X_PID[1]=-strtod(line,&line);
//  line++;
//  out_loop_X_PID[2]=-strtod(line,&line);
//  
//  /*----PID Y kp ki KD-----------------------------Y--------------------------*/ 
//  line++;
//  out_loop_Y_PID[0]=strtod(line,&line);
//  line++;
//  out_loop_Y_PID[1]=strtod(line,&line);
//  line++;
//  out_loop_Y_PID[2]=strtod(line,&line);
//  //--------------------------------------syc
//  /*----PID X kp ki KD-----------------------------D--------------------------*/ 
//  line++;
//  out_loop_D_PID[0]=-strtod(line,&line);
//  line++;
//  out_loop_D_PID[1]=-strtod(line,&line);
//  line++;
//  out_loop_D_PID[2]=-strtod(line,&line);
//  
//  /*----PID X kp ki KD-----------------------------pitch----------------------*/ 
//  line++;
//  LQR_pitchGain[0]=strtod(line,&line);
//  line++;
//  LQR_pitchGain[1]=strtod(line,&line);
//  line++;
//  LQR_pitchGain[2]=strtod(line,&line);
//  
//  /*----PID Y kp ki KD-----------------------------roll"-"-----------------*/ 
//  line++;
//  LQR_rollGain[0]=strtod(line,&line);
//  line++;
//  LQR_rollGain[1]=strtod(line,&line);
//  line++;
//  LQR_rollGain[2]=strtod(line,&line);
//  
//  /*----PID YAW kp ki KD-----------------------------yaw"-"-------------------------*/ 
//  line++;
//  in_loop_yaw_PID[0]=-strtod(line,&line);
//  line++;
//  in_loop_yaw_PID[1]=-strtod(line,&line);
//  line++;
//  in_loop_yaw_PID[2]=-strtod(line,&line);
//  line++;
//  double temp = strtod(line,&line);
//   
//  temp=(-1)*(temp-5);
//
//  //if(radius>=10)
//  //{
//    
//  //    radius=(-1)*(radius-10);
//  //}
//  trajectory.com_theta[0]=temp*C_DEG2RAD;
//  trajectory.com_theta[1]=temp*C_DEG2RAD;
//  //if(trajectory.heli_step == 1)
//  //trajectory.yaw_com_theta=-radius;
//  //init_velocity_on_circle = (temp >=5.0? 5.0:temp);
//  //trajectory.top_speed[0]= (temp >=10.0? 10.0:temp);
//  }
//}
//
//
