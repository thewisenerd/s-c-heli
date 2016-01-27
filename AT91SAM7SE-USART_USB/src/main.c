//*--------------------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*--------------------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*--------------------------------------------------------------------------------------
//* File Name           : main file
//* Object              :
//* Translator          :
//* 1.0 02/Mar/05 JPP	: Creation
//*--------------------------------------------------------------------------------------

#include <includs.h>
extern int first_entry_circle;
int round_count;

extern struct bmp085_t  bmp085;

extern void Usart_init ( void );
extern void AT91F_US_Put( char *buffer); // \arg pointer to a string ending by \0
extern void AT91F_DBGU_Init(void);
extern void AT91F_DBGU_Put(char *buffer);


struct  Target_Command     target_command;
struct  Target_Queue       target_queue;
struct  Position           position;
struct  Trajectory         trajectory;

double theta[ 3 ];
double angle_offset[3] = {3.0,1.0,0.0};//roll pitch yaw
int first_receive_target;
int manual_to_hover;
enum  Heli_mod
{	
    Off,
    Manual,
    Begin_Auto,
    Auto
} heli_mod;                     //飞机飞行模式：自控，手控..
/*
enum  Trajectory_mod
{
	Hover,
	Point_to_point,
	Circle,
	Mixture,
	Test
}trajectory_mod;
*/
//---------------------------------------------------------------------------------//     

//*--------------------------------------------------------------------------------------
//* Function Name       : main
//* Object              :
//*--------------------------------------------------------------------------------------
int main ( void )
{
   
    char ahrs_init_count=0;
    char ahrs_init_flag=1;
    heli_mod = Manual;
      
    AT91C_BASE_RSTC->RSTC_RMR = AT91C_RSTC_URSTEN | (0x4<<8) | (unsigned int)(0xA5<<24);
//----------------------systerm initial--------------------------------------//  
    init_translator();
    //PIOB0_init(); //去掉compass外部中断10.30 syc
    AT91F_SpiInit();
    Usart1_init(); 
    CDCLKRESET;//PA1=0;
    Usart_init();
    AT91F_DBGU_Init();
    //FIQ_init();//10.30 syc
    CDCLKSET;//PA1=1;
    timer_init();
    twi_init();
    scp1000_init();

    SSSET;//PA31没用到，这个没用
    //COMPASSRESET; //syc compass读取方式改变不需要管脚中断
   // CDCLKRESET; 
     reset_controller();
     target_command.reset();
     trajectory.position_t = & position;
     trajectory.target_queue_t = &target_queue;
     trajectory.reset();
    
//-----------------------------------------------------------------------------//
  
//-------------------kalman initial--------------------------------------------// 
while(1)
{
    if(READY_30HZ==1)
      {
        READY_30HZ=0;
      // AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA11;
       AT91F_TC_InterruptEnable(AT91C_BASE_TC0,AT91C_TC_CPAS);
       AT91C_BASE_TC0->TC_RA =312+AT91C_BASE_TC0 -> TC_CV; 
       HDG=1;
      if(ahrs_init_flag==1&&raw_imu_update==true&&ahrs_init_count<20)
           {
                 //IMU_OUTPUT();
                  ahrs_init_count++;
                  raw_imu_update=false;
                  if(ahrs_init_count==20)
                   ahrs_init_flag=0;
                  sensor_update();
           }
      else if(ahrs_init_count >= 20)
          {
            raw_imu_update=false;
            timer0_num=0;
            HDG=2;
            break;
          }
      }//end if(READY_30HZ==1)
}//while(1)
ahrs_init();
  
//-----------------------------------------------------------------------------//

for(;;)//main loop
{   
//---------- syc在定时器T0里面 read IMU(SPI)  bmp(I2C) compass(I2C) （与师兄区别最大地方）--------//
        
    
     if(HDG==2 && raw_imu_update==false)
     {
       HDG=1;   
       AT91C_BASE_TC0->TC_RA = 312+AT91C_BASE_TC0 -> TC_CV;
       if(timer0_num == 0)
       AT91F_TC_InterruptEnable(AT91C_BASE_TC0,AT91C_TC_CPAS); 
     }
     
       gps_update();

//*-----------------------------------------------------------------------------*//
      if(READY_30HZ==1)
      { 
         READY_30HZ=0;
         imu_monitor();
         gps_monitor();
                
         if(raw_imu_update==true)//raw_imu_update  in timer.c
         {
          
             raw_imu_update = false;
             sensor_update();
             accel_filter(ahrs_accel,imu_accel,15); 
			 ahrs_accel[2] = imu_accel[2];
             ahrs_update();     //kalman filter   
             //---------syc气压计更新-------------// 
             if(bmp_update_flag==2)
             {
                bmp_update_flag=1;
                scp1000_update();
                static double last_bmp_height = 0.0;
                if(bmp_height - last_bmp_height > 5.0 || bmp_height - last_bmp_height < -5.0) bmp_height = last_bmp_height;
                last_bmp_height = bmp_height;
             }
              
            //-----PID parameter update--------------//
               //-----PID parameter update--------------//
            if(groundtoarm_flag==1)
             {
               deal_groundtoarm_date_auto();//in pcm.c
               groundtoarm_flag = 0;
             }


            //-----------target update ----------------//
             if ( target_receive_flag == true )
             {
		if (false == target_queue.push_target(&target_command))   
                  	trajectory.brake_down_step();

                first_receive_target ++;
                target_receive_flag = false;
             }
           //-----------control step------------------//
             if( (control_mode==1) || (control_mode==0) ) //manual 
             {   
                 target_command.reset();
                 trajectory.reset();
                 first_receive_target  = 0;//
                 manual_to_hover = 0;
                 heli_mod = Manual ;			 
//----------------------路径规划模式选择---------------------------//			 
		 trajectory.trajectory_mod = Point_to_point;
//-----------------------------------------------------------------//		          			  
                 pwm_coll_mid = manual_servo_widths[1];//syc 4.16 高度项基值取手控值
                 temp_pwm_coll_mid = manual_servo_widths[1];//前飞高度补偿用 
             }
        
            if( control_mode==3 && last_control_mode==1 )
                 heli_mod = Begin_Auto;
             
             if( heli_mod == Begin_Auto )
             {
                 heli_mod = Auto;
                 trajectory.heli_step = 1;
                 first_entry_circle = 1;
                 set_init_state();		 
             }
             if (heli_mod == Auto)
             {
                 manual_to_hover ++;
                 if(manual_to_hover == 100)
                 {
                    trajectory.hover_xyz[0] = position.current_xyz[0] ;
                    trajectory.hover_xyz[1] = position.current_xyz[1] ;
                    trajectory.hover_xyz[2] = position.current_xyz[2] ;
                    manual_to_hover = 200;
                 }
                 if(manual_to_hover >= 10000)
                    manual_to_hover = 200;
                 //if(manual_to_hover>= 200)
                    trajectory.step(trajectory.trajectory_mod);
		//hover点在自控100次后变 不上传目标点 飞机位置是相对于第一次时飞机悬停点
		//上传目标点时 是相对于100次后的悬停点
             }
            //ahrs_adjust(ahrs_theta,angle_offset);//姿态矫正
//            input_yaw = yaw_convert(ahrs_theta[2],position.heading);  //heading 是目标航行，即yaw通道输入值
//	    xyz_transform(input_xyz, &position, &trajectory);//update input_xyz  //根据当前坐标和当前目标计算得到机体坐标
                                     
//***************************question*************************************************//
            //ahrs_theta[0] = ahrs_theta[0] - (double)amplify/572.9578;
//************************************************************************************//			
	    theta[2] = input_yaw;       //反馈值
            theta[1] = ahrs_theta[1];
            theta[0] = ahrs_theta[0];     
             
             if(round_count > 300)
             if(round_count > 10000)
                round_count = 500;   
                
         }//end if(raw_imu_update == 1)  
        round_count ++;
        servo_update_auto();
        output2ground();     
     }//end if(READY_30HZ==1)
     
  }//end for(;;) ,end mainloop

}//end all

