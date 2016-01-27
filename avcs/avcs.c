/******
2015-6-29 byM , 锁尾avcs

*/                                          
#include <includs.h>

#define _avcs_tuning_ //师父首次调的时候需要该定义!!!!!!!!!注意,调完参数之后,要把注释掉该定义,改回"固化参数模式"


/*******************************************************
//函数名:
//创建日期:2015/7/12
//最后修改日期:
//函数功能:
//使用方式:
*******************************************************/
void avcs(int *pwm)
{
#define init_gain_P 64          //锁尾模式的比例增益   >>>>>>解释:下面计算中3588(arm7PWM值)对应遥控器MODE中SERVO中的100(遥控器中SERVO值,不是END POINT值!) , 2088对应-100 , 所以((3588+2088)/2)对应0 , ((3588-2088)/2/100)对应SERVO中每变化一点pwm变化的量
                                  //师傅调好参数后,该参数的计算方法:gain_P=0.3240*(2846-pwm),其中pwm=((3588+2088)/2)-((3588-2088)/2)*(x/100),x为切锁尾开关在"一般模式"下遥控器MODE中SERVO中gain通道的量
                                  //简化算式:gain_P=0.3240*(8+7.5*x)
#define init_gain_I 8.8          //锁尾模式的积分增益
                                  //师傅调好参数后,该参数的计算方法:gain_I=0.0324*(pwm-2830),其中pwm=((3588+2088)/2)+((3588-2088)/2)*(y/100),y为切锁尾开关在"锁尾模式"下遥控器MODE中SERVO中gain通道的量
                                  //简化算式:gain_I=0.0324*(8+7.5*y)
#define init_gain_D 0.0;//3.86
#define output_limit 430.0        //output_limit是为了给尾舵限幅,让尾舵不堵转,对应servo_widths_temp的变化量最大值,取正反方向较小值
                                  //最简单的limit值测试办法,先把尾舵接在第5通道,gain通道,在遥控器MODE中END POINT中正负切换并调节gain通道的值,使达到不超出行程的最大值,然后切换至遥控器MODE中SERVO中,读第五通道的值并计算output_limit
                                  //简化算式:output_limit=7.5*z,z为正负行程中较小值
#define init_gain_coll_compensation 0.0//0.65                        //总距补偿用
  
/*固化锁尾参数模式*///新飞机先要进行"调节锁尾参数模式"
#ifndef _avcs_tuning_
  /*陀螺仪感度*/
  #define gain_P   init_gain_P  
  #define gain_I   init_gain_I      
  #define gain_D   init_gain_D//锁尾模式的微分增益   
  #define gain_coll_compensation  init_gain_coll_compensation  //总距补偿用
#endif 
  
/*调节锁尾参数模式*/
#ifdef _avcs_tuning_ 
  /*陀螺仪感度*/ 
  static double gain_P = init_gain_P;     //锁尾模式的比例增益
  static double gain_I = init_gain_I;      //锁尾模式的积分增益
  static double gain_D = init_gain_D;      //锁尾模式的微分增益
  static double gain_coll_compensation = init_gain_coll_compensation;      //总距补偿用
#endif
 
  
  
/*遥杆灵敏度*/
#define gain_remote_to_rate -0.007//摇杆量变成弧度制rad/s速度(300/180*3.14159)/((3588-2090)/2),对应摇杆满偏300°/s
/*锁尾陀螺仪输出方向*/
#define _Direction_ -       //反方向为"#define _Direction_ -"
/*锁尾通道输出的中位值*/
#define output_mid  2838           //对应初始化输出1.52ms的伺服脉冲,对应的servo_widths_temp的中点读数 
/*低通滤波器截止频率*/
#define alpha 1.0 //0.5945  //0.6768//0.8073//rate_error的低通滤波系数,计算方法:Dt = 0.033333;filt_hz=30;alpha = Dt/(Dt+1/(2*pi*filt_hz)),其中Dt是0.0333s,filt_hz是截至频率

  
/*用到的静态变量*/
static double manual_mid;               //摇杆偏移量,用于标记遥控中位值,开机时初始化
static double gyro_bias;                //角速度偏移量,用于校正角速度值,开机时初始化
static double _rate_error = 0.0;        //用于rate_error的低通滤波,记录旧的值
static double i_rate_error = 0.0;       //用于控制器,保存积分量的值
static int stage = 0;                   //用于锁尾初始化分阶段,在初始化阶段,确保遥控器的摇杆复位,不动直升机,因为该阶段会获取manual_mid和gyro_bias
static bool flag_breach_limit = 0;      //yaw_out超限标志位,0:没有超,1:超了
static int coll_zero;           //总距补偿用

static double tar_rate_pppppppp = 0.0;
static double tar_rate_ppppppp = 0.0;
static double tar_rate_pppppp = 0.0;
static double tar_rate_ppppp = 0.0;
static double tar_rate_pppp = 0.0;
static double tar_rate_ppp = 0.0;//用于积分控制器,p为previous,前一个的意思,舵机滞后越大,pppp越多
static double tar_rate_pp = 0.0;
static double tar_rate_p = 0.0;

double target_rate = 0.0;
double d_rate_error ;
double rate_error;
double yaw_out;


/*下面进行manual_mid与gyro_bias的获取*/
if (stage < 200)
  {
      stage++; 
      manual_mid += *pwm;
      gyro_bias += ahrs_pqr[2];//弧度制
      *pwm = output_mid;
  }
else if(stage < 201)
  { 
      stage++;  
      manual_mid = manual_mid/200.0;
      gyro_bias = gyro_bias/200.0;
      *pwm = output_mid;
      coll_zero = *(pwm+2);             //总距补偿用
  }
/*下面动作表示锁尾陀螺初始化已经完成*/
else if(stage < 216)
  {
      stage++;        
      *pwm = output_mid + output_limit;    
  }
else if(stage < 231)
  {
      stage++; 
      *pwm = output_mid - output_limit;
  }


/*下面是锁尾陀螺仪的控制算法,有锁尾模式和一般模式,一般模式用于调飞机的中位值*/
else 
  {
    /*期待角速度*/
    int target_pwm;
    target_pwm = *pwm - manual_mid;
     if(*(pwm+1) > 2838)//锁尾模式下偏航遥控量加入微弱摇杆控制死区，这样有助于降低由手产生的扰动
    {
    if(target_pwm > 23) target_pwm = target_pwm - 23;
    else if(target_pwm < -23) target_pwm = target_pwm + 23;
    else target_pwm = 0;
    }
    target_rate = target_pwm * gain_remote_to_rate;//摇杆信号转为角速度信号,单位rad/s 弧度制
    yaw_commend[1]= target_rate;//可删
     /*角速度误差*/
    rate_error = (target_rate - (ahrs_pqr[2] - gyro_bias));//转速误差,
    
    
    /*误差的低通滤波*/
    d_rate_error = (rate_error - _rate_error)*alpha;//d_rate_error复用,此处代表变化量(delta)
    rate_error = d_rate_error + _rate_error;
    d_rate_error = d_rate_error/0.03333;            //d_rate_error复用,此处代表变化率(derivative)
    _rate_error = rate_error;                       //_rate_error用于下一次滤波，滤掉不能要的高频，高频会使飞机尾震荡
    
    
#ifdef _avcs_tuning_   
//    if(*(pwm+4) <= 2838)
//    {
//      if(*(pwm+4) <= 2830)
//        gain_coll_compensation = 0.003165 * (2830 - *(pwm+4));//总距补偿用
//      else
//        gain_coll_compensation = 0.0;
//    }
 
    if(*(pwm+4) <= 2838)
    {
      if(*(pwm+4) <= 2830)
        gain_D = (2830 - *(pwm+4))/50.0;//微分用
      else
        gain_D = 0.0;
    }
#endif
    
    /*一般模式*/
    if(*(pwm+1) <= 2838)//非锁尾模式/非avcs
      {
#ifdef _avcs_tuning_
        gain_P = 0.3240 * (2846 - *(pwm+1));//如果100%感度都不够用,修改比例
#endif
        in_loop_yaw_PID[0] = gain_P;//可删
        in_loop_yaw_PID[1] = 0;//可删
        in_loop_yaw_PID[2] = gain_D;//gain_coll_compensation;//可删//总距补偿用
        
        i_rate_error = 0.0;
        flag_breach_limit = 0;
        
        /*P控制器*/
        yaw_out = rate_error * gain_P;
//        yaw_out += gain_coll_compensation * (coll_zero - *(pwm+2));        //总距补偿用
        
        if(yaw_out > output_limit) 
        {
          yaw_out = output_limit;
        }
        else if(yaw_out < -output_limit)
        {
          yaw_out = -output_limit;
        } 
        *pwm = output_mid + _Direction_(yaw_out);//_Direction_为输出方向的定义
      }
    
    /*锁尾模式 avcs*/
    else
      {
#ifdef _avcs_tuning_
        gain_I = 0.0324 * (*(pwm+1) - 2830);//如果100%感度都不够用,修改比例
#endif
        in_loop_yaw_PID[0] = gain_P;//可删
        in_loop_yaw_PID[1] = gain_I;//可删
        in_loop_yaw_PID[2] = gain_D;//gain_coll_compensation;//可删//总距补偿用
        if(((!flag_breach_limit) || (i_rate_error>0&&rate_error<0) || (i_rate_error<0&&rate_error>0)))//没有积分饱和,或者退出积分饱和
        {
//        if (abs(ahrs_pqr[2] - gyro_bias)<0.5)
        i_rate_error += (tar_rate_pppp - (ahrs_pqr[2] - gyro_bias));
        }
        /*PI控制器,PID都有,主要是PI参数作用,D增益为0不起主要作用*/
        if(rate_error <0.0)
          yaw_out = rate_error*gain_P*2.0*(1.0+abs(rate_error/6.0)) + i_rate_error*gain_I + d_rate_error*gain_D;
        else
          yaw_out = rate_error*gain_P*(1.0+abs(rate_error/6.0)) + i_rate_error*gain_I + d_rate_error*gain_D;
//        yaw_out += gain_coll_compensation * (coll_zero - *(pwm+2));        //总距补偿用
        
        if(yaw_out > output_limit) 
        {
          flag_breach_limit = 1;
          yaw_out = output_limit;
        }
        else if(yaw_out < -output_limit)
        {
          flag_breach_limit = 1;
          yaw_out = -output_limit;
        } 
        else
        {
          flag_breach_limit = 0;  
        }
        *pwm = output_mid + _Direction_(yaw_out);//_Direction_为输出方向的定义
      }
      tar_rate_pppppppp =   tar_rate_ppppppp;
      tar_rate_ppppppp =    tar_rate_pppppp;
      tar_rate_pppppp =     tar_rate_ppppp;
      tar_rate_ppppp =      tar_rate_pppp;
      tar_rate_pppp =       tar_rate_ppp;
      tar_rate_ppp =        tar_rate_pp;
      tar_rate_pp =         tar_rate_p;
      tar_rate_p =          target_rate;
  } 
}
