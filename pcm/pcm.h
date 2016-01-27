#ifndef _PCM_H_
#define _PCM_H_

#define CDREST      AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA0;
#define CDSET       AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA0;
#define CDCLKSET    AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA1;
#define CDCLKRESET  AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA1;

#define srevo_period 33658 //27000 by syc 2010 10.31
#define UPDATE_TARGET_SIZE 1    //地面站上传目标数组大小 ， 现在改为1个
#define MAX_TARGET_SIZE 20    // ARM系统存贮目标点最大数目

extern  int manual_servo_widths[3];
extern  int servo_widths_temp[10];
extern  int last_servo_widths_temp[7];
extern  int lim_servo_widths_temp[7];
extern  int servo_widths[ 10 ];
extern char control_mode;
extern char pcm_update_flag;
extern char pcm_rece_flag;
extern bool target_receive_flag;
//extern double target[UPDATE_TARGET_SIZE][4];
extern unsigned char  pcm_line[PCM_NUM-4];
//extern unsigned char pcm_err_char[PCM_NUM] ;

extern  int servo_widths_mid[4];
extern  int pwm_roll_mid,pwm_pitch_mid,pwm_yaw_mid,pwm_coll_mid;
extern  int temp_pwm_coll_mid;
extern  int amplify;
extern char last_control_mode;
extern bool receive_index;
extern double desired_x_rate;
extern double desired_y_rate;
extern double desired_d_rate;

void servo_update(void );

//------------------小直升飞机自主/2014/5/29--------------------
void servo_update_auto(void);
void deal_groundtoarm_date11(void);
//--------------------------------------------------------------

void init_translator(void);
void deal_groundtoarm_date_auto(void);
//bool target_monitor(void);
inline void uncharcpy(unsigned char* des,unsigned char* src, int num)
{
  for(int i=0;i<num;i++)
  {
    *(des+i)=*(src+i);
  }
}
#endif
