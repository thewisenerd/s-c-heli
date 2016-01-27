#ifndef __TIMER_H__
#define __TIMER_H__

#define TC0_INTERRUPT_LEVEL 4
#define TC1_INTERRUPT_LEVEL 3
#define TC2_INTERRUPT_LEVEL 7  //highest

extern char HDG;
extern char imu_num;
extern char READY_30HZ;

extern char DMP_GPS;


extern bool raw_imu_update;
extern volatile  char imu_update_flag;

extern  void timer_init(void);


#endif



