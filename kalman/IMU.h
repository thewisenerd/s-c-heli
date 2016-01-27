#ifndef _IMU_H_
#define _IMU_H_
/************************/
/*for the new main board*/
/************************/

#define INDEX_CX 0//INDEX_P 1//
#define INDEX_CY 1//INDEX_Q 6//
#define INDEX_CZ 2//INDEX_R 8//
                                                                                                 
#define INDEX_AX 4//INDEX_AX 0//
#define INDEX_AY 5//INDEX_AY 5//
#define INDEX_AZ 6//INDEX_AZ 7// 
                                                                                                 
#define INDEX_GX 1//INDEX_CX 3//
#define INDEX_GY 2//INDEX_CY 2//
#define INDEX_GZ 3//INDEX_CZ 4//

#define MAX_ADC_SAMPLES 9//没有用到

#define bias_ax 32768  //23124//22235 //18621//22235 //31605               //
#define bias_ay 32768  //18779//22913 //22937 //39296
#define bias_az 32768  //24731//17439 //24634  //37797
                                                                                                 
#define accel_scale_x  0.0333*0.98//-0.00116046//-0.00112843 //0.00123505  //0.00112843 //-0.000715902f      //   Unit:  G/bit
#define accel_scale_y  -0.0333*0.98//0.00123645//0.00123482 //-0.00115821  //-0.00123482 // 0.000830547f
#define accel_scale_z  -0.0333*0.98//0.00117513//0.00118435 //-0.00118300  //-0.00118435  // -0.000853266f
                                                                                                 
#define gyro_scale_x  0.05*C_DEG2RAD   //0.07526*1.5*C_DEG2RAD  
#define gyro_scale_y  -0.05*C_DEG2RAD  //-0.07526*1.5*C_DEG2RAD
#define gyro_scale_z  -0.05*C_DEG2RAD   //-0.07526*1.2*C_DEG2RAD


/*#define accel_scale_x  0.02522*0.98//-0.00116046//-0.00112843 //0.00123505  //0.00112843 //-0.000715902f      //   Unit:  G/bit
#define accel_scale_y  -0.02522*0.98//0.00123645//0.00123482 //-0.00115821  //-0.00123482 // 0.000830547f
#define accel_scale_z  -0.02522*0.98//0.00117513//0.00118435 //-0.00118300  //-0.00118435  // -0.000853266f
                                                                                                 
#define gyro_scale_x  0.07326*C_DEG2RAD   //0.07526*1.5*C_DEG2RAD  
#define gyro_scale_y  -0.07326*C_DEG2RAD  //-0.07526*1.5*C_DEG2RAD
#define gyro_scale_z  -0.07326*1.1*C_DEG2RAD   //-0.07526*1.2*C_DEG2RAD*/

                                                                                                 
#define gyro_zero_x 32768
#define gyro_zero_y 32768
#define gyro_zero_z 32768
                                                                                                 
#define compass_bias_x  -245
#define compass_bias_y  -488
#define compass_bias_z  95

/*define on 2009.03
#define compass_scale_x  0.8340
#define compass_scale_y  0.1515
#define compass_scale_z  0.1724
*/


//  define on 2009.05.11
#define compass_scale_x  0.3125
#define compass_scale_y  0.2778
#define compass_scale_z  0.3571

extern double ahrs_pqr[ 3 ];
extern double raw_ahrs_pqr[ 3 ];//*********************10.10.28*********************************
extern double ahrs_accel[ 3 ];
extern double ahrs_compass[ 3 ]; 

extern double imu_accel[3];


extern void sensor_update(void);
extern void  imu_monitor();
extern void accel_filter(double filter_accel[3],const double imu_accel[3],unsigned int len);

#endif
