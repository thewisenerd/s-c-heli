#include<includs.h>

double ahrs_pqr[3];
double raw_ahrs_pqr[ 3 ];//***********************10.10.28**********************************
double ahrs_accel[3];
double ahrs_compass[3];
double imu_accel[3];
//----------------------LQR 角速度滤波增加参数-------------------//
#define TIMES    6

double filter_pqr[3];
static double pqr_window[3][TIMES] = {0.0};
//-------------------------------------------------------------//

__inline  void IMU_OUTPUT(void)
{
     AT91F_US_Put("$GPADC");
	for( int i=0; i<8; i++ )
        {	
		//puts("0x");
		while (!AT91F_US_TxReady(AT91C_BASE_US0));
		  AT91F_US_PutChar(AT91C_BASE_US0, ',');
		put_uint16_t( imuframe[i] );
	}
        AT91F_US_Put("\r\n");
        AT91F_US_Put("$GPHDG");
	for( int i=0; i<3; i++ )
        {	
		//puts("0x");
		while (!AT91F_US_TxReady(AT91C_BASE_US0));
		  AT91F_US_PutChar(AT91C_BASE_US0, ',');
		put_uint16_t( pni_value[i] );
	}
        AT91F_US_Put("\r\n");
}


void accel_filter(double filter_accel[3],const double imu_accel[3],unsigned int len)
{
     static double filter_container[30][3];
     double sum[3] = {0,0,0};
     static unsigned int length = 1;
     for(unsigned int i = len-1;i>0;i--)
       for(unsigned int j=0;j<3;j++)
         filter_container[i][j] = filter_container[i-1][j];
     
     for(unsigned int j=0;j<3;j++)
       filter_container[0][j] = imu_accel[j];
     
     for(unsigned int i=0;i<length;i++)
       for(unsigned int j=0;j<3;j++)
         sum[j] += filter_container[i][j];
     
     for(unsigned int j=0;j<3;j++)
       filter_accel[j] = sum[j]/length;
     
     if(length < len)
       length++;
     else length = len;
     
}

/*--------------------------------------------------------------------------------------*/
        
        
/*--------------------------------------------------------------------------------------*/  
__inline void imu_update( void )
{
        //---------------------------------
        // 检查imuframe的标志位
       double sum_pqr[3] = {0.0};

       char temp1 = 0;
       char temp2 = 0;
       for( char i =1; i !=7; ++i)
       if((imuframe[i] & 0x4000) != 0)
               temp1 ++;
       if(imuframe[7] != 0)
               temp2 ++;
       if(temp1 >= 1)
       {
           imu_update_flag = 3;//溢出    
           return ;
       }
       if(temp2 >= 1)
       {
           imu_update_flag = 4;//IMU标志位非零，出错    
           return ;
       }

        int rate ;
	rate = imuframe[ INDEX_GX ]&0x3FFF;	
        if(rate>0x1fff)rate=rate-0x4000;
	raw_ahrs_pqr[ 0 ] = -rate * gyro_scale_x  ;
		
    rate = imuframe[ INDEX_GY ]&0x3FFF;
	//rate = imuframe[ INDEX_GY] - gyro_zero_y;
        if(rate>0x1fff)rate=rate-0x4000;
	raw_ahrs_pqr[ 1 ] = -rate * gyro_scale_y  ;
	
        rate = imuframe[ INDEX_GZ ]&0x3FFF;
	//rate = imuframe[ INDEX_GZ ] - gyro_zero_z;
        if(rate>0x1fff)rate=rate-0x4000;
	raw_ahrs_pqr[ 2 ] = rate * gyro_scale_z ;
	
        rate = imuframe[ INDEX_AX ]&0x3FFF;
	//rate = imuframe[ INDEX_AX ] - bias_ax;
        if(rate>0x1fff)rate=rate-0x4000;
        	imu_accel[ 0 ] = -rate * accel_scale_x;
  
        rate = imuframe[ INDEX_AY ]&0x3FFF;
	//rate = imuframe[ INDEX_AY ] - bias_ay;
        if(rate>0x1fff)rate=rate-0x4000;
	imu_accel[ 1 ] = -rate * accel_scale_y;
	rate = imuframe[ INDEX_AZ ]&0x3FFF;
        
	//rate = imuframe[ INDEX_AZ ] - bias_az;
        if(rate>0x1fff)rate=rate-0x4000;
	imu_accel[ 2 ] = rate * accel_scale_z;
        
       for( char i=0 ; i<3 ; i++ )
	{
    	 imu_accel[i] = limit(imu_accel[i],-40.0,40.0);
	}   
     // ***********增加对角速度的滑动窗口滤波2010.10.19*********************************
       for( char i=0 ; i<3 ; i++ )
	{
	    ahrs_pqr[i]=raw_ahrs_pqr[i];
	}
	
	for(char j=0; j<3; j++)
	{
	
		for(char i=1; i<TIMES; i++) 
    	{
			sum_pqr[j] += pqr_window[j][i];
        	pqr_window[j][i-1] = pqr_window[j][i];
    	}
	 
		pqr_window[j][TIMES-1] = raw_ahrs_pqr[j];
		sum_pqr[j] 			  += raw_ahrs_pqr[j];
		filter_pqr[j] 		   = sum_pqr[j] / TIMES;
	}	
    //----------------SYC去掉角速度均值滤波------------------------//  
    /* for( char i=0 ; i<3 ; i++ )
	{
	 raw_ahrs_pqr[i] = limit(raw_ahrs_pqr[i],-5.23,5.23);
	}   
       
    static double p_window[10],q_window[10], r_window[10];
    double sum_p = 0, sum_q = 0, sum_r = 0;
    double filter_pqr[3]={0.0,0.0,0.0};
    static unsigned int length_pqr = 1;
       //----------------------------
	for(char i=1;i<5;i++) 
    {
    	sum_p += p_window[i];
        p_window[i-1] = p_window[i];
    }
	 
       p_window[5-1]=raw_ahrs_pqr[0];
       sum_p += raw_ahrs_pqr[0];
      //--------------------------------
       for(char i=1;i<5;i++) 
    {
    	sum_q += q_window[i];
        q_window[i-1] = q_window[i];
    }
	 
       q_window[5-1]=raw_ahrs_pqr[1];
       sum_q += raw_ahrs_pqr[1];
      //-----------------------------------
       for(char i=1;i<5;i++) 
    {
    	sum_r += r_window[i];
        r_window[i-1] = r_window[i];
    }
	 
       r_window[5-1]=raw_ahrs_pqr[2];
       sum_r += raw_ahrs_pqr[2];
      //----------------------------------
       filter_pqr[0] = sum_p/length_pqr;
       filter_pqr[1] = sum_q/length_pqr;
       filter_pqr[2] = sum_r/length_pqr;

	if(length_pqr >= 5)//for length_pqr ,max = N
		length_pqr = 5;
	else length_pqr++;
       
       for( char i=0 ; i<3 ; i++ )
	{
	 ahrs_pqr[i] = limit(filter_pqr[i],-2.0,2.0);
	}   
 */
} 

/*--------------------------------------------------------------------------------------*/
__inline void compass_update( void )
{
    signed int rate;  
	if( pni_value[INDEX_CX] > 32767 )
	    pni_value[INDEX_CX] = pni_value[INDEX_CX] - 65536;
    if( pni_value[INDEX_CY] > 32767 )
	    pni_value[INDEX_CY] = pni_value[INDEX_CY] - 65536;
    if( pni_value[INDEX_CZ] > 32767 )
	    pni_value[INDEX_CZ] = pni_value[INDEX_CZ] - 65536;
   
        rate = pni_value[ INDEX_CX ] - compass_bias_x;
	ahrs_compass[ 0 ] = rate;
        
	rate = pni_value[ INDEX_CY ] - compass_bias_y;
	ahrs_compass[ 1 ] = rate;
        
	rate = pni_value[ INDEX_CZ] - compass_bias_z;
	ahrs_compass[ 2 ] = -rate ;      
}

 void sensor_update( void )
{
	imu_update();
	//printf("accel_x=%f\taccel_y=%f\taccel_z=%f\n",ahrs_accel[0],ahrs_accel[1],ahrs_accel[2]);
	compass_update();
}


/********************************************************
* 函数说明： *检查imuframe是否有连续一样的值,
              防止因为imu或者compass出错,这两者的读取进入死锁.
* 入口参数： * 无 
* 返回参数： * 无
* 输入数据示例：  
* 最后修改： * 陈勇，2009-03-26
* 备    注： * 无
********************************************************/ 
void imu_monitor( )
{
         static long last_imuframe[7] = {0,0,0,0,0,0,0};
         static char  imu_error = 0;    
         char imu_reset_flag = 0,     //重新读取compass和imu的标志
              temp = 0;
        //--------检查是否收到连续重复imuframe---------------
        for( char i= 1 ; i!= 7 ;++i){
            if(last_imuframe[i] == imuframe[i])
                  temp ++;
            last_imuframe[i] = imuframe[i];  
        }
        
        if(temp == 6)
            imu_error ++;
        else 
           imu_error = 0 ; 
        //---- ----------------------------------
        if(imu_error >= 4) {    //4组imuframe保持不变,认为出错
           imu_reset_flag = 1;
           imu_update_flag = 2;     
        }

        //----------reset compass&imu---------
        if(imu_reset_flag == 1) 
        {
          //-------compass reset-------- 
             HDG = 2;
             //read_compass=1;
             compass_update_flag=false;
             timer0_num=0;
             AT91F_TC_InterruptDisable(AT91C_BASE_TC0,AT91C_TC_CPAS);
             
    //----------imu reset-------
              imu_num = 0;
              raw_imu_update = false;
     //----------重启后reset 判断标志-----
              imu_reset_flag = 0;
              imu_error = 0;
        }
}
