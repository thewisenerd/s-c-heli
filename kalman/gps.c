#include <includs.h>

#define FILTER_LEN 3

     
int  gps_update_flag ;
double input_xyz[3];
double input_yaw=0;
double input_uvw[3];
char  zero_flag = 0;
//double current_vel[3]; 
double last_raw_current_xyz[3] = {0};
double last_raw_current_uvw[3] = {0};
char abnormal_limit_flag = 0;
double zero_position0= 23.0419933*C_DEG2RAD,//23.1565516*C_DEG2RAD  //0.404671487796//
       zero_position1= 113.40246*C_DEG2RAD,//113.341556*C_DEG2RAD
       zero_position2= 38.0;

	double gps_gga_time;
	double gps_gga_latitude;
	double gps_gga_longitude;
	int gps_gga_quality;
	int gps_gga_num_sats;
        int last_gps_gga_num_sats;
	double gps_gga_hdop;
	double gps_gga_altitude;
	double gps_gga_wgs_alt;

	double gps_vtg_course;
	double gps_vtg_speed;


double f,e,zero_position_ECEF[3];
double Re2t[3][3];
/*---------------------------------------------------------------------------------*/     
inline  double sqr(double	x)
{
	return x * x;
}         

void ned2bf2(const double v_in[3],
              const double theta,
              double v_out[3])
{
  static double     cos_theta   =   1.0;
  static  double    sin_theta   =   0.0;
  static  double    last_theta  =   0.0;
  if( last_theta != theta)
  {
      last_theta   =   theta;
      cos_theta    =   cos(theta) ;
      sin_theta    =   sin(theta) ;
  }
  
  v_out[0]  = v_in[0] * cos_theta + v_in[1] * sin_theta ;
  v_out[1]  = v_in[1] * cos_theta - v_in[0] * sin_theta ;     
  v_out[2]  = v_in[2] ;
}


/*---------------------------------------------------------------------------------*/
int strntol(
	char *			line,
	size_t			len,
	char **			line_out,
	int			base
)
{
	char			buf[ 8 ];
	if( len > 8 )
		len = 8;

	strncpy( buf, line, len );

	buf[len] = '\0';

	if( line_out )
		*line_out = line + len;

	return strtol( buf, 0, base );
}
/*---------------------------------------------------------------------------------*/
/********************************************************
* ����˵���� * gps���ݳ�ʼ��
               ȡ��10����Ч��GPS��Ϣ����ʼ��NEDԭ��ľ�γ�Ⱥͺ���
               ���������ʼNEDԭ�㣬���ECEFת����NED������ת������Re2t
* ��ڲ����� * �� 
* ���ز����� * ��
* ��������ʾ����  
* ����޸ģ� * ���£�2009-03-26
* ��    ע�� * ��
********************************************************/ 
void gps_date_init(void)
{
    static unsigned char first_entry = 1 ;
    if( first_entry == 100)/////100
    {
         zero_position0 = gps_gga_latitude*C_DEG2RAD;
         zero_position1 = gps_gga_longitude*C_DEG2RAD;
         zero_position2 = gps_gga_altitude+gps_gga_wgs_alt;//ע���Ǹü����Ǽ�????????????????
         
         f = ((C_WGS84_a - C_WGS84_b) / C_WGS84_a);
         e = sqrt( 2*f - f*f );
         double N = C_WGS84_a/sqrt( 1 - e*e*sqr(sin(zero_position0)));
         zero_position_ECEF[0]=(N + zero_position2) * cos(zero_position0) * cos(zero_position1);
         zero_position_ECEF[1]=(N + zero_position2) * cos(zero_position0) * sin(zero_position1);
         zero_position_ECEF[2]=(N*(1-e*e) + zero_position2) * sin(zero_position0);
		 //---------------------------
    
    double		clat = cos(zero_position0);
    double		clon = cos(zero_position1);
    double		slat = sin(zero_position0);
    double		slon = sin(zero_position1);

	Re2t[0][0] = -slat*clon;
	Re2t[0][1] = -slat*slon;
	Re2t[0][2] =  clat;

	Re2t[1][0] = -slon;
	Re2t[1][1] =  clon;
	Re2t[1][2] =  0.0;

	Re2t[2][0] = -clat*clon;
	Re2t[2][1] = -clat*slon;
	Re2t[2][2] = -slat;//Ϊʲôֻ������ֵһ�Σ�����syc
    zero_flag = 1;  //ȷ��λ�����֮�� ����gps��������
   }
   
    first_entry ++ ;
    if(first_entry >=200)
	{
		first_entry = 150;
	}

}

/*-------------------------------�����ECEF--------------------------------------------*/     
inline void llh2ECEF( double *llh)
{
	double value[3];
	double	N = C_WGS84_a/sqrt( 1 - e*e*sqr(sin(llh[0])));

	value[0]= (N + llh[2]) * cos(llh[0]) * cos(llh[1]);
        value[1]= (N + llh[2]) * cos(llh[0]) * sin(llh[1]);
	value[2]= (N*(1-e*e) + llh[2]) * sin(llh[0]);
        llh[0] = value[0];
        llh[1] = value[1];
        llh[2] = value[2];
	return ;
}
/*-------------------------------�����NED--------------------------------------------*/
inline void ECEF2Tangent(double *ECFF)
{
	double value[3];
        
        value[0] = Re2t[0][0]*ECFF[0]+Re2t[0][1]*ECFF[1]+Re2t[0][2]*ECFF[2];
        value[1] = Re2t[1][0]*ECFF[0]+Re2t[1][1]*ECFF[1]+Re2t[1][2]*ECFF[2];
        value[2] = Re2t[2][0]*ECFF[0]+Re2t[2][1]*ECFF[1]+Re2t[2][2]*ECFF[2];
        
        ECFF[0] = value[0];
        ECFF[1] = value[1];
        ECFF[2] = value[2];
        //value[0]=(Re2t*ECEF)[0];value[1]=(Re2t*ECEF)[1];value[2]=(Re2t*ECEF)[2];
	return ;
}          
          
/*---------------------------------------------------------------------------------*/

//--------------------------------------------------------------
/********************************************************
* ����˵���� * ��GPS�����е�GGA���ֽ�������
               ȫ�ֱ���gps_gga_time��gps_gga_latitude��gps_gga_longitude
               gps_gga_quality��gps_gga_num_sats��gps_gga_hdop�� gps_gga_altitude
               gps_gga_wgs_alt ����
* ��ڲ����� * �� 
* ���ز����� * ��
* ��������ʾ���� * $GPGGA,102134.00,1306.9847,N,11402.2986,W,2,5,1.0,50.3,M,-16.27,M,,*61\r\n  
* ����޸ģ� * ���£�2009-03-26
* ��    ע�� * ��
********************************************************/ 
void gpgga_update(void)
{
         int lat = 0 ;
         double lat_min = 0.0;
         double temp;
         char * line ;
         line = index(gga_save_line,',');
         
         line++; 
        gps_gga_time =  strtod(line,&line);
        int hour = (int)(gps_gga_time/10000.0);
        int min = (int)((gps_gga_time-hour*10000)/100.0);
        float sec   = gps_gga_time-hour*10000-min*100;
         gps_gga_time =  (min * 60 + sec)*100;  //��λ��0.01s(10ms),�������λ����֪��GPS��������
        
        //------γ��---- 
        line++;
        gps_gga_latitude     = strtod( line, &line);
        lat = (int)(gps_gga_latitude/100.0);
        lat_min = gps_gga_latitude - lat*100.0;
        gps_gga_latitude = lat + lat_min / 60.0;
        
        
        
       
	line++;
	if( *line == 'S' )      // Find the sign for the lattitude.  South is negative
	  gps_gga_latitude *= -1;
	line = index(line,',');               // Skip ,S,
        
        //------����---- 
        line++;
        gps_gga_longitude = strtod( line, &line);
	lat= (int)(gps_gga_longitude/100.0);
	lat_min= gps_gga_longitude - lat*100 ;
	gps_gga_longitude = lat + lat_min / 60.0;
	
        line++;
	if( *line == 'W' )
	   gps_gga_longitude *= -1;
	line = index(line,',');
	
        //----���״̬
        line++;
	gps_gga_quality = strtol( line, &line, 10 );
	line++;		// Skip ,
        //-------����
	gps_gga_num_sats = strtol( line, &line, 10 );
	line++;		// Skip ,
        
	gps_gga_hdop = strtod( line, &line );
	line++;		// Skip ,
//----------------����ȥ����ֵ--------------//	
	temp = gps_gga_altitude;
	gps_gga_altitude = strtod( line, &line );  //����
	if (temp > 0)
	{
		if ((gps_gga_altitude - temp) > 20 || (gps_gga_altitude - temp) < -20)
		{
			gps_gga_altitude = temp;
		}
	}
//----------------------------------------//
	line += 3;	// Skip ,M,
 

     //   line++;   �����˺���߶Ȳ�ķ�����
	gps_gga_wgs_alt = strtod( line, &line );  //���ˮƽ���������Բ��߶Ȳ�
	line += 3;	// Skip ,M,
        return;
}

/*---------------------------------------------------------------------------------*/     
          
          
/*---------------------------------------------------------------------------------*/
void gpvtg_update(void)
{
  char * line=NULL ; 
  line = index(vtg_save_line,',');
  if(!line)
      return ;
  line++;//skip ,
  gps_vtg_course = strtod( line, &line );
  line = index(line,'N');
  if(!line)
      return;
  line += 2;
  gps_vtg_speed = strtod( line, &line ) * 1000.0 / 3600.0;//��λm/s
  return;
}

  
          
/*---------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------*/
void gps2xyz(double *llh)
{

  llh[0] = gps_gga_latitude*C_DEG2RAD;
  llh[1] = gps_gga_longitude*C_DEG2RAD;
  llh[2] = gps_gga_altitude+gps_gga_wgs_alt;//ע���Ǹü����Ǽ�
  llh2ECEF( llh );

  llh[0] = llh[0] - zero_position_ECEF[0];
  llh[1] = llh[1] - zero_position_ECEF[1];
  llh[2] = llh[2] - zero_position_ECEF[2];
  
  ECEF2Tangent(llh);
  
  /*static double window_z[FILTER_LEN] = {0.0,0.0} ;
  double sum_z =0 ;     
  for( char i =FILTER_LEN-1 ; i != 0 ; --i )
  {
      window_z[i] = window_z[i-1];
      sum_z += window_z[i];
   }
  
   window_z[0] = gps_gga_altitude ;//����ΪʲôҪ�ú���ֵ?
   sum_z += window_z[0] ;
   llh[2] = sum_z/FILTER_LEN ;   // ���μ�һ��ƽ��ֵ�˲� */
}

/*---------------------------------------------------------------------------------*/     

 void gps2uvw(double *vel_NED)
{
    static double last_Z = 0.0;               
    double vel_z_current =0.0 ;
     
    vel_NED[0] = gps_vtg_speed * cos( gps_vtg_course * C_DEG2RAD );
    vel_NED[1]  = gps_vtg_speed * sin( gps_vtg_course * C_DEG2RAD );//������gps�ĺ�����
    if(last_gps_gga_num_sats != gps_gga_num_sats)
    {
       last_gps_gga_num_sats = gps_gga_num_sats;
       last_Z = position.current_xyz[2];
       return ;
    }
    vel_z_current = (position.current_xyz[2] - last_Z)*GPS_FREQ ;
    
    float sum_vel_z =0.0; 
    static float window_vel_z[FILTER_LEN] = {0.0,0.0} ;  // z���ٶȼ�2��ƽ��ֵ�˲�
    
    for( char i =FILTER_LEN-1 ; i != 0 ; --i )
    {
        window_vel_z[i] = window_vel_z[i-1];
        sum_vel_z += window_vel_z[i];
     }
    
     window_vel_z[0] = vel_z_current ;
     sum_vel_z += window_vel_z[0] ;
      vel_NED[2] = sum_vel_z/FILTER_LEN ;        
 
    last_Z = position.current_xyz[2];
     
    return ;   
  
}

void xyz_transform(double input_xyz[3], Position * position_t,Trajectory * trajectory_t)
{   
	//-------------------------ת������-------------------------//
    /*Znb[0][0] =  cos(ahrs_theta[1])*cos(ahrs_theta[2]);
    Znb[0][1] =  cos(ahrs_theta[1])*sin(ahrs_theta[2]);
    Znb[0][2] = -sin(ahrs_theta[1]);
    
    Znb[1][0] = -cos(ahrs_theta[0])*sin(ahrs_theta[2]) + sin(ahrs_theta[0])*sin(ahrs_theta[1])*cos(ahrs_theta[2]);
    Znb[1][1] =  cos(ahrs_theta[0])*cos(ahrs_theta[2]) + sin(ahrs_theta[0])*sin(ahrs_theta[1])*sin(ahrs_theta[2]);
    Znb[1][2] =  sin(ahrs_theta[0])*cos(ahrs_theta[1]);
    
    Znb[2][0] =  sin(ahrs_theta[0])*sin(ahrs_theta[2]) + cos(ahrs_theta[0])*sin(ahrs_theta[1])*cos(ahrs_theta[2]);
    Znb[2][1] =  cos(ahrs_theta[0])*sin(ahrs_theta[1])*sin(ahrs_theta[2]) - sin(ahrs_theta[0])*cos(ahrs_theta[2]);
    Znb[2][2] =  cos(ahrs_theta[0])*cos(ahrs_theta[1]); 
	*/
    double offset_xyz[3] = {0},sy,cy;
    offset_xyz[0]=position_t->raw_current_xyz[0]-position_t->current_target[0];
    offset_xyz[1]=position_t->raw_current_xyz[1]-position_t->current_target[1];
    offset_xyz[2]=position_t->raw_current_xyz[2]-position_t->current_target[2];
    sy = sin(input_yaw);
    cy = cos(input_yaw);
    
    input_xyz[0] = (offset_xyz[0])*cy  + (offset_xyz[1])*sy ; 		   //��gpsÿ�θ���ʱ�Ѿ�����	
    input_xyz[1] = (offset_xyz[0])*-sy + (offset_xyz[1])*cy ;    
    input_xyz[2] =  offset_xyz[2];    
    input_uvw[0] = (position_t->raw_current_uvw[0])*cy  + (position_t->raw_current_uvw[1])*sy; 	//��gpsÿ�θ���ʱ�Ѿ�����	            
    input_uvw[1] = (position_t->raw_current_uvw[0])*-sy + (position_t->raw_current_uvw[1])*cy; 
	

}    
void uvw_transform(double input_uvw[3], Position * position_t)
{   
	//�ٶ�ת������---�����ñ�׼�����µ��ٶ� �����ڴ����䶯
    /*input_uvw[0] =  position.current_uvw[0]*cos(ahrs_theta[1])*cos(ahrs_theta[2])+position.current_uvw[1]*cos(ahrs_theta[1])*sin(ahrs_theta[2])-position.current_uvw[2]*sin(ahrs_theta[1]); 		   
    input_uvw[1] = -position.current_uvw[0]*sin(ahrs_theta[2])*cos(ahrs_theta[0])+position.current_uvw[1]*cos(ahrs_theta[2])*cos(ahrs_theta[0])+position.current_uvw[2]*sin(ahrs_theta[0]);    
    input_uvw[2] =  position.current_uvw[0]*(abs(sin(ahrs_theta[1])))*cos(ahrs_theta[2])
                    +position.current_uvw[1]*(abs(sin(ahrs_theta[1])))*sin(ahrs_theta[2])
                    +position.current_uvw[2]*cos(ahrs_theta[1]);*/
    //input_uvw[0]=Cnb[0][0]*position_t->current_uvw[0]+Cnb[0][1]*position_t->current_uvw[1]+Cnb[0][2]*position_t->current_uvw[2];
    //input_uvw[1]=Cnb[1][0]*position_t->current_uvw[0]+Cnb[1][1]*position_t->current_uvw[1]+Cnb[1][2]*position_t->current_uvw[2];   
    //input_uvw[2]=Cnb[2][0]*position_t->current_uvw[0]+Cnb[2][1]*position_t->current_uvw[1]+Cnb[2][2]*position_t->current_uvw[2];  
}

   
          
/*---------------------------------------------------------------------------------*/

double yaw_convert(const double yaw, const double t_head)
{
   if(t_head > PI/2)
       if(yaw < -PI/2)
       return yaw+2*PI;
   if(t_head < -PI/2)
       if(yaw >  PI/2)
       return yaw-2*PI;
   return yaw; 
} 
/*---------------------------------------------------------------------------------*/     
          
          


/*---------------------------------------------------------------------------------*/     
          
          


void gps_monitor()
{
    static double last_vtg_course ,
                  last_vtg_speed  ,
                  last_gga_time   ;
    
    static int speed_flag = 0,
               course_flag = 0,
               time_flag = 0;
        
    if(last_vtg_course == gps_vtg_course )
       course_flag ++;
    else 
       course_flag = 0 ;
    last_vtg_course = gps_vtg_course;
    
    if(last_vtg_speed == gps_vtg_speed )
       speed_flag ++;
    else 
       speed_flag = 0 ;
    last_vtg_speed = gps_vtg_speed ;
    
    
    if(last_gga_time == gps_gga_time)
       time_flag ++;
    else
       time_flag =0 ;
    last_gga_time = gps_gga_time ;
    
    if(course_flag >=30 && speed_flag >= 30 && time_flag >=30 )  //30������������û�и���gpsֵ
    {   
        gps_update_flag = 2;        //gps�ź��ж�
        /*trajectory.reset();         // ********����gps�жϵ����************
        position.current_uvw[0]= 0;
        position.current_uvw[1]= 0;
        position.current_uvw[2]= 0;
        trajectory.heli_step = 1;   //�ֿ�״̬��Ӱ��*/
    }   
}

void gps_update()
{
//    static double output_xyz[3],output_uvw[3];
    
    bool gps_sample = false;
    
    
     if(gga_update_flag==true ) // ���յ�һ������
     {
         gpgga_update();
         gga_update_flag=false;    
     }
    if(vtg_update_flag==true && (gps_gga_quality!=0))   // in interrupt_usart.c
     {
        gpvtg_update();
        vtg_update_flag = false;
        gps_sample = true;
     } 
     
     
     if(gps_sample==true)
     {
         gps_sample = false;
         if((gps_gga_quality==1 ||gps_gga_quality==2 || gps_gga_quality ==5 ) &&
            (gps_gga_num_sats>=4))
         {
             gps_update_flag = 1;
             gps_date_init();
             gps2xyz(position.first_raw_current_xyz);    //update current_xyz��˳���ܷ�
             position.first_raw_current_xyz[0] = position.first_raw_current_xyz[0] + cos(input_yaw) * 0.08;//0.23�����������gpsƫ����
             position.first_raw_current_xyz[1] = position.first_raw_current_xyz[1] + sin(input_yaw) * 0.08;  
             
//           gps2uvw(position.first_raw_current_uvw);    //update speed
             double old_raw_current_x = position.raw_current_xyz[0];
             double old_raw_current_y = position.raw_current_xyz[1];
             double old_raw_current_u = position.raw_current_uvw[0];
             double old_raw_current_v = position.raw_current_uvw[1];
             
              if(zero_flag == 1)
                  {
                    for(int i=0;i<2;i++)
                        {
                           position.raw_current_xyz[i] +=  position.raw_current_uvw[i] * 0.1;//Ԥ��
                           position.raw_current_xyz[i] = abnormal_limit(position.first_raw_current_xyz[i], last_raw_current_xyz[i], &position.raw_current_xyz[i], 1.0);//5m/s
                           last_raw_current_xyz[i] = position.first_raw_current_xyz[i];
//                           position.raw_current_uvw[i] = abnormal_limit(position.first_raw_current_uvw[i], last_raw_current_uvw[i],&output_uvw[i], 4);//ע���޷�ֵ��С
//                           last_raw_current_uvw[i] = position.first_raw_current_uvw[i];
                        }
                        position.raw_current_xyz[2] = abnormal_limit(position.first_raw_current_xyz[2], last_raw_current_xyz[2], &last_raw_current_xyz[2], 100);
                        last_raw_current_xyz[2] = position.first_raw_current_xyz[2];
//                        position.raw_current_uvw[2] = abnormal_limit(position.first_raw_current_uvw[2], last_raw_current_uvw[2], &output_uvw[2], 20);//�߶�����û�б�Ҫ�ޣ���
//                        last_raw_current_uvw[2] = position.first_raw_current_uvw[2];
                  }


                  
                  position.raw_current_uvw[0] = (position.raw_current_xyz[0] - old_raw_current_x) / 0.1;
                  if(position.raw_current_uvw[0] == 0.0 && abs(old_raw_current_u - position.raw_current_uvw[0]) > 1.8)
                  {
                    position.raw_current_uvw[0] = old_raw_current_u;
                    position.raw_current_xyz[0] +=  position.raw_current_uvw[0] * 0.1;
                  }
//                  double abs_e = abs(old_raw_current_u - position.raw_current_uvw[0]);
//                  if(abs_e < 0.1);
//                  else if(abs_e < 0.2)
//                    position.raw_current_uvw[0] = 0.1*old_raw_current_u + 0.9*position.raw_current_uvw[0];  
//                  else if(abs_e < 0.3)
//                    position.raw_current_uvw[0] = 0.2*old_raw_current_u + 0.8*position.raw_current_uvw[0];
//                  else if(abs_e < 0.5)
//                    position.raw_current_uvw[0] = 0.5*old_raw_current_u + 0.5*position.raw_current_uvw[0];
//                  else if(abs_e < 1.0)
//                    position.raw_current_uvw[0] = 0.75*old_raw_current_u + 0.25*position.raw_current_uvw[0];
//                  else
//                    position.raw_current_uvw[0] = 0.85*old_raw_current_u + 0.15*position.raw_current_uvw[0];
                  
                  
                  position.raw_current_uvw[1] = (position.raw_current_xyz[1] - old_raw_current_y) / 0.1;
                  if(position.raw_current_uvw[1] == 0.0 && abs(old_raw_current_v - position.raw_current_uvw[1]) > 1.8)
                  {
                  position.raw_current_uvw[1] = old_raw_current_v;
                  position.raw_current_xyz[1] +=  position.raw_current_uvw[1] * 0.1;
                  }  
//                  abs_e = abs(old_raw_current_v - position.raw_current_uvw[1]);
//                  if(abs_e < 0.1);
//                  else if(abs_e < 0.2)
//                    position.raw_current_uvw[1] = 0.1*old_raw_current_v + 0.9*position.raw_current_uvw[1];  
//                  else if(abs_e < 0.3)
//                    position.raw_current_uvw[1] = 0.2*old_raw_current_v + 0.8*position.raw_current_uvw[1];
//                  else if(abs_e < 0.5)
//                    position.raw_current_uvw[1] = 0.5*old_raw_current_v + 0.5*position.raw_current_uvw[1];
//                  else if(abs_e < 1.0)
//                    position.raw_current_uvw[1] = 0.75*old_raw_current_v + 0.25*position.raw_current_uvw[1];
//                  else
//                    position.raw_current_uvw[1] = 0.85*old_raw_current_v + 0.15*position.raw_current_uvw[1];

              
              
         }
         else //set to abnormal mod, all to zero.
         {
              gps_update_flag = 2;
               trajectory.reset();     // ********����gps�жϵ����************
              position.current_uvw[0]= 0;
              position.current_uvw[1]= 0;
              position.current_uvw[2]= 0;
              position.raw_current_uvw[0]= 0;
              position.raw_current_uvw[1]= 0;
              position.raw_current_uvw[2]= 0;
              position.current_target[0] =  position.current_xyz[0] ;
              position.current_target[1] =  position.current_xyz[1];
              position.current_target[2] =  position.current_xyz[2];
              
              trajectory.heli_step = 1;   //�ֿ�״̬��Ӱ��  
         }
         
     }//end if(gps_sample = 1) 

}
 
