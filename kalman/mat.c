#include<includs.h>

//-------平均滤波--------//
double mean_filter(double * array , int len,double new_val)
{
    double sum =0 ;
    for(int i = len-1 ; i!=0 ; --i){
       sum += array[i];
       array[i]=array[i-1]; 
    }
    array[0] = new_val;
    sum += array[0];
    return sum/len;
}

//----------简化平均滤波---------//
double mean_filter_simple(int len ,double sum ,double * last_mean ,double new_val)
{    
    sum += new_val - *last_mean;   
    return (*last_mean = sum/len);
}

//--------一阶线性滤波------------//
double first_order_filter(double weight_factor, double last_val,double new_val )
{
   return (weight_factor*new_val + (1-weight_factor)*last_val) ;
}

//--------------------------------//
void eulerDC(
	double			tBL[3][3],
	double			phi,
	double			theta,
	double			psi
)
{
	const double		cpsi	= cos(psi);
	const double		cphi	= cos(phi);
	const double		ctheta	= cos(theta);
	const double		spsi	= sin(psi);
	const double		sphi	= sin(phi);
	const double		stheta	= sin(theta);

	tBL[0][0] = ctheta * cpsi ;
	tBL[0][1] = ctheta * spsi ;
	tBL[0][2] = -stheta;

	tBL[1][0] = -spsi*cphi + cpsi*stheta*sphi;
	tBL[1][1] =  cpsi*cphi + sphi*stheta*sphi;
	tBL[1][2] = ctheta*sphi;

	tBL[2][0] =  spsi*sphi + cpsi*stheta*cphi;
	tBL[2][1] = -cpsi*sphi + spsi*stheta*cphi;
	tBL[2][2] = ctheta*cphi;
}

/*-----------------------------------------------*/
void ned2bf3(	double  		v_out[3],
                const double  	        theta[3],
	        const double            v_in[3])
{
	double			cBE[3][3];
	eulerDC( cBE, theta[0],theta[1],theta[2] );
	v_out[0] = cBE[0][0] * v_in[0] + cBE[0][1] * v_in[1] + cBE[0][2] * v_in[2] ;
        v_out[1] = cBE[1][0] * v_in[0] + cBE[1][1] * v_in[1] + cBE[1][2] * v_in[2] ;
        v_out[2] = cBE[2][0] * v_in[0] + cBE[2][1] * v_in[1] + cBE[2][2] * v_in[2] ;
}

/*-----------------------------------------------*/
void ned2bf2( double v_out[3],
              const double theta,
              const double v_in[3])
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


void norm( double* q )
{
	unsigned char i;
	double mag = 0;
				
	for( i=0; i<4; i++ ){
		mag += q[i] * q[i];
	} 
	
	mag = sqrt( mag );
					
	for( i=0; i<4; i++ ){
		q[i] = q[i] / mag;
	}
}

 inline char is_zero( const double* p )
{
	const unsigned int* f = ( const unsigned int *) p;
			
	if( *f == 0 ) return 1;
				
	return 0;
						
}

/**********************************************************/
/**********************************************************/

 void mulNxM( void*  Out_ptr,
	     const void*  A_ptr,
	     const void*  B_ptr,
	     signed char n,
	     signed char m,
	     signed char p,
	     signed char transpose_B,
	     signed char add
	   )
{
	char i, j, k;
	
	double* OUT = (double*)Out_ptr; 
	const double* A =  (double*)A_ptr;
	const double* B =  (double*)B_ptr;
						
	for( i=0; i<n; i++ ){
		const double* A_i = A + i * m;    // A矩阵第i行首地址
		double* O_i = OUT + i * p;   // 输出矩阵(NxP)第i行首地址
		
		for( j=0; j<p; j++ ){
																	double  s = 0;            // 累加器 
			 double* O_i_j = O_i + j;  //输出矩阵第i行,j列地址																							

			 for( k=0; k<m; k++ ){
				 const double* a = A_i + k; //A(NxM)矩阵第i行中的第k列地址
				 const double* b;																			
				 if ( is_zero( a ) )				
					 continue;

				 if ( transpose_B )  //注意B矩阵作为参数传递过来并未转置
					 b = B + j * m + k;  //故这里实际是B的j行 k列地址
				 else 

					 b = B + k * p + j;  //B的第k行 j列地址

				 if ( is_zero( b ) )
					 continue;																					
				 s += *a * *b;

			 }
																										

			 if ( add == 0 )       *O_i_j  = s;

			 else if ( add  > 0 )  *O_i_j += s;

			 else                  *O_i_j -= s;

		}

	}
}

/**********************************************************/
/**********************************************************/

/**********************************************************/
/**********************************************************/

 double limit( const double f,const double min,const double max)
{
		if( f < min ) return min;
			
		if( f > max ) return max;

		return f;
}

unsigned int limit_int( const int f,const int min,const int max)
{
		if( f < min ) return min;
			
		if( f > max ) return max;

		return f;
}

/*********************************************************/
/*********************************************************/

void rad2deg( double* degree, double* rad, unsigned char index )
{
	unsigned char i;

	for( i=0; i<index; i++ ){
	       degree[ i ] = rad[ i ] * 180.0 / PI;
	}

}	

/**********************************************************/
/**********************************************************/
double compass_heading( double* euler, double * compass )
{	
	double compass_calib[2];
	double yaw;

        double phi = euler[0];
        double theta = -euler[1];   
        double sp,cp,st,ct;
        sp = sin(phi);
        st = sin(theta);
        cp = cos(phi);
        ct = cos(theta);    
      //---------------------------------修改于2014/7/28-------------------------------------------------      
//        compass_calib[0] = compass[0]*cos(theta) + compass[1]*sin(phi)*sin(theta);
//        compass_calib[1] = compass[1]*cos(phi);
        compass_calib[0] =compass[0]*ct + compass[1]*sp*st - compass[2]*st*cp;
        compass_calib[1] = compass[1]*cp + compass[2]*sp;
//        compass_calib[0] =compass[0]*cos(theta) + compass[1]*sin(phi)*sin(theta) - compass[2]*sin(theta)*cos(phi);
//        compass_calib[1] = compass[1]*cos(phi) + compass[2]*sin(phi);     
//--------------------------------------------------------------------------------------------------------

	if( compass_calib[0] != 0.000)   
        {
		yaw = atan2( compass_calib[1], compass_calib[0]);//*******根据compass安装位置确定是否加PI***********
	}
        else if ( compass_calib[1] < 0.000 )
        {
		yaw = -PI /2.0;
	}
        else
        {
	       	yaw = PI / 2.0;
	}
       
          
        /*
      //--------------------------------
       // double compass_all = sqrt( compass[0]*compass[0]+compass[1]*compass[1]+compass[2]*compass[2] );
       compass_calib[0] = compass[0] * cos(theta) + compass[1] * sin(theta) *sin(phi)+ compass[2]*sin(theta)*cos(phi);
       compass_calib[1] = compass[1] * cos(phi) *sin(phi)- compass[2]*sin(phi);
       	if( compass_calib[0] != 0.00000)   {
		yaw = atan2( compass_calib[1], compass_calib[0] );
	}else if ( compass_calib[1] < 0.00000 ) {
		yaw = -PI*3.0 /2.0;
        }else {
	       	yaw = PI / 2.0;
	} 
        
        */
        //-------------------------
	return  yaw;
}
/**********************************************************************/
/**********************************************************************/
unsigned char update_flag = 0;

void accel2euler( double* THETAm, const double* accel, double * compass  )
{

	double g;

	//g = sqrt( accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2] );
        g=accel_g;
        
	THETAm[0] =  atan2( accel[ 1 ],  accel[ 2 ] );

	THETAm[1] =  -asin( limit( accel[ 0 ] / g, -1, 1 ) );   
	
	THETAm[2] =   compass_heading( THETAm, compass ) ;
}


/**********************************************************/
/**********************************************************/

void quat2dcv( double* DCV, const double* q )
{
	DCV[0] =     2*(q[1]*q[3] - q[0]*q[2]);
	DCV[1] =     2*(q[2]*q[3] + q[0]*q[1]);
	DCV[2] = 1.0-2*(q[1]*q[1] + q[2]*q[2]);
}

/**********************************************************/
/**********************************************************/

void dcv2euler( double* THETAe, const double* DCV )
{
#ifdef __AVR__
		THETAe[0] = atan2( DCV[2], DCV[1] );
#else
		THETAe[0] = atan2( DCV[1], DCV[2] );
#endif

		THETAe[1] = -asin( DCV[0] );
				
}

/**********************************************************/
/**********************************************************/

void quat2euler( double* THETAe, const double * quat )
{
	THETAe[0] = atan2( 2 * (quat[2] * quat[3] + quat[0] * quat[1] ),\
				1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2] ) );
		
	THETAe[1] =  -asin( 2 * ( quat[1] * quat[3] - quat[0] * quat[2] ) );
				
	THETAe[2] = atan2( 2 * ( quat[1] * quat[2] + quat[0] * quat[3] ), \
				1 - 2 * ( quat[2] * quat[2] + quat[3] * quat[3] ) );
}

/**********************************************************/
/**********************************************************/

void euler2quat( double* quat, const double* myeuler )
{
	const double phi   = myeuler[0] / 2.0;
	const double theta = myeuler[1] / 2.0;
	const double psi   = myeuler[2] / 2.0;
					
	const double shphi0   = sin( phi );
	const double chphi0   = cos( phi );
							
	const double shtheta0 = sin( theta );
	const double chtheta0 = cos (theta );
									
	const double shpsi0   = sin( psi );
	const double chpsi0   = cos( psi );
											
	quat[0] =  chphi0 * chtheta0 * chpsi0 + shphi0 * shtheta0 * shpsi0;
	quat[1] = -chphi0 * shtheta0 * shpsi0 + shphi0 * chtheta0 * chpsi0;
	quat[2] =  chphi0 * shtheta0 * chpsi0 + shphi0 * chtheta0 * shpsi0;
	quat[3] =  chphi0 * chtheta0 * shpsi0 - shphi0 * shtheta0 * chpsi0;
}

/**********************************************************/
/**********************************************************/

void invert2( double (*A)[2] )
{
   double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
   double temp = A[0][0];			

   A[0][0] = A[1][1] / det;
   A[1][1] = temp /det;
   A[0][1] /= -det;
   A[1][0] /= -det;
}

/**********************************************************/
/**********************************************************/

void quat2DCM( double (*DD)[3], const double* quat )
{
	DD[0][0] = 1 - 2 * ( quat[2]*quat[2] + quat[3]*quat[3] );
	DD[0][1] = 2 * ( quat[1]*quat[2] + quat[0]*quat[3] );
	DD[0][2] = 2 * ( quat[1]*quat[3] - quat[0]*quat[2] );
					
	DD[1][0] = 2 * ( quat[1]*quat[2] - quat[0]*quat[3] );
	DD[1][1] = 1 - 2 * ( quat[1]*quat[1] + quat[3]*quat[3] );
	DD[1][2] = 2 * ( quat[2]*quat[3] + quat[0]*quat[1] );
								
	DD[2][0] = 2 * ( quat[1]*quat[3] + quat[0]*quat[2] );
	DD[2][1] = 2 * ( quat[2]*quat[3] - quat[0]*quat[1] );
	DD[2][2] = 1 - 2 * ( quat[1]*quat[1] + quat[2]*quat[2] );
}

/**********************************************************/
/**********************************************************/
	
void invert3( double (*matrix)[3], unsigned char order )
{	
	unsigned char m, i, j;
		
	for( m=0; m<order; m++ ){
   	  for( i=0; i<order; i++ ){
	    for( j=0; j<order; j++ ){
	      if( ( i!=m ) && ( j!=m ) ) {
		   matrix[i][j] = matrix[i][j] - matrix[i][m]*matrix[m][j]/matrix[m][m];
	      }	
            }
          }

	  for( j=0; j<order; j++ ){									      	         if( j != m ){
 		matrix[m][j] =  matrix[m][j]/matrix[m][m];
									
		matrix[j][m] = -matrix[j][m]/matrix[m][m];

	    }
   	  }

	  matrix[m][m] = 1.0/matrix[m][m];
       }
}

void invert6( double matrix[6][6], unsigned char order )
{	
	
	unsigned char m, i, j;

	
	for( m=0; m<order; m++ ){
   	  for( i=0; i<order; i++ ){
	    for( j=0; j<order; j++ ){
	      if( ( i!=m ) && ( j!=m ) ) {
		    matrix[i][j] = matrix[i][j] - matrix[i][m]*matrix[m][j]/matrix[m][m];
															   }	
            }
	  }
	  
	  for( j=0; j<order; j++ ){	
		  if( j != m ){
			  matrix[m][j] =  matrix[m][j]/matrix[m][m];
			  matrix[j][m] = -matrix[j][m]/matrix[m][m];
		  }
	  }

	
	  matrix[m][m] = 1.0/matrix[m][m];
	}
}

//-----------------syc 限制各传感器异常跳变函数----------------------//
double abnormal_limit(double raw_value, double last_value, double *out_value, double len)
{
    static double value = 0.0;
	static double out_result = 0.0;
	//static int count_point = 0;
	value = last_value - raw_value;
	if(value <= -len || value >= len)
	{
	    abnormal_limit_flag = 1; 
		out_result = *out_value;
	}		    
	else
	{   
		out_result = raw_value;	
	}
	*out_value = out_result;
	return out_result;
}
//--------------十字盘解耦加耦矩阵----------------------------//
void DecoupingMatrix(int out[3], int in[3])  //rcp
{
	out[0] = (in[0] - in[1] + 2*in[2]) / 3;//pitch
	out[1] = (in[0] + in[1]) / 2;//roll
	out[2] = (-in[0] + in[1] + in[2]) / 3;//coll
}
void CoupingMatrix(int out[3], int in[3])//prc
{
	out[0] = 0.5*in[0] + in[1] - in[2];//roll
	out[1] =-0.5*in[0] + in[1] + in[2];//coll
	out[2] = in[0] + in[2];//pitch
}

