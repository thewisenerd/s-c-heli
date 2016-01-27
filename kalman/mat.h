#ifndef _mat_h_
#define _mat_h_


#define PI 3.14159265


extern unsigned char update_flag;

/**********************************************************/
/**********************************************************/

extern void  mulNxM( void*  Out_ptr,
	     const void*  A_ptr,
	     const void*  B_ptr,
	     signed char n,
	     signed char m,
	     signed char p,
	     signed char transpose_B,
	     signed char add
	   );

extern void norm( double* q);

extern void rad2deg( double* degree, double* rad , unsigned char index );

extern void accel2euler( double* THETAm, const double* accel ,double* compass);			 

extern void quat2dcv( double* DCV, const double* q );
extern void dcv2euler( double* THETAe, const double* DCV );

extern void quat2euler( double* THETAe, const double* quat );
extern void euler2quat( double* quat, const double* euler);
extern void quat2DCM( double (*DD)[3], const double* quat );
extern void invert3( double (*matrix)[3], unsigned char order );
extern void invert6( double matrix[6][6], unsigned char order );

extern void invert2( double (*A)[2] );
extern double limit( const double f,const double min,const double max);
extern unsigned int limit_int( const int f,const int min,const int max);

extern double abnormal_limit(double raw_value, double last_value, double *out_value, double len);
void DecoupingMatrix(int out[3], int in[3]);
void CoupingMatrix(int out[3], int in[3]);

#endif

