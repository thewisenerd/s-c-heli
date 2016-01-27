#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_


extern double out_loop_X_PID[3];
extern double out_loop_Y_PID[3];
extern double out_loop_D_PID[3];

extern double in_loop_roll_PID[3];
extern double in_loop_pitch_PID[3];
extern double in_loop_yaw_PID[3];

extern double roll_commend[2];
extern double pitch_commend[2];
extern double yaw_commend[2];




//outputs and mids
extern double ahrs_yaw;

extern double outputs[4];

//####operaters####//
//if undefined limit()
/*inline double limit(const double rel,const double min,const double max)
{
	if(rel < min) return min;
	if(rel > max) return max;
	return rel;
}*/

//reset the controller
extern void reset_controller();
extern void set_init_state() ;
//give me the comment position and velocity you want!!
extern void out_loop_commend(const double C_position[3], 
					  const double C_velocity[3]);
//extern void out_loop_feedback(const double F_position[3],
	//				   const double F_velocity[3]);
//extern void out_loop_PID_step();

//inloop PID commend and feedback
//extern void in_loop_feedback(const double F_position[3],
	//				  const double F_velocity[3]);
//extern void in_loop_PID_step();

//PID base step



//total step

void PID_control(const double theta[3], const double pqr[3]);

#endif
