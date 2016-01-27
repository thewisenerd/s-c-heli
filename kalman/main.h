#ifndef _MAIN_H
#define _MAIN_H

/*#define lab_test*/
/*#define circle_test*/ 

extern struct  Target_Command     target_command;
extern struct  Target_Queue       target_queue;
extern struct  Position           position;
extern struct  Trajectory         trajectory;//



extern void uvw_transform(  double *vel_bf, Position * positon_t);
extern void xyz_transform(double input_xyz[3], Position * positon_t,Trajectory *trajectory_t);
extern  int round_count ;
extern  int first_receive_target;

extern void controller_step( const Trajectory * trajectory_t,
                     		 const double pos_body[3],
		     				 const double vel_body[3],
		    				 const double theta[3],
		     				 const double pqr[3]			);

extern void LQRControllerStep( const Trajectory * trajectory_t,
									const double pos_body[3],
		    						const double vel_body[3],
		    						const double theta[3],
		     						const double pqr[3]				);
extern void ahrs_adjust(double *theta,double *offset);
extern double filter_pqr[3];

#endif
