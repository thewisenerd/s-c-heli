#include <includs.h>

//------------------------LQR parameter-----------------------//
#define		Freq	30

double lastVelerrA1 = 0.0;
double lastVelerrB1 = 0.0;

//double LQR_rollGain[3] = {0.5247, 0.0108, 2.6586};//5.16_1
//double LQR_rollGain[3] = {0.8261,0.0071,1.1259};//5.16_2 Q=0.15
//double LQR_rollGain[3] = {0.8222,0.0070,1.1192};//5.31_3 Q=0.1
//double LQR_rollGain[3] = {0.8198,0.0070,1.1151};//5.31_4 Q=0.05
//double LQR_rollGain[3] = {0.8191,0.0070,1.1138};//5.31_5 Q=0.01
double LQR_rollGain[3] = {0.819,0.007,1.114};
//double LQR_pitchGain[3] = {0.8887,0.0314,1.0379};//5.16_3 Q=0.1
//double LQR_pitchGain[3] = {0.8858,0.0313,1.0342};//5.31_4 Q=0.05
//double LQR_pitchGain[3] = {0.8848,0.0312,1.0330};//5.31_5 Q=0.01
double LQR_pitchGain[3] = {0.685,0.031,0.533};
//---------------------------------------------------------//

double X_commend[2]; 
double Y_commend[2];
double D_commends[2];

double roll_commend[2];
double pitch_commend[2];
double yaw_commend[2]; 

double X_feedback[2];
double Y_feedback[2];
double D_feedback[2];

double roll_feedback[2];
double pitch_feedback[2];
double yaw_feedback[2];

//parameters kp ki kd.
double out_loop_X_PID[3];
double out_loop_Y_PID[3];
double out_loop_D_PID[3];

double in_loop_roll_PID[3];
double in_loop_pitch_PID[3];
double in_loop_yaw_PID[3];


struct limits
{
	double ProErrorLimits[2];
	double VelErrorLimits[2];
	double IntStateLimits[2];
	double OutErrorLimits[2];
	
	double int_state;
} X_limits,Y_limits,D_limits,roll_limits,pitch_limits,yaw_limits;

double outputs[4];

double PID_step(	const double commend[2],
                        const double feedback[2],
                        const double PID_gain[3], 
                        struct limits *lim	)
{        
	double err = limit((commend[0] - feedback[0]),lim->ProErrorLimits[0],
						lim->ProErrorLimits[1]);

	double velerr = limit((commend[1] - feedback[1]),lim->VelErrorLimits[0],
						lim->VelErrorLimits[1]);
	
        //NOTE HERE: PID_gain[0] is KP,PID_gain[2] is KD, PID_gain[1] is KI
	double result = 0.0 +PID_gain[0]*err+PID_gain[2]*velerr
			+PID_gain[1]*(lim->int_state); 
	
	lim->int_state = limit(	lim->int_state + err*dt,
			        lim->IntStateLimits[0],
                                lim->IntStateLimits[1]	);
	return limit(result,
				 lim->OutErrorLimits[0],
				 lim->OutErrorLimits[1]);
}

void set_limits()
{
	//--------X 轴限幅-----------
    X_limits.ProErrorLimits[0] = -8.0;
	X_limits.ProErrorLimits[1] =  8.0;
	X_limits.VelErrorLimits[0] = -10.0; 
	X_limits.VelErrorLimits[1] =  10.0;
	X_limits.IntStateLimits[0] = -2.0;
	X_limits.IntStateLimits[1] =  2.0;
	X_limits.OutErrorLimits[0] = -0.255;//rad 
	X_limits.OutErrorLimits[1] =  0.255;
	
        //--------Y 轴限幅-----------
	Y_limits.ProErrorLimits[0] = -8.0;
	Y_limits.ProErrorLimits[1] =  8.0;
	Y_limits.VelErrorLimits[0] = -10.0; 
	Y_limits.VelErrorLimits[1] =  10.0;
	Y_limits.IntStateLimits[0] = -2.0;
	Y_limits.IntStateLimits[1] =  2.0;
	Y_limits.OutErrorLimits[0] = -0.255;//rad 
	Y_limits.OutErrorLimits[1] =  0.255;
	
        //-----Z 轴限幅-------------
	D_limits.ProErrorLimits[0] =-5.0;      //  限定不能太大，否则上升速度过大;
	D_limits.ProErrorLimits[1] = 5.0;      //  8m 限定可以达到3m/s速度，太快；
	D_limits.VelErrorLimits[0] = -8.0; 
	D_limits.VelErrorLimits[1] =  8.0;
	D_limits.IntStateLimits[0] = -2.0;
	D_limits.IntStateLimits[1] =  2.0;
	D_limits.OutErrorLimits[0] = -10;//pwm without mutiply amplify(48) 
	D_limits.OutErrorLimits[1] =  10;
	
        //------roll 限幅-----------
	roll_limits.ProErrorLimits[0] = -20.0*C_DEG2RAD;
	roll_limits.ProErrorLimits[1] =  20.0*C_DEG2RAD;
	roll_limits.VelErrorLimits[0] = -20.0*C_DEG2RAD;//内环微分项限幅
	roll_limits.VelErrorLimits[1] =  20.0*C_DEG2RAD;
	roll_limits.IntStateLimits[0] = - 0.6*C_DEG2RAD;
	roll_limits.IntStateLimits[1] =   0.6*C_DEG2RAD;
	roll_limits.OutErrorLimits[0] = -20.0*C_DEG2RAD;
	roll_limits.OutErrorLimits[1] =  20.0*C_DEG2RAD;
        
        
        //------pitch 限幅-----------
	pitch_limits.ProErrorLimits[0] = -20.0*C_DEG2RAD;
	pitch_limits.ProErrorLimits[1] =  20.0*C_DEG2RAD;
	pitch_limits.VelErrorLimits[0] = -20.0*C_DEG2RAD;
	pitch_limits.VelErrorLimits[1] =  20.0*C_DEG2RAD;
	pitch_limits.IntStateLimits[0] = - 0.6*C_DEG2RAD;
	pitch_limits.IntStateLimits[1] =   0.6*C_DEG2RAD;
	pitch_limits.OutErrorLimits[0] = -20.0*C_DEG2RAD;
	pitch_limits.OutErrorLimits[1] =  20.0*C_DEG2RAD;
        
        //------yaw 限幅-----------
	yaw_limits.ProErrorLimits[0] = -30.0*C_DEG2RAD;
	yaw_limits.ProErrorLimits[1] =  30.0*C_DEG2RAD;
	yaw_limits.VelErrorLimits[0] = -100.0*C_DEG2RAD;
	yaw_limits.VelErrorLimits[1] =  100.0*C_DEG2RAD;
	yaw_limits.IntStateLimits[0] = -90.0*C_DEG2RAD;
	yaw_limits.IntStateLimits[1] =  90.0*C_DEG2RAD;
	yaw_limits.OutErrorLimits[0] = -30.0*C_DEG2RAD;
	yaw_limits.OutErrorLimits[1] =  30.0*C_DEG2RAD;
}
void set_init_state()
{
   	X_limits.int_state = 0.0;
	Y_limits.int_state = 0.0;
	D_limits.int_state = 0.0;
	roll_limits.int_state = 0.0;
	pitch_limits.int_state = 0.0;
	yaw_limits.int_state = 0.0;  
}
void reset_controller()
{
	//set the default target_commend; in hover state
	X_commend[0] = 0.0;
	X_commend[1] = 0.0;
	Y_commend[0] = 0.0;
	Y_commend[1] = 0.0;
	D_commends[0] = 0.0;
	D_commends[1] = 0.0;
	yaw_commend[0] = 0.0;
	yaw_commend[1] = 0.0;
	//set the default PID gain ,from t1.NOTE the sign
	out_loop_X_PID[0] = 3.0;
	out_loop_X_PID[1] = 0.0;
	out_loop_X_PID[2] = 13.0;//-0.05;

	out_loop_Y_PID[0] = 3.0;
	out_loop_Y_PID[1] = 0.0;
	out_loop_Y_PID[2] = 13.0;//0.05;
	
	out_loop_D_PID[0] = -50.0;//-40
	out_loop_D_PID[1] = 0.0;
	out_loop_D_PID[2] = -60.0;//-90
	
        in_loop_roll_PID[0] = 2800.0;   //-0.245;
	in_loop_roll_PID[1] = -0.0;
	in_loop_roll_PID[2] = 30.0;    //-0.007;//-0.005

	in_loop_pitch_PID[0] = -2300.0;// -850.0;  //0.5;
	in_loop_pitch_PID[1] = 0.0;
	in_loop_pitch_PID[2] = -30.0;//-22.0;   //0.15;//0.1

	in_loop_yaw_PID[0] = -0.15;
	in_loop_yaw_PID[1] = -0.002;
	in_loop_yaw_PID[2] = -0.0;
	
	//set limits ,call the above function
	set_limits();
	//set the int_state;
    set_init_state();

}

//out loop
void out_loop_commend(const double C_position[3], 
					  const double C_velocity[3])
{
	X_commend[0] = C_position[0];
	X_commend[1] = C_velocity[0];//X
	
	Y_commend[0] = C_position[1];
	Y_commend[1] = C_velocity[1];//Y

	D_commends[0] = C_position[2];//+trajectory.yaw_com_theta;//航向角阶跃响应测试
	D_commends[1] = C_velocity[2];//D
}

void out_loop_feedback(const double F_position[3],
					   const double F_velocity[3])
{
	X_feedback[0] = F_position[0];
	X_feedback[1] = F_velocity[0];//X

	Y_feedback[0] = F_position[1];
	Y_feedback[1] = F_velocity[1];//Y

	D_feedback[0] = F_position[2];
	D_feedback[1] = F_velocity[2];//D
}
/*
void out_loop_PID_step(const Trajectory * trajectory_t )
{	
	//roll  trajectory_t->com_theta是用来产生内环阶跃响应 看内环跟踪效果
	roll_commend[0] = trajectory_t->com_theta[0]+PID_step(Y_commend,Y_feedback,out_loop_Y_PID,&Y_limits);
	roll_commend[1] = trajectory_t->com_pqr[0];
	//pitch
	//pitch_commend[0] = PID_step_pitch(X_commend,X_feedback,out_loop_X_PID,&X_limits);
    pitch_commend[0] = trajectory_t->com_theta[1]+PID_step(X_commend,X_feedback,out_loop_X_PID,&X_limits);
	pitch_commend[1] = trajectory_t->com_pqr[1];
	//yaw
	yaw_commend[0] = position.heading;//+trajectory_t->yaw_com_theta;  //目标航向角 .
	yaw_commend[1] = trajectory_t->com_pqr[2];

	//out put coll_pwm
	outputs[0] = PID_step(D_commends,D_feedback,out_loop_D_PID,&D_limits);
}
*/
//in_loop
void in_loop_feedback(const double F_position[3],
					  const double F_velocity[3])
{
	//roll
	roll_feedback[0] = F_position[0];
	roll_feedback[1] = F_velocity[0];
	//pitch
	pitch_feedback[0] = F_position[1];
	pitch_feedback[1] = F_velocity[1];
	//yaw
	yaw_feedback[0] = F_position[2];
	yaw_feedback[1] = F_velocity[2];
}

void in_loop_PID_step()
{	
	//roll_out rad
	outputs[1] = PID_step(roll_commend,roll_feedback,in_loop_roll_PID,&roll_limits);
	//pitch_out rad .NOTE:there is a "-" here
	outputs[2] = -PID_step(pitch_commend,pitch_feedback,in_loop_pitch_PID,&pitch_limits);
	//yaw_out rad
	outputs[3] = PID_step(yaw_commend,yaw_feedback,in_loop_yaw_PID,&yaw_limits);
	
}
/****************************************************************
 *函数名: void out_loop_PID_step(const Trajectory * trajectory_t )
 *
 *参数: const Trajectory * trajectory_t为路径规划的结构体
 *      	
 *功能：PID控制器外环
 *				
 *返回值：暂无
 *
 *作    者：徐志恒
 *完成日期: 2013-3-29
 ****************************************************************/
 void out_loop_PID_step(const Trajectory * trajectory_t )
{	
	//roll
	//roll_commend[0] = trajectory_t->com_theta[0]; //PID_step(Y_commend,Y_feedback,out_loop_Y_PID,&Y_limits);
	
	roll_commend[0] = trajectory_t->com_theta[0] + PID_step(Y_commend,Y_feedback,out_loop_Y_PID,&Y_limits);
	roll_commend[1] = trajectory_t->com_pqr[0];
	
	//pitch
    //pitch_commend[0] = trajectory_t->com_theta[1]; //PID_step(X_commend,X_feedback,out_loop_X_PID,&X_limits);
	
	pitch_commend[0] = trajectory_t->com_theta[1] + PID_step(X_commend,X_feedback,out_loop_X_PID,&X_limits);
	pitch_commend[1] = trajectory_t->com_pqr[1];
	
	//yaw
	yaw_commend[0] = position.heading;//+trajectory_t->yaw_com_theta;  //目标航向角 .
	yaw_commend[1] = trajectory_t->com_pqr[2];

	//out put coll_pwm
	outputs[0] = PID_step(D_commends,D_feedback,out_loop_D_PID,&D_limits);
}

/****************************************************************
 *函数名: LonLQRStep( const double 	commend[2], 
 *				  	  const double 	feedback[2],
 *				      const double 	LQR_gain[3],   
 *				      struct 	   	limits *lim		)
 *
 *参数: const double commend[2]为pitch向内环控制量
 *		const double feedback[2]为pitch向内环反馈量（角度、角速度、挥舞角）
 *      const double LQR_gain[3]为LQR控制器增益
 *		struct limits *lim 为限幅值结构体
 *
 *功能：Pitch向LQR控制器
 *			
 *返回值：暂无
 *
 *作    者：徐志恒
 *完成日期: 2013-3-29
 ****************************************************************/
double LonLQRStep(const double 	commend[2], 
				  const double 	feedback[2],
				  const double 	LQR_gain[3],   
				  struct 	   	limits *lim)
{
	double err, velerr, a1err, result;
	
	err = limit( (commend[0] - feedback[0]), 
				 lim->ProErrorLimits[0], 
				 lim->ProErrorLimits[1]);

	velerr = limit( (commend[1] - feedback[1]),
				    lim->VelErrorLimits[0], 
					lim->VelErrorLimits[1]);

	a1err = (velerr - lastVelerrA1) * Freq / 426.82;
	
	result = 0.0 + LQR_gain[0]*err 
		         + LQR_gain[1]*velerr 
				 + LQR_gain[2]*a1err;
	
	lastVelerrA1 = velerr;
	
	return limit(result, 
				 lim->OutErrorLimits[0], 
				 lim->OutErrorLimits[1]);
}

/****************************************************************
 *函数名:double LatLQRStep( const double commend[2], 
 *				  			const double feedback[2],
 *				  			const double LQR_gain[3],   
 *				  			struct limits *lim         )
 *
 *参数：const double commend[2]为roll向内环控制量
 *		const double feedback[2]为roll向内环反馈量（角度、角速度、挥舞角）
 *      const double LQR_gain[3]为LQR控制器增益
 *		struct limits *lim 为限幅值结构体
 *      
 *功能：roll向LQR控制器
 *		
 *返回值：暂无
 *
 *作    者：徐志恒
 *完成日期: 2013-3-29
 ****************************************************************/
double LatLQRStep(const double 	commend[2], 
				  const double 	feedback[2],
				  const double 	LQR_gain[3],   
				  struct 	   	limits *lim)
{
	double err, velerr, b1err, result;
	
	err = limit( (commend[0] - feedback[0]), 
		  		 lim->ProErrorLimits[0], 
				 lim->ProErrorLimits[1]);

	velerr = limit( (commend[1] - feedback[1]), 
				  	lim->VelErrorLimits[0], 
					lim->VelErrorLimits[1]);

	b1err = (velerr - lastVelerrB1) * Freq / 1338.83;//挥舞角
	
	result = 0.0 + LQR_gain[0]*err 
				 + LQR_gain[1]*velerr 
			     + LQR_gain[2]*b1err;
	
	lastVelerrB1 = velerr;
	
	return limit(result, 
				 lim->OutErrorLimits[0], 
				 lim->OutErrorLimits[1]);
}

/****************************************************************
 *函数名: void InLoopLQRStep()
 *
 *参数: 无
 *      
 *功能：内环控制器包括两个Pitch和Roll的LQR控制器和Yaw向的PID控制器
 *			
 *返回值：暂无
 *
 *作    者：徐志恒
 *完成日期: 2013-3-29
 ****************************************************************/
void InLoopLQRStep()
{	
	//roll_out rad
	outputs[1] = -LatLQRStep(roll_commend, 
							 roll_feedback, 
							 LQR_rollGain, 
							 &roll_limits);
	
	//pitch_out rad .NOTE:there is a "-" here
	outputs[2] = -LonLQRStep(pitch_commend,
							 pitch_feedback,
							 LQR_pitchGain,
							 &pitch_limits);
	//yaw_out rad
	outputs[3] = PID_step(yaw_commend, 
						  yaw_feedback, 
						  in_loop_yaw_PID, 
						  &yaw_limits);
	
}

/*
void controller_step(	const Trajectory * trajectory_t,
                     	const double pos_body[3],
		     			const double vel_body[3],
		   	 			const double  theta[3],
		     			const double pqr[3]				)
{	
	//step 0 ,set the commends.usually before entering controller.by default,
	// hover target.but also need the target_heading.
    out_loop_commend(trajectory_t->com_position, trajectory_t->com_velocity );

	//step 1, set the outloop feedbacks.
	out_loop_feedback(pos_body,vel_body);
	//step 2, outloop PID 
	out_loop_PID_step(trajectory_t);
        
	//step 3, set the inloop feedbacks
	in_loop_feedback(theta,pqr);
	//step 4, inloop PID
	in_loop_PID_step();

}
*/
/****************************************************************
 *函数名: void LQRControllerStep( const	Trajectory * trajectory_t, 
 *							  	  const double pos_body[3],
 *		    					  const double vel_body[3],
 *		    			  		  const double theta[3],
 *		     					  const double pqr[3]				)
 *
 *参数: const	Trajectory * trajectory_t为路径规划结构体
 * 		const double pos_body[3]为机体坐标
 *		const double vel_body[3]为机体速度
 *		const double theta[3]为姿态角（Pitch、Roll、Yaw)
 *		const double pqr[3]为角速度
 *
 *功能：更新外环控制量
 *		更新外环反馈量
 *		外环PID控制器且更新内环控制量
 *		更新外环反馈量
 *		内环Pitch和Roll向LQR控制器和Yaw向PID控制器
 *
 *返回值：暂无
 *
 *作    者：徐志恒
 *完成日期: 2013-3-29
 ****************************************************************/
void LQRControllerStep( const 		 Trajectory * trajectory_t,
						const double pos_body[3],
		    			const double vel_body[3],
		    			const double theta[3],
		     			const double pqr[3])
//LQRControllerStep(&trajectory,input_xyz, input_uvw, theta, filter_pqr);
{
	out_loop_commend(trajectory_t->com_position, trajectory_t->com_velocity );
	//step 1, set the outloop feedbacks.
	out_loop_feedback(pos_body, vel_body);
	//step 2, outloop PID 
	out_loop_PID_step(trajectory_t);
	
	in_loop_feedback(theta,pqr);
	
	InLoopLQRStep();
}

void PID_control(const double theta[3], const double pqr[3])
{
    in_loop_feedback(theta, pqr);    
    //roll_out rad
    outputs[1] = PID_step(roll_commend,roll_feedback,in_loop_roll_PID,&roll_limits);
    //pitch_out rad
    outputs[2] = PID_step(pitch_commend,pitch_feedback,in_loop_pitch_PID,&pitch_limits);
}

