#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#define FLY_TIME            16000    
#define ACCEL               2.0
#define MAX_COM_SPEED       20.0
#define BOUNDARY_DISTANCE   8.0
#define DEFAULT_HOVER_TIEM  450

extern double init_velocity_on_circle ;

enum  Trajectory_mod
{
	Hover,                      //0
	Point_to_point,				//1
	Circle,						//2
	Mixture,					//3
	Test,					    //4
	False
};

struct Position
{
	void      reset();
	void      current_target_update(double next_target[3]);
                            
	double current_xyz[3];   //�ɻ�λ�á��ٶȣ�NED���ͺ���
	double current_uvw[3];
              
    double raw_input_uvw[3];
    double raw_current_xyz[3];   
	double raw_current_uvw[3];  
	//----syc
	 double first_raw_current_xyz[3];   
	double first_raw_current_uvw[3]; 
        
	double current_target[3];   //����״̬�͵�ǰĿ���
	double heading ;            //��ǰ����Ŀ��Ƕ�
};

struct Trajectory
{
	public:
//--------------������-----------------//		
	  void          step(int mode);
//-------------ģʽ����----------------//
	  void   		Hover_Mode_Step();
	  void			Point_to_point_Mode_Step();
	  void			Circle_Mode_Step();
	  void			Mixture_Mode_Step();
	  void			Test_Mode_Step();
//-------------�ײ㺯��----------------//     
      void          schedule(int max_speed, int mode);
      bool          target_update(int mode);
      void          distance_calculate(int mode); 
      void          speed_calculate(int max_com_speed, int trajectory_mod); 
      void          turn_heading(double *end_heading_point);
	  void          reset();
      void          hover_step();                       //heli_step = 1
      void          turning_head_step();                //heli_step = 2
      void          speed_up_step();					//heli_step = 3
      void          speed_down_step();					//heli_step = 4
      void          brake_down_step();					//heli_step = 8
      bool          climbing_step();
      void          speed_up_and_down_step(double *velocity_array);
      void          circle_step(double circle_velocity, int radius,
								double helix_velocity, double angle);//heli_step = 9
	  
	public:
      double        top_speed[3];
      double        how_long[3] ;
      
      Target_Queue* target_queue_t;
      Position  *   position_t;
      
      double        next_target[4];
      double        end_heading;
      double        hover_time;
      double        hover_xyz[3];      //�滮��������,�����ڻ����⻷����
      double        com_velocity[3];
      double        com_position[3];
      double        com_pqr[3];
      double        com_theta[3];
      int           heli_step;
      int           Isturn; 			
      double        yaw_com_theta;
	  enum Trajectory_mod trajectory_mod;
//-----------circle---------------------//	  
	  double   		sector_angle;    //Բ�����νǶ�
	  double   		circle_velocity; //Բ�����ٶ�
	  double 		helix_velocity;  //�����ߴ�ֱ�����ٶ�
	  double		circle_radius;	 //Բ�ܰ뾶
	  double 		next_next_target[4];
	  int         	circle_turn;
};

#endif
