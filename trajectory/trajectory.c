#include "includs.h"

//int radius = 25;//�뾶��ʼ��
double init_velocity_on_circle = 5.0;
//int num = 0, speed_count = 0;
//double *ps_velocity = NULL;
int num = 0;
int first_entry_circle = 1;
bool turn_accel_flag = false;
bool circle_finish_flag = false;
int turn_accel=0;
bool Mixture_to_Pointtopoint_flag = false;
//static int circle_count = 0;

void Position::reset()
{
    for(int i =0 ; i!=3; ++i) 
      current_target[i] = current_xyz[i];
    heading = ahrs_theta[2];
}

void Position::current_target_update(double * next_target)
{
    for(int i=0 ; i!=3 ; ++i)
       current_target[i] = next_target[i];
}

//----------------------------------------

void Trajectory::reset()
{

    heli_step = 0;
    
    end_heading = ahrs_theta[2];
    hover_time = FLY_TIME;
    Isturn = 0;
    num = 1;
    
    position_t->reset();
    target_queue_t->reset();
    for(int i =0 ; i!= 3 ;  ++i)
    {
        hover_xyz[i] = position_t->current_xyz[i];
        next_target[i] = hover_xyz[i]; 
        com_position[i] = 0.0;
        com_velocity[i] = 0.0;
        com_pqr[i]      = 0.0;
        com_theta[i]    = 0.0;
        
        top_speed[i] = 0.0;
        how_long[i] = 0.0;
    }
  
}

/****************************************************************
 *������: ahrs_adjust(double *theta,double *angle_offset)
 *
 *����: double *thetaΪ������3����̬��
 *		double *angle_offsetΪ�����Ƕ�
 *      	
 *���ܣ���̬��ˮƽ�����
 *				
 *����ֵ������
 *
 *��    �ߣ������
 *�������: 2013-4-1
 ****************************************************************/
void ahrs_adjust(double *theta,double *angle_offset)
{
	for (int i = 0;i < 3;i++)
	{	
		angle_offset[i] = angle_offset[i]*C_DEG2RAD;
	}
	for (int j = 0;j < 3;j++)
	{
		theta[j] -= angle_offset[j];
	}
}

bool Trajectory::target_update(int trajectory_mod)
{  
    if (trajectory_mod == Point_to_point)
	{
		if (hover_time <=-1 && (!target_queue_t->is_queue_empty()) || first_receive_target == 1)
    	{
        	if(!target_queue_t->pop_target(next_target))
            	return false;

        	first_receive_target++;
        	return true;
    	}
		if (target_queue_t->is_queue_empty() && hover_time <= -2000)
      	//��ͣʱ��������ȴ���1����Ŀ����л��ǿ� 
    	{ 
        	next_target[0] = hover_xyz[0] ;
        	next_target[1] = hover_xyz[1] ;
        	next_target[2] = hover_xyz[2] ;
        	hover_time = DEFAULT_HOVER_TIEM ;
        	return true;
    	}
	}
	
    if (trajectory_mod == Circle)
	{
		if ((!target_queue_t->is_queue_empty()) && first_receive_target == 1)
    	{
        	if(!target_queue_t->pop_target(next_target))
            	return false;

        	first_receive_target++;
        	return true;
    	}
		if ((!target_queue_t->is_queue_empty()) 
			&& circle_finish_flag == true 
			&& hover_time <=-1 )
    	{
        	if(!target_queue_t->pop_target(next_target))
            	return false;

        	circle_finish_flag = false;
        	return true;
    	}		
/*		
		if (target_queue_t->is_queue_empty() && hover_time <= -2000)
      	//��ͣʱ��������ȴ���1����Ŀ����л��ǿ� 
    	{ 
        	next_target[0] = hover_xyz[0] ;
        	next_target[1] = hover_xyz[1] ;
        	next_target[2] = hover_xyz[2] ;
        	hover_time = DEFAULT_HOVER_TIEM ;
        	return true;
    	}
*/
	}
	
	if (trajectory_mod == Mixture)
	{
		if (circle_finish_flag == true)
		{	
			for (int i=0; i!=5; i++)
			{
				next_target[i] = next_next_target[i];
			}
			if(!target_queue_t->pop_target(next_next_target))
			{
				heli_step = 8;
				return false;
			}
			
			return true;
		}
		if ((!target_queue_t->is_queue_empty()) && first_receive_target == 1)
		{
			if(!target_queue_t->pop_target(next_target))
            	return false;
			
			if(!target_queue_t->pop_target(next_next_target))
				return false;
			
			first_receive_target++;
			return true;
		}
	}
	

	
    return false;
}

//  �ж�Ŀ��������ڷɻ����ʲôλ�ã�ȷ��תͷĿ��Ƕ�--end_heading��תͷ����--ISturn 
// end_heading_point �ɻ���Ҫת�����ĽǶȣ�
//n_target ��һ��Ŀ��㣻current_heading ��ǰ�ɻ����������;current_xyz ��ǰλ��
void Trajectory::turn_heading(double * end_heading_point)
{
   double temp = atan2(next_target[0]-position_t->current_xyz[0], 
                       next_target[1]-position_t->current_xyz[1]);
   // atan2����ĽǶ�����ͨ����ϵX����нǣ���end_heading�Ƕ���NED����ϵ�ļн�
	if ( temp > -C_PI /2.0 )
      	*end_heading_point = C_PI /2.0 - temp ;
    else 
      	*end_heading_point = - 3.0 * C_PI /2.0 - temp ;
	
   	if ( *end_heading_point - position_t->heading > C_PI )
	{   
      Isturn = 1;     //��ת
	  return;
	}
   	if ( *end_heading_point - position_t->heading < -C_PI )
	{
      Isturn = 2;     //��ת
	  return;
	}
   	if (*end_heading_point - position_t->heading > 0.00 
	   	&& *end_heading_point - position_t->heading < C_PI)
	{
      Isturn = 3;     //��ת
	  return;
	}
   	if (*end_heading_point - position_t->heading < 0.00 
		&& *end_heading_point - position_t->heading > -C_PI)
	{
      Isturn = 4;    //��ת
	  return;
	}
}
//--------------------------------syc
void Trajectory::distance_calculate(int trajectory_mod)
{
     if (trajectory_mod == Point_to_point || trajectory_mod == Circle)
	 {
	 	how_long[0] =sqrt((position_t->current_xyz[0]-next_target[0])*(position_t->current_xyz[0]-next_target[0]) 
                      + (position_t->current_xyz[1]-next_target[1])*(position_t->current_xyz[1]-next_target[1]));      
     	how_long[2] = hover_xyz[2]-next_target[2];
		return;
	 }
	 if (trajectory_mod == Mixture)
	 {
	 /*��������v1(x1, y1)��v2(x2, y2)�����v1��v2=y1x2-x1y2��
  	�����v1��v2��˳ʱ��ת�������Ϊ������֮Ϊ����
  	Ϊ0��ʾ���߷�����ͬ��ƽ�У���*/
		double cross_product = (next_target[0]-position_t->current_xyz[0])*(next_next_target[1]-position_t->current_xyz[1])
							   - (next_target[1]-position_t->current_xyz[1])*(next_next_target[0]-position_t->current_xyz[0]);
		if (cross_product == 0)
		{
			for (int i=0; i!=5; i++)
			{
				next_target[i] = next_next_target[i];
			}
			if(!target_queue_t->pop_target(next_next_target))
			{
				heli_step = 8;
				return;
			}
		}
		else 
		{
			if (cross_product < 0)
			{
				circle_turn = 2;	//Բ��������ʱ��ת
			}
			else
			{
				circle_turn = 1;	//Բ�����ң�˳ʱ��ת
			}
		}
			
		how_long[0] = sqrt((position_t->current_xyz[0]-next_target[0])*(position_t->current_xyz[0]-next_target[0]) 
                      + (position_t->current_xyz[1]-next_target[1])*(position_t->current_xyz[1]-next_target[1]));
		how_long[2] = hover_xyz[2]-next_target[2];
			 
		double b = sqrt((next_target[0]-next_next_target[0])*(next_target[0]-next_next_target[0]) 
                      	+ (next_target[1]-next_next_target[1])*(next_target[1]-next_next_target[1]));
		double c = sqrt((position_t->current_xyz[0]-next_next_target[0])*(position_t->current_xyz[0]-next_next_target[0]) 
                      	+ (position_t->current_xyz[1]-next_next_target[1])*(position_t->current_xyz[1]-next_next_target[1]));
		
		circle_velocity = init_velocity_on_circle;
		sector_angle = acos((pow(how_long[0],2)+pow(b,2)-pow(c,2)) / 2*abs(how_long[0]*b));
		circle_radius = 2*circle_velocity; 		 
		if (sector_angle >= 0.785 && sector_angle <= 2.356)//�Ƕ�ѡ����45-135��֮��
		{
			 double r = 0;
			 sector_angle = PI - sector_angle;
			 r = circle_radius*tan(sector_angle/2);
			 how_long[0] = how_long[0] - r;
			 next_target[0] -= r*cos(ahrs_theta[2]);
			 next_target[1] -= r*sin(ahrs_theta[2]);
			 return;
		}
		else
		{
			trajectory_mod = Point_to_point;
			Mixture_to_Pointtopoint_flag = true;
			return;
		}	 
	 }
}

void Trajectory::speed_calculate(int max_com_speed, int trajectory_mod)
{   
	 double speed_temp = -how_long[2]/15.0 ;  //15������ɴ�ֱ����
     if( speed_temp >= 0.0 )
          top_speed[2] =(speed_temp >2.0) ? 2.0 : speed_temp ;
     if( speed_temp < 0.0 )
          top_speed[2] = speed_temp <-2.0 ? -2.0 : speed_temp ;
	
     if (trajectory_mod == Point_to_point)
	 {
		   //if ( 4.0 * (MAX_COM_SPEED * MAX_COM_SPEED)/ACCEL > how_long[0] ) //�Ӽ���ռ1/4·��
           //top_speed[0] = sqrt(ACCEL * how_long[0]/4.0 );
     	if (2.0 * (max_com_speed * max_com_speed)/ACCEL > how_long[0]) //���ٶ�2
		{	
           	top_speed[0] = sqrt(ACCEL * how_long[0]/2.0 );// �Ӽ���ռ1/2·��
      /* if ( 4.0 * (MAX_COM_SPEED * MAX_COM_SPEED)/MAX_ACCEL + MAX_ACCEL*MAX_ACCEL*MAX_ACCEL/(3*a1*a1) > how_long[0] ) //�ȼ��� �Ӽ���ռ1/4·��
           top_speed[0] = sqrt(MAX_ACCEL * how_long[0]/4.0 - MAX_ACCEL*MAX_ACCEL*MAX_ACCEL*MAX_ACCEL/(12*a1*a1));*/
		}	
     	else
     	{     
           	top_speed[0] = max_com_speed;
           	top_speed[2] = -top_speed[0] * how_long[2]/how_long[0];
     	}
		return;
	 }
	 if (trajectory_mod == Circle)
	 {
		 if (0.5*(max_com_speed*max_com_speed)/ACCEL > how_long[0])
		 {
			 top_speed[0] = 0;
		 }
		 else
		 {
			 top_speed[0] = max_com_speed;
		 }
		 return;
	 }	 
	 if (trajectory_mod == Mixture)
	 {
		 if (circle_finish_flag == false)
		 {	 
		 	if (0.5*(max_com_speed*max_com_speed)/ACCEL > how_long[0])
		 	{
				top_speed[0] = 0;
		 	}
		 	else
		 	{
			 	top_speed[0] = max_com_speed;
		 	}
		 	return;
		 }
		 else
		 {
			 top_speed[0] = max_com_speed;
		 }
		
	 }	 
}



void Trajectory::schedule(int max_speed, int mode)  //��next_targe���й滮
{
     distance_calculate(mode);              //�������how_long
     speed_calculate(max_speed, mode);                //����ο�����ٶ�top_speed 

/*-----------------------------------------
     if( how_long[0] >BOUNDARY_DISTANCE )
     {
          turn_heading( &end_heading) ;
          heli_step = 2;
          return ;
      }
     if( how_long[0] <=BOUNDARY_DISTANCE  )
      {
          position_t->current_target_update(next_target);
          position_t->heading = ahrs_theta[2];
          end_heading =ahrs_theta[2]; 
          heli_step = 1;  
          return ;
      } 
------------------------------*/
}

void Trajectory::hover_step()
{
/*------------------------      
      com_position[0] = 0.0;
      com_velocity[0] = 0.0;
      com_pqr[2]  = 0.0;
      com_theta[2]  = 0.0;
--------------------------*/
	for(int i =0 ; i!= 3 ;  ++i)
    { 
        com_position[i] = 0.0;
        com_velocity[i] = 0.0;
        com_pqr[i]      = 0.0;
        com_theta[i]    = 0.0;
        
        top_speed[i] = 0.0;
        how_long[i] = 0.0;
    }
	  
      if(true == climbing_step())
     {   
         hover_time--;
         num++;
     }
     if(num >= 10000)
        num = 2000;
}
//-------------------------------syc--2012.4.13--------------------//
void Trajectory::turning_head_step()
{  	 
    if ( abs(end_heading - position.heading) < 0.04 )  //0.04rad��ӦΪ2.3deg 
    {
        Isturn = 0 ;
        position_t->heading = end_heading;
        turn_accel_flag=true;
		return;
    }
	 
	if ( Isturn == 1 || Isturn ==4 )
    { //��ת
    	if ( position_t->heading < - C_PI )
            position_t->heading +=2* C_PI;
        position_t->heading -= 0.015 ; //by syc
        //position_t->heading -= 0.015 ; 
		return;
    }
    if ( Isturn == 2 || Isturn ==3 )
    { //��ת
       	if ( position_t->heading > C_PI )
            position_t->heading -=2* C_PI;
       	position_t->heading += 0.015 ;
		return;
    }
/*-----------------------------
     if(turn_accel_flag==true)
     {
       turn_accel++;
     }
     if(turn_accel==90)//3��֮�� ǰ��
     {
      turn_accel=0;
      turn_accel_flag=false;
      heli_step = 3;
      position_t->current_target_update(next_target);      
     }
------------------------------*/  
}

void Trajectory::speed_up_step()
{
        /*if ( ACCEL <= MAX_ACCEL)
         ACCEL += a1*dt;
         else
         ACCEL -= a1*dt; */

     if (com_velocity[0] < top_speed[0])
         com_velocity[0] +=  dt * ACCEL ; 
     else
         com_velocity[0] -=  dt * ACCEL ; 
     how_long[0] -= com_velocity[0]*dt; 
      //��GPS�õ���ʵʱλ������Ŀ�����Ϊԭ�㣬��ȷ���滮λ��ʱҲӦ��Ŀ�����Ϊԭ�㣬10.11.03
      //********************************************
     com_position[0]= -how_long[0];
     //*********************************************
     climbing_step();
     
#ifdef circle_test
     if(com_velocity[0] >= 0.5)
     {   
         heli_step = 9;
         return ;
     }
#endif
/*----------------------------
     if ( how_long[0] <= 0.5 * ( top_speed[0] * top_speed[0] )/ACCEL ) **��2.0��Ϊ0.5,0,75****************
     {    
         heli_step = 4;
     }

-----------------------------*/
}

void Trajectory::speed_down_step()
{
    /*if(com_velocity[0] <= MAX_ACCEL*MAX_ACCEL/(2*a1))
        ACCEL -= dt * a1;
	if(ACCEL <= 0)
	ACCEL = 0;
    */
    if ( com_velocity[0] > 0) 
         com_velocity[0] -=  dt *ACCEL ; 
    else
          how_long[0] -= dt; //????
    how_long[0] -= com_velocity[0]*dt;
     //******************10.11.03*******************
    com_position[0]= -how_long[0];
      //*********************************************
    climbing_step();
/*------------------------
    if ( how_long[0] <=0.0)
	{
        how_long[0] = 0.0 ;
        heli_step = 1;
    }
---------------------------*/     
}

void Trajectory::brake_down_step()  // �ɵ���վ�ġ�ȡ��Ŀ����С������
{
    hover_time = DEFAULT_HOVER_TIEM;
    
    if ( com_velocity[0] > 0)
	{ 
		for(int i =0 ; i!= 3 ;  ++i)
       { 
        	com_position[i] = 0.0;
        	com_pqr[i]      = 0.0;
        	com_theta[i]    = 0.0;       
        	top_speed[i] 	= 0.0;
        	how_long[i] 	= 0.0;
       }
        com_velocity[0] -=  dt *ACCEL ;  // ɲ������ֻ���ٶȽ��й滮
        heli_step = 8;
    }
    else
	{
        com_velocity[0] = 0;
        heli_step = 1;
    }
    position_t->current_target_update(position_t->current_xyz);
    position_t->heading = ahrs_theta[2];
}
//------------------------------syc------------------//
/*bool Trajectory::climbing_step()
{  
    if(abs(how_long[2]) >2)
    {
	if (  com_velocity[2]  < top_speed[2])
              com_velocity[2] +=  dt * ACCEL *0.5; 
        else
              com_velocity[2] -=  dt * ACCEL *0.5; 
        how_long[2] += com_velocity[2]*dt;
    }
    
    com_position[2] = how_long[2] ;
    if(abs(how_long[2]) <= 2)
    {   
        how_long[2] = 0.0 ;
        top_speed[2] = 0.0 ;
        
        if (  com_velocity[2]  < top_speed[2])
              com_velocity[2] +=  dt * ACCEL *0.5; 
        else
              com_velocity[2] -=  dt * ACCEL *0.5; 
        
        hover_xyz[2] = next_target[2];
        return true ;
    }
    return false ;
}*/

bool Trajectory::climbing_step()
{
    com_position[2] = how_long[2] ;
    com_velocity[2] = top_speed[2] ;
    
    if(abs(how_long[2]) > 2)
	{
        how_long[2] += com_velocity[2]*dt;
		return false;
	}
  
    if(abs(how_long[2]) <= 2)
    {   
        how_long[2] = 0.0 ;
        top_speed[2] = 0.0 ;
        hover_xyz[2] = next_target[2];
        return true;
    }
    return false;
}
/****************************************************************
 *������: void Trajectory::circle_step(double _circle_velocity, int _radius,
 *							 		   double _helix_velocity, double _angle)
 *����: circle_velocityΪԲ�����ٶ�
 *      radiusΪԲ�ܰ뾶(�Ժ�������������ꡢԲ���Ƕȣ�   
 *		climbing_velocityΪ��ֱ�����ٶȣ���ֵ���ϣ�����������
 *		angleΪ��Բ�������νǶ�
 *
 *���ܣ��ȼ��ٲ����ٵ�circle_velocity
 *		Ȼ�����ROLL��Ǻ�PITCH���ٶ�����Բ��
 *		�뾶Ϊradius,��ʼ�ٶ�Ϊinit_velocity_on_circle
 *		���ٶ�Ϊinit_velocity_on_circle/radius����
 *      circle_turn = 1ʱ˳ʱ������(��ͷ����)
 *		circle_turn = 2ʱ��ʱ������(��ͷ����)
 *
 *����ֵ������
 *
 *��    �ߣ������
 *�������: 2012-11-15
 *
 *ԭ �� �ߣ���ԥ��
 *������ڣ�2012-7-21
 *
 ****************************************************************/
void Trajectory::circle_step(double _circle_velocity, int _radius,
							 double _helix_velocity, double _angle)

{
	static double init_theta,init_position[2],circle_angle;

	if (first_entry_circle == 1)
    {
		com_position[0] = 0.0;
		com_position[1] = 0.0;
		com_position[2] = 0.0;
		com_velocity[2] = _helix_velocity;
        
        com_velocity[0] = _circle_velocity; //���ٶȳ�ʼ��
        com_pqr[2] = com_velocity[0]/_radius;       //������ٶȳ�ʼ��
        position_t->heading = ahrs_theta[2];	  //����ǳ�ʼ��

		circle_angle = 0.0;                    //Բ��ת�ǣ����ν�
        init_theta = ahrs_theta[2];           //��ʼ�����     
        init_position[0] = position_t->current_xyz[0]; //��ʼλ��
        init_position[1] = position_t->current_xyz[1];
        position_t->current_target[2] = next_target[2];
    }

		circle_angle += com_pqr[2]*dt;	
		if (circle_angle >= _angle)
		{
			for(int i =0 ; i!= 3 ;  ++i)
    		{ 
        		com_position[i] = 0.0;
        		com_pqr[i]      = 0.0; 
        		com_theta[i]    = 0.0;
        
        		top_speed[i] = 0.0;
        		how_long[i] = 0.0;
    		}
			circle_finish_flag = true;
			first_entry_circle = 1;
			heli_step = 8;//Բ�����������û�и����¸�Ŀ��㣬��Ĭ��ɲ��ģʽ��
			return;
		}
		
		if (circle_turn == 1)//˳ʱ��ת
		{
			position_t->heading += com_pqr[2]*dt;
			if (position_t->heading > PI)
			{
        		position_t->heading -= 2*PI;
			}
			position_t->current_target[0] = init_position[0]+_radius*(-sin(init_theta)+sin(position_t->heading));
    		position_t->current_target[1] = init_position[1]+_radius*(cos(init_theta)-cos(position_t->heading));		
        	
			//com_theta[0] = atan2((com_velocity[0]*com_pqr[2]),accel_g);//roll��Բ���		
		}
		if (circle_turn == 2)//��ʱ��ת
		{
			position_t->heading -= com_pqr[2]*dt;
			if (position_t->heading < -PI)
			{
        		position_t->heading += 2*PI;
			}
			position_t->current_target[0] = init_position[0]+_radius*(sin(init_theta)-sin(position_t->heading));
    		position_t->current_target[1] = init_position[1]+_radius*(-cos(init_theta)+cos(position_t->heading));		
		
			//com_theta[0] = -atan2((com_velocity[0]*com_pqr[2]),accel_g);//roll��Բ���
		}		
    	
		position_t->current_target[2] -= com_velocity[2]*dt ;
		//com_pqr[1] = com_pqr[2]*cos(com_theta[0]);//pitch��Բ���ٶ�
	
	first_entry_circle ++;
    if(first_entry_circle >= 10000)
      first_entry_circle = 200;
}
/****************************************************************
 *�� �� ��: void Trajectory::speed_up_and_down_step(double *velocity_array)
 *��    ��: *velocity_array  ָ��滮ֱ�߼Ӽ�������������ͷ��ַ��ԭ��Ĭ��Ϊ0
 *                 
 *
 *��    �ܣ�֧��·���滮ֱ�߼Ӽ��٣����ļ�ȫ���������speed_up_down_velocity
 *			Ϊ�������ʼ��ϣ�ԭ����յ�Ĭ��Ϊ��
 *                  
 *
 *�� �� ֵ: ����
 *                
 *
 *��    �ߣ������
 *������ڣ�2012-11-15
 ****************************************************************/
/*
void Trajectory::speed_up_and_down_step(double *velocity_array)
{
	//double velocity1 = 0, velocity2 = 0;
	
	ps_velocity = velocity_array;
    
	if (num == speed_count)
	{
		if (*ps_velocity < *(ps_velocity+1))
		{
			how_long[0] = 0.25*(*(ps_velocity+1)-*ps_velocity)
							  *(*(ps_velocity+1)-*ps_velocity);
    		speed_count += (*(ps_velocity+1)-*ps_velocity)*30/ACCEL;
		}
		else
		{
			how_long[0] = 0.25*(*ps_velocity-*(ps_velocity+1))
							  *(*ps_velocity-*(ps_velocity+1));
    		speed_count += (*ps_velocity-*(ps_velocity+1))*30/ACCEL;
			
		}
	   
    position_t->current_target[0] += how_long[0]*cos(position_t->heading);
	position_t->current_target[1] += how_long[0]*sin(position_t->heading);
	position_t->current_target[2] = hover_xyz[2];
	
	ps_velocity++;
	}
	
    if (num < speed_count)
    {   
		//velocity1 = (*ps_velocity) > *(ps_velocity-1) ? *ps_velocity : *(ps_velocity-1);//High value speed
		//velocity2 = (*ps_velocity) < *(ps_velocity-1) ? *ps_velocity : *(ps_velocity-1);//low value speed
        
        if (com_velocity[0] < *ps_velocity)
		{
			com_velocity[0] += ACCEL*dt;
		}
		else
		{
         	com_velocity[0] -= ACCEL*dt;	
		}
		
		
     	how_long[0] -= com_velocity[0]*dt; 
	    com_position[0]= -how_long[0];
		climbing_step();
	}	
	num++;     
}
*/
void Trajectory::step(int trajectory_mode)
{
    switch (trajectory_mode)
	{
		case Hover : Hover_Mode_Step();//��ͣģʽ
					 break;
				 
		case Point_to_point : Point_to_point_Mode_Step();//��Ե�ģʽ
							  break;
				 
		case Circle : Circle_Mode_Step();//��Բģʽ
					  break;
				 
		case Mixture : Mixture_Mode_Step();//���ģʽ
			 		   break; 
				 
		case Test : Test_Mode_Step();//����ģʽ
			 		break;
				 
		default : brake_down_step();	//ɲ��
				  break;
		
	}
}

void Trajectory::Hover_Mode_Step()
{
   	if (heli_step == 1) 
      	hover_step();
	if (hover_time <= 0)
		hover_time = FLY_TIME;
}

void Trajectory::Point_to_point_Mode_Step()
{
	if (target_update(Point_to_point))					// �ж��Ƿ����next_target
	{	
    	schedule(MAX_COM_SPEED, Point_to_point);		// ��next_target���й滮
		if (how_long[0] > BOUNDARY_DISTANCE)
     	{
          	turn_heading(&end_heading);
          	heli_step = 2;
      	}
     	if (how_long[0] <= BOUNDARY_DISTANCE)
      	{
          	position_t->current_target_update(next_target);
          	position_t->heading = ahrs_theta[2];
          	end_heading = ahrs_theta[2]; 
          	heli_step = 1;
      	} 
	}	
   	if (heli_step == 1)
	{	
      	hover_step();//��ͣ
	}
   	if (heli_step == 2)
	{
      	turning_head_step();   //ת��
		
		if(turn_accel_flag==true)
     	{
       		turn_accel++;
     	}
		if(turn_accel==90)//3��֮�� ǰ��
     	{
      		turn_accel=0;
      		turn_accel_flag=false;
      		heli_step = 3;
      		position_t->current_target_update(next_target);      
     	}
	}
   	if (heli_step == 3)
	{
      	speed_up_step();        //����
		if (how_long[0] <= 0.5*(top_speed[0]*top_speed[0])/ACCEL) //**��2.0��Ϊ0.5,0,75****************
     	  	heli_step = 4;
	}
   	if (heli_step == 4)
	{
      	speed_down_step();      //����
	    if (how_long[0] <= 0.0)
		{
        	how_long[0] = 0.0 ;
        	heli_step = 1;
    	}
	}
	if (heli_step == 8)
	{	
		brake_down_step();
	}
}

void Trajectory::Circle_Mode_Step()
{
	if (target_update(Circle))
	{	
    	schedule(init_velocity_on_circle, Circle);
		if (how_long[0] > 0.5*init_velocity_on_circle*init_velocity_on_circle/ACCEL)
     	{
          	turn_heading(&end_heading);
          	heli_step = 2;
      	}
     	else
      	{
          	position_t->current_target_update(hover_xyz);
          	position_t->heading = ahrs_theta[2];
          	end_heading = ahrs_theta[2]; 
          	heli_step = 1;  
      	}
		
	}
	if (heli_step == 1)
	{
      	hover_step();
    }
	if (heli_step == 2)
	{
      	turning_head_step();   //ת��
		
		if(turn_accel_flag==true)
     	{
       		turn_accel++;
     	}
		if(turn_accel==90)//3��֮�� ǰ��
     	{
      		turn_accel=0;
      		turn_accel_flag=false;
      		heli_step = 3;
      		position_t->current_target_update(next_target);      
     	}
	}
	if (heli_step == 3)
	{
      	speed_up_step();        //����
		if (how_long[0] <= 0.0)
		{
        	how_long[0] = 0.0;
			heli_step = 9;
			circle_finish_flag = false;
		}
	}
	if (heli_step == 9 && circle_finish_flag == false)
	{
		circle_turn = 1;
		//circle_step(5.0, 25, 0.0, 15*PI);//����by wybb
		circle_step(5.0, 25, 0.0, 1*PI);//����by wybb
		return;
	}
    if (heli_step == 8)
	{	
		brake_down_step();
	}
}

void Trajectory::Mixture_Mode_Step()
{
	if (target_update(Mixture))
	{	
    	schedule(init_velocity_on_circle, Mixture);
		if (Mixture_to_Pointtopoint_flag == true)
		{
			Mixture_to_Pointtopoint_flag = false;
			if (how_long[0] > BOUNDARY_DISTANCE)
     		{
          	turn_heading(&end_heading);
          	heli_step = 2;
      		}
     		if (how_long[0] <= BOUNDARY_DISTANCE)
      		{
          	position_t->current_target_update(next_target);
          	position_t->heading = ahrs_theta[2];
          	end_heading = ahrs_theta[2]; 
          	heli_step = 1;  
      		}
			return;
		}
		if (circle_finish_flag == true)
		{
			circle_finish_flag = false;
			position_t->current_target_update(next_target);
			position_t->heading = ahrs_theta[2];
			heli_step = 3;
		}
		if (how_long[0] > 0.5*init_velocity_on_circle*init_velocity_on_circle/ACCEL
			&& circle_finish_flag == false)
     	{
          	turn_heading(&end_heading);
          	heli_step = 2;
      	}
     	else
      	{
          	position_t->current_target_update(hover_xyz);
          	position_t->heading = ahrs_theta[2];
          	end_heading = ahrs_theta[2]; 
          	heli_step = 1;  
      	}
	}
	if (heli_step == 1)
	{
      	hover_step();
    }
	if (heli_step == 2)
	{
      	turning_head_step();   //ת��
		
		if(turn_accel_flag==true)
     	{
       		turn_accel++;
     	}
		if(turn_accel==90)//3��֮�� ǰ��
     	{
      		turn_accel=0;
      		turn_accel_flag=false;
      		heli_step = 3;
      		position_t->current_target_update(next_target);      
     	}
	}
	if (heli_step == 3)
	{
      	speed_up_step();        //����
		if (how_long[0] <= 0.0)
		{
        	how_long[0] = 0.0;
			heli_step = 9;
		}
	}
	if (heli_step == 9 && circle_finish_flag == false)
	{
		circle_step(circle_velocity, circle_radius, helix_velocity, sector_angle); //����by wybb	
		return;
	}
    if (heli_step == 8)
	{	
		brake_down_step();
	}
}

void Trajectory::Test_Mode_Step()
{
	if (heli_step == 1)
	{
		hover_step();	
		if (num == 300)
    	{
    		com_theta[0]  = 0.035;//roll
        	com_theta[1]  = -0.035;//pitch
    	}
    	if (num == 330)
    	{
      		com_theta[0]  = 0;
      		com_theta[1]  = 0;
    	}
    	if (num == 630)
    	{
      		com_theta[0]  = -0.035;//roll
      		com_theta[1]  = 0.035;//pitch
    	}
    	if (num == 660)
    	{
      		com_theta[0]  = 0;
      		com_theta[1]  = 0;
    	}
	}

}

