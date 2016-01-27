#include "target.h"
#include "includs.h"

//--Target_Command实现--------------//
//void get_command(char * line) :读取地面站发来的目标指令
//void reset_command():每次push完一次command后，清除该组
//----------------------------------
void  Target_Command::get_command(char * line)
{
  int i =0,j=0;
  for(i =0 ; i!= MAX_COMMAND_SIZE; ++i)
  {
       for(j =0 ; j!=4 ; ++j)
	   {
         target[i][j] = strtod(line,&line);
         line++;
       }
       target[i][2] = -1*target[i][2];//syc 高度转成NED向下为正坐标值
       if( -1 == target[i][3])
           break;
  }
  count = i ;
}

void Target_Command::reset()
{
  for(int i =0 ; i!= MAX_COMMAND_SIZE; ++i)
  {
     for(int j =0 ; j!=3 ; ++j)
       target[i][j] = 0.0;
     target[i][3] = -1;
  }
  count = 0 ;
}

//----------Target_Queue的实现--------//
void Target_Queue::reset(){
  index = 0;  //queue中等待执信的目标个数
  for(int i =0 ; i!=MAX_QUEUE_SIZE; ++i)
  {  
    for(int j =0 ; j!=3 ; ++j)
        queue[i][j] =0;   
    queue[i][3] = -1 ;
  }
}

bool Target_Queue::push_target(Target_Command * target_c){
    bool brake_flag = true;
    for( int i =0 ; target_c->count >0 && index != MAX_QUEUE_SIZE; ++i)
    {  
        for(int j=0; j!=4;++j)
           queue[index][j]=target_c->target[i][j];        
        target_c->count--;   
        index ++ ;
        if(-2 == queue[i][3])
        {  
            reset();
            brake_flag = false;
            break;
        }  
    }
    target_c->reset();   
    return brake_flag; 
}

bool Target_Queue::pop_target(double * target_p)
{
  if(index <= 0)
    return false ;   //没有目标点了
  for(int i =0 ; i!=3 ; ++i)
    *(target_p+i)=  queue[0][i]+trajectory.hover_xyz[i];
  trajectory.hover_time =  queue[0][3]*30;  
  
  for(int i =0 ; i !=MAX_QUEUE_SIZE-1 ; ++i )
    for(int j =0 ; j!= 4 ; ++j)
       queue[i][j] =  queue[i+1][j];
  
  queue[MAX_QUEUE_SIZE-1][3] = -1;
  index--;
  return true;
  
} 

bool Target_Queue::is_queue_empty(){
  return index<=0? true:false;
}

/*bool target_monitor(void)
{
   if(target[0][0] >= 500 || target[0][0] <=-500)
     return false ;
   
   if(target[0][1] >= 500 || target[0][1] <=-500)
     return false ;
   
   if(target[0][2] > 100 || target[0][2] <-80)
     return false ;
   
   if(target[0][3] >= 150 || target[0][3] <=0)
     return false ;

   return true;
  
}
*/

