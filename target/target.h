#ifndef _TARGET_H
#define _TARGET_H

#define MAX_QUEUE_SIZE 20
#define MAX_COMMAND_SIZE 6

#include <stdlib.h> 
struct Target_Command{
	void   get_command(char * target_p);
        void   reset();
	double target[MAX_COMMAND_SIZE][4];
	int	   count;
};

struct Target_Queue{	
	bool 	push_target(Target_Command * target_c);
	bool	pop_target(double * target_p);	
	void    reset();
        bool    is_queue_empty();

private:
	double 	        queue[MAX_QUEUE_SIZE][4];
	int  	index;
};
 

extern struct Target_Command target_command;
extern struct Target_Queue   target_queue;

#endif
