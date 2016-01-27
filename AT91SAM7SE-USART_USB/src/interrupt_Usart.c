#include<includs.h>

	
AT91PS_USART COM0;
AT91PS_USART COM1;

//extern unsigned int imu_count;
//extern volatile  bool compass_update_flag ;

unsigned char AVR_PCM[PCM_NUM];
unsigned char raw_AVR_PCM[PCM_NUM];
unsigned char PCM_buffer[PCM_BUF_NUM] ;//11.01

unsigned char last_char;
unsigned char new_char;


//int round_count;
//int gps_len=0;
char groundtoarm[UPLOAD_NUM];

char groundtoarm_flag=0;

char hmc_save_line[GPS_BUF_NUM];
volatile bool hmc_update_flag = false  ;


 
char put2ground[400]="$DATA";  //********数组长度要足够大，防止指针越界
bool to_ground_transmit_flag=true;


volatile bool gga_update_flag = false  ;
volatile bool vtg_update_flag = false ;

char GPS_buffer[GPS_BUF_NUM] ;
char gga_save_line[GPS_BUF_NUM];
char vtg_save_line[GPS_BUF_NUM];
//extern inline void uncharcpy(unsigned char* des,unsigned char* src, int num);
	
//*------------------------- Interrupt Function -------------------------------

//*----------------------------------------------------------------------------
//* Function Name       : Usart_c_irq_handler
//* Object              : C handler interrupt function called by the interrupts
//*                       assembling routine
//*----------------------------------------------------------------------------
void Usart_c_irq_handler(void)
{
	AT91PS_USART USART_pt = COM0;
	unsigned int status;

	//* get Usart status register and active interrupt
	status = USART_pt->US_CSR ;
        status &= USART_pt->US_IMR;

	if ( status & AT91C_US_RXBUFF)
	{
	    if(*groundtoarm=='$')
            {
 	     COM0->US_RPR = (unsigned int) groundtoarm;
	     COM0->US_RCR = UPLOAD_NUM;
             groundtoarm_flag=1;
            }
            else
            {
 	     COM0->US_RPR = (unsigned int) groundtoarm;
	     COM0->US_RCR = 1;
            } 
            
	}
        
        if ( status & AT91C_US_TXBUFE)
	{ 
          AT91F_US_DisableIt(AT91C_BASE_US0,AT91C_US_TXBUFE  );
          to_ground_transmit_flag=true;
          AT91C_BASE_US0->US_TPR = (unsigned int)NULL;
          AT91C_BASE_US0->US_TCR = 0;
         
	}
   //* Reset the satus bit for error
	 USART_pt->US_CR = AT91C_US_RSTSTA;
       // AT91F_US_DisableIt(COM0,AT91C_US_RXBUFF  );
}


//*-------------------------- External Function -------------------------------
//    receive the PCM
//*---------------------------------------------------------------------------
void Usart1_c_irq_handler(void)
{
	AT91PS_USART USART_pt = COM1;
	unsigned int status;

	// get Usart status register and active interrupt
	    status = USART_pt->US_CSR ;
        status &= USART_pt->US_IMR;
        static unsigned int pcm_index = 0;
        static unsigned char newchar = 'A';
        static unsigned char lastchar = 'B';
        unsigned char r = '\r' , n = '\n';
        
        if ( status & AT91C_US_RXRDY)
          {	
            
             lastchar = newchar;
             newchar = AT91C_BASE_US1->US_RHR;
             
             PCM_buffer[pcm_index] = newchar;  
         
            if((lastchar == r && newchar != n) ||( newchar == n && lastchar != r ) )
            {
                   pcm_index = 0;
                   memset(PCM_buffer,'\0',PCM_BUF_NUM);
                   USART_pt->US_CR = AT91C_US_RSTSTA;
                  return;   
             }
                 
            if(pcm_index >= PCM_BUF_NUM-10)   //不能超过一定长度,防止溢出出错
            {
                pcm_index = 0;
                memset(PCM_buffer,'\0', PCM_BUF_NUM);
                USART_pt->US_CR = AT91C_US_RSTSTA;
                 //AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
                return;
            }  
                    
            if(lastchar == r && newchar == n)//handle a line end with \r\n
             { 
                    PCM_buffer[pcm_index+1] = '\0';
                    if(PCM_buffer[0]== '$' && PCM_buffer[1]== '$' && pcm_index==(PCM_NUM-1))
                    {
                      uncharcpy(AVR_PCM,PCM_buffer,PCM_NUM);
                      memset(PCM_buffer,'\0',PCM_BUF_NUM);
                      pcm_rece_flag = 1 ; 
                    }
                     
                     pcm_index = 0;
					 memset(PCM_buffer,'\0',PCM_BUF_NUM);
                     USART_pt->US_CR = AT91C_US_RSTSTA;
                     //AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
                    return;
            }
            pcm_index++ ;     //正常读完
                     
         }	

        
    
       //Reset the satus bit for error
	USART_pt->US_CR = AT91C_US_RSTSTA;
       // AT91F_US_DisableIt(COM1,AT91C_US_RXBUFF  );
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_US_Printk
//* \brief This function is used to send a string through the US channel
//*----------------------------------------------------------------------------
void AT91F_US_Put( char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady(COM0));
		AT91F_US_PutChar(COM0, *buffer++);
	}
}

void AT91F_US1_Put( char *buffer) // \arg pointer to a string ending by \0
{
	while(*buffer != '\0') {
		while (!AT91F_US_TxReady(COM1));
		AT91F_US_PutChar(COM1, *buffer++);
	}
}

void AT91F_DBGU_Put(char *buffer)
{
  while(*buffer != '\0'){
	//while (!((AT91PS_USART)AT91C_BASE_DBGU->DBGU_CSR & AT91C_US_TXRDY));
        while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_DBGU));
                 AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_DBGU, *buffer++);
               // AT91C_BASE_DBGU->DBGU_THR = (*buffer & 0x1FF);
	}
}
//*----------------------------------------------------------------------------
//* Function Name       : Usart_init
//* Object              : USART initialization
//* Input Parameters    : none
//* Output Parameters   : TRUE
//*----------------------------------------------------------------------------

//*---------------------------------------------------------------------------
//*   Receive Ground2Arm and Transmit DATA to GroundStation
//*---------------------------------------------------------------------------

void Usart_init ( void )
//* Begin
{
    COM0= AT91C_BASE_US0;
    //* Define RXD and TXD as peripheral
    // Configure PIO controllers to periph mode
    AT91F_PIO_CfgPeriph(
	 AT91C_BASE_PIOA, // PIO controller base address
	 ((unsigned int) AT91C_PA5_RXD0    ) |
	 ((unsigned int) AT91C_PA6_TXD0    ) , // Peripheral A
	 0 ); // Peripheral B

    //* First, enable the clock of the PIOB
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_US0 ) ;

    //* Usart Configure
    AT91F_US_Configure (COM0, AT91B_MCK,AT91C_US_ASYNC_MODE,115200 , 0);

    //* Enable usart
    COM0->US_CR = AT91C_US_TXEN |AT91C_US_RXEN;

    //* open Usart interrupt
    AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_US0, USART_INTERRUPT_LEVEL,
                           AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, Usart_c_irq_handler);
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_US0);
    // Set the PDC
    AT91F_PDC_Open (AT91C_BASE_PDC_US0);
    //* address to the next bloc to be received
    AT91C_BASE_US0->US_RPR = (unsigned int) groundtoarm;
    COM0->US_RCR = 1;//UPLOAD_NUM;//* 139个字符
    //*  and AT91C_US_ENDRX
    AT91F_US_EnableIt(COM0,AT91C_US_RXBUFF  );//* RX Buffer Full Interrupt Enable
  //  AT91F_US_EnableIt(AT91C_BASE_US0,AT91C_US_TXBUFE  );
//* End
}

//*---------------------------------------------------------------------------
//*   receive AVR_PCM
//*---------------------------------------------------------------------------
void Usart1_init ( void )
// Begin
{
    COM1= AT91C_BASE_US1;
    // Define RXD and TXD as peripheral
    // Configure PIO controllers to periph mode
    AT91F_PIO_CfgPeriph(
	 AT91C_BASE_PIOA, // PIO controller base address
	 ((unsigned int) AT91C_PA21_RXD1    ) |
	 ((unsigned int) AT91C_PA22_TXD1    ) , // Peripheral A
	 0 ); // Peripheral B

    //First, enable the clock of the PIOB
    AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_US1 ) ;

    // Usart Configure
    AT91F_US_Configure (COM1, AT91B_MCK,AT91C_US_ASYNC_MODE,19200, 0);

    // Enable usart
    COM1->US_CR = AT91C_US_RXEN | AT91C_US_TXEN;

    // open Usart interrupt
    AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_US1, USART1_INTERRUPT_LEVEL,
                           AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, Usart1_c_irq_handler);
    AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_US1);
    // Set the PDC
   // AT91F_PDC_Open (AT91C_BASE_PDC_US1);//11.01
    
    AT91F_US_EnableIt(COM1,AT91C_US_RXRDY);//RX Buffer Full Interrupt Enable
    
// End
}

//*---------------------------------------------------------------------------
//* receive the GPS 
//*---------------------------------------------------------------------------

/********************************************************
* 函数说明： * debug接口中断处理函数
               接受来自debug的gps字符，并做字头、字尾、逗号判断。
               接收完一串完整gga后置位gga_update_flag，接收到完整vtg后置位vtg_update_flag
* 入口参数： * 无
* 返回参数： * 无
* 输入数据示例： $GPGGA,075402.00,2302.4770,N,11321.1163,E,2,09,0.9,6.30,M,-6.50,M,02,AAAA*70
                 $GPVTG,133.955,T,133.955,M,0.047,N,0.087,K,D*2A
* 最后修改： * 陈勇 2009-03-24 
* 备    注： * 无
********************************************************/
 
void Debug_irq_handler(void)
{

    static unsigned char gps_index = 0;
    static unsigned char new_char = 'A';
    static unsigned char last_char = 'B';
    unsigned char r = '\r' , n = '\n';//***********注意,实验室实验用r&n
    unsigned int status;
    status  = AT91C_BASE_DBGU->DBGU_CSR ;
    status &= AT91C_BASE_DBGU->DBGU_IMR;
	
   
    
    if ( status & AT91C_US_RXRDY)
      {	
	
         last_char = new_char;
         new_char = AT91C_BASE_DBGU->DBGU_RHR;
          
		GPS_buffer[gps_index] = new_char;  

	if((last_char == r && new_char != n) ||( new_char == n && last_char != r ) )
	{
              gps_index = 0;
	       memset(GPS_buffer,'\0', GPS_BUF_NUM);
                AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
              return;   
	 }
		
	if(gps_index >= GPS_BUF_NUM-10)   //不能超过一定长度,防止溢出出错
        {
            gps_index = 0;
            memset(GPS_buffer,'\0', GPS_BUF_NUM);
             AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
            return;
        }  
		
        if(last_char == r && new_char == n)//handle a line end with \r\n
         {  
                GPS_buffer[gps_index+1] = '\0';//2011.10.24syc
                 if( strncmp(GPS_buffer,"$GPGGA",6)== 0)//$GPGGA
                {
                     if((CharCounter(GPS_buffer,',') == 14) &&(gga_update_flag == false ))
                        {
							//if(strlen(GPS_buffer)==72||strlen(GPS_buffer)>72)
							//{
								strncpy(gga_save_line,GPS_buffer,gps_index+1);
								gga_update_flag = true ;
							//}
								memset(GPS_buffer,'\0',GPS_BUF_NUM);
                        }
                    
                }   

                if(strncmp(GPS_buffer,"$GPVTG",6) == 0)//$GPVTG
                {
                     if((CharCounter(GPS_buffer,',') == 9) && (vtg_update_flag == false) )
                       {
			 strncpy(vtg_save_line,GPS_buffer,gps_index+1);
		  	 memset(GPS_buffer,'\0',GPS_BUF_NUM);
			 vtg_update_flag = true;
                       }
                }
		if(strncmp(GPS_buffer,"$HMC",4) == 0)//$GPVTG
                {
                     if((CharCounter(GPS_buffer,',') == 3) && (hmc_update_flag == false) )
                       {
			 strncpy(hmc_save_line,GPS_buffer,gps_index+1);
		  	 memset(GPS_buffer,'\0',GPS_BUF_NUM);
			 hmc_update_flag = true;
                       }
                }
                gps_index = 0;
                 AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
                return;
        }
	gps_index++ ;     //正常读完
		 
    }	
   
	 AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RSTSTA;
} 


//*---------------------------------------------------------------------------

void AT91F_DBGU_Init(void)
{
  AT91F_DBGU_CfgPIO();
  AT91F_DBGU_CfgPMC();
  AT91F_US_Configure ((AT91PS_USART)AT91C_BASE_DBGU, AT91B_MCK,
                          AT91C_US_ASYNC_MODE,19200, 0); //bysyc 10.29 把波特率从38400改为19200
  AT91C_BASE_DBGU->DBGU_CR = AT91C_US_RXEN | AT91C_US_TXEN;

  AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_SYS, DBGU_USART_INTERRUPT_LEVEL,
                                         AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, Debug_irq_handler);
  AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_SYS);
  
  //AT91F_PDC_Open (AT91C_BASE_PDC_DBGU);// 11 01
  //AT91C_BASE_DBGU->DBGU_RPR = (unsigned int) buff_rx1;
  //AT91C_BASE_DBGU->DBGU_RCR = 8;
  AT91F_DBGU_InterruptEnable(AT91C_BASE_DBGU,AT91C_US_RXRDY );
  //AT91F_DBGU_InterruptEnable(AT91C_BASE_DBGU,AT91C_US_TXBUFE );
}


void output2ground(void)
{
  int len=0;
  if(to_ground_transmit_flag==true)
  {
//-------------------roll pitch degree-------------------  
      len += sprintf(put2ground+len,"%.2f",ahrs_theta[0]);      //1
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",ahrs_theta[1]);      //2
//------------------roll pitch degree command------------------
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",roll_commend[0]);    //3
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",pitch_commend[0]);   //4 
      
//--------------------manual roll pwm------------------------  
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%d",servo_widths_temp[1]); //5
//--------------------manual pitch pwm----------------------      
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%d",servo_widths_temp[2]); //6
      
      
////-------------------remote aux1 pwm------------------------      
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",servo_widths[7]);        
////-------------------remote aux2 pwm------------------------      
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",servo_widths[8]);        
    
//-------------------roll PD para------------------------  
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%.0f",in_loop_roll_PID[0]);//7
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%.0f",in_loop_roll_PID[2]);//8
//-------------------pitch PD para----------------------      
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%.0f",in_loop_pitch_PID[0]);//9 
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%.0f",in_loop_pitch_PID[2]); //10
    
//-------------------yaw rate------------------
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.3f",raw_ahrs_pqr[2]);      //11
//-------------------yaw rate command-------------------------    
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",yaw_commend[1]); //12      
//-------------------yaw pwm-------------------------    
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%d",servo_widths[4]);      //13
//-------------------remote gain pwm-------------------------    
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%d",servo_widths_temp[5]); //14
//-------------------yaw PI & compensaton para----------------------      
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.0f",in_loop_yaw_PID[0]);//15
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",in_loop_yaw_PID[1]); //16
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",in_loop_yaw_PID[2]); //17 
      
//------------------------altitude---------------------------      
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",bmp_height);      //18     
//-------------------remote throttle pwm--------------------   
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%d",servo_widths_temp[3]);//19
//-------------------remote coll pwm------------------------      
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%d",servo_widths_temp[6]);//20     
//-------------------remote heading pwm------------------------      
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",input_yaw);//21
//-------------------quality---------------------------------    
      put2ground[len++]=',';
      put2ground[len++]= gps_gga_quality+48; //22    
//-------------------num_sats--------------------------------     
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%d",gps_gga_num_sats);//23
//------------------------current_xyz----------------------  
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_xyz[0]);//24
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_xyz[1]);//25
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_xyz[2]);//26
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",position.current_target[0]);//27
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",position.current_target[1]);//28
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",position.current_target[2]);//29
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_uvw[0]);//30
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_uvw[1]);//31
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.2f",input_uvw[2]);//32
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",desired_x_rate);//33
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",desired_y_rate);//34
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.1f",desired_d_rate);//35
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.f",out_loop_X_PID[0]);//36
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.f",out_loop_X_PID[2]);//37
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.f",out_loop_Y_PID[0]);//38
      put2ground[len++]=',';
      len += sprintf(put2ground+len,"%.f",out_loop_Y_PID[2]);//39
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.f",out_loop_D_PID[0]); //40
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.f",out_loop_D_PID[2]); //41
         put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_xyz[0]);//27
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_xyz[1]);//28
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_xyz[2]);//29
          put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_uvw[0]);//27
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_uvw[1]);//28
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.2f",position.raw_current_uvw[2]);//29
        put2ground[len++]=',';
        len += sprintf(put2ground+len,"%.3f",lidar_lite_distance);//29
        
//   put2ground[len++]=',';
//        len += sprintf(put2ground+len,"%.2f",ahrs_compass[0]);//29
//   put2ground[len++]=',';
//        len += sprintf(put2ground+len,"%.2f",ahrs_compass[1]);//29
//   put2ground[len++]=',';
//        len += sprintf(put2ground+len,"%.2f",ahrs_compass[2]);//29
//---------------round_count---------//
    put2ground[len++]=',';
    len += sprintf(put2ground+len,"%d",round_count);

//现在len长度至少要留30个的裕度，否则实际使用可能导致越界/////
    put2ground[len++]='\r';
    put2ground[len++]='\n';
    to_ground_transmit_flag=false; 

    AT91C_BASE_US0->US_TPR = (unsigned int)put2ground;
    AT91C_BASE_US0->US_TCR = len;
    AT91F_US_EnableIt(AT91C_BASE_US0,AT91C_US_TXBUFE  );
   

  }
}


//void output2ground(void)
//{
//  char i;
//  int len=5;
//  int pcm_up_flag;
//  static int err_count=0;
//  if(to_ground_transmit_flag==true)
//  {
// //-------------------degree------------------------------------  
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.3f",ahrs_theta[0]);
//	  //len += sprintf(put2ground+len,"%.3f",roll_commend[0]);
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.3f",ahrs_theta[1]);
//	  //len += sprintf(put2ground+len,"%.3f",pitch_commend[0]);
//      put2ground[len++]=',';
//3      len += sprintf(put2ground+len,"%.3f",ahrs_theta[2]);
//  //------------------------roll pitch yaw commend------------
// 
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.3f",roll_commend[0]);
//
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.3f",pitch_commend[0]);
//      
//      put2ground[len++]=',';
//6      len += sprintf(put2ground+len,"%.3f",yaw_commend[0]);
//
//
// //-------------------(raw_ahrs_theta)PQR----------------------------------
//      for( i =0 ; i!=3; ++i )
//      {
//        put2ground[len++]=',';
// 9       len += sprintf(put2ground+len,"%.2f",ahrs_pqr[i]);//raw_ahrs_theta[i]
//      }
// //-------------------accel---------------------------------     
//    for( i =0 ; i!=3; ++i )
//   {
//      put2ground[len++]=',';
// 12     len += sprintf(put2ground+len,"%.2f",ahrs_accel[i]);
//   }    
// //-------------------servo_widths  mode--------------------
//   put2ground[len++]=',';
//   put2ground[len++]=control_mode+48;
//       
////-------------------manual roll pwm------------------------  
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%d",manual_servo_widths[0]);
////--------------------manual pitch pwm----------------------      
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%d",manual_servo_widths[2]);
////-------------------manual throttle pwm--------------------   
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%d",servo_widths_temp[3]);
////-------------------manual yaw pwm-------------------------    
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",servo_widths_temp[4]);
////-------------------manual coll pwm------------------------      
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",manual_servo_widths[1]);
////-------------------auto_roll--------------------------------------------------      
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",auto_servo_widths_mid[1]);
////-------------------auto_pitch------------------------------      
//      put2ground[len++]=',';
//20      len += sprintf(put2ground+len,"%d",auto_servo_widths_mid[0]);
////-------------------auto_coll--------------------------------     
//        put2ground[len++]=',';
//        len += sprintf(put2ground+len,"%d",auto_servo_widths_mid[2]);
////-------------------auto_yaw--------------------------------           
//        put2ground[len++]=',';
//        len += sprintf(put2ground+len,"%d",auto_servo_widths_mid[3]);
// //-------------------latitude-------------------------------
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.7f",gps_gga_latitude);//
//
// //-------------------longitude------------------------------   
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.7f",gps_gga_longitude);//
// //-------------------quality---------------------------------    
//      put2ground[len++]=',';
//       put2ground[len++]= gps_gga_quality+48;
//       
////-------------------num_sats--------------------------------     
//       put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%d",gps_gga_num_sats);
//	 
////------------------------altitude---------------------------      
//      put2ground[len++]=',';
//27      len += sprintf(put2ground+len,"%.1f",bmp_height);
//
////------------------------current_xyz----------------------  
//     for( i =0 ; i!=3; ++i )
//      {
//          put2ground[len++]=',';
// 30         len += sprintf(put2ground+len,"%.1f",position.current_xyz[i]);
//      }
// //------------------------current_uvw---------------------------
//      //for( i =0 ; i!=3; ++i )
//      //{
//          //put2ground[len++]=',';
//          //len += sprintf(put2ground+len,"%.1f",position.raw_current_uvw[i]);//
//     // }
// //------------------------input_xyz------------------------------ 
//      for( i =0 ; i!=3; ++i )
//        {
//            put2ground[len++]=',';
//            len += sprintf(put2ground+len,"%.1f",input_xyz[i]);
//        }
//
//    //------------------------input_uvw-----------------------------
//      for( i =0 ; i!=3; ++i )
//      {
//          put2ground[len++]=',';
//          len += sprintf(put2ground+len,"%.2f",input_uvw[i]);
//      }
//     //------------------------course /track-------------------------  
//      //put2ground[len++]=',';
//      //len += sprintf(put2ground+len,"%.1f",gps_vtg_course);//
//  //------------------------speed------------------------------------ 
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.2f",gps_vtg_speed);    
//   //---------------hover_xyz[3]-------------------------    
//    for( i =0 ; i!=3; ++i )
//    {
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.1f",trajectory.hover_xyz[i]);
//    }
//  //-------------------- current_target[3]------------------------
//    //当前目标点，相对于悬停点的位置
//    for( i =0 ; i!=3; ++i )
//    {
//      put2ground[len++]=',';
//      len += sprintf(put2ground+len,"%.1f",position.current_target[i]);
//    }
//    //-------------heli_step--------------//1
//     put2ground[len++]=',';
//     put2ground[len++]=trajectory.heli_step + 48;   
//
//   //--------------------receive_index-----------------//2    
//    put2ground[len++]=',';
//    put2ground[len++]=receive_index+48;
//    if(receive_index==true) receive_index=false;
// //---------------pcm_update_flag------------------------//3  
//     put2ground[len++]=',';
//
//    if(pcm_update_flag == 0)
//     {
//       err_count++;
//       if(err_count>=2)
//       {
//         pcm_up_flag=0;
//         err_count=0;
//       }
//       else
//       {
//          pcm_up_flag=1;
//       }
//     }
//     else 
//     {
//        err_count=0;
//        pcm_up_flag=pcm_update_flag;
//     }
//     put2ground[len++]=pcm_up_flag + 48;
//
//     //put2ground[len++]=pcm_update_flag + 48;
//     pcm_update_flag = 0;
//
//     //---------compass_update_flag--------//4
//     put2ground[len++]=',';
//     put2ground[len++]=compass_update_flag + 48;
//     compass_update_flag = false;
//     
//     //---------imu_update_flag----------//5
//     put2ground[len++]=',';
//     put2ground[len++]=imu_update_flag+48;   
//     imu_update_flag = 0;
//     
//     //---------gps_update_flag----------//6
//     put2ground[len++]=',';
//    put2ground[len++]=gps_update_flag+48;           //gps_update_flag;
//   //---------------round_count---------//7
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%d",round_count);  //round_count 
//   //-----------how_long---------------//8 
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%.1f",trajectory.how_long[0]); //how_long[0]  
//
//   //------------------------hover_time--------------------------------//9 
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%.0f",trajectory.hover_time); //hover_time 
//    //-----------top_speed---------------// 
//   // put2ground[len++]=',';
//    //len += sprintf(put2ground+len,"%.1f",trajectory.top_speed[2]); // 
//    //---------------------com_position[0]----------------------------//10
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%.1f",trajectory.com_position[0]);
//    
//  //-----------com_velocity[0]----------//11
//  put2ground[len++]=',';
//  len += sprintf(put2ground+len,"%.1f",trajectory.com_velocity[0]); //com_velocity[0]   
//  //-----------gps_gga_time----------
//  //-put2ground[len++]=',';
//  //-len += sprintf(put2ground+len,"%.0f",gps_gga_time); //gps_gga_time//
//  
////----------------------bmp085------------------------------------//
//	
//     // put2ground[len++]=',';
//     //len += sprintf(put2ground+len,"%d",temperature);
//      put2ground[len++]=',';
//      put2ground[len++]=abnormal_limit_flag+48;//gps跳变标志
//      bmp_update_flag = 0;
//	  abnormal_limit_flag = 0;
//      //put2ground[len++]=',';
//      //len += sprintf(put2ground+len,"%.2f",bmp_height);
//      
//
////---------------len---------------------------------------------------//12
//    put2ground[len++]=',';
//    len += sprintf(put2ground+len,"%d",len);
//   
//
////现在len长度至少要留30个的裕度，否则实际使用可能导致越界/////
//    put2ground[len++]='\r';
//    put2ground[len++]='\n';
//    to_ground_transmit_flag=false; 
//
//    AT91C_BASE_US0->US_TPR = (unsigned int)put2ground;
//    AT91C_BASE_US0->US_TCR = len;
//    AT91F_US_EnableIt(AT91C_BASE_US0,AT91C_US_TXBUFE  );
//   
//
//  }
//}
//
