#include  <includs.h>

char READY_30HZ=0;
char DMP_GPS=0;
char servo_num=0;
static bool cd_update = true;


/*************************************************************************************/

void timer1_c_irq_handler(void)
{
  int dummy;
  dummy =AT91C_BASE_TC1->TC_SR;
  dummy =dummy;

	
          AT91C_BASE_TC1->TC_RA = AT91C_BASE_TC1->TC_CV+62400;//30ms
          READY_30HZ=1;
        
       // AT91F_TC_InterruptDisable(AT91C_BASE_TC1,AT91C_TC_CPAS);
           //READY_30HZ=1;
       
  //AT91F_TC_InterruptDisable(AT91C_BASE_TC0,AT91C_TC_LDRAS);
  // AT91F_US_Put("scut\r\n");
}



 /*************************************************************************************/
/*
void timer2_c_irq_handler(void)
{
  int dummy;
  //int temp;
  CDCLKSET;
  dummy =AT91C_BASE_TC2->TC_SR;
  dummy =dummy;
  AT91C_BASE_TC2->TC_RA = AT91C_BASE_TC2->TC_CV +servo_widths[ servo_num ];
  servo_num++;
  servo_num=servo_num%10;
  CDCLKRESET; 
  //AT91F_TC_InterruptDisable(AT91C_BASE_TC0,AT91C_TC_LDRAS); 
}*/
void timer2_c_irq_handler(void)//4017每个周期都复位
{
  int dummy;
  //int temp;
  dummy =AT91C_BASE_TC2->TC_SR;
  dummy =dummy;
  
  if(cd_update == false)
  {
    CDCLKSET;
    AT91C_BASE_TC2->TC_RA = AT91C_BASE_TC2->TC_CV +100;
    cd_update = true;
    CDSET;
  }
  else
  {
    if(servo_num == 0)
    {
      CDREST;
    }
    CDCLKSET;

    // AT91C_BASE_TC2->TC_RA =AT91C_BASE_TC2->TC_CV +500000;
    /*while(1)
    {
        temp ++;
        if (temp >10)
          break;
    }*/
    AT91C_BASE_TC2->TC_RA = AT91C_BASE_TC2->TC_CV + servo_widths[ servo_num ];
    servo_num++;
    if(servo_num >= 10)
    {  
      servo_num=servo_num%10;
      cd_update = false;
    }
    CDCLKRESET; 
  //AT91F_TC_InterruptDisable(AT91C_BASE_TC0,AT91C_TC_LDRAS); 
  } 
}

/*************************************************************************************/
 //T1 is 30HZ
 //T2 is used to produce PWM (  
/*************************************************************************************/
void timer_init(void)
{
   // AT91F_PIO_CfgPeriph(AT91C_BASE_PIOB,(unsigned int)AT91C_PB0_TIOA0,0);
 //AT91F_TC0_CfgPIO();
  AT91F_TC0_CfgPMC();
  AT91F_TC1_CfgPMC();
  AT91F_TC2_CfgPMC();
  
 // AT91C_BASE_TC0 ->TC_CMR= AT91C_TC_CLKS_TIMER_DIV2_CLOCK|AT91C_TC_LDRA_RISING
                   //       |AT91C_TC_ABETRG|AT91C_TC_LDBDIS|AT91C_TC_ETRGEDG_FALLING ;
  AT91C_BASE_TC2 ->TC_CMR=AT91C_TC_CLKS_TIMER_DIV3_CLOCK|AT91C_TC_WAVE;
  AT91C_BASE_TC1 ->TC_CMR=AT91C_TC_CLKS_TIMER_DIV3_CLOCK|AT91C_TC_WAVE;
  AT91C_BASE_TC0 ->TC_CMR=AT91C_TC_CLKS_TIMER_DIV2_CLOCK|AT91C_TC_WAVE;
  /************************************TC0*************************************************/  
   AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC0,
	                   TC0_INTERRUPT_LEVEL,
	                   AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE,
	                   timer0_c_irq_handler);
   AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_TC0);
  // AT91C_BASE_TC0->TC_RC = 0xffff;
   
  // AT91F_TC_InterruptEnable(AT91C_BASE_TC0,AT91C_TC_CPAS);
  /************************************TC1 30HZ*************************************************/
  
  AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC1,
	                   TC1_INTERRUPT_LEVEL,
	                   AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE,
	                   timer1_c_irq_handler);
   AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_TC1);
   
   AT91F_TC_InterruptEnable(AT91C_BASE_TC1,AT91C_TC_CPAS);
   
 /*************************************************************************************/
    AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC2,
	                   TC2_INTERRUPT_LEVEL,
	                   AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE,
	                   timer2_c_irq_handler);
   AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_TC2);
   
   AT91F_TC_InterruptEnable(AT91C_BASE_TC2,AT91C_TC_CPAS);
   CDREST;/*注意：它是CD4017的RESET信号，它使CD4017从'0'开始产生PWM信号。它的位置有待考量*/
   AT91C_BASE_TC2 ->TC_RA = 20;
   
 //  AT91C_BASE_TC2 ->TC_RA = servo_widths[0];
  AT91C_BASE_TC0 -> TC_CCR = AT91C_TC_CLKEN |AT91C_TC_SWTRG ; 
  AT91C_BASE_TC1 -> TC_CCR = AT91C_TC_CLKEN |AT91C_TC_SWTRG ;
  AT91C_BASE_TC2 -> TC_CCR = AT91C_TC_CLKEN |AT91C_TC_SWTRG ;
}


