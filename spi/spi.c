#include<includs.h>

AT91PS_SPI SPI;
//char pni_axis=1;
//long pni_value[3];
long  imuframe[8]; //imu information
int    imuaddress[8]= {0x0200,0x0400,0x0600,0x0800,0x0A00,0x0C00,0x0E00,0x3C00};
//char DRDY=0;
//char    avrframe[10];

/*void FIQ_c_irq_handler(void)
{
  int dummy;
  dummy =AT91D_BASE_PIO_SW->PIO_ISR;
  dummy =dummy;

} 

void pio_c_irq_handler(void)
{
  int dummy;
  dummy =AT91D_BASE_PIO_SW->PIO_ISR;
  dummy =dummy;
  DRDY=1;
*/    
     // AT91F_SPI_PutChar(AT91C_BASE_SPI,0,1);
     // pni_value[pni_axis]=AT91F_SPI_GetChar(AT91C_BASE_SPI);
      // pni_value[pni_axis]=AT91F_SPI_GetChar(AT91C_BASE_SPI);
   /*   pni_axis++;
   if(pni_axis<=3)
   {
     COMPASSSET;
     AT91C_BASE_SPI->SPI_CSR[1]= AT91C_SPI_BITS_8|((unsigned int) 0xff <<8) |AT91C_SPI_CPOL;
     COMPASSRESET;
     AT91F_SPI_PutChar(AT91C_BASE_SPI,pni_axis,1);
   }
   else
    {
      pni_axis=0;
      SSSET;
    }*/
   //AT91F_US_Put("scut\r\n");
//}


void AT91F_SpiInit(void)
{  
    SPI=AT91C_BASE_SPI;
/*------------------------------IMU---------------------------------------------------*/    
    AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, // PIO controller base address
	  (((unsigned int) AT91C_PA11_NPCS0 ) |((unsigned int) AT91C_PA12_MISO  )|
           ((unsigned int)AT91C_PA13_MOSI ) |((unsigned int) AT91C_PA14_SPCK)),0) ;// Peripheral A
/*---------------------------------------------------------------------------------*/
    
/*------------------------------AVR---------------------------------------------------*/
  //  AT91F_PIO_CfgPeriph(AT91C_BASE_PIOA, // PIO controller base address
	//  (((unsigned int) AT91C_PA11_NPCS0 ) |((unsigned int) AT91C_PA12_MISO  )|((unsigned int) AT91C_PA31_NPCS1  )|
      //     ((unsigned int)AT91C_PA13_MOSI ) |((unsigned int) AT91C_PA14_SPCK)),0) ;// Peripheral A
/*---------------------------------------------------------------------------------*/
    
    AT91F_SPI_CfgPMC() ;
     // Reset SPI0
   // SPI->SPI_CR = AT91C_SPI_SWRST;
 
      // Configure SPI0 in Master Mode with No CS selected 
   // *(AT91C_SPI_CSR + 0) = AT91C_SPI_CPOL | AT91C_SPI_BITS_8 | ((AT91B_MASTER_CLOCK / AT91C_SPI_CLK) << 8);
             
     SPI->SPI_MR=AT91C_SPI_MSTR | AT91C_SPI_PS_VARIABLE | AT91C_SPI_MODFDIS ;
                          //  ((unsigned int) 0xE << 16);
                            
     SPI->SPI_CR=AT91C_SPI_SPIEN;
     
     SPI->SPI_CSR[0]= AT91C_SPI_BITS_16 |IMU_SPI_BAUD_RATE \
                      |AT91C_SPI_CPOL|AT91C_SPI_CSAAT;//|AT91C_SPI_DLYBCT;
     //--------by syc compass spi¶ÁÈ¡ÅäÖÃ
     //SPI->SPI_CSR[1]= AT91C_SPI_BITS_8 | COMPASS_SPI_BAUD_RATE |AT91C_SPI_CPOL|AT91C_SPI_NCPHA;
     
     //SPI->SPI_CSR[2]= AT91C_SPI_BITS_8 | ((unsigned int) 0x08 <<  8) |AT91C_SPI_CPOL|AT91C_SPI_NCPHA;
     //SPI->SPI_CSR[3]= AT91C_SPI_BITS_8 | ((unsigned int) 0x08 <<  8) |AT91C_SPI_CPOL|AT91C_SPI_NCPHA;
     
     // AT91F_PDC_Open (AT91C_BASE_PDC_SPI);
   //  AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_SPI, SPI_INTERRUPT_LEVEL,
    //                       AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SPI_c_irq_handler);
    // AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_SPI);
     
  //   SPI->SPI_RPR = (unsigned int) avrframe;
    // SPI->SPI_RCR = 10;
    
  //   AT91F_SPI_EnableIt(SPI,AT91C_SPI_RXBUFF  );
     
}

/*void FIQ_init(void)  
{
  AT91F_PIO_CfgPeriph(AT91C_BASE_PIOB,(unsigned int)AT91C_PB19_FIQ,0);
  
  AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_FIQ) ;
  
  AT91F_AIC_ConfigureIt (AT91C_BASE_AIC, AT91C_ID_FIQ, FIQ_INTERRUPT_LEVEL,
                           AT91C_AIC_SRCTYPE_POSITIVE_EDGE, FIQ_c_irq_handler);
  //AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_FIQ);
}

void PIOB0_init(void)
{
   AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOB ) ;

   AT91F_PIO_CfgInput(AT91C_BASE_PIOB, AT91B_COMPASS );
   AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91D_ID_PIO_SW,
	                   PIO_INTERRUPT_LEVEL,
	                   AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE,
	                   pio_c_irq_handler);
   
   AT91F_PIO_InterruptEnable(AT91D_BASE_PIO_SW,AT91B_COMPASS);
   
   AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91D_ID_PIO_SW);
}*/





