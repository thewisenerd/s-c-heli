#ifndef _SPI_H_
#define _SPI_H_


#define FIQ_INTERRUPT_LEVEL     5
#define PIO_INTERRUPT_LEVEL     6 

#define SSNOT AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA31
#define SSSET AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA31
//#define COMPASSRESET AT91C_BASE_PIOB->PIO_CODR = AT91C_PIO_PB25 //change from PC4 by chen
//#define COMPASSSET   AT91C_BASE_PIOB->PIO_SODR = AT91C_PIO_PB25

#define IMU_SPI_BAUD_RATE              ( (unsigned int) 0x50<< 8)    //  MCK/IMU_SPI_BAUD_RATE
#define COMPASS_SPI_BAUD_RATE          ( (unsigned int) 0x50<< 8)


//extern char pni_axis;
//extern long pni_value[3];  //compass information
//extern char  DRDY;
extern long    imuframe[8];    //imu information
extern int    imuaddress[8];

void AT91F_SpiInit(void);
void FIQ_init(void);
void PIOB0_init(void);

#endif

