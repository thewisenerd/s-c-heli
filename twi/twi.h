#ifndef _TWI_H_
#define _TWI_H_

//#define AT91B_SCP AT91C_PIO_PB26
//#define PIO_SCP_INTERRUPT_LEVEL 6

#define DEVICE_SLAVE_ADDRESS_WRITE  (0x22<<16)
#define DEVICE_SLAVE_ADDRESS_READ   (0x23<<16)
#define TWI_BUS_CLOCK   400000 /* in Hz*/
#define ERROR (AT91C_TWI_NACK_SLAVE)


extern void twi_init(void);
extern void AT91F_SetTwiClock(int TwiClock);



#endif




