#include  <includs.h>

/*----------------------------------------------------------------------------
Function: AT91F_SetTwiClock (int TwiClock)
Arguments: - <TwiClock> TWI bus clock in Hertz
Comments : TO DO:

Return Value: none
-----------------------------------------------------------------------------*/
void AT91F_SetTwiClock(int TwiClock)
{
  int cldiv,ckdiv=1 ;

  /* CLDIV = ((Tlow x 2^CKDIV) -4) x Tmck */
  /* CHDIV = ((THigh x 2^CKDIV) -4) x Tmck */
  /* Only CLDIV is computed since CLDIV = CHDIV (50% duty cycle) */

  while ( ( cldiv = ( (AT91B_MCK/(2*TwiClock))-4 ) / pow(2.0,ckdiv)) > 255 )
   ckdiv++ ;

  AT91C_BASE_TWI->TWI_CWGR =(ckdiv<<16)|((unsigned int)cldiv << 8)|(unsigned int)cldiv  ;
}



/*----------------------------------------------------------------------------
Function : AT91F_TWI_ProbeDevices
Arguments: 
           <SlaveAddr>: Address of the slave device to probe.

Comments : Write a single data into a slave device to see if it is connected on
           the bus.

Return Value: AT91C_TWI_NACK_MASTER if so, SlaveAddress otherwise.
-----------------------------------------------------------------------------*/
int AT91F_TWI_ProbeDevices(int SlaveAddr)
{
    unsigned int end = 0, status, Return;

    /* Enable Master Mode */
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN|AT91C_TWI_SVDIS ;

    /* Set the TWI Master Mode Register */
    AT91C_BASE_TWI->TWI_MMR =  SlaveAddr & ~AT91C_TWI_MREAD;

    /* Write the data to send into THR. Start conditionn DADDR and R/W bit
       are sent automatically */
    AT91C_BASE_TWI->TWI_THR = 0x55;

    while (!end)
    {
      status = AT91C_BASE_TWI->TWI_SR;
      if ((status & AT91C_TWI_NACK_MASTER) == AT91C_TWI_NACK_MASTER)
      {
        Return = AT91C_TWI_NACK_MASTER ;
        end=1;
      }
      /*  Wait for the Transmit ready is set */
      else if ((status & AT91C_TWI_TXRDY_MASTER) == AT91C_TWI_TXRDY_MASTER)
      {
        end=1;
        Return = SlaveAddr>>16;
      }
    }

    /* Wait for the Transmit complete is set */
    status = AT91C_BASE_TWI->TWI_SR;
    while (!(status & AT91C_TWI_TXCOMP_MASTER))
      status = AT91C_BASE_TWI->TWI_SR;

    return (Return);
}

//*----------------------------------------------------------------------------
//* \fn twi_init
//* \brief Initializes TWI device
//*----------------------------------------------------------------------------
void twi_init(void)
{
    /* Configure the PIO pins corresponding to the TWI pins */
    AT91F_TWI_CfgPIO ();

    /* Configure PMC by enabling TWI clock */
    AT91F_TWI_CfgPMC ();

    /* Reset the TWI */
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_SWRST;

    /* Configure TWI in master mode */
    AT91F_TWI_Configure (AT91C_BASE_TWI);

    /* Set TWI Clock Waveform Generator Register */
    AT91F_SetTwiClock(TWI_BUS_CLOCK);
}

