#include <includs.h>


struct bmp085_t  bmp085;
struct bmp085_t * p_bmp085=&bmp085; 

volatile  bool compass_update_flag = false; //Compass更新标志
char bmp_update_flag=0;
char imu_num=0;
char bmphmc_num = 0;
//bool imu_compass_update = false;
char HDG=2;
volatile  char imu_update_flag=0;
bool   raw_imu_update=false;
unsigned int  imu_count = 0;
//char read_compass=0;
int timer0_num=0;

/**< pointer to SMD500 / BMP085 device area */
unsigned long up = 0;
unsigned long ut = 0;
short temperature = 0;
long pressure = 0;
double bmp_height = 0.0;
double zero_bmp_height=0.0;

long pni_value[3] = {0,0,0};

//----------------------------------------------------------------


unsigned int bmp085_t::BMP085_BUS_WRITE_FUNC(const AT91PS_TWI pTwi ,int mode, int int_address, unsigned char *data2send, unsigned int nb)
{
	unsigned int status,counter=0,error=0;
        unsigned int temp=0;
	// Set TWI Internal Address Register
	if ((mode & AT91C_TWI_IADRSZ) != 0) pTwi->TWI_IADR = int_address;

	// Set the TWI Master Mode Register
	  pTwi->TWI_MMR = mode & ~AT91C_TWI_MREAD;
	if(nb <2){
		pTwi->TWI_CR = AT91C_TWI_START | AT91C_TWI_MSEN | AT91C_TWI_STOP;
		pTwi->TWI_THR = *data2send;
	}
	else
	{
	// Set the TWI Master Mode Register
	  for(counter=0;counter<nb;counter++){
          pTwi->TWI_CR = AT91C_TWI_START | AT91C_TWI_MSEN;
          if (counter == (nb - 1)) pTwi->TWI_CR = AT91C_TWI_STOP;
          status = pTwi->TWI_SR;
          if ((status & ERROR) == ERROR) error++;
          while (!(status & AT91C_TWI_TXRDY_MASTER))
          {
               status = pTwi->TWI_SR;
               if ((status & ERROR) == ERROR) error++;
               if(temp >2000)
                {
                  temp=0;
                  break;
                } 
               temp ++;
          }
          pTwi->TWI_THR = *(data2send+counter);
	   }
	}
	status = pTwi->TWI_SR;
	if ((status & ERROR) == ERROR) error++;
	while (!(status & AT91C_TWI_TXCOMP_MASTER))
        {
    		status = pTwi->TWI_SR;
    		if ((status & ERROR) == ERROR) error++;
                if(temp >2000)
                {
                  temp=0;
                  break;
                } 
                temp ++;
        }
	return error;
}

//*=========================================================
//*		READ
//*=========================================================
//*----------------------------------------------------------------------------
//* \fn    AT91F_TWI_ReadByte
//* \brief Read a byte from a slave device
//*----------------------------------------------------------------------------
unsigned int bmp085_t::BMP085_BUS_READ_FUNC(const AT91PS_TWI pTwi ,int mode, int int_address, unsigned char *data, unsigned int nb)
{
	unsigned int status,counter=0,error=0;
        unsigned int temp=0;
	// Set TWI Internal Address Register
	if ((mode & AT91C_TWI_IADRSZ) != 0) pTwi->TWI_IADR = int_address;

	// Set the TWI Master Mode Register
	pTwi->TWI_MMR = mode | AT91C_TWI_MREAD;

	// Start transfer
	if (nb == 1){
	   pTwi->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
	   status = pTwi->TWI_SR;
    	   if ((status & ERROR) == ERROR) error++;
	   while (!(status & AT91C_TWI_TXCOMP_MASTER))
           {
    	      status = pTwi->TWI_SR;
              if ((status & ERROR) == ERROR) error++;
              if(temp >2000)
                {
                  temp=0;
                  break;
                } 
              temp ++;
    	   }
	   *(data) = pTwi->TWI_RHR;
	}
 	else{
 	   pTwi->TWI_CR = AT91C_TWI_START | AT91C_TWI_MSEN;
	   status = pTwi->TWI_SR;
	   if ((status & ERROR) == ERROR) error++;

	// Wait transfer is finished
           while (!(status & AT91C_TWI_TXCOMP_MASTER))
           {
   		status = pTwi->TWI_SR;
   		if ((status & ERROR )== ERROR) error++;
                if(temp >2000)
                {
                  temp=0;
                  break;
                } 
                temp ++;
    		if(status & AT91C_TWI_RXRDY)
                {
			*(data+counter++) = pTwi->TWI_RHR;
			if (counter == (nb - 1)) pTwi->TWI_CR = AT91C_TWI_STOP;
		}
	   }
	}
	return 0;
}






/*-----------------------------------------------------------------------------
/ Wait function with the Periodic Interval Timer (PIT)
/ The wait time is from 1ms to 999ms.
/----------------------------------------------------------------------------*/
void bmp085_t::AT91F_WaitMiliSecond (unsigned int MiliSeconds)
{
   unsigned int PitStatus = 0;     /* Status register of the PIT */
   unsigned int PitLoop = 0;    /* Store the number of PIT Loop */

   AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN|PIT_PIV_MILI_SECOND_VALUE;

   for( PitLoop=0; PitLoop <(MiliSeconds*3);)   /* One PIT loop equals 333ms */
   {
    /* Wait for the PIT counter overflow occurs */
    while ((AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS)==0);
    /* Read the PIT Interval Value Reg. to clear it for the next overflow */
    PitStatus = AT91C_BASE_PITC->PITC_PIVR ;
    /* dummy access to avoid IAR warning */
    PitStatus = PitStatus ;
    PitLoop++;
   }
}

//----------------------------------------------------------------
/** initialize BMP085 / SMD500 

  This function initializes the BMP085 pressure sensor/ the successor SMD500 is also supported.
  The function automatically detects the sensor type and stores this for all future communication and calculation steps
  \param *bmp085_t pointer to bmp085 device data structure
  \return result of communication routines

*/

int bmp085_init() //(bmp085_t *bmp085) 
{
  char comres=0;
  unsigned char data;

  //p_bmp085 = bmp085;                                      /* assign BMP085 ptr */
  //p_bmp085->sensortype = E_SENSOR_NOT_DETECTED;
  //p_bmp085->dev_addr = BMP085_I2C_ADDR;                   /* preset BMP085 I2C_addr */
  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_CHIP_ID__REG, &data, 1);  /* read Chip Id */
   
  p_bmp085->number_of_samples = 1;  
  p_bmp085->oversampling_setting=2;

  //p_bmp085->sensortype = BOSCH_PRESSURE_BMP085;
    
  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_VERSION_REG, &data, 1); /* read Version reg */
    
  p_bmp085->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);        /* get ML Version */
  p_bmp085->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);        /* get AL Version */
  bmp085_get_cal_param( ); /* readout bmp085 calibparam structure */

  return comres;

}

/** read out parameters cal_param from BMP085 memory
   \return result of communication routines
*/

//int bmp085_read_cal_param(void)
int bmp085_get_cal_param(void)
{
  int comres;
  unsigned char data[22];
  comres = p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_PROM_START__ADDR, data, BMP085_PROM_DATA__LEN);
  
  /*parameters AC1-AC6*/
  p_bmp085->cal_param.ac1 =  (data[0] <<8) | data[1];
  p_bmp085->cal_param.ac2 =  (data[2] <<8) | data[3];
  p_bmp085->cal_param.ac3 =  (data[4] <<8) | data[5];
  p_bmp085->cal_param.ac4 =  (data[6] <<8) | data[7];
  p_bmp085->cal_param.ac5 =  (data[8] <<8) | data[9];
  p_bmp085->cal_param.ac6 =  (data[10] <<8) | data[11];
  
  /*parameters B1,B2*/
  p_bmp085->cal_param.b1 =  (data[12] <<8) | data[13];
  p_bmp085->cal_param.b2 =  (data[14] <<8) | data[15];
  
  /*parameters MB,MC,MD*/
  p_bmp085->cal_param.mb =  (data[16] <<8) | data[17];
  p_bmp085->cal_param.mc =  (data[18] <<8) | data[19];
  p_bmp085->cal_param.md =  (data[20] <<8) | data[21];
  
  return comres;  
  
}


/** calculate temperature from ut
  ut was read from the device via I2C and fed into the right calc path for either SMD500 or BMP085
  \param ut parameter ut read from device
  \return temperature in steps of 0.1 deg celsius
  \see bmp085_read_ut()
*/

//short bmp085_calc_temperature(unsigned long ut) 
short bmp085_get_temperature(unsigned long ut) 
{
  //short temperature;
  long x1,x2;    

  x1 = (((long) ut - (long) p_bmp085->cal_param.ac6) * (long) p_bmp085->cal_param.ac5) >> 15;
  x2 = ((long) p_bmp085->cal_param.mc << 11) / (x1 + p_bmp085->cal_param.md);
  p_bmp085->param_b5 = x1 + x2;


  temperature = ((p_bmp085->param_b5 + 8) >> 4);  // temperature in 0.1癈

  return (temperature);
}

/** calculate pressure from up
  up was read from the device via I2C and fed into the right calc path for either SMD500 or BMP085
  In case of SMD500 value averaging is done in this function, in case of BMP085 averaging is done through oversampling by the sensor IC

  \param ut parameter ut read from device
  \return temperature in steps of 1.0 Pa
  \see bmp085_read_up()
*/

//long bmp085_calc_pressure(unsigned long up)
long bmp085_get_pressure(unsigned long up)
{
   long x1,x2,x3,b3,b6;
   unsigned long b4, b7;
   
   b6 = p_bmp085->param_b5 - 4000;
   //*****calculate B3************
   x1 = (b6*b6) >> 12;	 	 
   x1 *= p_bmp085->cal_param.b2;
   x1 >>=11;

   x2 = (p_bmp085->cal_param.ac2*b6);
   x2 >>=11;

   x3 = x1 +x2;

   b3 = (((((long)p_bmp085->cal_param.ac1 )*4 + x3) <<p_bmp085->oversampling_setting) + 2) >> 2;

   //*****calculate B4************
   x1 = (p_bmp085->cal_param.ac3* b6) >> 13;
   x2 = (p_bmp085->cal_param.b1 * ((b6*b6) >> 12) ) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (p_bmp085->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;
     
   b7 = ((unsigned long)(up - b3) * (50000>>p_bmp085->oversampling_setting));   
   if (b7 < 0x80000000)
   {
     pressure = (b7 << 1) / b4;
   }
   else
   { 
     pressure = (b7 / b4) << 1;
   }
   
   x1 = pressure >> 8;
   x1 *= x1;
   x1 = (x1 * 3038) >> 16;
   x2 = (pressure * (-7357)) >> 16;
   pressure += (x1 + x2 + 3791) >> 4;	// pressure in Pa  

   return (pressure);
}


/** read out ut for temperature conversion
   \return ut parameter that represents the uncompensated temperature sensors conversion value
*/

//unsigned short bmp085_read_ut ()
unsigned short bmp085_get_ut ()
{
  unsigned char data[2];    
  unsigned char ctrl_reg_data;
  int wait_time;
  int comres;
 
  ctrl_reg_data = BMP085_T_MEASURE;
  wait_time = BMP085_TEMP_CONVERSION_TIME;
  
  comres = p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
  
 p_bmp085->AT91F_WaitMiliSecond (wait_time);  
  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_ADC_OUT_MSB_REG, data, 2);
  ut = ((long)data[0] <<8) | data[1];
  return (ut);
  
}

double height_average_filter(double temp2)//气压计高度平均值滤波
{
    static double bmp_h_window[7];
    double bmp_sum_h = 0;
    static int bmp_length = 1;
    double bmp_height_filter;
    for(char i=1; i<7; i++) //for sum_e and e_window
    {
    	bmp_sum_h += bmp_h_window[i];
        bmp_h_window[i-1] = bmp_h_window[i];
    }
	 //   e_window[8-1]=e_speed;sum_e += e_speed;
       bmp_h_window[7-1]=temp2;
       bmp_sum_h += temp2;
       bmp_height_filter = bmp_sum_h/bmp_length;
       
      if(bmp_length >= 7)//for length ,max = N
         bmp_length = 7;
      else bmp_length++;
      
     return bmp_height_filter;
}


/** read out up for pressure conversion
  depending on the oversampling ratio setting up can be 16 to 19 bit
   \return up parameter that represents the uncompensated pressure value
*/

//unsigned long bmp085_read_up ()
unsigned long bmp085_get_up ()
{
  unsigned char data[3];    
  unsigned char ctrl_reg_data;
  int comres=0;
 
  ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
  comres = p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
  
 
  p_bmp085->AT91F_WaitMiliSecond ( 2 + (3 << (p_bmp085->oversampling_setting) ) );
  
  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_ADC_OUT_MSB_REG, data, 3);
  up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-p_bmp085->oversampling_setting);
  p_bmp085->number_of_samples = 1;
   
  return (up);
  
}

//----------------------------------------------------------------
/** initialize HMC5843

*/

int hmc_init() 
{
  char comres=0;
  unsigned char cle_reg_data = 0x00;
  unsigned char cle_reg_data1 = 0x14;
  comres = p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,HMC5843_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, HMC5843_MODE_ADDR, &cle_reg_data, 1);
  comres = p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,HMC5843_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, HMC5843_CONFIGUREA_ADDR, &cle_reg_data1, 1);
  //comres += p_bmp085->BMP085_BUS_WRITE_FUNC(HMC5843_I2C_ADDR_write, HMC5843_MODE_ADDR,AT91C_TWI_IADRSZ_1_BYTE,&cle_reg_data);

  return comres;

}

/** read out parameters cal_param from HMC5843 memory
   \return result of communication routines
*/

long hmc_get_data(char reg_adrr)
{
  int comres = 0;
  unsigned char data[2];
  long value;
  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,HMC5843_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, reg_adrr, data, 2);
  //comres += p_bmp085->BMP085_BUS_READ_FUNC(HMC5843_I2C_ADDR_read,reg_adrr,AT91C_TWI_IADRSZ_1_BYTE,data,2);
  //comres = p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_PROM_START__ADDR, data, BMP085_PROM_DATA__LEN);
  
  /*parameters*/
  value =  (data[0] <<8) | data[1];
  return value;   
}
//------------------------------------------syc
void bmp085_update(void)
{ 
  //double limit=0;
  static unsigned int zero_count = 1 ;
  //static double last_bmp_height=0;
  
  bmp085_get_temperature(ut);
  bmp085_get_pressure(up);
  bmp_height = 44330*(1-pow((pressure/101325.0),0.190294957));
  bmp_height=height_average_filter(bmp_height);
  
   if(zero_count==300)
       zero_bmp_height=bmp_height;//取高度项零点
   zero_count ++ ;
   if(zero_count >=400)
      zero_count = 350;
   
   bmp_height=zero_bmp_height-bmp_height;
}

void timer0_c_irq_handler(void)
{
  int dummy;
  unsigned char data[2];
  unsigned char data1[3];
  unsigned char ctrl_reg_data;
  int comres;
  dummy =AT91C_BASE_TC0->TC_SR;
  dummy =dummy;
  //timer0_num++;
  if(HDG == 1)
  {  timer0_num++;
     AT91F_SPI_PutChar(AT91C_BASE_SPI,imuaddress[imu_num],0);
     imu_count ++;
   
     int temp = 0;
     while((AT91C_BASE_SPI->SPI_SR & AT91C_SPI_TXEMPTY)==0)
     {
        temp ++;
        if (temp >200)
         break;
      }
       if(imu_num!=0)
              imuframe[imu_num-1]=AT91F_SPI_GetChar(AT91C_BASE_SPI);
          else 
             imuframe[7]=AT91F_SPI_GetChar(AT91C_BASE_SPI);    
   
        imu_num ++;
        imu_num=imu_num%8;
       if(imu_num==0) 
        {
          raw_imu_update = true ;
          imu_update_flag = 1;
          HDG=0; 
        }
       AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+250;
  }
  else
  {
    if(HDG==0)
    {  timer0_num++;
       if(bmphmc_num == 6)
       {
         bmphmc_num = 0;         
         HDG=2;
         bmp_update_flag=2;
         AT91F_TC_InterruptDisable(AT91C_BASE_TC0,AT91C_TC_CPAS);
         timer0_num=0;
       }
        else
        {
		  hmc_init();
          switch(bmphmc_num)
            {
               
               case 0 :  ctrl_reg_data = BMP085_T_MEASURE;
                         comres = p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
                         bmphmc_num++;
                         AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+37500;
                         break;
               case 1 :  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_ADC_OUT_MSB_REG, data, 2);
                         ut = ((unsigned long)data[0] <<8) | (unsigned long)data[1];
                        // bmp085_get_temperature(ut);
                         bmphmc_num++;
                         AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+200;
                         break;
               case 2 :  ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
                         comres += p_bmp085->BMP085_BUS_WRITE_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_write | AT91C_TWI_IADRSZ_1_BYTE, BMP085_CTRL_MEAS_REG, &ctrl_reg_data, 1);
                         bmphmc_num++;
                         AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+52500;
                         break;
               case 3:   bmphmc_num++;
                         AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+52500;
                         break;   
               case 4 :  comres += p_bmp085->BMP085_BUS_READ_FUNC(AT91C_BASE_TWI,BMP085_I2C_ADDR_read | AT91C_TWI_IADRSZ_1_BYTE, BMP085_ADC_OUT_MSB_REG, data1, 3);
                         up = (((unsigned long) data1[0] << 16) | ((unsigned long) data1[1] << 8) | (unsigned long) data1[2]) >> (8-p_bmp085->oversampling_setting);
                         p_bmp085->number_of_samples = 1;
                         //bmp085_get_pressure(up);
                         //bmp_height = 44330*(1-pow((pressure/101325.0),0.190294957));
                         //bmp_height=height_average_filter(bmp_height);
                         bmphmc_num++;
                         AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+3500;
                         break;
               case 5 :   pni_value[COMP_CX] = hmc_get_data(HMC5843_X_ADDR);
                          pni_value[COMP_CY] = hmc_get_data(HMC5843_Y_ADDR);
                          pni_value[COMP_CZ] = hmc_get_data(HMC5843_Z_ADDR);
                          compass_update_flag = true;
                          bmphmc_num++;
                          AT91C_BASE_TC0->TC_RA = AT91C_BASE_TC0 -> TC_CV+350;
                          break;
              default : break;
            }
        }
     }//HDG==0
  }
}
