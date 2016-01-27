#ifndef __BMP085_H__
#define __BMP085_H__



extern void timer0_c_irq_handler(void);
#define  PIV_5_MS 18720  //5ms for 60MHZ
#define RTTC_INTERRUPT_LEVEL 1 

extern long pni_value[3];
extern char bmp_update_flag;
extern volatile  bool compass_update_flag ;
extern unsigned int imu_count;
//extern bool imu_compass_update;
//extern char read_compass;
extern int timer0_num;
/*

	HMC5843 I2C Address
*/

#define HMC5843_I2C_ADDR_read		((0x3D>>1)<<16)
#define HMC5843_I2C_ADDR_write		((0x3C>>1)<<16)



/* 
 *	
 *	register definitions 	
 *
 */

#define HMC5843_MODE_ADDR		0x02
#define HMC5843_CONFIGUREA_ADDR         0x00

#define HMC5843_X_ADDR		    0x03
#define HMC5843_Y_ADDR			0x07
#define HMC5843_Z_ADDR		    0x05

#define COMP_CX 0  //INDEX_P 1//
#define COMP_CY 1  //INDEX_Q 6//
#define COMP_CZ 2  //INDEX_R 8//



/* General Setup Functions */



/** HMC5843_init 
   
   input : 	Pointer to bmp085_t 
   output:  -		
   return:  result of communication function
   notes :   
*/
 
extern int hmc_init();
//short bmp085_calc_temperature(unsigned long ut);
long hmc_get_data(char reg_adrr);


/*
	CHIP_TYPE CONSTANTS
*/

#define BMP085_CHIP_ID		0x55
#define BOSCH_PRESSURE_BMP085	85

/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */



#define BMP085_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS


#define BMP085_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)




/*
	BMP085 I2C Address
*/

#define BMP085_I2C_ADDR_read		((0xEF>>1)<<16)
#define BMP085_I2C_ADDR_write		((0xEE>>1)<<16)



/* 
 *	
 *	register definitions 	
 *
 */

#define BMP085_PROM_START__ADDR		0xaa
#define BMP085_PROM_DATA__LEN		  22

#define BMP085_CHIP_ID_REG			0xD0
#define BMP085_VERSION_REG			0xD1

#define BMP085_CTRL_MEAS_REG		0xF4
#define BMP085_ADC_OUT_MSB_REG		0xF6
#define BMP085_ADC_OUT_LSB_REG		0xF7

#define BMP085_SOFT_RESET_REG		0xE0

#define BMP085_T_MEASURE        0x2E				// temperature measurent 
#define BMP085_P_MEASURE        0x34				// pressure measurement

#define BMP085_TEMP_CONVERSION_TIME  5				// TO be spec'd by GL or SB



/* 
 *	
 *	bit slice positions in registers
 *
 */

#define BMP085_CHIP_ID__POS		0
#define BMP085_CHIP_ID__MSK		0xFF
#define BMP085_CHIP_ID__LEN		8
#define BMP085_CHIP_ID__REG		BMP085_CHIP_ID_REG


#define BMP085_ML_VERSION__POS		0
#define BMP085_ML_VERSION__LEN		4
#define BMP085_ML_VERSION__MSK		0x0F
#define BMP085_ML_VERSION__REG		BMP085_VERSION_REG



#define BMP085_AL_VERSION__POS  	4
#define BMP085_AL_VERSION__LEN  	4
#define BMP085_AL_VERSION__MSK		0xF0
#define BMP085_AL_VERSION__REG		BMP085_VERSION_REG


/* General Setup Functions */

/*PIT_interrupt_init*/
extern void PIT_interrupt_init(void);

/** BMP085_init 
   
   input : 	Pointer to bmp085_t 
   output:  -		
   return:  result of communication function
   notes :   
*/
//int bmp085_init(bmp085_t *bmp085);
extern int bmp085_init();
extern void bmp085_update(void);
//short bmp085_calc_temperature(unsigned long ut);
short bmp085_get_temperature(unsigned long ut);

//long bmp085_calc_pressure(unsigned long up);
long bmp085_get_pressure(unsigned long up);

//unsigned short bmp085_read_ut(void);
//unsigned long  bmp085_read_up(void);
unsigned short bmp085_get_ut(void);
unsigned long  bmp085_get_up(void);

extern void hmc_update(void);

/* API internal helper functions */
int bmp085_get_cal_param(void);

extern short temperature;
extern long pressure;
extern double bmp_height;


extern unsigned long up;
extern unsigned long ut;

/* value of the PIT value to have 333ns @ 48 MHz
668 -1 since real time out = PIV + 1 */
#define PIT_PIV_MILI_SECOND_VALUE  0x667


/** this structure holds all device specific calibration parameters 
*/
struct bmp085_smd500_calibration_param_t{
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;      		   
};


/** BMP085 image registers data structure

*/
struct  bmp085_t{	
	bmp085_smd500_calibration_param_t  cal_param;	
	unsigned char mode;
	unsigned char ml_version,	al_version;//chip_id,	
	unsigned char dev_addr;	
	unsigned char sensortype;

	long param_b5;
	int number_of_samples;
	short oversampling_setting;
        unsigned int BMP085_BUS_WRITE_FUNC(const AT91PS_TWI pTwi ,int mode, int int_address, unsigned char *data2send, unsigned int nb);
        unsigned int BMP085_BUS_READ_FUNC(const AT91PS_TWI pTwi ,int mode, int int_address, unsigned char *data, unsigned int nb);
        void AT91F_WaitMiliSecond (unsigned int MiliSeconds);
};

//extern struct bmp085_t p_bmp085;

//scp1000 
#define SPC1000AddrR    ((0x23>>1)<<16)//read
#define SPC1000AddrW	 ((0x22>>1)<<16) //write

//measurement mode
#define High_resolution  0x0A
#define High_speed       0x09

//TWI relative registers
#define OPERATION        0x03
#define OPSTATUS         0x04
#define STATUS           0x07
#define DATARD8          0x7F
#define DATARD16         0x80

extern double scp_pressure ;
extern unsigned char scp_status;
extern unsigned char scp_opstatus;
extern double scp_height;
extern unsigned int scp1000_init(void);
extern unsigned int scp1000_pressure(double *p);
extern void scp1000_update(void);
extern int flag;

extern double lidar_lite_distance;
#endif   // __BMP085_H__





