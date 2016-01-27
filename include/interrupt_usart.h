#ifndef _INTERRUPT_USART_H_
#define _INTERRUPT_USART_H_

#define USART_BAUD_RATE 		38400
#define USART_INTERRUPT_LEVEL		3
#define DBGU_USART_INTERRUPT_LEVEL      3
#define USART1_INTERRUPT_LEVEL          6
#define PCM_NUM 28
//#define part_num 10

#define UPLOAD_NUM 40//139
#define GPS_BUF_NUM 100
#define PCM_BUF_NUM 100
//#define part_len 10
//extern unsigned char receiveimusignal;
extern unsigned char AVR_PCM[PCM_NUM];
extern unsigned char raw_AVR_PCM[PCM_NUM];

extern AT91PS_USART COM0;
//extern   char pcm_update_flag ;



extern volatile bool hmc_update_flag ;
extern volatile bool gga_update_flag ;
extern volatile bool vtg_update_flag ;

extern char gga_save_line[GPS_BUF_NUM];
extern char vtg_save_line[GPS_BUF_NUM];
extern char hmc_save_line[GPS_BUF_NUM];

extern char groundtoarm[UPLOAD_NUM];
extern  char groundtoarm_flag;

extern void put_uint16_t(int		i);
extern void AT91F_US_Put(char*);
extern void AT91F_US1_Put(char*);
extern void AT91F_DBGU_Put(char*);
extern void Usart1_init(void);

//extern char ftoa(double dat,char *s,unsigned char jd); 

extern void output2ground(void);

extern void output2ground11(void);
#endif
