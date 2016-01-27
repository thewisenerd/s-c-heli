#include<includs.h>

 

void
put_hexdigit(unsigned char		i)
{
	if( i < 0x0A )
		i += '0';
	else
		i += 'A' - 0x0A;
	while (!AT91F_US_TxReady(AT91C_BASE_US0));
		AT91F_US_PutChar(AT91C_BASE_US0, i);
}	


void
put_uint8_t(
	unsigned char		i
)
{
	put_hexdigit( (i >> 4) & 0x0F );
	put_hexdigit( (i >> 0) & 0x0F );
}


void
put_uint16_t(
	int		i
)
{
	put_uint8_t(  (i >> 8) & 0xFF );
	put_uint8_t(  (i >> 0) & 0xFF );
}


__inline char ftoa(float dat,char *s,unsigned char jd)
{
  int len,temp,i;
  char flag=dat<0?dat=-dat,1:0;
  char t[10];
  temp=(int)dat;

  put_uint16_t( temp );
  for(len=0;temp>0;temp/=10,len++)
  { t[len]=temp%10+48;
  //while (!AT91F_US_TxReady(AT91C_BASE_US0));
//  AT91F_US_PutChar(AT91C_BASE_US0, t[len]);
  }
  for(i=0;i<=len;i++)
     s[len-i-1]=t[i];
     s[len++]='.';
    
  for(i=0,temp=(int)((dat-(int)dat)*pow(10,jd));temp>0;temp/=10,i++)
      t[i]=temp%10+48;
  
  for(i=0;i<jd;i++)
   s[len++]=t[jd-i-1];
   s[len]=0;
   return flag;
}


