#include<includs.h>

char *index(char *line,char character)
{
  unsigned char i;
  for (i=0;i<255;i++)
      if(line[i]==character)return &line[++i];
  return 0;
}


static inline unsigned char nmea_split( char *line,int *values,unsigned char max_values)
{
	unsigned char	i;
	line = index( line, ',' );
	if( !line )return 0;
	for( i=0 ; i<max_values ; i++ )
	{
		char *			end_ptr;
		values[i] = strtol( line+1, &end_ptr, 16 );
		line = end_ptr;
		if( ! *line )	break;
	}
	return i+1;
}


