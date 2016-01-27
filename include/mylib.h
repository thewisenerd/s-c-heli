#ifndef _MYLIB_H_
#define _MYLIB_H_

//I don't know why,but the "inline" is needed . or linking error will occure. 
//The terminating NULL charater  is considered to be a part of the strings.
inline char *index( char*s, int c)
{
	char *line_in = s;
	while(*line_in != c)
	{
		if( *line_in++ == '\0')
			return NULL;
	}
	return line_in;
}

//The terminating NULL charater  is considered to be a part of the strings.
//return how many 'c' in the string s
inline unsigned int CharCounter( char*s, int c)
{
	unsigned int sum = 0;
	char *line_in = s;
	while(*line_in != '\0')
	{
		if( *line_in++ == c)
			sum++;
	}
	return sum;
}

#endif
