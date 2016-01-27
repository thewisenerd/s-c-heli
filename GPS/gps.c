#include<includs.h>

/* These come from GPGGA sentences */
int			time;		// seconds past midnight
	
float			latitude;	// N=+, S=-
float			longitude;	// E=+, W=-
unsigned char quality;	// 0=invalid, 1=gps, 2=dgps
unsigned char	num_sats;
float			hdop;		// meters
float			altitude;	// meters, if available
float			wgs_alt;	// meters

/* These are GPVTG sentences */
float			track;		// magnetic
float			ground_speed;	// km/h

void gps_update(const char *	line)
{
	if( line[3] == 'G' )
		gpgga_update( line );
	else
		gpvgt_update( line );
}

/*
 * $GPGGA,020314.0,3902.848,N,07706.833,W,1,4,002.3,,M,-033,M,,*50
 */
void gpgga_update(const char *		line_in)
{
	// Gross hack to get around strtol's non-const second argument
	char *			line = (char*) line_in;
 // Skip $GPGGA
	line = index( line, ',' );
	if( !line )
		return;
	line++;

	// Convert the time to seconds past midnight (GMT)
	int			hours	= strntol( line, 2, &line, 10 );
	int			min	= strntol( line, 2, &line, 10 );
	int			sec	= strntol( line, 2, &line, 10 );

	time = hours * 3600 + min * 60 + sec;

	// Skip .0,
	line = index( line, ',' );
	if( !line )
		return;
	line++;

	// Convert the lattitude
	float			lat	= strntol( line, 2, &line, 10 );
	float			lat_min	= strtod( line, &line );

	latitude = lat + lat_min / 60.0;

	// Find the sign for the lattitude.  South is negative
	if( line[1] == 'S' )
	latitude *= -1;
	line += 3; // Skip ,S,

	// Convert the longitude
	float			lng	= strntol( line, 3, &line, 10 );
	float			lng_min	= strtod( line, &line );

	longitude = lng + lng_min / 60.0;

	if( line[1] == 'W' )
		 longitude	*= -1;
	line += 3; // Skip ,W,
	

	// Now for the easier ones...

	quality	= strtol( line, &line, 10 );
	line++;		// Skip ,

	num_sats	= strtol( line, &line, 10 );
	line++;		// Skip ,

	hdop	= strtod( line, &line );
	line++;		// Skip ,

	altitude	= strtod( line, &line );
	line += 3;	// Skip ,M,

	wgs_alt	= strtod( line, &line );
	line += 3;	// Skip ,M,
}

void gpvgt_update(const char *		line_in)
{
/*****************added by ourselves***************************************************/

	char * line = ( char * )line_in;
	line = index( line, ',' );                  //skip $GPVTG
	if( !line )
		return;
	line++;

	track = strtod( line, &line );        // magnetic track

	line = index( line, 'N' );
        if( !line )
	    return;
        line = line + 2; 
   
             
	ground_speed = strtod( line, &line ) * 1000.0 / 3600.0;   // m/s
/**************************************************************************************/
}

