/* 
**
** Copyright 2006, The Android Open Source Project
** Copyright 2009, Michael Trimarchi <michael@panicking.kicks-ass.org>
** Copyright 2011, Eduardo Jos� Tagle <ejtagle@tutopia.com>
** Copyright 2014, Alexey Hohlov <root@amper.me>
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free
** Software Foundation; either version 2, or (at your option) any later
** version.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
** FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
** more details.
**
** You should have received a copy of the GNU General Public License along with
** this program; if not, write to the Free Software Foundation, Inc., 59
** Temple Place - Suite 330, Boston, MA 02111-1307, USA.
**/

#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <linux/socket.h>
#include <sys/socket.h>

#define  LOG_TAG  "GPS"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

#define  GPS_DEBUG  0

#define LOGD(...) ((void)ALOG(LOG_DEBUG, LOG_TAG, __VA_ARGS__))
#define LOGE(...) ((void)ALOG(LOG_ERROR, LOG_TAG, __VA_ARGS__))
#define LOGV(...) ((void)ALOG(LOG_VERBOSE, LOG_TAG, __VA_ARGS__))

#define D(level, ...) if (gpsDebug >= level) LOGD(__VA_ARGS__)

typedef void (*start_t)(void*);

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  200

enum {
	STATE_QUIT  = 0,
	STATE_INIT  = 1,
	STATE_START = 2
};

typedef struct {
	int		pos;
	int		overflow;
	int		utc_year;
	int		utc_mon;
	int		utc_day;
	int		utc_diff;
	GpsLocation	fix;
	GpsSvStatus	sv_status;
	int		sv_status_changed;
	int		active_count;
	char		in[ NMEA_MAX_SIZE+1 ];
	
	bool		gga_changed;
	bool		gll_changed;
	bool		gsa_changed;
	bool		gsv_changed;
	bool		rmc_changed;
	bool		vtg_changed;
	bool		zda_changed;
} NmeaReader;


typedef struct {
	volatile int		init;
	GpsCallbacks		callbacks;
	AGpsCallbacks		a_callbacks;
	GpsXtraCallbacks	xtra_callbacks;
	GpsStatus		gps_status;
	char			nmea_buf[512];
	int			nmea_len;
	pthread_t		thread;
	sem_t			fix_sem;
	pthread_t		tmr_thread;
	int			control[2];
	int			min_interval; // in ms
	NmeaReader		reader;

	int			prev_act;
} GpsState;


/* Since NMEA parser requires locks */
/*#define GPS_STATE_LOCK_FIX(_s)			\
{						\
	int ret;				\
	do {					\
		ret = sem_wait(&(_s)->fix_sem);	\
	} while (ret < 0 && errno == EINTR);	\
}

#define GPS_STATE_UNLOCK_FIX(_s)		\
	sem_post(&(_s)->fix_sem)
*/
static GpsState _gps_state[1];
static GpsState *gps_state = _gps_state;

//static void *gps_timer_thread( void* arg );

static void gps_status_cb( GpsState* state , GpsStatusValue status);
static void gps_location_cb( GpsState* state );
static void gps_sv_status_cb( GpsState* state );

static char gpsDebug = 0;

typedef struct {
	unsigned int speed;
	unsigned int value;
} speed_map;

static const speed_map speeds[] = {
	{B0,		0},
	{B50,		50},
	{B75,		75},
	{B110,		110},
	{B134,		134},
	{B150,		150},
	{B200,		200},
	{B300,		300},
	{B600,		600},
	{B1200,		1200},
	{B1800,		1800},
	{B2400,		2400},
	{B4800,		4800},
	{B9600,		9600},
	{B19200,	19200},
	{B38400,	38400},
	{B57600,	57600},
	{B115200,	115200},
	{B230400,	230400},
	{B460800,	460800},
};

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

enum { NUM_SPEEDS = COUNT_OF(speeds) };

static unsigned int baud2int(speed_t speed)
{
	unsigned char i;

	for (i = 0; i < NUM_SPEEDS; i++) {
		if (speed == speeds[i].speed) {
			return speeds[i].value;
		}
	}

	return 0;
}

static speed_t int2baud(unsigned int value)
{
	unsigned char i;

	for (i = 0; i < NUM_SPEEDS; i++) {
		if (value == speeds[i].value) {
			return speeds[i].speed;
		}
	}

	return B0;
}

static int serial_open( const char* name, const int baud )
{
	int fd = -1;

	do {
		fd = open( name, O_RDWR );
	} while (fd < 0 && errno == EINTR);

	if (fd < 0) {
		LOGE("could not open serial device %s: %s", name, strerror(errno) );
		return fd;
	}

	// disable echo on serial lines
	if ( isatty( fd ) ) {
		struct termios ios;
		speed_t speed;

		tcgetattr( fd, &ios );
		ios.c_lflag = 0;			/* disable ECHO, ICANON, etc... */
		ios.c_oflag &= (~ONLCR);		/* Stop \n -> \r\n translation on output */
		ios.c_iflag &= (~(ICRNL | INLCR));	/* Stop \r -> \n & \n -> \r translation on input */
		ios.c_iflag |= (IGNCR | IXOFF);		/* Ignore \r & XON/XOFF on input */

		speed = int2baud(baud);

		if (speed != B0) {
			cfsetispeed(&ios, speed);
			cfsetospeed(&ios, speed);
		}

		tcsetattr( fd, TCSANOW, &ios );
	}

	return fd;
}

/*
static void serial_write(int fd, char *msg)
{
	int i, n, ret;

	i = strlen(msg);
	n = 0;

	D("serial_write: start %s", msg);

	do {
		ret = write(fd, msg + n, i - n);

		if (ret < 0 && errno == EINTR) {
			continue;
		}

		if (ret < 0) {
			LOGE("Error writing to serial port: %d",errno);
			return;
		}

		n += ret;

	} while (n < i);

	D("serial_write: end %s", msg);

	return;
}
*/
static int serial_get_ok(int fd)
{
	char buf[64+1];
	int n;
	int ret;

	do {
		n = 0;
		do {
			ret = read(fd, &buf[n], 1);

			if (ret < 0 && errno == EINTR) {
				continue;
			}

			if (ret < 0) {
				LOGE("Error reading from serial port: %d",errno);
				return 0;
			}

			n ++;

		} while (n < 64 && buf[n-1] != '\n');

		// Remove the trailing spaces
		while (n > 0 && buf[n-1] <= ' ' && buf[n-1] != 0)
			n--;
		buf[n] = 0;

		// Remove the leading spaces
		n = 0;
		while (buf[n]<= ' ' && buf[n] != 0)
			n++;

	} while (buf[0] == 0); // Ignore blank lines

	D(1, "got answer : '%s'",&buf[n]);
	return strcmp(&buf[n],"OK") == 0;
}


/**************************************************************/
/**************************************************************/
/*******						*******/
/*******	D E V I C E				*******/
/*******						*******/
/**************************************************************/
/**************************************************************/

static void dev_power(int state)
{
	char	prop[PROPERTY_VALUE_MAX];
	int	fd;
	char	cmd = '0';
	int	ret;

	if (property_get("gps.power_on",prop,"") == 0) {
		LOGE("no gps power interface name");
		return;
	}

	do {
		fd = open( prop, O_RDWR );
	} while (fd < 0 && errno == EINTR);

	if (fd < 0) {
		LOGE("could not open gps power interface: %s", prop );
		return;
	}

	if (state) {
		cmd = '1';
	} else {
		cmd = '0';
	}

	do {
		ret = write( fd, &cmd, 1 );
	} while (ret < 0 && errno == EINTR);

	close(fd);

	D(1, "gps power state = %c", cmd);

	if (state)
		usleep(500*1000);

	return;
}

/**************************************************************/
/**************************************************************/
/*******						*******/
/*******	N M E A   T O K E N I Z E R		*******/
/*******						*******/
/**************************************************************/
/**************************************************************/

typedef struct {
	const char* p;
	const char* end;
} Token;

#define MAX_NMEA_TOKENS 32

typedef struct {
	int	count;
	Token	tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int nmea_tokenizer_init( NmeaTokenizer* t, const char* p, const char* end )
{
	int	count = 0;
	char*	q;

	// the initial '$' is optional
	if (p < end && p[0] == '$')
		p += 1;

	// remove trailing newline
	if (end > p && end[-1] == '\n') {
		end -= 1;

		if (end > p && end[-1] == '\r')
			end -= 1;
	}

	// get rid of checksum at the end of the sentecne
	if (end >= p+3 && end[-3] == '*') {
		end -= 3;
	}

	while (p < end) {
		const char*  q = p;

		q = memchr(p, ',', end-p);
		if (q == NULL)
			q = end;

		if (count < MAX_NMEA_TOKENS) {
			t->tokens[count].p   = p;
			t->tokens[count].end = q;
			count += 1;
		}

		if (q < end)
			q += 1;

		p = q;
	}

	t->count = count;
	return count;
}

static Token nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
	Token  tok;
	static const char*  dummy = "";

	if (index < 0 || index >= t->count) {
		tok.p = tok.end = dummy;
	} else
		tok = t->tokens[index];

	return tok;
}


static int str2int( const char*  p, const char*  end )
{
	int result = 0;
	int len = end - p;

	if (len == 0) {
		return -1;
	}

	for ( ; len > 0; len--, p++ )
	{
		int  c;

		if (p >= end)
			goto Fail;

		c = *p - '0';
		if ((unsigned)c >= 10)
			goto Fail;

		result = result*10 + c;
	}
	return  result;

Fail:
	return -1;
}

static double str2float( const char*  p, const char*  end )
{
	//int   result = 0;
	int   len    = end - p;
	char  temp[16];
	int	  i;

	if (len == 0) {
		return -1.0;
	}

	if (len >= (int)sizeof(temp))
		return 0.;

	//memcpy( temp, p, len );
	for (i = 0; i < len; i++)
		temp[i] = p[i];

	temp[len] = 0;
	return strtod( temp, NULL );
}

/** @desc Convert struct tm to time_t (time zone neutral).
 *
 * The one missing function in libc: It works basically like mktime, with the main difference that
 * it does no time zone-related processing but interprets the members of the struct tm as UTC.
 * Unlike mktime, it will not modify any fields of the tm structure; if you need this behavior, call
 * mktime before this function.
 *
 * @param t Pointer to a struct tm containing date and time. Only the tm_year, tm_mon, tm_mday,
 * tm_hour, tm_min and tm_sec members will be evaluated, all others will be ignored.
 *
 * @return The epoch time (seconds since 1970-01-01 00:00:00 UTC) which corresponds to t.
 *
 * @author Originally written by Philippe De Muyter <phdm@macqel.be> for Lynx.
 * http://lynx.isc.org/current/lynx2-8-8/src/mktime.c
 */

static time_t mkgmtime(struct tm *t)
{
	short month, year;
	time_t result;
	static const int m_to_d[12] =
		{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

	month = t->tm_mon;
	year = t->tm_year + month / 12 + 1900;
	month %= 12;
	if (month < 0) {
		year -= 1;
		month += 12;
	}
	result = (year - 1970) * 365 + m_to_d[month];
	if (month <= 1)
		year -= 1;
	result += (year - 1968) / 4;
	result -= (year - 1900) / 100;
	result += (year - 1600) / 400;
	result += t->tm_mday;
	result -= 1;
	result *= 24;
	result += t->tm_hour;
	result *= 60;
	result += t->tm_min;
	result *= 60;
	result += t->tm_sec;
	return (result);
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
	time_t		now = time(NULL);
	struct tm	tm_local;
	struct tm	tm_utc;
	long		time_local, time_utc;

	gmtime_r( &now, &tm_utc );
	localtime_r( &now, &tm_local );

	time_local = mktime(&tm_local);
	time_utc = mktime(&tm_utc);

	r->utc_diff = time_local - time_utc;
}


static void nmea_reader_init( NmeaReader*  r )
{
	int i;
	memset( r, 0, sizeof(*r) );

	// Initialize the sizes of all the structs we use
	r->fix.size = sizeof(GpsLocation);
	r->sv_status.size = sizeof(GpsSvStatus);
	for (i = 0; i < GPS_MAX_SVS; i++) {
		r->sv_status.sv_list[i].size = sizeof(GpsSvInfo);
	}

	r->pos      = 0;
	r->overflow = 0;
	r->utc_year = -1;
	r->utc_mon  = -1;
	r->utc_day  = -1;

	// not sure if we still need this (this module doesn't use utc_diff)
	nmea_reader_update_utc_diff( r );
}

static int nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
	int	hour, minute, seconds, milliseconds;
	struct	tm tm;
	time_t	fix_time;

	if (tok.p + 6 > tok.end)
		return -1;

	if (r->utc_year < 0) {
		// no date, can't return valid timestamp (never ever make up a date, this could wreak havoc)
		return -1;
	}
	else
	{
		tm.tm_year = r->utc_year - 1900;
		tm.tm_mon  = r->utc_mon - 1;
		tm.tm_mday = r->utc_day;
	}

	hour    = str2int(tok.p,   tok.p+2);
	minute  = str2int(tok.p+2, tok.p+4);
	seconds = str2int(tok.p+4, tok.p+6);

	// parse also milliseconds (if present) for better precision
	milliseconds = 0;
	if (tok.end - (tok.p+7) == 2) {
		milliseconds = str2int(tok.p+7, tok.end) * 10;
	}
	else if (tok.end - (tok.p+7) == 1) {
		milliseconds = str2int(tok.p+7, tok.end) * 100;
	}
	else if (tok.end - (tok.p+7) >= 3) {
		milliseconds = str2int(tok.p+7, tok.p+10);
	}

	// the following is only guaranteed to work if we have previously set a correct date, so be sure
	// to always do that before

	tm.tm_hour = hour;
	tm.tm_min  = minute;
	tm.tm_sec  = seconds;

	fix_time = mkgmtime( &tm );

	r->fix.timestamp = (long long)fix_time * 1000 + milliseconds;
	return 0;
}

static int nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{

	if ( (tok_d.p + 2 > tok_d.end) ||
		(tok_m.p + 2 > tok_m.end) ||
		(tok_y.p + 4 > tok_y.end) )
			return -1;

	r->utc_day = str2int(tok_d.p,   tok_d.p+2);
	r->utc_mon = str2int(tok_m.p, tok_m.p+2);
	r->utc_year = str2int(tok_y.p, tok_y.p+4);

	return 0;
}

static int nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
	Token  tok = date;
	int    day, mon, year;

	if (tok.p + 6 != tok.end) {
		D(1, "date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
		return -1;
	}
	day  = str2int(tok.p, tok.p+2);
	mon  = str2int(tok.p+2, tok.p+4);
	year = str2int(tok.p+4, tok.p+6) + 2000;

	if ((day|mon|year) < 0) {
		D(1, "date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
		return -1;
	}

	r->utc_year  = year;
	r->utc_mon   = mon;
	r->utc_day   = day;

	return nmea_reader_update_time( r, time );
}


static double convert_from_hhmm( Token  tok )
{
	double	val	= str2float(tok.p, tok.end);
	int	degrees	= (int)(floor(val) / 100);
	double	minutes	= val - degrees*100.;
	double	dcoord	= degrees + minutes / 60.0;
	return dcoord;
}


static int nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
	double   lat, lon;
	Token    tok;

	tok = latitude;
	if (tok.p + 6 > tok.end) {
		D(1, "latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
		return -1;
	}
	lat = convert_from_hhmm(tok);
	if (latitudeHemi == 'S')
	lat = -lat;

	tok = longitude;
	if (tok.p + 6 > tok.end) {
		D(1, "longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
		return -1;
	}
	lon = convert_from_hhmm(tok);
	if (longitudeHemi == 'W')
		lon = -lon;

	r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
	r->fix.latitude  = lat;
	r->fix.longitude = lon;
	return 0;
}

static int nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
	//double  alt;
	Token   tok = altitude;

	if (tok.p >= tok.end)
		return -1;

	r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
	r->fix.altitude = str2float(tok.p, tok.end);
	return 0;
}

static int nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
	//double  acc;
	Token   tok = accuracy;

	if (tok.p >= tok.end)
		return -1;

	r->fix.accuracy = str2float(tok.p, tok.end);

	if (r->fix.accuracy >= 99.99){
		return 0;
	}

	D(1, "Accuracy: %.3f", r->fix.accuracy);

	r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
	return 0;
}

static int nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
	//double  alt;
	Token   tok = bearing;

	if (tok.p >= tok.end)
		return -1;

	r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
	r->fix.bearing  = str2float(tok.p, tok.end);
	return 0;
}


static int nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
	//double  alt;
	Token   tok = speed;

	if (tok.p >= tok.end)
		return -1;

	r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
	// convert knots into m/sec (1 knot equals 1.852 km/h, 1 km/h equals 3.6 m/s)
	// since 1.852 / 3.6 is an odd value (periodic), we're calculating the quotient on the fly
	// to obtain maximum precision (we don't want 1.9999 instead of 2)
	r->fix.speed    = str2float(tok.p, tok.end) * 1.852 / 3.6;
	return 0;
}


static void nmea_reader_parse( NmeaReader*  r )
{
	/* we received a complete sentence, now parse it to generate
	* a new GPS fix...
	*/
	NmeaTokenizer	tzer[1];
	Token		tok;

	D(2, "Received: '%.*s'", r->pos, r->in);

	if (r->pos < 9) {
		D(2, "Too short. discarded.");
		return;
	}

	nmea_tokenizer_init(tzer, r->in, r->in + r->pos);

	tok = nmea_tokenizer_get(tzer, 0);

	if (tok.p + 5 > tok.end) {
		D(2, "sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
		return;
	}

	// ignore first two characters.
	tok.p += 2;

	if ( !memcmp(tok.p, "GGA", 3) ) {
		// GPS fix
		Token tok_fixstaus = nmea_tokenizer_get(tzer,6);

		if ((tok_fixstaus.p[0] > '0') && (r->utc_year >= 0)) {
			// ignore this until we have a valid timestamp
			Token tok_time		= nmea_tokenizer_get(tzer,1);
			Token tok_latitude	= nmea_tokenizer_get(tzer,2);
			Token tok_latitudeHemi	= nmea_tokenizer_get(tzer,3);
			Token tok_longitude	= nmea_tokenizer_get(tzer,4);
			Token tok_longitudeHemi	= nmea_tokenizer_get(tzer,5);
			Token tok_altitude	= nmea_tokenizer_get(tzer,9);
			Token tok_altitudeUnits	= nmea_tokenizer_get(tzer,10);

			// don't use this as we have no fractional seconds and no date; there are better ways to
			// get a good timestamp from GPS
			//nmea_reader_update_time(r, tok_time);
			nmea_reader_update_latlong(r, tok_latitude,
				tok_latitudeHemi.p[0],
				tok_longitude,
				tok_longitudeHemi.p[0]);
			nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
			
			r->gga_changed = true;
		}

		Token tok_active = nmea_tokenizer_get(tzer,7);
		D(1, "Active satellites: %i", str2int(tok_active.p, tok_active.end));

	} else if ( !memcmp(tok.p, "GLL", 3) ) {

		Token tok_fixstaus = nmea_tokenizer_get(tzer,6);

		if ((tok_fixstaus.p[0] == 'A') && (r->utc_year >= 0)) {
			// ignore this until we have a valid timestamp
			Token tok_latitude	= nmea_tokenizer_get(tzer,1);
			Token tok_latitudeHemi	= nmea_tokenizer_get(tzer,2);
			Token tok_longitude	= nmea_tokenizer_get(tzer,3);
			Token tok_longitudeHemi	= nmea_tokenizer_get(tzer,4);
			Token tok_time		= nmea_tokenizer_get(tzer,5);

			// don't use this as we have no fractional seconds and no date; there are better ways to
			// get a good timestamp from GPS
			//nmea_reader_update_time(r, tok_time);
			nmea_reader_update_latlong(r, tok_latitude,
				tok_latitudeHemi.p[0],
				tok_longitude,
				tok_longitudeHemi.p[0]);
			
			r->gll_changed = true;
		}

	} else if ( !memcmp(tok.p, "GSA", 3) ) {

		Token tok_fixStatus = nmea_tokenizer_get(tzer, 2);
		int i;

		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {
			Token tok_accuracy = nmea_tokenizer_get(tzer, 15);

			nmea_reader_update_accuracy(r, tok_accuracy);

			r->sv_status.used_in_fix_mask = 0ul;
			r->active_count = 0;

			for (i = 3; i <= 14; ++i) {
				Token tok_prn = nmea_tokenizer_get(tzer, i);
				int prn = str2int(tok_prn.p, tok_prn.end);

				if (prn > 0) {
					r->sv_status.used_in_fix_mask |= (1ul << (prn - 1));
					r->active_count++;
				}
			}
			r->sv_status_changed = 1;
			r->gsa_changed = true;
			//D(1, "Active satellites: %i, Total satellites: %i", cuur, r->sv_status.num_svs);
		}

	} else if ( !memcmp(tok.p, "GSV", 3) ) {

		Token tok_noSatellites = nmea_tokenizer_get(tzer, 3);
		int noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

		if (noSatellites > 0) {
			Token tok_noSentences	= nmea_tokenizer_get(tzer, 1);
			Token tok_sentence	= nmea_tokenizer_get(tzer, 2);

			int sentence = str2int(tok_sentence.p, tok_sentence.end);
			int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
			int curr;
			int i;

			if (sentence == 1) {
				r->sv_status_changed = 0;
				r->sv_status.num_svs = 0;

				for (i = 0; i < 32; i++) {
					r->sv_status.sv_list[i].prn = 0;
					r->sv_status.sv_list[i].elevation = 0.0;
					r->sv_status.sv_list[i].azimuth = 0.0;
					r->sv_status.sv_list[i].snr = 0;
				}
			}

			curr = r->sv_status.num_svs;
			i = 0;

			while (i < 4 && r->sv_status.num_svs < noSatellites) {
				Token tok_prn		= nmea_tokenizer_get(tzer, i * 4 + 4);
				Token tok_elevation	= nmea_tokenizer_get(tzer, i * 4 + 5);
				Token tok_azimuth	= nmea_tokenizer_get(tzer, i * 4 + 6);
				Token tok_snr		= nmea_tokenizer_get(tzer, i * 4 + 7);
				
				r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
				r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
				r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
				r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);
				if (r->sv_status.sv_list[curr].snr < 0)
					r->sv_status.sv_list[curr].snr = 0;


				curr++;
				i++;
				
				r->sv_status.num_svs++;
			}

			if (sentence == totalSentences) {
				r->sv_status_changed = 1;
			}

			D(2, "%s: GSV message with total satellites %d", __FUNCTION__, noSatellites);
		}

	} else if ( !memcmp(tok.p, "RMC", 3) ) {

		Token tok_fixStatus = nmea_tokenizer_get(tzer,2);

		if (tok_fixStatus.p[0] == 'A') {
			Token tok_time		= nmea_tokenizer_get(tzer,1);
			Token tok_latitude	= nmea_tokenizer_get(tzer,3);
			Token tok_latitudeHemi	= nmea_tokenizer_get(tzer,4);
			Token tok_longitude	= nmea_tokenizer_get(tzer,5);
			Token tok_longitudeHemi	= nmea_tokenizer_get(tzer,6);
			Token tok_speed		= nmea_tokenizer_get(tzer,7);
			Token tok_bearing	= nmea_tokenizer_get(tzer,8);
			Token tok_date		= nmea_tokenizer_get(tzer,9);

			nmea_reader_update_date( r, tok_date, tok_time );

			nmea_reader_update_latlong( r, tok_latitude,
				tok_latitudeHemi.p[0],
				tok_longitude,
				tok_longitudeHemi.p[0] );

			nmea_reader_update_bearing( r, tok_bearing );
			nmea_reader_update_speed  ( r, tok_speed );
			
			r->rmc_changed = true;
		}

	} else if ( !memcmp(tok.p, "VTG", 3) ) {

		Token tok_fixStatus = nmea_tokenizer_get(tzer,9);

		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N') {
			Token tok_bearing	= nmea_tokenizer_get(tzer,1);
			Token tok_speed		= nmea_tokenizer_get(tzer,5);

			nmea_reader_update_bearing( r, tok_bearing );
			nmea_reader_update_speed( r, tok_speed );
			
			r->vtg_changed = true;
		}

	} else if ( !memcmp(tok.p, "ZDA", 3) ) {

		Token tok_time = nmea_tokenizer_get(tzer,1);
		Token tok_year = nmea_tokenizer_get(tzer,4);

		if ((tok_year.p[0] != '\0') && (tok_time.p[0] != '\0')) {
			// make sure to always set date and time together, lest bad things happen
			Token tok_day	= nmea_tokenizer_get(tzer,2);
			Token tok_mon	= nmea_tokenizer_get(tzer,3);

			nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
			nmea_reader_update_time(r, tok_time);
			
			r->zda_changed = true;
		}
	} else {
		tok.p -= 2;
		D(2, "unknown sentence '%.*s", tok.end-tok.p, tok.p);
	}

//	if (r->sv_status_changed != 0) {
	if ((r->fix.flags != 0) && (r->sv_status_changed != 0)) {
	    if (r->gga_changed &&  r->gsa_changed &&
		    r->rmc_changed) {// && r->vtg_changed && r->zda_changed) {
		GpsState*  state = _gps_state;
//		D(2, "gps sv status callback");
//		//D(1, "fix mask: %u", state->reader.sv_status.used_in_fix_mask);
		gps_location_cb( state );
		gps_sv_status_cb( state );
		//gps_sv_status_cb( state );
//		if (state->callbacks.sv_status_cb) {
//			state->callbacks.sv_status_cb( &r->sv_status );
//			r->sv_status_changed = 0;
//		}
//		gps_location_cb( state );
		
		r->gga_changed = false;
		//r->gll_changed = false;
		r->gsa_changed = false;
		r->rmc_changed = false;
		//r->vtg_changed = false;
		//r->zda_changed = false;
		
//		GPS_STATE_UNLOCK_FIX(state);
	    }
	}

//	if (r->sv_status_changed != 0) {
//	if (r->fix.flags != 0) {
//		GpsState*  state = _gps_state;
//		gps_location_cb( state );
//	}
}

static void nmea_reader_addc( NmeaReader*  r, int  c )
{
	if (r->overflow) {
		r->overflow = (c != '\n');
		return;
	}

	if (r->pos >= (int) sizeof(r->in)-1 ) {
		r->overflow = 1;
		r->pos = 0;
		return;
	}

	r->in[r->pos] = (char)c;
	r->pos += 1;

	if (c == '\n') {
		nmea_reader_parse( r );
		r->pos = 0;
	}
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
	CMD_QUIT  = 0,
	CMD_START = 1,
	CMD_STOP  = 2
};


static void gps_state_start( GpsState*  s )
{
	char  cmd = CMD_START;
	int   ret;

	do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

	if (ret != 1)
		D(1, "%s: could not send CMD_START command: ret=%d: %s",
			__FUNCTION__, ret, strerror(errno));
}


static void gps_state_stop( GpsState*  s )
{
	char  cmd = CMD_STOP;
	int   ret;

	do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

	if (ret != 1)
		D(1, "%s: could not send CMD_STOP command: ret=%d: %s",
			__FUNCTION__, ret, strerror(errno));
}


static int epoll_register( int  epoll_fd, int  fd )
{
	struct epoll_event  ev;
	int                 ret, flags;

	/* important: make the fd non-blocking */
	flags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	ev.events  = EPOLLIN;
	ev.data.fd = fd;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
	} while (ret < 0 && errno == EINTR);
	return ret;
}


static int epoll_deregister( int  epoll_fd, int  fd )
{
	int  ret;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
	} while (ret < 0 && errno == EINTR);
	return ret;
}

static void gps_nmea_thread_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	state->callbacks.nmea_cb(state->reader.fix.timestamp,&state->nmea_buf[0],state->nmea_len);
	//GPS_STATE_UNLOCK_FIX(state);
}

static void gps_nmea_cb( GpsState* state , const char* buf, int len)
{
	int i;

	D(2, "%s()", __FUNCTION__ );
	// Forward NMEA sentences ....
	if (state->callbacks.nmea_cb) {

		//GPS_STATE_LOCK_FIX(state);
		//memcpy(&state->nmea_buf[0],buf,len);
		for (i = 0; i < len; i++)
			state->nmea_buf[i] = buf[i];

		state->nmea_buf[len] = 0;
		state->nmea_len = len;
		state->callbacks.create_thread_cb("nmea",(start_t)gps_nmea_thread_cb,(void*)state);
	}
}

static void gps_status_thread_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	state->callbacks.status_cb(&state->gps_status);
	//GPS_STATE_UNLOCK_FIX(state);
}

static void gps_status_cb( GpsState* state , GpsStatusValue status)
{
	D(2, "%s()", __FUNCTION__ );
	if (state->callbacks.status_cb) {
		//GPS_STATE_LOCK_FIX(state);

		state->gps_status.size = sizeof(GpsStatus);
		state->gps_status.status = status;
		state->callbacks.create_thread_cb("status",(start_t)gps_status_thread_cb,(void*)state);

//state->callbacks.status_cb(&state->gps_status);
//GPS_STATE_UNLOCK_FIX(state);

		D(2, "gps status callback: 0x%x", status);
	}
}

static void gps_set_capabilities_cb( GpsState* state , uint32_t caps)
{
	D(2, "%s()", __FUNCTION__ );
	if (state->callbacks.set_capabilities_cb) {
		state->callbacks.create_thread_cb("caps",(start_t)state->callbacks.set_capabilities_cb,(void*)caps);
	}
}

static void gps_location_thread_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	//if (state->reader.fix.flags > 0)
	{
		state->callbacks.location_cb( &state->reader.fix );
		state->reader.fix.flags = 0;
		//GPS_STATE_UNLOCK_FIX(state);
	}
}


static void gps_location_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	if (state->callbacks.location_cb) {
		//GPS_STATE_LOCK_FIX(state);
		state->callbacks.create_thread_cb("fix",(start_t)gps_location_thread_cb,(void*)state);
	}
}

static void gps_sv_status_thread_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	if (state->reader.sv_status_changed > 0)
	{
		state->callbacks.sv_status_cb( &state->reader.sv_status );
		state->reader.sv_status_changed = 0;
		//GPS_STATE_UNLOCK_FIX(state);
	}
}


static void gps_sv_status_cb( GpsState* state )
{
	D(2, "%s()", __FUNCTION__ );
	if (state->callbacks.sv_status_cb) {
		//GPS_STATE_LOCK_FIX(state);
		state->callbacks.create_thread_cb("sv-status",(start_t)gps_sv_status_thread_cb,(void*)state);
	}
}

static int open_gps()
{
	char device[128], prop_value[PROPERTY_VALUE_MAX+20];
	int  gps_fd = -1;

	property_get("gps.debug", prop_value, "0");
	gpsDebug = atoi(prop_value);

	D(1, "open_gps start");

	// Power up the GPS interface
	dev_power(1);

	// look for a kernel-provided gps_data port
	property_get("ro.kernel.android.gps",prop_value,"ttyS4");
	snprintf(device, sizeof(device), "/dev/%s", prop_value);

	property_get("ro.kernel.android.gps_baud", prop_value, "9600");

	gps_fd = serial_open( device, atoi(prop_value) );
	if (gps_fd < 0) {
        	LOGE("could not open gps serial device %s: %s", device, strerror(errno) );
		dev_power(0);
		return -1;
	}

	D(1, "gps will read from %s at %i baud", device, atoi(prop_value));

	// And return the GPS data handle
	return gps_fd;
}

static void close_gps(int gps_fd)
{
	//int gps_ctl_fd = -1;

	D(1, "close_gps start");

	if ( gps_fd >= 0 ) {
		// Reduce NMEA message ratess
		close(gps_fd);
	}

	// Power down the GPS interface
	dev_power(0);

	D(1, "close_gps end");
}


/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void* gps_state_thread( void* arg )
{
	GpsState*	state = (GpsState*) arg;
	NmeaReader	*reader;
	int		epoll_fd = epoll_create(2);
	int		gps_fd = -1;
	int		control_fd = state->control[1];

	reader = &state->reader;

	nmea_reader_init( reader );

	gps_fd = open_gps();

	// register control file descriptors for polling
	epoll_register( epoll_fd, control_fd );
	epoll_register( epoll_fd, gps_fd );

	D(1, "gps thread running");

	gps_set_capabilities_cb( state , GPS_CAPABILITY_MSA | GPS_CAPABILITY_MSB );

	gps_status_cb( state , GPS_STATUS_ENGINE_ON);

	// now loop
	for (;;) {
		struct epoll_event	events[2];
		int			ne, nevents;

		nevents = epoll_wait( epoll_fd, events, 2, -1 );
		if (nevents < 0) {
			if (errno != EINTR)
				LOGE("epoll_wait() unexpected error: %s", strerror(errno));
			continue;
		}

		D(2, "gps thread received %d events", nevents);
		for (ne = 0; ne < nevents; ne++) {
			if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
				D(2, "EPOLLERR or EPOLLHUP after epoll_wait() - Means GPS was turned off");
				goto Exit;
			}

			if ((events[ne].events & EPOLLIN) != 0) {
				int  fd = events[ne].data.fd;

				if (fd == control_fd)
				{
					char  cmd = 255;
					int   ret;
					D(2, "gps control fd event");
					do {
						ret = read( fd, &cmd, 1 );
					} while (ret < 0 && errno == EINTR);

					if (cmd == CMD_QUIT) {
						D(2, "gps thread quitting on demand");
						goto Exit;
					} else if (cmd == CMD_START) {
						//if (gps_fd < 0)
						{
							D(2, "gps thread starting  location_cb=%p", state->callbacks.location_cb);

							// Open the GPS
							//gps_fd = open_gps();
							if ( gps_fd < 0 ) {
								state->init = STATE_INIT;
								goto Exit;
							}

							// Add it to the monitoring set
							//epoll_register( epoll_fd, gps_fd );

							// Initialize NMEA GPS
							gps_status_cb( state , GPS_STATUS_SESSION_BEGIN);

							state->init = STATE_START; /* The timer thread depends on this to properly start */

							//if ( pthread_create( &state->tmr_thread, NULL, gps_timer_thread, state ) != 0 ) {
							//	LOGE("could not create gps timer thread: %s", strerror(errno));

								// Remove it from the monitoring set
								//epoll_deregister( epoll_fd, gps_fd );
								//close_gps( gps_fd );
								//gps_fd = -1;

							//	state->init = STATE_INIT;
							//	goto Exit;
							//}
						}
					} else if (cmd == CMD_STOP) {
						if (gps_fd >= 0) {
							//void *dummy;
							D(2, "gps thread stopping");

							state->init = STATE_INIT; /* Must be here, as timer thread will only quit with this */
							//pthread_join(state->tmr_thread, &dummy);
							//pthread_destroy(state->tmr_thread);

							gps_status_cb( state , GPS_STATUS_SESSION_END);

							// Remove it from the monitoring set
							//epoll_deregister( epoll_fd, gps_fd );
							//close_gps( gps_fd );
							//gps_fd = -1;

						}
					}
				} else if (fd == gps_fd) {
					char buf[512];
					int  nn, ret;

					do {
						ret = read( fd, buf, sizeof(buf) );
					} while (ret < 0 && errno == EINTR);

					if (ret < 0 && errno == EIO) {
						// Probably, the GPS turned off...
						//void *dummy;
						D(2, "gps was suspended ... ");

						state->init = STATE_INIT; /* Must be here, as timer thread will only quit with this */
						//pthread_join(state->tmr_thread, &dummy);
						//pthread_destroy(state->tmr_thread);

						gps_status_cb( state , GPS_STATUS_SESSION_END);

						// Remove it from the monitoring set
						//epoll_deregister( epoll_fd, gps_fd );
						//close_gps( gps_fd );
						//gps_fd = -1;

					} else if (ret > 0) {
						gps_nmea_cb( state , &buf[0], ret);

						//GPS_STATE_LOCK_FIX(state);
						for (nn = 0; nn < ret; nn++)
							nmea_reader_addc( reader, buf[nn] );
						//GPS_STATE_UNLOCK_FIX(state);
					}
					D(2, "gps fd event end");
				} else {
					LOGE("epoll_wait() returned unkown fd %d ?", fd);
				}
			}
		}
	}
Exit:
	gps_status_cb( state , GPS_STATUS_ENGINE_OFF);

	epoll_deregister( epoll_fd, gps_fd );
	close ( epoll_fd );

	return NULL;
}

//static void* gps_timer_thread( void*  arg )
//{
//	GpsState *state = (GpsState *)arg;

//	D(2, "gps entered timer thread");

//	do {
//		D(2, "gps timer exp");

//		if ((state->reader.fix.flags != 0) && (state->reader.sv_status_changed != 0)) {
//			if (state->reader.active_count <= state->reader.sv_status.num_svs)
			//if (state->prev_act >= state->reader.active_count / 2)
//			{
//			D(2, "gps fix cb: 0x%x", state->reader.fix.flags);
//				gps_sv_status_cb( state );
//				gps_location_cb( state );
//			}
			//state->prev_act = state->reader.active_count;
//		}

//		if (state->reader.sv_status_changed != 0) {
//			D(2, "gps sv status callback");
//			//D(1, "fix mask: %u", state->reader.sv_status.used_in_fix_mask);
//			gps_sv_status_cb( state );
//		}

//		usleep(state->min_interval*1000);

//	} while (state->init == STATE_START);

//	D(2, "gps timer thread destroyed");

//	return NULL;
//}

static void gps_state_done( GpsState*  s )
{
	// tell the thread to quit, and wait for it
	char   cmd = CMD_QUIT;
	void*  dummy;
	int ret;

	D(1, "gps send quit command");

	do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

	D(1, "gps waiting for command thread to stop");

	pthread_join(s->thread, &dummy);
	//pthread_destroy(s->thread);

	/* Timer thread depends on this state check */
	s->init = STATE_QUIT;
	s->min_interval = 1000;

	// close the control socket pair
	close( s->control[0] ); s->control[0] = -1;
	close( s->control[1] ); s->control[1] = -1;

	sem_destroy(&s->fix_sem);

	memset(s, 0, sizeof(*s));

	D(1, "gps deinit complete");
}

static void gps_state_init( GpsState*  state )
{
	int	ret;

	//struct sigevent tmr_event;

	state->init		= STATE_INIT;
	state->control[0]	= -1;
	state->control[1]	= -1;
	state->min_interval	= 1000;

	if (sem_init(&state->fix_sem, 0, 1) != 0) {
		LOGE("gps semaphore initialization failed! Error %d: %s", errno, strerror(errno));
		return;
	}

	if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
		LOGE("could not create thread control socket pair: %s", strerror(errno));
		goto Fail;
	}

	if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
		LOGE("could not create gps thread: %s", strerror(errno));
		goto Fail;
	}

	D(1, "gps state initialized");

	return;

Fail:
	gps_state_done( state );
}



/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int gps_init(GpsCallbacks* callbacks)
{
	GpsState*  s = _gps_state;

	if (!s->init)
		gps_state_init(s);

	s->callbacks = *callbacks;

	return 0;
}

static void gps_cleanup(void)
{
	GpsState*  s = _gps_state;

	if (s->init)
		gps_state_done(s);
}


static int gps_start(void)
{
	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	D(1, "%s: called", __FUNCTION__);
	gps_state_start(s);
	return 0;
}


static int gps_stop(void)
{
	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	D(1, "%s: called", __FUNCTION__);
	gps_state_stop(s);
	//gps_state_start(s);
	return 0;
}


static int gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
	return 0;
}

/** Injects current location from another location provider
 *  (typically cell ID).
 *  latitude and longitude are measured in degrees
 *  expected accuracy is measured in meters
 */
static int gps_inject_location(double latitude, double longitude, float accuracy)
{
	return 0;
}

static void gps_delete_aiding_data(GpsAidingData flags)
{
}

static int gps_set_position_mode(GpsPositionMode mode,  GpsPositionRecurrence recurrence,
		 uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	// only standalone supported for now.
	if (mode == GPS_POSITION_MODE_STANDALONE) {
	} else
		if (mode == GPS_POSITION_MODE_MS_BASED ||
		mode == GPS_POSITION_MODE_MS_ASSISTED) {
	}

	if (min_interval == 0)
		min_interval = 1000;
	s->min_interval = min_interval;

	D(1, "gps fix frquency set to %d secs", min_interval);

	return 0;
}

/***** AGpsInterface *****/

/**
 * Opens the AGPS interface and provides the callback routines
 * to the implemenation of this interface.
 */
static void agps_init( AGpsCallbacks* callbacks )
{
	D(1, "%s() is called", __FUNCTION__);

	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return;
	}

	s->a_callbacks = *callbacks;
}

/**
 * Notifies that a data connection is available and sets
 * the name of the APN to be used for SUPL.
 */
static int agps_conn_open( const char* apn )
{
	D(1, "%s() is called", __FUNCTION__);
	D(1, "apn=%s", apn);

	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return 0;
	}

	return 0;
}

/**
 * Notifies that the AGPS data connection has been closed.
 */
static int agps_conn_closed( void )
{
	D(1, "%s() is called", __FUNCTION__);

	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return 0;
	}

	return 0;
}

/**
 * Notifies that a data connection is not available for AGPS.
 */
static int agps_conn_failed( void )
{
	D(1, "%s() is called", __FUNCTION__);

	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return 0;
	}

	return 0;
}

/**
 * Sets the hostname and port for the AGPS server.
 */
static int agps_set_server( AGpsType type, const char* hostname, int port )
{
	char buf[512];

	D(1, "%s() is called", __FUNCTION__);
	D(1, "type=%d, hostname=%s, port=%d", type, hostname, port);

	GpsState*  s = _gps_state;

	if (!s->init) {
		D(1, "%s: called with uninitialized state !!", __FUNCTION__);
		return 0;
	}

	return 0;
}

static const AGpsInterface  sAGpsInterface = {
	sizeof(AGpsInterface),
	agps_init,
	agps_conn_open,
	agps_conn_closed,
	agps_conn_failed,
	agps_set_server
};


/***** GpsXtraInterface *****/
static int xtra_init(GpsXtraCallbacks* callbacks)
{
	D(1, "%s() is called", __FUNCTION__);
	GpsState*  s = _gps_state;

	s->xtra_callbacks = *callbacks;

	return 0;
}

static int xtra_inject_xtra_data(char* data, int length)
{
	D(1, "%s() is called", __FUNCTION__);
	D(1, "xtra size = %d, data ptr = 0x%x\n", length, (int) data);

	GpsState*  s = _gps_state;
	if (!s->init)
		return 0;

	return 0;
}

static const GpsXtraInterface sGpsXtraInterface = {
	sizeof(GpsXtraInterface),
	xtra_init,
	xtra_inject_xtra_data,
};

static const void* gps_get_extension(const char* name)
{
	D(1, "%s('%s') is called", __FUNCTION__, name);

	if (!strcmp(name, GPS_XTRA_INTERFACE)) {
		return &sGpsXtraInterface;
	} else if (!strcmp(name, AGPS_INTERFACE)) {
		return &sAGpsInterface;
	}
	return NULL;
}

static const GpsInterface sGpsInterface = {
	sizeof(GpsInterface),
	gps_init,
	gps_start,
	gps_stop,
	gps_cleanup,
	gps_inject_time,
	gps_inject_location,
	gps_delete_aiding_data,
	gps_set_position_mode,
	gps_get_extension,
};

// As under Android, there are no exceptions in C++ base system, we will hide all exported symbols
// except the required ones. This will generate better code!

#if __GNUC__ >= 4
	#define DLL_PUBLIC __attribute__ ((visibility ("default")))
	#define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
	#define DLL_PUBLIC
	#define DLL_LOCAL
#endif

static const GpsInterface* gps_get_hardware_interface(struct gps_device_t* dev)
{
	LOGV("get_interface was called");
	return &sGpsInterface;
}

static int open_gps_module(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
	struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
	memset(dev, 0, sizeof(*dev));

	dev->common.tag = HARDWARE_DEVICE_TAG;
	dev->common.version = 0;
	dev->common.module = (struct hw_module_t*)module;
	dev->get_gps_interface = gps_get_hardware_interface;

	*device = (struct hw_device_t*)dev;
	return 0;
}
 

static struct hw_module_methods_t gps_module_methods = {
	.open = open_gps_module
};

DLL_PUBLIC struct hw_module_t HAL_MODULE_INFO_SYM = {
	.tag = HARDWARE_MODULE_TAG,
	.version_major = 1,
	.version_minor = 0,
	.id = GPS_HARDWARE_MODULE_ID,
	.name = "GPS Module",
	.author = "Alexey Hohlov",
	.methods = &gps_module_methods,
};
