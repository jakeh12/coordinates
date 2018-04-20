#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#define deg_to_rad(deg) (deg * M_PI / 180.0)
#define rad_to_deg(rad) (rad * 180.0 / M_PI)


struct coord
{
	double lat;
	double lon;
	double timestamp;
};


int gps_init(char* serial_path)
{
	/* open serial port */
	int fd = open(serial_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (fd == -1) {
		fprintf(stderr, "opening serial port %s failed: %s\n", serial_path, strerror(errno));
		return fd;
	}
	
	/* set up serial port */
	struct termios options;
	tcgetattr(fd, &options);
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	options.c_iflag |= IGNCR;
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CRTSCTS;
	tcsetattr(fd, TCSAFLUSH, &options);
	
	/* return file descriptor */
	return fd;
}

void gps_close(int fd)
{
	close(fd);
}

void gps_readline(int fd, char* buffer)
{
	int i;
	char c;
	
	/* wait for '$' character */
	do
	{
		read(fd, &c, 1);
	}
	while(c != '$');
	
	/* buffer string until '\n' character is encountered */
	buffer[0] = '$';
	read(fd, &c, 1);
	for (i = 1; c != '\n' && i < 82; i++)
	{
		buffer[i] = c;
		read(fd, &c, 1);
	}
	
	/* null terminate the string */
	buffer[i+1] = '\0';
}

int gps_is_gll(char* buffer)
{
	if (strncmp(buffer, "$GPGLL", 6) == 0)
		return 1;
	return 0;
}

struct coord gps_parse_gll(char* gll_string)
{
	char buffer[16];
	struct coord parsed;

	/* parse latitude */
	strncpy(buffer, gll_string+7, 2);
	buffer[2] = '\0';
	int lat_whole = atoi(buffer);
	strncpy(buffer, gll_string+9, 8);
	buffer[8] = '\0';
	double lat_frac = atof(buffer);
	parsed.lat = lat_whole + lat_frac / 60.0;
	if (gll_string[18] == 'S')
		parsed.lat = -parsed.lat;
	
	/* parse longitude */
	strncpy(buffer, gll_string+20, 3);
	buffer[3] = '\0';
	int lon_whole = atoi(buffer);
	strncpy(buffer, gll_string+23, 8);
	buffer[8] = '\0';
	double lon_frac = atof(buffer);
	parsed.lon = lon_whole + lon_frac / 60.0;
	if (gll_string[32] == 'W')
		parsed.lon = -parsed.lon;
	
	/* parse timestamp */
	strncpy(buffer, gll_string+34, 2);
	buffer[2] = '\0';
	int hours = atoi(buffer);
	strncpy(buffer, gll_string+36, 2);
	buffer[2] = '\0';
	int minutes = atoi(buffer);
	strncpy(buffer, gll_string+38, 5);
	buffer[5] = '\0';
	double seconds = atof(buffer);
	double timestamp = hours * 3600.0 + minutes * 60.0 + seconds;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	int unix_epoch_midnight = (tv.tv_sec / 86400) * 86400;
	parsed.timestamp = timestamp + unix_epoch_midnight;
	
	/* return result */
	return parsed;
}


double coord_distance(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians and change east longitude positive by formula sign convention */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=-deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=-deg_to_rad(destination.lon);
	
	/* formula from http://www.edwilliams.org/avform.htm#Dist */
	double distance = 2*asin(sqrt(pow(sin((origin.lat-destination.lat)/2.0),2.0)+cos(origin.lat)*cos(destination.lat)*pow(sin((origin.lon-destination.lon)/2.0),2.0)));
	
	/* convert radial distance to meters (1 deg = 60 nmi, 1 nmi = 1852 m) */
	distance = rad_to_deg(distance) * 60.0 * 1852.0;
	
	/* return result */
	return distance;
}


struct coord coord_dist_radial(struct coord origin, double distance, double radial)
{
	/* convert parameters to radians and change west longitude positive by formula sign convention */
	origin.lat = deg_to_rad(origin.lat);
	origin.lon = -deg_to_rad(origin.lon);
		
	/* invert radial to use positive clockwise angle and convert to radians */ 
	radial = deg_to_rad(-radial);
	
	/* convert distance to angular distance (1 deg = 60 nmi, 1 m = 0.000539957 nmi) */
	distance = deg_to_rad(distance * 0.000539957 / 60.0);
	
	/* create struct for result coordinate */
	struct coord result;

	/* formulas from http://www.edwilliams.org/avform.htm#LL */
	result.lat = asin(sin(origin.lat)*cos(distance)+cos(origin.lat)*sin(distance)*cos(radial));
	result.lon = fmod(origin.lon-atan2(sin(radial)*sin(distance)*cos(origin.lat),cos(distance)-sin(origin.lat)*sin(result.lat))+M_PI,2.0*M_PI)-M_PI;
  
	/* convert result coordinate to degrees and switch to west longitude negative */
	result.lat = rad_to_deg(result.lat);
	result.lon = -rad_to_deg(result.lon);
  
	/* return result pointer */
	return result;
}


double coord_course(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians and east longitude positive by formula sign convention */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=-deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=-deg_to_rad(destination.lon);
	
	/* formula from http://www.edwilliams.org/avform.htm#Crs */
	double course = fmod(atan2(sin(origin.lon-destination.lon)*cos(destination.lat),cos(origin.lat)*sin(destination.lat)-sin(origin.lat)*cos(destination.lat)*cos(origin.lon-destination.lon)),2*M_PI);
	
	/* convert course to degrees and add offset for standard course range*/
	course = -rad_to_deg(course);
	
	// flip negative values around
	if (course < 0.0)
		course = course + 360.0;
	
	/* return result */
	return course;
}


void coord_print(struct coord coord)
{
	printf("%.2f: %f, %f\n", coord.timestamp, coord.lat, coord.lon);
}


int main(void)
{
	/* create struct for origin coordinate */
	struct coord origin;
	
	/* create buffer for gps messages */
	char buffer[82];

	/* initialize gps */
	int gps_fd = gps_init("/dev/tty.usbmodem1411");
	
	while(1)
	{
		/* read a message from gps */	
		gps_readline(gps_fd, buffer);
		
		/* check if message is gll */
		if (gps_is_gll(buffer))
		{
			/* parse gll message */
			origin = gps_parse_gll(buffer);
		}
		
		/* set distance and radial from the origin coordinate */
		double distance = 100.0;
		double radial = 90.0;
	
		/* calculate target coordinate */
		struct coord result = coord_dist_radial(origin, distance, radial);
	
		/* print coordinates */
		coord_print(origin);
		coord_print(result);
	
		/* calculate distance and course */
		printf("distance: %f\n", coord_distance(origin, result));
		printf("course: %f\n\n", coord_course(origin, result));
	}
	
	/* close gps */
	gps_close(gps_fd);
	
	return EXIT_SUCCESS;
}
