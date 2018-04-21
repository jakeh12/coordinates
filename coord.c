#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <pthread.h>

#define deg_to_rad(deg) (deg * M_PI / 180.0)
#define rad_to_deg(rad) (rad * 180.0 / M_PI)

static volatile int running = 1;

struct coord
{
	double lat;
	double lon;
	double timestamp;
	int valid;
};


struct gps_worker_struct
{
	int gps_fd;
	double time_offset;
	struct coord current;
	struct coord previous;
	struct coord oldest;
	pthread_mutex_t lock;
};

void keyboard_interrupt_handler(int sig_num);
void* gps_worker(void *args);
struct coord gps_get_instant(struct gps_worker_struct* gps_args);
int gps_init(char* serial_path);
void gps_close(int fd);
void gps_readline(int fd, char* buffer);
int gps_is_gll(char* buffer);
struct coord gps_parse_gll(char* gll_string);
double coord_distance(struct coord origin, struct coord destination);
struct coord coord_dist_radial(struct coord origin, double distance, double radial);
double coord_course(struct coord origin, struct coord destination);
void coord_print(struct coord coord);
void log_coord_string(struct coord coord, char* buffer);
void log_coord_string_kml(struct coord coord, char* buffer);


int main(int argc, const char * argv[])
{	
	/* catch keyboard interrupt signal (ctrl+c) and handle it */
    signal(SIGINT, keyboard_interrupt_handler);

	/* check if enough arguments are provided */
	if (argc < 2)
	{
		fprintf(stderr, "missing arguments\n");
	    printf("usage: coord serial_port [update_rate] [log_file]\n");
	    exit(EXIT_FAILURE);
	}

	/* get serial port device path */
	char serial_path[120];
	strcpy(serial_path, argv[1]);

	/* get update rate */
	int update_rate = 1;
	if (argc > 2)
	{
		update_rate = atoi(argv[2]);
		if (update_rate < 1 || update_rate > 1000000)
		{
			fprintf(stderr, "update rate needs to be between 1 and 1000000\n");
		    exit(EXIT_FAILURE);
		}
	}

	/* get log file path */
	FILE *log_file_kml = NULL;
	FILE *log_file = NULL;
	if (argc > 3)
	{
		char log_path[120];
		strcpy(log_path, argv[3]);

		/* open log file for writing */
		log_file = fopen(log_path, "w");


		///////
	    strcat(log_path, "_.kml");
		log_file_kml = fopen(log_path, "w");
		
		fprintf(log_file_kml, "%s",	"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
									"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n"
									"<Document>\n"
									"<Placemark>\n"
									"<name>Path A</name>\n"
									"<LineString>\n"
									"<tessellate>1</tessellate>\n"
									"<coordinates>\n");

		///////
	}

	/* initialize gps */
	int gps_fd = gps_init(serial_path);

	/* intialize gps polling thread */
	struct gps_worker_struct gps_args;
	gps_args.current.valid = 0;
	gps_args.previous.valid = 0;
	gps_args.oldest.valid = 0;
	gps_args.gps_fd = gps_fd;
	pthread_mutex_lock(&gps_args.lock);
	pthread_t gps_thread;
	pthread_create(&gps_thread, NULL, gps_worker, (void*)&gps_args);

	/* create coordinate to hold current position */
	struct coord current;

	/* buffer for logging coordinates */
	char log_string_buffer[64];

	/* infinite while loop to calculate instantaneous coordinates at a set rate */
	while(running)
	{

		/* get instantaneous position */
		pthread_mutex_lock(&gps_args.lock);
		current = gps_get_instant(&gps_args);
		pthread_mutex_unlock(&gps_args.lock);

		/* print the coordinate */
		coord_print(current);

		/* output to file if log file is open */
		if (log_file != NULL && current.valid > 0)
		{
			log_coord_string(current, log_string_buffer);
			fprintf(log_file, "%s\n", log_string_buffer);
			log_coord_string_kml(current, log_string_buffer);
			fprintf(log_file_kml, "%s\n", log_string_buffer);


		}

		/* wait for a period defined by the update rate */
		usleep((int)((1.0 / update_rate) * 1000000.0));
	}

	pthread_join(gps_thread, NULL);

	/* stopping message */
	printf("\nstopping...\n");

	/* close gps */
	gps_close(gps_fd);

	/* close log file */
	fclose(log_file);


	fprintf(log_file_kml, "%s",	"</coordinates>\n"
								"</LineString>\n"
								"<Style>\n"
								"<LineStyle>\n"
								"<color>#ff0000ff</color>\n"
								"<width>1</width>\n"
								"</LineStyle>\n"
								"</Style>\n"
								"</Placemark>\n"
								"</Document>\n"
								"</kml>\n");

	fclose(log_file_kml);

	return EXIT_SUCCESS;
}


void keyboard_interrupt_handler(int sig_num)
{
    running = 0;
}


void* gps_worker(void *args)
{
	/* get arguments from parameter pointer */
	struct gps_worker_struct* gps_args = (struct gps_worker_struct*) args;
	
	/* create buffer for gps messages */
	char buffer[82];
	
	/* buffer for coordinate read */
	struct coord message_coord;
	
	int sync_time = 1;
	int valid_counter = 0;
	
	//////////////////////////////////////////
	char raw_log_buffer[64];
	
	FILE *raw_log = fopen("raw_log.txt", "a+");
	//////////////////////////////////////////
	
	/* infinite while loop for polling gps messages */
	while(running)
	{
		/* read gps message */
		gps_readline(gps_args->gps_fd, buffer);
		
		/* check if message is gll */
		if (gps_is_gll(buffer))
		{
			/* parse gll message into a coordinate */
			message_coord = gps_parse_gll(buffer);
			
			/* check if coordinate is valid */
			if (message_coord.valid == 1)
			{
				//////////////////////////////////////////
				/* log coordinate into file */
				log_coord_string(message_coord, raw_log_buffer);
				fprintf(raw_log, "%s\n", raw_log_buffer);
				//////////////////////////////////////////
				
				/* sync time if second coordinate is received */
				if (sync_time && valid_counter == 1)
				{
					/* get system real time */
					struct timeval tv;
					gettimeofday(&tv, NULL);
					double time_real = tv.tv_sec + tv.tv_usec / 1000000.0;
					
					/* calculate time offset between real and gps time */
					gps_args->time_offset = time_real - message_coord.timestamp;
					
					/* do not sync time again */
					sync_time = 0;
				}
				/* update coordinate history */
				pthread_mutex_lock(&gps_args->lock);
				gps_args->oldest = gps_args->previous;
				gps_args->previous = gps_args->current;
				gps_args->current = message_coord;
				pthread_mutex_unlock(&gps_args->lock);
				valid_counter++;
			}
		}
	}
	//////////////////////////////////////////
	fclose(raw_log);
	//////////////////////////////////////////
	
	pthread_exit(NULL);
}


struct coord gps_get_instant(struct gps_worker_struct* gps_args)
{
	/* create coordinate to hold the result */
	struct coord instant;
	
	/* create timeval struct to retrieve absolute time */
	struct timeval tv;
	gettimeofday(&tv, NULL);
	
	/* calculate real time into seconds and add the skew from gps */
	double time_real = gps_args->time_offset + tv.tv_sec + tv.tv_usec / 1000000.0;
	
	/* calculate time passed since last known gps coordinate was obtained */
	double time_new = time_real - gps_args->current.timestamp;
	
	if (time_new < 0.0)
		time_new = 0.000001;
	printf("time passed: %f\n", time_new);
	
	/* calculate courses between the last three known points */
	double course_current = coord_course(gps_args->previous, gps_args->current);
	double course_previous = coord_course(gps_args->oldest, gps_args->previous);
	
	/* calculate turn rate based on the last two known courses */
	double turn_rate = (course_current - course_previous) / (gps_args->current.timestamp - gps_args->previous.timestamp);
	
	/* calculate distances between the last three known points */
	double distance_current = coord_distance(gps_args->previous, gps_args->current);
	/* double distance_previous = coord_distance(gps_args->oldest, gps_args->previous); */ /* use for averaging */
	
	/* calculate speeds between the last three known points */
	double speed_current = distance_current / (gps_args->current.timestamp - gps_args->previous.timestamp);
	/* double speed_previous = distance_old / (gps_args->previous.timestamp - gps_args->oldest.timestamp); */ /* use for averaging */
	
	/* calculate distance change and course change since last known gps coordinate */
	double distance_new = time_new * speed_current;
	/* double distance_new = time_new * ((speed_current + speed_previous) / 2); */ /* use for averaging */
	double course_new = turn_rate * time_new + course_current;
	
	/* calculate new coordinate based on original coordinate, new distance change since, and course change since */
	instant = coord_dist_radial(gps_args->current, distance_new, course_current); /* <<<<<< change to course_current to avoid course prediction */
	
	/* update time stamp of the result to current time */
	instant.timestamp = time_real;
	
	instant.valid = 0;
	if (gps_args->current.valid > 0 && gps_args->previous.valid > 0 && gps_args->oldest.valid > 0)
		instant.valid = 2;
	
	/* return result */
	return instant;
}


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
	
	/* pass validity flag */
	strncpy(buffer, gll_string+44, 1);
	buffer[1] = '\0';	
	if (buffer[0] == 'A')
		parsed.valid = 1;
	else
		parsed.valid = 0;
	
	/* return result */
	return parsed;
}


double coord_distance(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=deg_to_rad(destination.lon);
	
	/* formula from http://www.edwilliams.org/avform.htm#Dist */
	double distance = 2*asin(sqrt(pow(sin((origin.lat-destination.lat)/2.0),2.0)+cos(origin.lat)*cos(destination.lat)*pow(sin((origin.lon-destination.lon)/2.0),2.0)));
	
	/* convert radial distance to meters (1 deg = 60 nmi, 1 nmi = 1852 m) */
	distance = rad_to_deg(distance) * 60.0 * 1852.0;
	
	/* return result */
	return distance;
}


struct coord coord_dist_radial(struct coord origin, double distance, double radial)
{
	/* convert parameters to radians */
	origin.lat = deg_to_rad(origin.lat);
	origin.lon = deg_to_rad(origin.lon);
		
	/* invert radial to use positive clockwise angle and convert to radians */ 
	radial = deg_to_rad(-radial);
	
	/* convert distance to angular distance (1 deg = 60 nmi, 1 m = 0.000539957 nmi) */
	distance = deg_to_rad(distance * 0.000539957 / 60.0);
	
	/* create struct for result coordinate */
	struct coord result;

	/* formulas from http://www.edwilliams.org/avform.htm#LL */
	result.lat = asin(sin(origin.lat)*cos(distance)+cos(origin.lat)*sin(distance)*cos(radial));
	result.lon = fmod(origin.lon-atan2(sin(radial)*sin(distance)*cos(origin.lat),cos(distance)-sin(origin.lat)*sin(result.lat))+M_PI,2.0*M_PI)-M_PI;
  
	/* convert result coordinate to degrees */
	result.lat = rad_to_deg(result.lat);
	result.lon = rad_to_deg(result.lon);
  
	/* return result pointer */
	return result;
}


double coord_course(struct coord origin, struct coord destination)
{
	/* convert coordinates to radians */
	origin.lat=deg_to_rad(origin.lat);
	origin.lon=deg_to_rad(origin.lon);
	destination.lat=deg_to_rad(destination.lat);
	destination.lon=deg_to_rad(destination.lon);
	
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


void log_coord_string(struct coord coord, char* buffer)
{
	sprintf(buffer, "%.10f,%.10f,%.10f", coord.timestamp, coord.lat, coord.lon);
}


void log_coord_string_kml(struct coord coord, char* buffer)
{
	sprintf(buffer, "%.10f,%.10f,%.10f", coord.lon, coord.lat, 0.0);
}
