#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>

#include <termios.h>
#include <fcntl.h>   // File control definitions

#include <common/mavlink.h>


#define PORTNUM 6666
//#define IPADDR "192.168.2.1"
#define IPADDR "192.168.56.101"
#define CLIENTNUM 10
#define UART_NAME "/dev/ttyUSB0"
#define UART_NAME_EX "/dev/ttyS0"
#define BAUDRATE  57600 
#define DATA_BITS  8
#define STOP_BITS  1
#define PARITY  0
#define HARDWARE_CONTROL  0


#define READ_THREAD_ID 0x1
#define WRITE_THREAD_ID 0x2
#define RC_THREAD_ID 0x4


#define USE_MAVLINK 1


typedef struct __g_data_t{
	//server sock
	int sockfd;
	struct sockaddr_in server_addr;
	//client sock
	int client_sockfd;
	struct sockaddr_in client_addr;
	//var
	int need_close ;
	int need_read_thread_quit;
	int need_write_thread_quit;
	//thread
	pthread_t read_thread_id;
	pthread_t write_thread_id;
	pthread_t rc_thread_id;
	char thread_status;//b0=1/0 read on/off b1 = 1/0 write on/off
	pthread_mutex_t mutex;
}g_data_t;

typedef struct __g_uart_t{
	char name[32];
	int baudrate;
	int data_bits;
	int stop_bits;
	int parity;
	int hardware_control;
	int fd;
	pthread_mutex_t lock;
	int packet_rx_drop_count;
}g_uart_t;

char ip_addr[32];
int portnumber = PORTNUM;
g_data_t g_data;
g_uart_t g_uart;
int debug = 0;

typedef struct __g_rc_data_t{
	mavlink_message_t rc_message;
	int freq;
	int sleep_time; // 1000000 us / freq
	struct timeval time_stamp;
	int max_lost_time;// 2s , lost connect   s
	pthread_mutex_t lock;
	int stop;
}g_rc_data_t;
g_rc_data_t g_rc_data;

#define debugMsg(format, ...) if(debug==1) fprintf(stderr, format, ## __VA_ARGS__)
#define msleep(x) usleep(x*1000)

int byte_every_s = 0;
int fps_every_s = 0;
void do_time_alloc_msg(int msg_size)
{
	static int sum_size=0;
	static struct timeval tv_last;
	struct timeval tv_now ;
	static int fps = 0;

	int diff_sec=0;

	if( !debug ) return;

pthread_mutex_lock(&g_data.mutex);
	if( sum_size == 0 ){
		gettimeofday(&tv_last, NULL);
	}

	gettimeofday(&tv_now, NULL);
	diff_sec = tv_now.tv_sec - tv_last.tv_sec ;
	if( diff_sec > 1 ){
		byte_every_s = sum_size;
		fps_every_s = fps;
		debugMsg(">>>>>>>>>>>>>>>>>>>>>>> net status: %d byte/s, %d fps \n",byte_every_s,fps_every_s);
		fps = 0;
		sum_size = 0;
		tv_last.tv_sec = tv_now.tv_sec;
		tv_last.tv_usec = tv_now.tv_usec;
	}

	fps ++;
	sum_size += msg_size;
pthread_mutex_unlock(&g_data.mutex);
}

int do_write(int fd,void *buffer,int length)
{
	int bytes_left;
	int written_bytes;
	char *ptr;
	int retry;

	ptr=buffer;
	bytes_left=length;

	retry  = 5;
	while(retry-- > 0){
		written_bytes = write(fd,buffer,length);
		if( written_bytes <0 ){
			if(written_bytes == EINTR || written_bytes == EWOULDBLOCK || written_bytes == EAGAIN)
				debugMsg("write sockfd failse> %s ; maybe network interupt, retry\n",strerror(errno));
			else
				return -1;
		}else if( written_bytes == 0){
			debugMsg("write sockfd error client lost?\n");
			return -1;
		}else{
			if( written_bytes < length )
				debugMsg("write sockfd wrong !!!!!!!!!!!!!!   %d of %d\n",written_bytes,length);
			debugMsg("write sockfd %d of %d\n",written_bytes,length);
			do_time_alloc_msg(written_bytes);
			return written_bytes;
		}
	}

	return 0;
}
int do_read(int client_sockfd,void *buf,int len)
{
	int glen;
	int retry = 5;

	for( retry; retry >0 ; retry--){
		glen = read(client_sockfd,buf,len);
		if( glen < 0){
			if(glen == EINTR || glen == EWOULDBLOCK || glen == EAGAIN)
				debugMsg("read sockfd failse> %s ; maybe network interupt, retry\n",strerror(errno));
			else
				return -1;
			//return -1;
		}else if( glen == 0){
			debugMsg("read sockfd error client lost?\n");
			return -1;
		}else{
			debugMsg("read sockfd byte %d\n",glen);
			do_time_alloc_msg(glen);
			return glen;
		}
	}
	return 0;
}

int do_recv(int client_sockfd ,char *buf,int len)
{
	int glen;
	int retry = 5;

	for( retry; retry >0 ; retry--){
		glen = recv(client_sockfd,buf,len,0);
		if( glen < 0 ){
			if(glen == EINTR )
				continue;
			else
				return -1;
		}else
			break;
	}
	return glen;
}

int do_open_uart()
{
	struct termios  config;
	int fd;
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(g_uart.name, O_RDWR|O_NOCTTY|O_NDELAY);
	if (fd == -1){
#if 0
		if( strcmp(UART_NAME,g_uart.name) == 0){
			debugMsg("open %s file error, try %s\n",g_uart.name,UART_NAME_EX);
			fd = open(UART_NAME_EX, O_RDWR|O_NOCTTY|O_NDELAY);
		}else{
			debugMsg("open %s file error, try %s\n",g_uart.name,UART_NAME);
			fd = open(UART_NAME, O_RDWR|O_NOCTTY|O_NDELAY);
		}
		if( fd == -1 )
#endif
			goto err_fd;
	}

	//fcntl(fd, F_SETFL, 0);
	//fcntl(fd, F_SETFL, FNDELAY);
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		goto err_check;
	}
	// Read file descritor configuration
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		goto err_check;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (g_uart.baudrate)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			debugMsg("user 57600 \n");
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", g_uart.baudrate);
				goto err_check;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", g_uart.baudrate);
			goto err_check;

			break;
	}

	if( g_uart.parity ) config.c_cflag |= PARENB;
	else config.c_cflag &= ~PARENB;

	if( g_uart.stop_bits ) config.c_cflag |= CSTOPB;
	else config.c_cflag &= ~CSTOPB;

	//data_bits = 8
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;
	//hardware_control
	if( g_uart.hardware_control ) config.c_cflag |= CRTSCTS;
	else config.c_cflag &= ~CRTSCTS;

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		goto err_check;
	}

	debugMsg("open Uart %s OK \n",g_uart.name);
	g_uart.fd = fd;
	return 0;

err_check:
	close(fd);
err_fd:
	return -1;
}
int do_close_uart()
{
	if( g_uart.fd > 0 )
		close(g_uart.fd);
	return 0;
}

int do_write_uart_mavlink_msg(g_uart_t *uart, mavlink_message_t *message)
{
	char buf[300];
	int res;
	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
	// Lock
	pthread_mutex_lock(&uart->lock);
	// Write packet via serial link
	res = write(uart->fd, buf, len);
	// Wait until all data has been written
	tcdrain(uart->fd);
	// Unlock
	pthread_mutex_unlock(&uart->lock);
	return res ;
}

int do_write_socket_mavlink_msg(int sockfd, mavlink_message_t *message)
{
	char buf[300];
	int res,len;
	// Translate message to buffer
	unsigned len_msg = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
	// Lock
	len = do_write(sockfd,buf, len_msg);
	return len ;
}

int do_copy_mavlink_message_from_buffer(char * buffer, int buff_len, mavlink_message_t *message)
{

	int lest_len,msg_len;
	int index;

	for ( index = 0; index < buff_len ; index ++)
	{
		if( buffer[index] == MAVLINK_STX )
		{
			lest_len = buff_len-index; //include the stx
			if( lest_len < 2){
				msg_len = 0;
				break;
			}
			msg_len = buffer[index+1] + MAVLINK_NUM_HEADER_BYTES;// MAVLINK_STX magic ~ payload
			if( lest_len < msg_len ){
				msg_len = 0;
				break;
			}
			memcpy(&message->magic,buffer,msg_len);
			index += msg_len;
			break;
		}
	}
	return index;
}


int do_read_mavlink_message_from_buffer(char * buffer, int buff_len, mavlink_message_t *message,int *res)
{//return index, res : 1:mavlink_message 0: no ;

	int result;
	int index;
	mavlink_status_t status;

	*res = 0;
	for ( index = 0; index < buff_len ; index ++)
	{
		if( mavlink_parse_char(MAVLINK_COMM_1, buffer[index], message, &status) )
		{
			if ( (g_uart.packet_rx_drop_count != status.packet_rx_drop_count) )
			{
				g_uart.packet_rx_drop_count != status.packet_rx_drop_count;
				debugMsg("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			}
			debugMsg("read message：head is %02x msgid=%d,sysid=%d,compid=%d\n",buffer[index],message->msgid,message->sysid,message->compid);
			*res = 1;
			break;
		}
	}
	return index;
}

int do_write_uart_raw(char *data, int len)
{
	int res =0;

	pthread_mutex_lock(&g_uart.lock);
	// Write packet via serial link
	res = write(g_uart.fd, data, len);
	// Wait until all data has been written
	tcdrain(g_uart.fd);
	// Unlock
	pthread_mutex_unlock(&g_uart.lock);

	return res;
}

int do_read_uart_raw(char *data, int len)
{
	int result;
	// Lock
	pthread_mutex_lock(&g_uart.lock);
	result = read(g_uart.fd, data, len);
	// Unlock
	pthread_mutex_unlock(&g_uart.lock);
	return result;
}

void do_initdata(){
	int i;
	g_data.need_close = 0;

	g_data.sockfd = -1;
	g_data.client_sockfd = 0;
	bzero(&g_data.server_addr,sizeof(struct sockaddr_in));
	bzero(&g_data.client_addr,sizeof(struct sockaddr_in));

	g_data.need_read_thread_quit = 0;
	g_data.need_write_thread_quit = 0;
	g_data.read_thread_id = 0;
	g_data.write_thread_id = 0;
	g_data.rc_thread_id = 0;

	portnumber = PORTNUM;
	sprintf(ip_addr,"%s",IPADDR);

	g_data.thread_status = 0;
	pthread_mutex_init(&g_data.mutex,NULL);


	sprintf(g_uart.name,"%s",UART_NAME);
	g_uart.baudrate = BAUDRATE;
	g_uart.data_bits = DATA_BITS;
	g_uart.stop_bits = STOP_BITS;
	g_uart.parity = PARITY;
	g_uart.hardware_control = HARDWARE_CONTROL;
	g_uart.fd = -1;
	pthread_mutex_init(&g_uart.lock,NULL);

	g_rc_data.freq = 20;
	bzero(&g_rc_data.rc_message, sizeof(mavlink_message_t));
	pthread_mutex_init(&g_rc_data.lock,NULL);

}


void do_release_client_socket()
{
	if( g_data.client_sockfd > 0 )
		close(g_data.client_sockfd);
	g_data.client_sockfd = -1;
}
void do_release_server_socket()
{
	if( g_data.sockfd > 0 )
		close(g_data.sockfd);
	g_data.sockfd = -1;
}
void do_release_socket(){
	do_release_client_socket();
	do_release_server_socket();
}
void do_wait_thread()
{
	if( g_data.rc_thread_id > 0 )
		pthread_join(g_data.rc_thread_id,NULL);
	if( g_data.read_thread_id > 0 )
		pthread_join(g_data.read_thread_id,NULL);
	if( g_data.write_thread_id > 0 )
		pthread_join(g_data.write_thread_id,NULL);
	g_data.write_thread_id = 0;
	g_data.read_thread_id = 0;
	g_data.rc_thread_id = 0;
}
void do_release_thread()
{
	int i;
	g_data.need_read_thread_quit =1;
	g_data.need_write_thread_quit =1;

	do_wait_thread();
}
void quit_handler( int sig )
{
	int i;

	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
		
	do_release_thread();
	do_release_socket();

	do_close_uart();

	// end program here
	printf("Ruan: exit by ctrl-c\n");
	exit(0);

}
int is_thread_running(int thread_id)
{
	int ret = 0;

	pthread_mutex_lock(&g_data.mutex);

	if( (g_data.thread_status & thread_id)==thread_id )
		ret = 1;

	pthread_mutex_unlock(&g_data.mutex);
	return ret ;
}
void set_thread_status(int thread_id,int val)
{
	pthread_mutex_lock(&g_data.mutex);
	if( val == 1)
		g_data.thread_status |= thread_id;
	else 
		g_data.thread_status &= ~thread_id;
	pthread_mutex_unlock(&g_data.mutex);
}
int is_fd_ready(int fd,int timeout_ms)
{// 0 is on ready, 1 is ready , -1 error
	struct timeval timeout={0,70000}; 
	fd_set fdset;
	int ret;

	if( timeout_ms >= 1000 )
		timeout.tv_sec = timeout_ms/1000;
	timeout.tv_usec = (timeout_ms - timeout.tv_sec*1000)*1000;

	FD_ZERO(&fdset);
	FD_SET(fd,&fdset); 

       	ret = select(fd+1 ,&fdset,NULL,NULL,&timeout); //select使用 

	if( ret < 0 ) return -1;
	if( ret > 0 && FD_ISSET(fd,&fdset) )
		return 1;
	else
		return 0;

}


void do_rc_lost_connect()
{
	
	mavlink_rc_channels_override_t rc_packet;
	mavlink_message_t message;

	bzero(&rc_packet,sizeof(mavlink_rc_channels_override_t));
	rc_packet.target_system = mavlink_msg_rc_channels_override_get_target_system(&g_rc_data.rc_message);
	rc_packet.target_component = mavlink_msg_rc_channels_override_get_target_component(&g_rc_data.rc_message);
	mavlink_msg_rc_channels_override_encode(g_rc_data.rc_message.sysid,g_rc_data.rc_message.compid, &message, &rc_packet);
	do_write_uart_mavlink_msg(&g_uart,&message);
	usleep(20000);
	do_write_uart_mavlink_msg(&g_uart,&message);
	usleep(20000);
	do_write_uart_mavlink_msg(&g_uart,&message);

	//printf("do_rc_lost_connect ... the last time rc packet here is > 2s\n");
}

void do_update_rc_value(mavlink_rc_channels_override_t *rc_packet)
{
	
	pthread_mutex_lock(&g_rc_data.lock);

	//bzero(&rc_packet,sizeof(mavlink_rc_channels_override_t));
	rc_packet->target_system = mavlink_msg_rc_channels_override_get_target_system(&g_rc_data.rc_message);
	rc_packet->target_component = mavlink_msg_rc_channels_override_get_target_component(&g_rc_data.rc_message);

	rc_packet->chan1_raw = mavlink_msg_rc_channels_override_get_chan1_raw(&g_rc_data.rc_message);
	rc_packet->chan2_raw = mavlink_msg_rc_channels_override_get_chan2_raw(&g_rc_data.rc_message);
	rc_packet->chan3_raw = mavlink_msg_rc_channels_override_get_chan3_raw(&g_rc_data.rc_message);
	rc_packet->chan4_raw = mavlink_msg_rc_channels_override_get_chan4_raw(&g_rc_data.rc_message);
	rc_packet->chan5_raw = mavlink_msg_rc_channels_override_get_chan5_raw(&g_rc_data.rc_message);
	rc_packet->chan6_raw = mavlink_msg_rc_channels_override_get_chan6_raw(&g_rc_data.rc_message);
	rc_packet->chan7_raw = mavlink_msg_rc_channels_override_get_chan7_raw(&g_rc_data.rc_message);
	rc_packet->chan8_raw = mavlink_msg_rc_channels_override_get_chan8_raw(&g_rc_data.rc_message);


	pthread_mutex_unlock(&g_rc_data.lock);
}
void* rc_override_thread_worker(void *args)
{
	mavlink_rc_channels_override_t rc_packet;
	mavlink_message_t message;
    	struct timeval tv;
	int id = g_data.rc_thread_id;


	set_thread_status(RC_THREAD_ID,1);
	g_rc_data.sleep_time = 1000000 / g_rc_data.freq; //us
	
	while( !g_rc_data.stop && !g_data.need_read_thread_quit && !g_data.need_write_thread_quit)
	{
    		gettimeofday(&tv, NULL);
		if(  (tv.tv_sec - g_rc_data.time_stamp.tv_sec) >= g_rc_data.max_lost_time) 
		{
			do_rc_lost_connect();
			break;
		}
		
		do_update_rc_value(&rc_packet);
		mavlink_msg_rc_channels_override_encode(g_rc_data.rc_message.sysid,g_rc_data.rc_message.compid, &message, &rc_packet);
		do_write_uart_mavlink_msg(&g_uart,&message);
		//printf("!!!!!!!!!!!! id(%d) do_update_rc_value after %d s: %d us\n",id,tv.tv_sec-g_rc_data.time_stamp.tv_sec,tv.tv_usec-g_rc_data.time_stamp.tv_usec );
		usleep(g_rc_data.sleep_time);
	}
	do_rc_lost_connect();
	set_thread_status(RC_THREAD_ID,0);
}
void do_start_rc_thread()
{
	int res;
	g_rc_data.freq = 20;	
	g_rc_data.stop = 0;
	g_rc_data.max_lost_time = 5;
	res= pthread_create( &g_data.rc_thread_id, NULL, &rc_override_thread_worker, &g_data );
}
void do_stop_rc_thread()
{
	g_rc_data.stop = 0;
	if( g_data.rc_thread_id > 0 )
		pthread_join(g_data.rc_thread_id,NULL);
	g_data.rc_thread_id = 0;
}

int handle_rc_override_message(mavlink_message_t * message)
{
    	struct timeval tv;
	int res;
    	gettimeofday(&tv, NULL);
	//mavlink_rc_channels_override_t rc_packet;

	//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!get a rc override message after %ds:%dus\n",tv.tv_sec-g_rc_data.time_stamp.tv_sec,tv.tv_usec-g_rc_data.time_stamp.tv_usec);
	//mavlink_msg_rc_channels_override_decode(message,&rc_packet);
	pthread_mutex_lock(&g_rc_data.lock);
	memcpy(&g_rc_data.rc_message , message, sizeof(mavlink_message_t));
	gettimeofday(&g_rc_data.time_stamp,NULL);

	if( !is_thread_running(RC_THREAD_ID) )
	{
		do_start_rc_thread();
	}

	pthread_mutex_unlock(&g_rc_data.lock);
}

int handle_qgc_mavlink_message_sevice(mavlink_message_t *message)
{
	mavlink_rc_channels_override_t rc_msg;
	int need_send_to_copter= 0;
	int res = 0;

	switch ( message->msgid ){
		case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
			res = handle_rc_override_message(message);
			break;

		default :
			need_send_to_copter = 1;
			break;
	}
	if( need_send_to_copter ){
		res = do_write_uart_mavlink_msg(&g_uart,message);
		if ( 0 > res)
		{
			res = -1;
			debugMsg("write msg failse %d, may be uart error \n",res);
		}	
	}
	return res;
}

void* write_flight_mavlink_thread_worker(void *args)
{
	int len,i,ret,res;
	char buf[1024];
	mavlink_message_t message;
	
	set_thread_status(WRITE_THREAD_ID,1);
	while(!g_data.need_read_thread_quit){
		//bzero(buf,1024);
		ret = is_fd_ready(g_data.client_sockfd,50);
		if( ret < 0 ){
			//select fd error;
			debugMsg("Select socket fd error, exit ??\n");
			//break;
		}else if (ret == 0){
			//no fd ready
			//debugMsg("Select socket fd no data\n");
			continue;
		}

		len = do_read(g_data.client_sockfd ,buf, 1024);
		if( len < 0 ){
			debugMsg("Client connect error, exit !!\n");
			break;
		}else if( len == 0){
			;//no data to deal;maybe network problem
		}else{
			ret = 0;
			//debugMsg("############### ret=%d,res=%d\n",ret,res);
			for( ret = 0; ret < len ;ret ++){
				ret += do_read_mavlink_message_from_buffer(&buf[ret],len-ret,&message,&res);
				if( res ) {
					//deal with a mavlink message
					if ( 0 > handle_qgc_mavlink_message_sevice(&message))
					{
						debugMsg("handle_mavlink_message_sevice return error\n");
						break;
					}
				}
				//debugMsg("ret=%d,res=%d\n",ret,res);
			}
			//debugMsg("############### ret=%d,res=%d\n",ret,res);
		}
	}
	g_data.need_read_thread_quit = 1;
	g_data.need_write_thread_quit = 1;
	set_thread_status(WRITE_THREAD_ID,0);
	debugMsg("recive_msg_thread_worker exit\n");
}
void* read_flight_mavlink_thread_worker(void *args)
{
	int len,len_msg;
	char buf[1024];
	mavlink_message_t message;
	char *ptr;
	int i,ret,res;

	fd_set fdsr;

	bzero(buf,1024);
	sprintf(buf,"OK");
	set_thread_status(READ_THREAD_ID,1);

	tcflush(g_uart.fd, TCIOFLUSH);

	while(!g_data.need_write_thread_quit){
		//do_read_mavlink_msg(&message);
		ret = is_fd_ready(g_uart.fd ,70);
		if( ret < 0 ){
			//select fd error;
			debugMsg("Select uart fd error, exit ??\n");
			//break;
		}else if (ret == 0){
			//no fd ready
			//debugMsg("Select uart fd no data\n");
			continue;
		}

		len_msg = do_read_uart_raw(buf,1024);
		if( len_msg > 0)
		{
			ret = 0;
			//debugMsg("@@@@@@@@@@@@@@@@@@@@@@@ ret=%d,res=%d\n",ret,res);
			for( ret = 0; ret < len_msg ;ret ++){
				ret += do_read_mavlink_message_from_buffer(&buf[ret],len_msg-ret,&message,&res);
				if( res ) {
					//do_write_uart_mavlink_msg(&g_uart,&message);
					len = do_write_socket_mavlink_msg(g_data.client_sockfd, &message);
					if( len < 0 ){
						debugMsg("send msg failse %d, may be client leave\n",len);
						break;
					}
				}
				//debugMsg("ret=%d,res=%d\n",ret,res);
			}
			//debugMsg("@@@@@@@@@@@@@@@@@@@@@@@ ret=%d,res=%d\n",ret,res);

		}else if(len_msg == 0){
			debugMsg("read no data from uart result %d\n",len_msg);
		}else{
			debugMsg("read uart len =%d, error %d !!!!!\n",len_msg,errno);
			fprintf(stderr,"TcpToUart  error:%s\n\a",strerror(errno));
		}
		//msleep(70);
	}
	g_data.need_read_thread_quit = 1;
	g_data.need_write_thread_quit = 1;
	set_thread_status(READ_THREAD_ID,0);
	debugMsg("send_msg_thread_worker exit\n");
}





//data raw
void* write_flight_thread_worker(void *args)
{
	int len,i,ret;
	char buf[1024];
	
	set_thread_status(WRITE_THREAD_ID,1);
	while(!g_data.need_read_thread_quit){
		//bzero(buf,1024);
		ret = is_fd_ready(g_data.client_sockfd,50);
		if( ret < 0 ){
			//select fd error;
			debugMsg("Select socket fd error, exit ??\n");
			//break;
		}else if (ret == 0){
			//no fd ready
			//debugMsg("Select socket fd no data\n");
			continue;
		}

		len = do_read(g_data.client_sockfd ,buf, 1024);
		if( len < 0 ){
			debugMsg("Client connect error, exit !!\n");
			break;
		}else if( len == 0){
			;//no data to deal;maybe network problem
		}else{
			/*
			debugMsg("recive msg: \n");
			for(i=0 ;i< len ; i++)
				debugMsg("%x, ",buf[i]);
			*/
			if( 0 > do_write_uart_raw(buf,len) ){
				debugMsg("write uart error \n");
				fprintf(stderr,"TcpToUart  error:%s\n\a",strerror(errno));
			}
		}
	}
	g_data.need_read_thread_quit = 1;
	g_data.need_write_thread_quit = 1;
	set_thread_status(WRITE_THREAD_ID,0);
	debugMsg("recive_msg_thread_worker exit\n");
}
void* read_flight_thread_worker(void *args)
{
	int len,len_msg;
	char buf[1024];
	//mavlink_message_t message;
	char *ptr;
	int i,ret;

	fd_set fdsr;

	bzero(buf,1024);
	sprintf(buf,"OK");
	set_thread_status(READ_THREAD_ID,1);

	tcflush(g_uart.fd, TCIOFLUSH);

	while(!g_data.need_write_thread_quit){
		//do_read_mavlink_msg(&message);
		ret = is_fd_ready(g_uart.fd ,70);
		if( ret < 0 ){
			//select fd error;
			debugMsg("Select uart fd error, exit ??\n");
			//break;
		}else if (ret == 0){
			//no fd ready
			//debugMsg("Select uart fd no data\n");
			continue;
		}

		len_msg = do_read_uart_raw(buf,1024);
		if( len_msg > 0)
		{
		#if 0
			//ptr = (char *) &message;
			debugMsg("get a message, len=%d,max len=%d: \n",len_msg,(int)sizeof(mavlink_message_t));
			for(i=0 ; i<len_msg; i++) 
			{
				debugMsg("%2x,",buf[i]);
			}
			debugMsg("\n");
		#endif
			len = do_write(g_data.client_sockfd,buf, len_msg);
			//len = send(g_data.client_sockfd,buf, len_msg, 0);
			if( len < 0 ){
				debugMsg("send msg failse %d, may be client leave\n",len);
				break;
			}

		}else if(len_msg == 0){
			debugMsg("read no data from uart result %d\n",len_msg);
		}else{
			debugMsg("read uart len =%d, error %d !!!!!\n",len_msg,errno);
			fprintf(stderr,"TcpToUart  error:%s\n\a",strerror(errno));
		}
		//msleep(70);
	}
	g_data.need_read_thread_quit = 1;
	g_data.need_write_thread_quit = 1;
	set_thread_status(READ_THREAD_ID,0);
	debugMsg("send_msg_thread_worker exit\n");
}
int do_creat_thread()
{
	int res ;
	
	g_data.need_read_thread_quit = 0;
	g_data.need_write_thread_quit = 0;
#if USE_MAVLINK
	res= pthread_create( &g_data.read_thread_id, NULL, &read_flight_mavlink_thread_worker, &g_data );
#else
	res= pthread_create( &g_data.read_thread_id, NULL, &read_flight_thread_worker, &g_data );
#endif
	if( res != 0)
		goto thread_err;
#if USE_MAVLINK
	res= pthread_create( &g_data.write_thread_id, NULL, &write_flight_mavlink_thread_worker, &g_data);
#else
	res= pthread_create( &g_data.write_thread_id, NULL, &write_flight_thread_worker, &g_data);
#endif
	if( res != 0){
		g_data.need_read_thread_quit = 1;
		pthread_join(g_data.read_thread_id,NULL);
		goto thread_err;
	}
	return 0;
thread_err:
	g_data.read_thread_id = 0;
	g_data.write_thread_id = 0;
	return -1;
}

int do_creat_service_socket()
{
        /* 服务器端开始建立socket描述符 */
        if((g_data.sockfd=socket(AF_INET,SOCK_STREAM,0))==-1)  
        {
                fprintf(stderr,"TcpToUart Socket error:%s\n\a",strerror(errno));
		return -1;
        }

        /* 服务器端填充 sockaddr结构  */ 
        bzero(&g_data.server_addr,sizeof(struct sockaddr_in));
        g_data.server_addr.sin_family=AF_INET;
        g_data.server_addr.sin_addr.s_addr= inet_addr(ip_addr);//htonl(INADDR_ANY);
        g_data.server_addr.sin_port=htons(portnumber);

        /* 捆绑sockfd描述符  */ 
        if(bind(g_data.sockfd,(struct sockaddr *)(&g_data.server_addr),sizeof(struct sockaddr))==-1)
        {
                fprintf(stderr,"TcpToUart Bind Server Sockfd error:%s\n\a",strerror(errno));
		return -1;
        }

        /* 监听sockfd描述符  */
        if(listen(g_data.sockfd,CLIENTNUM)==-1)
        {
                fprintf(stderr,"TcpToUart Server Listen error:%s\n\a",strerror(errno));
		close(g_data.sockfd);
		g_data.sockfd = -1;
		return -1;
        }
	return 0;

}
int main(int argc, char *argv[])
{

        int sin_size;
	int tmp_sock;

	fprintf(stderr,"Usage:%s ip port /dev/ttyxxx baudrate  debug[0,1]\n",argv[0]);
	do_initdata();
        if(argc >=3 && atoi(argv[2])>0)
        {
		portnumber = atoi(argv[2]);
		sprintf(ip_addr,"%s",argv[1]);
	}else{
                fprintf(stderr,"TcpToUart  Usage:%s ip port /dev/ttyxxx baudrate \n",argv[0]);
        }
        debugMsg("TcpToUart  Use addr,portnumber= %s:%d\a\n",ip_addr,portnumber);
	if( argc >=5 )
	{
		sprintf(g_uart.name,"%s",argv[3]);
		g_uart.baudrate = atoi(argv[4]);
	}
	if( argc >= 6 ){
		debug = atoi(argv[5]);
	}
        debugMsg("TcpToUart  serial = %s %d\n",g_uart.name,g_uart.baudrate);


	if( 0 > do_open_uart() ){
		debugMsg("open uart %s error \n",g_uart.name);
		return -1;
	};
	signal(SIGINT,quit_handler);



        while(!g_data.need_close)
        {
		if( g_data.sockfd <= 0 ){
			if( do_creat_service_socket() < 0 ){
				debugMsg("creat socket failse, Waiting and retry  ...\n");
				sleep(1);
				continue;
			}
		}

		debugMsg("socket %d Wait for client ...\n",g_data.sockfd);
                /* 服务器阻塞,直到客户程序建立连接  */
                sin_size=sizeof(struct sockaddr_in);
		tmp_sock = accept(g_data.sockfd,(struct sockaddr *)(&g_data.client_addr),&sin_size);
                if(tmp_sock != -1)
                {
			if( is_thread_running(READ_THREAD_ID) || is_thread_running(WRITE_THREAD_ID) ){//has connected a client
				#if 1
				debugMsg("thread is running , close new clinet\n");
				close(tmp_sock);
				continue;
				#else
				debugMsg("thread is running , connect new clinet\n");
				do_release_thread();
				close(g_data.client_sockfd);
				g_data.client_sockfd = tmp_sock;
				do_creat_thread();
				sleep(1);
				#endif
			}else{
				debugMsg("no clinet servering, creat thread for new clinet\n");
				do_release_client_socket();
				g_data.client_sockfd = tmp_sock;
				do_creat_thread();
				sleep(1);
			}
                }else{
                        fprintf(stderr,"TcpToUart Accept error:%s, recreat sockfd\n",strerror(errno));
			do_release_thread();
			do_release_socket();
#if 0 // loop try
			continue;
#else	//loop by script
			break;
#endif
		}
        }

        //close(g_data.sockfd);
	debugMsg("quit normally \n");
	quit_handler(0);
        exit(0);

listen_error:
	close(g_data.sockfd);
bind_error:
socket_error:
	do_close_uart();
	exit(1);
}
