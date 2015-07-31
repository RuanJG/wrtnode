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

//#include <common/mavlink.h>


#define PORTNUM 6666
//#define IPADDR "192.168.2.1"
#define IPADDR "192.168.56.101"
#define CLIENTNUM 10
#define UART_NAME "/dev/ttyUSB0"
#define BAUDRATE  57600 
#define DATA_BITS  8
#define STOP_BITS  1
#define PARITY  0
#define HARDWARE_CONTROL  0


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

#define debugMsg(format, ...) fprintf(stderr, format, ## __VA_ARGS__)
#define msleep(x) usleep(x*1000)
char ip_addr[32];
int portnumber = PORTNUM;
g_data_t g_data;
g_uart_t g_uart;



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
				debugMsg("write sockfd %d of %d\n",written_bytes,length);
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
			//debugMsg("read sockfd byte %d\n",glen);
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
		debugMsg("open %s file error\n",g_uart.name);
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
/*
int do_read_mavlink_msg(mavlink_message_t *message)
{
	int result;
	uint8_t cp;
	mavlink_status_t status;
	// Lock
	pthread_mutex_lock(&g_uart.lock);
	result = read(g_uart.fd, &cp, 1);
	// Unlock
	pthread_mutex_unlock(&g_uart.lock);

	if (result > 0)
	{
		debugMsg("read data,,,\n");
		// the parsing
		result = mavlink_parse_char(MAVLINK_COMM_1, cp, message, &status);

		// check for dropped packets
		if ( (g_uart.packet_rx_drop_count != status.packet_rx_drop_count) )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			fprintf(stderr,"head is %02x ", cp);
		}
		debugMsg("read message：head is %02x msgid=%d,sysid=%d,compid=%d\n",cp,message->msgid,message->sysid,message->compid);
		g_uart.packet_rx_drop_count != status.packet_rx_drop_count;
	}else{
		fprintf(stderr, "ERROR: Could not read from fd %d\n", g_uart.fd);
	}

	return result>0?0:-1;
}
void do_write_mavlink_msg(mavlink_message_t *message)
{
	char buf[300];
	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
	// Lock
	pthread_mutex_lock(&g_uart.lock);
	// Write packet via serial link
	write(g_uart.fd, buf, len);
	// Wait until all data has been written
	tcdrain(g_uart.fd);
	// Unlock
	pthread_mutex_unlock(&g_uart.lock);
	return;
}
*/
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
	if( g_data.read_thread_id > 0 )
		pthread_join(g_data.read_thread_id,NULL);
	if( g_data.write_thread_id > 0 )
		pthread_join(g_data.write_thread_id,NULL);
	g_data.write_thread_id = 0;
	g_data.read_thread_id = 0;
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

int is_thread_running()
{
	int ret = 0;

	pthread_mutex_lock(&g_data.mutex);

	if( (g_data.thread_status & 0x1)==0x1 ||  (g_data.thread_status & 0x2)==0x2 )
		ret = 1;

	pthread_mutex_unlock(&g_data.mutex);
	return ret ;
}
void set_thread_status(int bit_mask,int val)
{
	pthread_mutex_lock(&g_data.mutex);
	if( val == 1)
		g_data.thread_status |= bit_mask;
	else 
		g_data.thread_status &= ~bit_mask;
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
void* recive_msg_thread_worker(void *args)
{
	int len,i,ret;
	char buf[1024];
	
	set_thread_status(0x2,1);
	while(!g_data.need_read_thread_quit){
		//bzero(buf,1024);
		ret = is_fd_ready(g_data.client_sockfd,70);
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
	set_thread_status(0x2,0);
	debugMsg("recive_msg_thread_worker exit\n");
}
void* send_msg_thread_worker(void *args)
{
	int len,len_msg;
	char buf[1024];
	//mavlink_message_t message;
	char *ptr;
	int i,ret;

	fd_set fdsr;

	bzero(buf,1024);
	sprintf(buf,"OK");
	set_thread_status(0x1,1);

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
	set_thread_status(0x1,0);
	debugMsg("send_msg_thread_worker exit\n");
}
int do_creat_thread()
{
	int res ;
	
	g_data.need_read_thread_quit = 0;
	g_data.need_write_thread_quit = 0;
	res= pthread_create( &g_data.read_thread_id, NULL, &recive_msg_thread_worker, &g_data );
	if( res != 0)
		goto thread_err;
	res= pthread_create( &g_data.write_thread_id, NULL, &send_msg_thread_worker, &g_data);
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

	do_initdata();
        if(argc >=3 && atoi(argv[2])>0)
        {
		portnumber = atoi(argv[2]);
		sprintf(ip_addr,"%s",argv[1]);
	}else{
                fprintf(stderr,"TcpToUart  Usage:%s ip port /dev/ttyxxx baudrate \n",argv[0]);
        }
        debugMsg("TcpToUart  Use addr,portnumber= %s:%d\a\n",ip_addr,portnumber);
	if( argc ==5 )
	{
		sprintf(g_uart.name,"%s",argv[3]);
		g_uart.baudrate = atoi(argv[4]);
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
			if( is_thread_running() ){//has connected a client
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
			continue;
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
