#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <wait.h>
#define PORTNUM 6666
//#define IPADDR "192.168.8.1"
#define IPADDR "10.0.2.15"
#define CLIENTNUM 1


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
}g_data_t;



#define debugMsg(format, ...) fprintf(stderr, format, ## __VA_ARGS__)
#define msleep(x) usleep(x*1000)
char ip_addr[]=IPADDR;
int portnumber = PORTNUM;
g_data_t g_data;



int do_write(int fd,void *buffer,int length)
{
	int bytes_left;
	int written_bytes;
	char *ptr;

	ptr=buffer;
	bytes_left=length;

	while(bytes_left>0)
	{
        /* 开始写*/
        	written_bytes=write(fd,ptr,bytes_left);
        	if(written_bytes<=0)/* 出错了*/
        	{       
                	if(errno==EINTR) /* 中断错误 我们继续写*/
                        	written_bytes=0;
                	else /* 其他错误 没有办法,只好撤退了*/
                        	return(-1);
        	}
        	bytes_left-=written_bytes;
        	ptr+=written_bytes;     /* 从剩下的地方继续写  */
	}
	return(length-bytes_left);
}
int do_read(int fd,void *buffer,int length)
{
	int bytes_left;
	int bytes_read;
	char *ptr = buffer;
   
	bytes_left=length;
	while(bytes_left>0)
	{
   		bytes_read=read(fd,ptr,bytes_read);
   		if(bytes_read<0)
   		{
		     if(errno==EINTR)
		        bytes_read=0;
		     else
		        return(-1);
   		}else if(bytes_read==0)
       			break;
		bytes_left-=bytes_read;
		ptr+=bytes_read;
	}
	return(length-bytes_left);
}

int do_recv(int client_sockfd ,char *buf,int len, int flag)
{
	int glen;
	int retry = 5;

	for( retry; retry >0 ; retry--){
		glen = recv(client_sockfd,buf,len,flag);
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

do_initdata(){
	int i;
	g_data.need_close = 0;

	g_data.sockfd = 0;
	g_data.client_sockfd = 0;
	bzero(&g_data.server_addr,sizeof(struct sockaddr_in));
	bzero(&g_data.client_addr,sizeof(struct sockaddr_in));

	g_data.need_read_thread_quit = 0;
	g_data.need_write_thread_quit = 0;
	g_data.read_thread_id = 0;
	g_data.write_thread_id = 0;
}

void do_service(struct sockaddr_in *client_addr,int client_sockfd)
{
	char buf[100];
	int len;
	int ret;
	fprintf(stderr,"Server get connection from %s\n",inet_ntoa(client_addr->sin_addr));

	while(!g_data.need_close)
	{
		//len = do_read(client_sockfd,buf,1);
		bzero(buf,100);
		len = recv(client_sockfd,buf, 100, 0);
		if( len > 0 ){
			debugMsg("Client #> %s\n",buf);
			ret = do_write(client_sockfd,buf,len);
			if( ret != len )
				debugMsg("write not complie !!!\n");
			if(0 == strcmp(buf,"exit") )
				g_data.need_close = 1;
		}
	}
        /* 这个通讯已经结束     */
        close(client_sockfd);
}

void do_release_client_socket()
{
	if( g_data.client_sockfd > 0 )
		close(g_data.client_sockfd);
	g_data.client_sockfd = 0;
}
void do_release_server_socket()
{
	if( g_data.sockfd > 0 )
		close(g_data.sockfd);
	g_data.sockfd = 0;
}
void do_release_socket(){
	do_release_client_socket();
	do_release_server_socket();
}
void do_wait_thread()
{
	pthread_join(g_data.read_thread_id,NULL);
	pthread_join(g_data.write_thread_id,NULL);
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
		
	do_release_socket();
	do_release_thread();

	// end program here
	printf("Ruan: exit by ctrl-c\n");
	exit(0);

}

int need_response=0;
void* recive_msg_thread_worker(void *args)
{
	int len;
	char buf[1024];

	while(!g_data.need_read_thread_quit){
		bzero(buf,1024);
		len = do_recv(g_data.client_sockfd ,buf, 1024, 0);
		if( len == 0 ){
			// get 0 data ,maybe error;
		}else if( len < 0 ){
			debugMsg("Client connect error, exit !!\n");
			break;
		}else{
			debugMsg("Client#> %s, len =%d\n",buf,len);
			if( 0 == strcmp(buf,"exit") ){
				break;
			}
			need_response = 1;
		}
	}
	g_data.need_read_thread_quit = 1;
	g_data.need_write_thread_quit = 1;
	debugMsg("recive_msg_thread_worker exit\n");
}
void* send_msg_thread_worker(void *args)
{
	int len;
	char buf[1024];

	bzero(buf,1024);
	sprintf(buf,"OK");
	while(!g_data.need_write_thread_quit){
		if( need_response ){
			len = do_write(g_data.client_sockfd,buf, 1024);
			need_response = 0;
		}
		msleep(200);
	}
	debugMsg("send_msg_thread_worker exit\n");
}
int do_creat_thread()
{
	int res ;
	
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
int main(int argc, char *argv[])
{

        int sin_size;

        if(argc!=3 || atoi(argv[2])<0)
        {
                fprintf(stderr,"TcpToUart  Usage:%s addr portnumber, now use default  %s:%d\a\n",argv[0],ip_addr,portnumber);
        }


	do_initdata();
	signal(SIGINT,quit_handler);

        /* 服务器端开始建立socket描述符 */
        if((g_data.sockfd=socket(AF_INET,SOCK_STREAM,0))==-1)  
        {
                fprintf(stderr,"TcpToUart Socket error:%s\n\a",strerror(errno));
		goto socket_error;
        }

        /* 服务器端填充 sockaddr结构  */ 
        //bzero(&server_addr,sizeof(struct sockaddr_in));
        g_data.server_addr.sin_family=AF_INET;
        g_data.server_addr.sin_addr.s_addr= inet_addr(ip_addr);//htonl(INADDR_ANY);
        g_data.server_addr.sin_port=htons(portnumber);

        /* 捆绑sockfd描述符  */ 
        if(bind(g_data.sockfd,(struct sockaddr *)(&g_data.server_addr),sizeof(struct sockaddr))==-1)
        {
                fprintf(stderr,"TcpToUart Bind Server Sockfd error:%s\n\a",strerror(errno));
		goto bind_error;
        }

        /* 监听sockfd描述符  */
        if(listen(g_data.sockfd,CLIENTNUM-1)==-1)
        {
                fprintf(stderr,"TcpToUart Server Listen error:%s\n\a",strerror(errno));
		goto listen_error;
        }


        while(!g_data.need_close)
        {
		debugMsg("Wait for client ...\n");

                /* 服务器阻塞,直到客户程序建立连接  */
                sin_size=sizeof(struct sockaddr_in);
		g_data.client_sockfd = accept(g_data.sockfd,(struct sockaddr *)(&g_data.client_addr),&sin_size);
                if(g_data.client_sockfd != -1)
                {
			//do_service(&g_data.client_addr,g_data.client_sockfd);
			do_creat_thread();
			do_wait_thread();
                }else{
                        fprintf(stderr,"TcpToUart Accept error:%s,retry ...\n\a",strerror(errno));
		}

		//init , ready to accept the new client
		debugMsg("reinit for next client\n");
		g_data.need_read_thread_quit = 0;
		g_data.need_write_thread_quit = 0;
		close(g_data.client_sockfd);
		g_data.client_sockfd = 0;
        }

        //close(g_data.sockfd);
	debugMsg("quit normally \n");
	quit_handler(0);
        exit(0);

listen_error:
	close(g_data.sockfd);
bind_error:
socket_error:
	exit(1);
}
