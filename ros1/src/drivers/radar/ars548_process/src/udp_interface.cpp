#include "udp_interface.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>  // close 函数用于关闭文件描述符

UdpInterface::UdpInterface()
{

}

int UdpInterface::initUdpMulticastServer(const string& _group_ip, int _group_port)
{
    char group_ip[20];
    int group_port = _group_port;
    strcpy(group_ip, _group_ip.c_str());

    addr_len = sizeof(struct sockaddr_in);

    // 建立UDP套接字
    socket_server_fd=socket(AF_INET,SOCK_DGRAM,0);
    if(socket_server_fd < 0)
    {
        perror("socket multicast!");
        return(socket_server_fd);
    }

    /*set up the local address*/
    // 设置本地接收地址
    struct sockaddr_in local_addr;
    memset(&local_addr,0,sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    // 绑定的端口号必须和多播组发送方使用的端口相同，否则收不到数据
    local_addr.sin_port = htons(group_port);        

    /*bind local address*/
    if(bind(socket_server_fd,(struct sockaddr *)&local_addr,sizeof(local_addr)) == -1)
    {
        perror("Binding the multicast!");
        return(-1);
    }   

    /*set up the destination address*/
    // 设置目标多播地址（接收数据的多播组地址）
    memset(&group_addr,0,sizeof(struct sockaddr_in));
    group_addr.sin_family = AF_INET;
    group_addr.sin_port = htons(group_port);
    group_addr.sin_addr.s_addr = inet_addr(group_ip);   

    // 设置组播选项，加入组播组
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr=inet_addr(group_ip);
    mreq.imr_interface.s_addr=htonl(INADDR_ANY);

    if (setsockopt(socket_server_fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0)
    {
        perror("setsockopt multicast!");
        return(-1);
    }

    return(0);
}

int UdpInterface::initUdpMulticastServer(const string& _group_ip, int _group_port, const string& _local_ip)
{
    char group_ip[20];  // "224.0.2.2"
    char local_ip[20];  // "10.13.1.166"
    int group_port = _group_port;  // 42102
    strcpy(group_ip, _group_ip.c_str());
    strcpy(local_ip, _local_ip.c_str());

    // 使用 sizeof(struct sockaddr_in) 更加通用和规范，
    // 且不依赖于任何特定的变量，能清晰表达代码意图，减少潜在错误。
    addr_len = sizeof(struct sockaddr_in);

    // 1. 建立UDP套接字
    socket_server_fd=socket(AF_INET,SOCK_DGRAM,0);
    if(socket_server_fd < 0)
    {
        perror("socket multicast!");
        return(socket_server_fd);
    }

    // 2. 设置 SO_REUSEADDR 选项
    // 允许多个套接字绑定到相同的端口，通常用于多播接收。
    int reuse = 1;
    if (setsockopt(socket_server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("setsockopt SO_REUSEADDR");
        close(socket_server_fd);
        return -1;
    }

    // 3. 配置本地地址（绑定套接字）
    /*set up the local address*/
    struct sockaddr_in local_addr;
    memset(&local_addr,0,sizeof(local_addr));
    local_addr.sin_family = AF_INET;  // 地址族
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // INADDR_ANY代表本机所有地址
    // 该端口必须与组端口相同
    local_addr.sin_port = htons(group_port);

    // 绑定本地地址到套接字
    /*bind local address*/ 
    if(bind(socket_server_fd,(struct sockaddr *)&local_addr,sizeof(local_addr)) == -1)
    {
        perror("Binding the multicast!");
        close(socket_server_fd);
        return(-1);
    } 

    // 4. 配置组播地址（目标地址）
    struct ip_mreq mreq; // 定义组播结构体
    // inet_pton(AF_INET,"224.0.2.2",&mreq.imr_multiaddr); // 组播地址
    if (inet_pton(AF_INET, group_ip, &mreq.imr_multiaddr) <= 0) {
        perror("inet_pton group_ip");
        return -1;
    }
    // inet_pton(AF_INET,"10.13.1.166",&mreq.imr_interface); // 需要添加到组的ip
    // 设置本机接口地址
    if (inet_pton(AF_INET, local_ip, &mreq.imr_interface) <= 0) {
        perror("inet_pton local_ip");
        return -1;
    }

    // 5. 加入组播组
    // 加入组播属性（也就是设置这个套接字 可以接收组播信息）
    if (setsockopt(socket_server_fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0)
    {
        perror("setsockopt multicast!");
        close(socket_server_fd);
        return(-1);
    }

    return(0);
}

// 发送 单个雷达的配置；使用全局变量 addr_serv，无法被二次绑定
int UdpInterface::initUdpUnicastClient(const string& dest_ip, int dest_port, int local_port)
{
    struct sockaddr_in addr;
  
    /* set up udp socket */  
    socket_client_fd = socket(AF_INET, SOCK_DGRAM, 0);  
    if(socket_client_fd < 0)  
    {  
        perror("socket");  
        return(socket_client_fd);
    } 

	bzero(&addr, sizeof(struct sockaddr_in));
	addr.sin_family=AF_INET;
	addr.sin_addr.s_addr=htonl(INADDR_ANY);
	addr.sin_port=htons(local_port);

    // 在同一台机器上，同一个 IP:port 组合只能绑定一次
	if(bind(socket_client_fd,(struct sockaddr *)(&addr),sizeof(struct sockaddr_in))<0)
	{
		perror("Binding the socket!");
		return(-1);
	}	    

    memset(&addr_serv, 0, sizeof(addr_serv));  
    addr_serv.sin_family = AF_INET;  
    addr_serv.sin_addr.s_addr = inet_addr(dest_ip.c_str());  
    addr_serv.sin_port = htons(dest_port);  

    return(0);
}

// 可用于 多个雷达的配置；使用传入的 addr，可以多次绑定
int UdpInterface::initUdpUnicastClient(const string& local_ip, int local_port)
{
    struct sockaddr_in addr;
  
    /* set up udp socket */  
    socket_client_fd = socket(AF_INET, SOCK_DGRAM, 0);  
    if(socket_client_fd < 0)  
    {  
        perror("socket");  
        return(socket_client_fd);
    } 

	bzero(&addr, sizeof(struct sockaddr_in));
	addr.sin_family=AF_INET;
    // 建议别用 INADDR_ANY，最好绑具体网卡IP
    addr.sin_addr.s_addr=inet_addr(local_ip.c_str());
	addr.sin_port=htons(local_port);

    // 在同一台机器上，同一个 IP:port 组合只能绑定一次
	if(bind(socket_client_fd,(struct sockaddr *)(&addr),sizeof(struct sockaddr_in))<0)
	{
		perror("Binding the socket!");
		return(-1);
	}	    

    return(0);
}

// 使用的是通用的 sockaddr
void UdpInterface::receiveFromRadar(char* data, int &len)
{
    len = recvfrom(socket_server_fd, data, 40000, 0, (struct sockaddr *)&addr_clie, &addr_len);
}

// 专门用于 IPv4 的 sockaddr_in
void UdpInterface::receiveFromRadar(struct sockaddr_in* addr, char* data, int &len)
{
    len = recvfrom(socket_server_fd, data, 40000, 0, (struct sockaddr *)addr, &addr_len);

    // Debug
    // char *ip = NULL;
    // int port = 0;
    // // 获取IPv4地址
    // ip = inet_ntoa(addr->sin_addr);
    // // 获取端口号
    // port = ntohs(addr->sin_port);
}

int UdpInterface::sendToRadar(char* data, int len)
{
    int n;
    n = sendto(socket_client_fd, data, len, 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));    
    return(n);  
}

int UdpInterface::sendToRadar(const string& dest_ip, int dest_port, char* data, int len)
{
    // struct sockaddr_in addr_serv_tmp{} 是局部变量，每次调用都会新建一份在栈上的地址结构，线程之间互不影响。
    // sendto() 本身在 Linux 下是线程安全的，多个线程可以同时对同一个 socket 发送数据，内核会保证数据报不会被拼接或损坏。

    struct sockaddr_in addr_serv_tmp{};
    addr_serv_tmp.sin_family = AF_INET;
    addr_serv_tmp.sin_addr.s_addr = inet_addr(dest_ip.c_str());
    addr_serv_tmp.sin_port = htons(dest_port);

    int n;
    n = sendto(socket_client_fd, data, len, 0, (struct sockaddr *)&addr_serv_tmp, sizeof(addr_serv_tmp));    
    return(n);  
}

UdpInterface::~UdpInterface()
{


}