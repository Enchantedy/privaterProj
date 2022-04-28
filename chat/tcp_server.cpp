#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
int fd;
int main(int argc, char const *argv[])
{
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd == -1) {
        perror("socket failed");
        return 1;
    }
    struct sockaddr_in s_addr;
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(atoi(argv[1]));
    s_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int rc = bind(fd, (const struct sockaddr *)&s_addr, sizeof(s_addr));
    if(rc) {
        perror("bind failed");
    }
    //设置监听状态
	listen(fd, 1);
    struct sockaddr_in c_addr;
    while(true) {
		bzero(&c_addr,sizeof(c_addr));
		socklen_t len = sizeof(c_addr);
        std::cout << "waiting....." << std::endl;
        int conect_fd = accept(fd, (struct sockaddr *)&c_addr, &len);
        std::cout << "connect!" << std::endl;
    }
}