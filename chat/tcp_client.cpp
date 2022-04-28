#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <thread>
int fd;
void recv_server()
{
    char recv_buf[255] {0};
    char last_recv_buf[255] {0};
    while(true) {
        bzero(recv_buf, sizeof(recv_buf));
        recv(fd, (void *)recv_buf, sizeof(recv_buf), 0);
        printf("\nGet：%s\n", recv_buf);
		fprintf(stderr, "Pls Input: ");
    }
    
}

int main(int argc, char const *argv[])
{
    int rc = 0;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd == -1) {
        perror("socket failed");
        return 1;
    }
    struct sockaddr_in c_addr;
    bzero(&c_addr, sizeof(c_addr));
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(atoi(argv[1]));
    c_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rc = bind(fd, (const struct sockaddr *)&c_addr, sizeof(c_addr));
    if(rc) {
        perror("bind failed");
        return 1;
    }
    struct sockaddr_in s_addr;
	bzero(&s_addr, sizeof(s_addr));
	s_addr.sin_family = AF_INET;
	s_addr.sin_port = htons(atoi(argv[3]));
    inet_pton(AF_INET, argv[2], (void *)&s_addr.sin_addr);
    rc = connect(fd, (const struct sockaddr *)&s_addr, sizeof(s_addr));
    if(rc == -1) {
        perror("connect failed");
        close(fd);
        return 2;
    }
    else
        std::cout << "connect server successful" << std::endl;
    std::shared_ptr<std::thread> recv = std::make_shared<std::thread>(recv_server);
    char msg[255] {0};
    while(true) {
        std::cout << "if quit, intput: exit" << std::endl;
        std::cout << "input:" << std::endl;
        fgets(msg, sizeof(msg), stdin);strtok(msg, "\n");
        if(!strcmp(msg, "exit"))
            break;
        send(fd, msg, strlen(msg), 0);
    }
    recv->join();
    close(fd);
    return 0;
}