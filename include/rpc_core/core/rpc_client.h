#ifndef RPC_CLIENT_H
#define RPC_CLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string>
#include <core/skeleton_thread.h>

namespace core {

class RpcClient : public SkeletonThread
{
public:
    RpcClient();

    void setServerAddress(std::string server_address);

    bool connectServer();

    void disconnect();

protected:
    virtual void handleNewConnection(int fd);

    virtual bool isDisconnectDetected();

    virtual bool threadLoop();

private:
    std::string server_address;
    int client_socket;
    struct sockaddr_un server_sockaddr;
    bool sock_connected_;
};

}

#endif
