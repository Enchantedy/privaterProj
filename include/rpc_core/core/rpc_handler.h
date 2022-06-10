#ifndef _RPC_HANDLER_H_
#define _RPC_HANDLER_H_

#include <map>
#include <mutex>
#include <string>
#include <sstream>
#include <memory>
#include <thread>
#include <vector>
#include <future>
#include <chrono>
#include <stdint.h>

#include <msgpack.hpp>
#include <core/queue.h>
#include <core/header.h>
#include <core/ptr_buffer.h>

#define LIBCORE_VERSION     "1.0.0"

namespace core {

class RpcHandler
{
public:
    class FutureObject {
    public:
        bool success;
        struct core::header header;
        std::vector<char> packed_data;
    };

    class RpcSendContext {
    public:
        uint32_t seq;
        bool send;
        bool recv;
        struct core::header header;
        std::vector<char> packed_data;
        bool has_promise;
        std::shared_ptr<std::promise<std::shared_ptr<FutureObject>>> promise;
    };

    class RpcRecvContext {
    public:
        uint32_t seq;
        struct core::header header;
        std::vector<char> packed_data;
    };

    class PromiseContext {
    public:
        int seq;
        std::shared_ptr<std::promise<std::shared_ptr<FutureObject>>> promise;
    };

public:
    RpcHandler() {
        fd = -1;
        seq_ = 1;
        send_running_ = false;
        recv_running_ = false;
        deal_running_ = false;
        send_thread_ = nullptr;
        recv_thread_ = nullptr;
        deal_thread_ = nullptr;
    }

    void setSocketFd(int fd) {
        this->fd = fd;
        sock_connected_ = true;
    }

    void start();

    void stop();

    bool isSendAlive();

    bool isRecvAlive();

    bool isTransceiverAlive();

protected:
    virtual bool threadSendLoop();
    virtual bool threadRecvLoop();
    virtual bool threadDealLoop();

    uint32_t generateSeq();

    bool submitToSendQueue(std::shared_ptr<RpcSendContext> sendContext);
    bool submitToRecvQueue(std::shared_ptr<RpcRecvContext> recvContext);

    uint32_t sendRequestWithPayload(struct header &header, std::vector<char> &payload);
    uint32_t sendRequest(struct header &header);
    bool waitResponseWithPayload(uint32_t seq, struct header &header, std::vector<char> &payload, int timeout = 100);
    bool waitResponse(uint32_t seq, struct header &header, int timeout = 100);

    bool sendResponseWithPayload(struct header &header, std::vector<char> &payload);
    bool sendResponse(struct header &header); 

    virtual bool handleRequest(struct header &header, std::vector<char> &payload) = 0;

    template<typename T>
    uint32_t sendRequest(struct header &header, T &t)
    {
        if (isTransceiverAlive() == false)
            return 0;

        std::stringstream buffer;
        msgpack::pack(buffer, t);
        buffer.seekg(0);
        std::string str(std::move(buffer.str()));
        std::vector<char> payload(str.begin(), str.end()); // fixme: rvalue later

        header.data_len = payload.size();

        return sendRequestWithPayload(header, payload);
    }

    template<typename T>
    bool sendResponse(struct header &header, T &t)
    {
        if (isTransceiverAlive() == false)
            return false;

        std::stringstream buffer;
        msgpack::pack(buffer, t);
        buffer.seekg(0);
        std::string str(std::move(buffer.str()));
        std::vector<char> payload(str.begin(), str.end()); // fixme: rvalue later

        header.data_len = payload.size();

        return sendResponseWithPayload(header, payload);
    }

    template<typename T>
    bool waitResponse(uint32_t seq, struct header &header, T &t, int timeout = 100)
    {
        bool success;
        std::vector<char> payload;

        if (seq == 0) return false;

        success = waitResponseWithPayload(seq, header, payload, timeout);
        if (success == false) {
            return false;
        }

        msgpack::object_handle oh = msgpack::unpack(payload.data(), payload.size());
        msgpack::object deserialized = oh.get();

        deserialized.convert(t);

        return true;
    }

private:
    void threadSendFunction();
    void threadRecvFunction();

    void process_message(int cmdid,  unsigned char *body, unsigned int body_len);
    void process_recv_buf();
    void threadDealFunction();

protected:
    int fd;
    std::shared_ptr<std::thread> send_thread_;
    std::shared_ptr<std::thread> recv_thread_;
    std::shared_ptr<std::thread> deal_thread_;
    volatile bool send_running_;
    volatile bool recv_running_;
    volatile bool deal_running_;
    volatile bool sock_connected_;
    uint32_t seq_;
    std::mutex seq_mutex;
    core::Queue<std::shared_ptr<RpcSendContext>> send_queue;
    std::condition_variable event_condition_;
    std::mutex event_mutex_;
    core::Queue<std::shared_ptr<RpcRecvContext>> recv_queue;
    std::condition_variable recv_condition_;
    std::mutex recv_mutex_;
    std::map<uint32_t, std::shared_ptr<PromiseContext>> promise_map;
    std::mutex promise_mutex_;
    std::mutex stop_mutex_;
    unsigned char static_sendbuf[1024];
    ptr_buffer_t sendbuffer;
    unsigned char static_recvbuf[2*1024];
    ptr_buffer_t recvbuffer;
    unsigned char static_bodybuf[1024];
};

}

#endif /* _RPC_HANDLER_H_ */
