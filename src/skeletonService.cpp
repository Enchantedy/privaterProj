#include "include/skeletonService.h"
#include<typeinfo>
SkeletonService::SkeletonService() 
{
    running_ = false;
}

SkeletonService::~SkeletonService() 
{
    if(running_) {
        std::cout << "~SKeletonService" << std::endl;
        stop();
    }
}

void SkeletonService::start()
{
    thread_ = std::make_shared<std::thread>(&SkeletonService::threadFunction, this);
}

void SkeletonService::stop()
{
    if (running_) {
        running_ = false;
        printf("exit thread id 0x%08x\n", thread_->get_id());
        thread_->join();
        thread_ = nullptr;
    }
}

bool SkeletonService::threadLoop()
{
    return false;
}

void SkeletonService::threadFunction()
{
    bool exec;
    name_ = typeid(*this).name();
    std::string tmp_name;
    if (name_.size() > 15) {
        tmp_name = name_.substr(0, 15);
    } else {
        tmp_name = name_;
    }
    std::cout << "thread_name:" << tmp_name << std::endl;
    pthread_setname_np(pthread_self(), tmp_name.c_str());
    while (running_)
    {
        exec = threadLoop();
        if(!exec) {
            break;
        }
    }
    
}