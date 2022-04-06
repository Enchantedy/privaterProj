#include <iostream>
#include <thread>
#include <memory>
#include <iomanip>
#include <unistd.h>
#include <mutex>
class SkeletonService 
{
    public:
        SkeletonService();
        ~SkeletonService();
        void start();
        void stop();
        void isRunning();
    protected:
        bool running_;
        std::shared_ptr<std::thread> thread_;
        std::string name_;
        virtual bool threadLoop();
    private:
        void threadFunction();

};