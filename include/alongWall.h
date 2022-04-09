#include "skeletonService.h"

class AlongWall:public SkeletonService
{
public:
    struct along_wall_para
    {
    };
    
public:
    AlongWall(){}
    ~AlongWall() 
    { 
        if (thread_->joinable()) {
            thread_->join();
            thread_ = nullptr;
        }
    }
    int init();
    void startAlongWall();
    void stopAlongWall();
    bool wheelSpin();
    bool wheelBack();
    int getState();
protected:
    bool threadLoop();
private:
    bool along_wall_running_ = false;

};
