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
    void startAlongWall();
    void stopAlongWall();
    bool wheelSpin();
    bool wheelBack();
protected:
    bool threadLoop();
private:
    bool along_wall_running_ = false;

};
