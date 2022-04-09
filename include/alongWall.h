#include "skeletonService.h"

class AlongWall:public SkeletonService
{
public:
    struct along_wall_para
    {
    };
    
public:
    void startAlongWall();
    void stopAlongWall();
    bool wheelSpin();
    bool wheelBack();
protected:
    bool threadLoop();
private:
    bool along_wall_running_ = false;

};
