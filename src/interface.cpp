#include "include/interface.h"

int along_wall_init()
{
    int rc;
    along_wall_ = std::make_shared<AlongWall>();
    if(along_wall_ == nullptr) {
        return -1;
    }
    along_wall_->start();
}

void start_along_wall()
{
    along_wall_->startAlongWall();
}