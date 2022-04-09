#include "include/alongWall.h"


void AlongWall::startAlongWall()
{
    along_wall_running_ = true;
}

void AlongWall::stopAlongWall()
{
    along_wall_running_ = false;
}

bool AlongWall::threadLoop()
{
    if(!along_wall_running_) {
        usleep(100 * 1000);
        return true;
    }
}