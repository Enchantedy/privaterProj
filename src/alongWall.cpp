#include "include/alongWall.h"


void AlongWall::startAlongWall()
{
    along_wall_running_ = true;
}

void AlongWall::stopAlongWall()
{
    along_wall_running_ = false;
}

int AlongWall::init()
{
    memset(&wall_para, 0, sizeof(along_wall_para));
}

void AlongWall::bumpDetect()
{
    
}

void AlongWall::bumpDeal()
{

}

void AlongWall::alongWallDeal()
{

}

bool AlongWall::threadLoop()
{
    if(!along_wall_running_) {
        usleep(100 * 1000);
        return true;
    }
    if(!wall_para.bump_flag)
        bumpDetect();
    if(wall_para.bump_flag)
        bumpDeal();
    else
        alongWallDeal();
}
