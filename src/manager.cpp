#include "include/manager.h"

bool Manager::threadLoop()
{
    usleep(500 * 1000);
    std::cout << "threadLoop" << std::endl;
}