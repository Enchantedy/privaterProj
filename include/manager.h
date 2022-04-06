#include <iostream>
#include "skeletonService.h"


class Manager : public::SkeletonService
{
    public:
    protected:
        virtual bool threadLoop();
    private:
        std::shared_ptr<std::thread> manager_;
        std::mutex manager_lock_;
};
