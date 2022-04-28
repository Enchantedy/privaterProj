#include "skeletonService.h"
#include <string.h>
class AlongWall:public SkeletonService
{
public:
    enum BumpList
    {

        BumpNo = 0,           //没有碰撞0

        OBSSlow,              //全方位减速1

        InsideVirtual,        //靠墙一侧有虚拟墙信号2

        FrontVirtual,         //有虚拟墙信号3

        BumpLeft ,            //左碰撞4

        BumpRight,            //右碰撞5

        BumpInter,            //中碰撞6

        CliffLeft,            //左探地7

        CliffInter,           //中探地8

        CliffRight,           //右探地9

        MagVirtualLeft,       //磁性虚拟墙左10

        MagVirtualInter,      //磁性虚拟墙中11

        MagVirtualRight,      //磁性虚拟器右12

        OBSLeft,                //全方位左13

        OBSFront,               //全方位中14

        OBSRight,               //全方位右15

        ForbidChargeFront,      //前禁区16

        ForbidChargeLeft,       //左禁区17

        ForbidChargeRight,      //右禁区18

        ForbidFront,            //前禁区19

        ForbidLeft,             //左禁区20

        ForbidRight,            //右禁区21

        RadarFront,             //前雷达信号22

        RadarLeft,              //左雷达信号23

        RadarRight,             //右雷达信号24

        VirFront,

        Virleftt,

        VirRight,

        IntoVir

    };
    
    enum BumpDealProcess
    {
        Idle = 0,

        BumpSignalBack,

        BackWithoutSignal,

        Turn,
        
        TurnVirtual,
    };

    struct PID
    {
        int p;
        int i;
        int d;
        int last_p;
    };

    enum AlongWallDir
    {   
        NO_DIR = 0
        LEFT_ALONG_WALL = 1,
        RIGHT_ALONG_WALL = 2,
    };

    enum SpeedMode
    {
        SLOW = 0,
        QUICK = 1,
        EXPRESS = 2
    };

    struct along_wall_para
    {
        BumpList bump_state;
        AlongWallDir dir;
        PID pid;
        BumpDealProcess bump_deal_state;
        SpeedMode speed_mode;
        long long start_time;
        long long quit_time;
        int add_angle;
        int aim_wall_dis;
        int now_wall_dis;
        int bump_flag;
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
    void bumpDetect();
    void bumpDeal();
    void alongWallDeal();
protected:
    bool threadLoop();
private:
    bool along_wall_running_ = false;
    float record_forwar = 0;
    along_wall_para wall_para;
};
