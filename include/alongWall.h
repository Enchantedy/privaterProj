#include "skeletonService.h"
#include <string.h>
class AlongWall:public SkeletonService
{
public:
    enum BumpList
    {

        BumpNo = 0,           //没有碰撞0

        InsideVirtual,        //靠墙一侧有虚拟墙信号1

        FrontVirtual,         //有虚拟墙信号2

        Bump,                   //碰撞3

        CliffLeft,            //左探地4

        CliffInter,           //中探地5

        CliffRight,           //右探地6

        MagVirtualLeft,       //磁性虚拟墙左7

        MagVirtualInter,      //磁性虚拟墙中8

        MagVirtualRight,      //磁性虚拟器右9

        OBSLeft,                //全方位左10

        OBSFront,               //全方位中11

        OBSRight,               //全方位右12

        ForbidChargeFront,      //前禁区13

        ForbidChargeLeft,       //左禁区14

        ForbidChargeRight,      //右禁区15

        ForbidFront,            //前禁区16

        ForbidLeft,             //左禁区17

        ForbidRight,            //右禁区18

        RadarFront,             //前雷达信号19

        RadarLeft,              //左雷达信号20

        RadarRight,             //右雷达信号21

        InCharge                 //陷入回充座禁区22

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
        NO_DIR = 0,
        LEFT_ALONG_WALL = 1,
        RIGHT_ALONG_WALL = 2,
    };

    enum SpeedMode
    {
        SLOW = 0,
        QUICK = 1,
        EXPRESS = 2
    };

    enum AlongWallMode
    {
        PID_ALONG_WALL = 0,
        CHARGE_ALONG_WALL = 1,
    };

    struct along_wall_para
    {
        BumpList bump_record;
        BumpList bump_state;
        AlongWallDir dir;
        PID pid;
        BumpDealProcess bump_deal_state;
        SpeedMode speed_mode;
        AlongWallMode mode;
        int last_add_angle;
        int vir_turn_cnt;
        long long start_time;
        long long quit_time;
        int add_angle;
        int aim_wall_dis;
        int now_wall_dis;
        int bump_flag;
        float small_place_x;
        float small_place_y;
        int fast_wall_count;
        int restricted_zone;
        bool first_time_flag = false;
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

    void wallFollowDeal();

    void initPid();
    
    int wallPid(int now_value, int aim_value);

    bool isNearForbidArea(float x, float y, int area_type);

    float getForbidAreaDis(int dir, int area_type);

    bool isInChargeArea();

    bool isInForbidArea();

    bool isAwayChargeArea();

    bool isAwayForbidArea();
protected:
    bool threadLoop();
private:
    chassisBase chassis_;
    Grid current_pos_;
    Sensor current_sensor_;
    bool along_wall_running_ = false;
    float record_forward = 0;
    along_wall_para wall_para_;
};
