/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-24 11:48:32
 * @Project      : UM_path_planning
 */


#include "navigation_algorithm/InternalSpiralPlanning.h"
extern useerobot::Maze _maze;
extern float gyo_angle_;
// 常数定义
//-----------------------------------------------------------------------------
#define FIXED_HIGH_SP 260 //高速轮速度
#define FIXED_LOW_SP 0   //低速轮速度
#define BEHIND_SPEED_CONST 160 //后退时速度
#define BUMP_BEHIND_TIME            (200)      //碰撞之后退时间 ms
 
#define FIXED_MAX_RADIUS            (22)   //定点清扫最大半径

//定点清扫方向定义
typedef enum {
    FIXED_CW = 0, //顺时针
    FIXED_CCW = !FIXED_CW,//逆时针
} CLEAN_DIR;

typedef enum {
    INCREASE = 0,//半径递增
    DECRE = !INCREASE,//半径递减
}RADIU_ENU;

typedef enum {
    FIXED_BEHIND_IDLE = 0,//空闲
    FIXED_BEHIND_START = 1,//后退开始
    FIXED_BEHIND_ING = 2,//后退中
    FIXED_BEHIND_END = 3,//后退结束
    FIXED_BEHIND_ADJ = 4,//后退结束调整角度
    FIXED_BEHIND_SPIN_DETC = 5,//后退，360自旋检测
    FIXED_SPIN_DETC_WAIT = 6,// 自旋等待
    FIXED_OBS_STOP_WAIT = 7,//全方位停止等待
}FIXED_BEHIND_ENU;

typedef enum {
    FIXED_IDLE = 0, //空闲
    FIXED_UNIL_ING = 1,//单边旋中
    FIXED_END = 2,    //定点清扫结束
    FIXED_WAIT = 3,    //定点清扫主模式等待
} CLEAN_ENU;

//定点清扫函数参数结构体
typedef struct {
    uint8_t bumpValid;          //碰撞有效标志
    uint8_t fixedState;        //定点清扫主状态机
    uint8_t behindState;       //定点清扫后退状态机
    uint8_t radiIncrDecDir;     //半径增减方向
    uint8_t radiChangeFlag;     //半径改变标志
    int16_t currHighSpeed;      //当前高速轮速度
    int16_t currLowSpeed;       //当前低速轮速度
    int16_t currFixedDir;       //当前清扫方向
    int16_t currFixedTurnNum;   //当前清扫圈数
    int16_t currFixedRadius;    //当前清扫半径
    int32_t radiuCalcCnt;       //半径计算计数
    int32_t currAngleDiff;      //当前清扫旋转角度
    int32_t startAngle;         //开始清扫的角度
    int32_t intoFixedAngle;     //进入定点清扫时的角度
    int32_t behindCompleWaitTime;//后退完等待时间
}fixdPara_t;

fixdPara_t Para;
static int spinWaitTime = 0;
static bool fixedCiifFlagBack = false;
static bool spinbumpFlag = false;
static int b_time = 0;
static int behindSumSpd = 0;

namespace useerobot
{
    
    InternalSpiralPlanning::InternalSpiralPlanning()
    {
    }    

    InternalSpiralPlanning::~InternalSpiralPlanning()
    {
        
    }

    bool InternalSpiralPlanning::goPoint(Point &startPoint,Point &endPoint,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start to go to the clean point in InternalSpiralPlanning::goPoint model ");
        auto astarMaze = Maze(_maze);
        astarMaze.startPoint = startPoint;
        astarMaze.endPoint =  endPoint;
        auto Astar = aStar();
        auto Motioncontrol = MotionControl();

        if(robotShape == RobotType::circle)
        {
            astarResult = Astar.findPath(astarMaze.startPoint, astarMaze.endPoint, robotShape);
            astarPath = Astar.getPath(astarResult);
            Motioncontrol.pathTransformtoOrder(astarPath); //路径到motioncontrol模块 是否需要在这里先执行？？？
            FRIZY_LOG(LOG_INFO, "successful to reach the  clean point in InternalSpiralPlanning::goPoint model ");
            return true;
        }
        else if(robotShape == RobotType::rectangle)
        {
            //need to do
        }

    }

    bool InternalSpiralPlanning::internalSpiralCleanPoint(Point &cleanPoint,RobotType &robotShape,int &layers)
    {
        FRIZY_LOG(LOG_INFO, "start to clean point in InternalSpiralPlanning::goPoint model ");

        if(robotShape == RobotType::circle)
        {


            InteralSpiralPath.push_back(&cleanPoint);
            tempPoint = cleanPoint;
            for(int a = 0;a <layers ; a++ )
            {
                for(int i =0;i<2*(a+1);i++) //右Y轴boundary
                {
                    tempPoint.x = tempPoint.x + cleaning_interval;
                    tempPoint.y =  tempPoint.y + cleaning_interval*i;
                    InteralSpiralPath.push_back(&tempPoint);
                }
                for(int i =0;i<2*(a+1);i++)//底X轴boundary
                {
                    tempPoint.x = tempPoint.x - cleaning_interval*(i+1);
                    tempPoint.y =  tempPoint.y;
                    InteralSpiralPath.push_back(&tempPoint);                    
                }
                for(int i =0;i<2*(a+1);i++)//左Y轴boundary
                {
                    tempPoint.x = tempPoint.x;
                    tempPoint.y =  tempPoint.y - cleaning_interval*(i+1);
                    InteralSpiralPath.push_back(&tempPoint);                       
                }
                for(int i =0;i<2*(a+1);i++)//顶X轴boundary
                {
                    tempPoint.x = tempPoint.x + cleaning_interval*(i+1);
                    tempPoint.y =  tempPoint.y;
                    InteralSpiralPath.push_back(&tempPoint);                     
                }                                                

            }

        }

        else if(robotShape == RobotType::rectangle)
        {
            //need to do
        }        

    }

    //定点清扫初始化
    void InternalSpiralPlanning::fixPointInit()
    {
        chassis_.GetSensor(&fix_sensor_);
        memset(&Para,0,sizeof(fixdPara_t));
        FRIZY_LOG(LOG_DEBUG, "Para %d %d", Para.behindState, Para.fixedState);
        Para.currHighSpeed = FIXED_HIGH_SP;
        Para.currLowSpeed = FIXED_LOW_SP;
        Para.currFixedDir = FIXED_CW;//定点清扫方向，顺时针
        Para.intoFixedAngle = fix_sensor_.addAngle;
        Para.radiIncrDecDir = INCREASE;
        Para.behindState = FIXED_BEHIND_IDLE;
        b_time = 0;
        behindSumSpd = 0;
        FRIZY_LOG(LOG_DEBUG, "fix point clean init");
    }

    
    void InternalSpiralPlanning::backDis(int speed, int dis)
    {
        int currSpdTmp = 0;
        int cnt = 20;
        while(running_) 
        {
            if(cnt > 0)
                cnt--;
            else
            {
                chassis_.chassisSpeed(0, 0, 1);
                return;
            }
            chassis_.chassisSpeed(-160, -160, 1);
            // chassis_.GetSensor(&fix_sensor_);
            // currSpdTmp = (fix_sensor_.rightw + fix_sensor_.leftw)/2;
            // behindSumSpd += currSpdTmp;
            // b_time++;

            // FRIZY_LOG(LOG_DEBUG, "b_time:%d,%d", abs(behindSumSpd*b_time*10),(dis-5)*1000);
            // chassis_.chassisSpeed(speed, speed, 1);

            // if(abs(behindSumSpd*b_time*10) >= (dis-5)*1000) //mm/s ms
            // {
            //     chassis_.chassisSpeed(0, 0, 1);
            //     b_time = 0;
            //     behindSumSpd = 0;
            //     return;
            // }
            usleep(20 * 1000);
        }
        chassis_.chassisSpeed(0, 0, 1);
        return;
    }

    int InternalSpiralPlanning::spin(int lSpeed, int rSpeed, int angle)
    {
        float aimForward = angle + (360-gyo_angle_*180/_Pi);
        if(aimForward >= 360)
            aimForward = aimForward - 360;
        while(running_)
        {
            chassis_.chassisSpeed(lSpeed, rSpeed, 1);
            chassis_.GetSensor(&fix_sensor_);
            if(fix_sensor_.cliff || fix_sensor_.bump/* || fix_sensor_.magnVirWall*/)
            {
                chassis_.chassisSpeed(0, 0, 1);
                spinbumpFlag = true;
                spinWaitTime = 0;
                FRIZY_LOG(LOG_DEBUG, "spin 180... bump");
                return 0;
            }
            // FRIZY_LOG(LOG_INFO,"aimForward:%f, gyo_angle_:%f", aimForward, (360-gyo_angle_*180/_Pi));
            if(fabs(aimForward - (360-gyo_angle_*180/_Pi)) < 4 || fabs(aimForward - (360-gyo_angle_*180/_Pi)) > 356)
            {
                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");   
                chassis_.chassisSpeed(0, 0, 1);  
                return 1;
            }
            usleep(20 * 1000);
        }
        chassis_.chassisSpeed(0, 0, 1);
        return -1;
    }

    //碰撞类检查并处理
    void InternalSpiralPlanning::fixPointDeal()
    {
        static int16_t bumpInvlidCnt = 0;
        static int16_t fixedBumpNoCnt = 0;
        static int32_t fixedSpinSratAgl = 0;
        //只在后退完和自旋的时候检测碰撞、探地
        if(FIXED_BEHIND_IDLE == Para.behindState || FIXED_OBS_STOP_WAIT == Para.behindState || spinbumpFlag)
        {
            //探地
            if(fix_sensor_.cliff)
            {
                chassis_.chassisSpeed(0, 0, 1);
                Para.behindState = FIXED_BEHIND_START;
                Para.fixedState = FIXED_WAIT;
                FRIZY_LOG(LOG_DEBUG, "fix cliff");
            }
            //碰撞
            else if(fix_sensor_.bump)
            {
                chassis_.chassisSpeed(0, 0, 1);
                Para.behindState = FIXED_BEHIND_START;
                Para.fixedState = FIXED_WAIT;
                FRIZY_LOG(LOG_DEBUG, "fix bump");
            }
            //磁虚拟墙
            // else if(fix_sensor_.leftFrontVir || fix_sensor_.rightFrontVir ||
            //         fix_sensor_.leftVir || fix_sensor_.rightVir)
            // {
            //     chassis_.chassisSpeed(0, 0, 1);s
            //     Para.behindState = FIXED_BEHIND_START;
            //     Para.fixedState = FIXED_WAIT;
            //     FRIZY_LOG(LOG_DEBUG, "fix magnVirWall");
            // }
            //回充座 红外信号
            else if(fix_sensor_.Infrared)
            {
                chassis_.chassisSpeed(0, 0, 1);
                Para.behindState = FIXED_BEHIND_END;
                Para.fixedState = FIXED_WAIT;
                FRIZY_LOG(LOG_DEBUG, "fix irWall");
            }
            //全方位
            else if(fix_sensor_.midOmnibearingTurn)
            {
                chassis_.chassisSpeed(0, 0, 1);
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    Para.behindState = FIXED_OBS_STOP_WAIT;//进入全方位停止等待
                    Para.fixedState = FIXED_WAIT;
                }
                FRIZY_LOG(LOG_DEBUG, "fix obs stop");
            }
            //前方位减速
            else if(fix_sensor_.midOmnibearingSlow && !fix_sensor_.midOmnibearingTurn)
            {
                int tmpl = chassis_.getSendSpeed().first * 0.5;
                int tmpr = chassis_.getSendSpeed().second * 0.5;
                chassis_.chassisSpeed(tmpl, tmpr, 1);
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    Para.behindState = FIXED_OBS_STOP_WAIT;//进入全方位停止等待
                    Para.fixedState = FIXED_WAIT;
                }
                FRIZY_LOG(LOG_DEBUG, "fix obs slow");
            }
            else
            {   
                FRIZY_LOG(LOG_DEBUG, "Para.currFixedDir == FIXED_CW");
                //全方位减速解除
                if(Para.currFixedDir == FIXED_CW)
                {
                    chassis_.chassisSpeed(Para.currHighSpeed, Para.currLowSpeed, 100);
                }
                else
                {
                    chassis_.chassisSpeed(Para.currLowSpeed, Para.currHighSpeed, 100);
                }
            }
            spinbumpFlag = false;
        }
        FRIZY_LOG(LOG_DEBUG, "behindState:%d", Para.behindState);
        switch (Para.behindState)
        {
            case FIXED_BEHIND_IDLE: //空闲 0
            {
            }
            break;
        
            case FIXED_BEHIND_START: //后退开始 1
            {
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    Para.behindCompleWaitTime = 0;
                    fixedBumpNoCnt = 0;
                    Para.behindState = FIXED_BEHIND_ING;
                }
                if(fix_sensor_.cliff)
                {
                    fixedCiifFlagBack = true;
                    chassis_.chassisSpeed(-BEHIND_SPEED_CONST, -BEHIND_SPEED_CONST, 1);
                }
                // else if(fix_sensor_.magnVirWall)
                // {
                //     fixedCiifFlagBack = true;
                //     chassis_.chassisSpeed(-BEHIND_SPEED_CONST, -BEHIND_SPEED_CONST, 1);
                // }
                else
                {
                    backDis(-BEHIND_SPEED_CONST, 15);
                }
            }
            break;

            case FIXED_BEHIND_ING: //后退中 2
            {
                if(fixedCiifFlagBack)//探地、磁条后退
                {
                    if(!fix_sensor_.cliff && !fix_sensor_.bump)//后退到没有碰撞、探地信号
                    {
                        if(fixedBumpNoCnt > 100/20)
                        {
                            fixedBumpNoCnt = 0;
                            fixedCiifFlagBack = false;
                            chassis_.chassisSpeed(0, 0, 1);
                            Para.behindState = FIXED_BEHIND_END;
                            FRIZY_LOG(LOG_DEBUG, "realCliff OK");
                        }
                        else
                        {
                            fixedBumpNoCnt++;
                        }
                    }
                    else
                    {
                        if(fixedBumpNoCnt>5)
                            fixedBumpNoCnt -= 5;
                        
                        //还没退出探地或者磁条信号就停下来了
                        if(chassis_.getWheelState() == WHEELSTOP)
                            Para.behindState = FIXED_BEHIND_START;
                    }
                }
                else
                {
                    if(chassis_.getWheelState() == WHEELSTOP)
                    {
                        if(!fix_sensor_.cliff && !fix_sensor_.bump)//后退到没有碰撞、探地信号
                        {
                            Para.behindState = FIXED_BEHIND_END;
                        }
                        else//碰撞后退还有碰撞，则继续后退
                        {
                            Para.behindState = FIXED_BEHIND_START;
                        }
                        
                    }
                }
                
            }
            break;

            case FIXED_OBS_STOP_WAIT://7 全方位停止等待
            {
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    Para.behindState = FIXED_BEHIND_END;
                }
                else
                    chassis_.chassisSpeed(0, 0, 1);
            }
            break;

            // case FIXED_BEHIND_SPIN_DETC://后退超出距离 360旋转判定
            // {}
            // break;

            // case FIXED_SPIN_DETC_WAIT:
            // {}
            // break;

            case FIXED_BEHIND_END://后退结束 3
            {
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    FRIZY_LOG(LOG_DEBUG, "spin 180");
                    int rc = 0;
                    rc = spin(120, -120 , 180);
                    if(rc != 1)
                        break;
                    Para.behindState = FIXED_BEHIND_ADJ;
                    Para.behindCompleWaitTime = 0;
                }
                else
                    chassis_.chassisSpeed(0, 0, 1);
            }
            break;

            case FIXED_BEHIND_ADJ: //等待调整完成 4
            {
                if(chassis_.getWheelState() == WHEELSTOP)
                {
                    if(spinWaitTime < 200/20)
                    {
                        spinWaitTime++;
                    }
                    else
                    {
                        spinWaitTime = 0;
                        Para.behindState = FIXED_BEHIND_IDLE;
                        Para.currFixedDir = !Para.currFixedDir;
                        Para.fixedState = FIXED_IDLE;
                        Para.radiChangeFlag = true;
                        FRIZY_LOG(LOG_DEBUG, "back ok dir.%d",Para.currFixedDir);
                    }
                }
                else
                    chassis_.chassisSpeed(0, 0, 1);
            }
            break;
            
            default:
                break;
        }
    }
    void InternalSpiralPlanning::stopPointClean()
    {
        FRIZY_LOG(LOG_DEBUG, "stop fix point clean");
        running_ = false;
        chassis_.chassisSpeed(0, 0, 1);
    }
    bool InternalSpiralPlanning::getRunning()
    {
        return running_;
    }
    bool InternalSpiralPlanning::pointClean(RobotType &robotShape)
    {
        // _motioncontrol.robotchassiscontrol.chassisSpeed(30,200,1);
        // pointCleanTime--;
        // FRIZY_LOG(LOG_DEBUG, "pointCleanTime*20 = %d",pointCleanTime);
        static int StepValue = 12;
        int tempRadius;
        running_ = true;
        while(running_)
        {
            chassis_.GetSensor(&fix_sensor_);
            fixPointDeal();
            FRIZY_LOG(LOG_DEBUG, "fixedState:%d", Para.fixedState);
            switch(Para.fixedState)
            {
                case FIXED_IDLE:
                {
                    //顺时针
                    if(Para.currFixedDir == FIXED_CW)
                    {
                        chassis_.chassisSpeed(Para.currHighSpeed, Para.currLowSpeed, 100);
                    }
                    else
                    {
                        chassis_.chassisSpeed(Para.currLowSpeed, Para.currHighSpeed, 100);
                    }
                    
                    //记录开始单边旋的角度
                    Para.startAngle = fix_sensor_.addAngle;
                    Para.fixedState = FIXED_UNIL_ING;
                }
                break;

                case FIXED_UNIL_ING:
                {
                    //如果中途被打断，重新启动
                    if(chassis_.getWheelState() == WHEELSTOP)
                    {
                        Para.fixedState = FIXED_IDLE;
                    }
                    Para.currAngleDiff = abs(Para.startAngle - fix_sensor_.addAngle);
                    //旋转超过360 或者 因为碰撞 方向行走 半径需要改变
                    if(Para.currAngleDiff > 900 || Para.radiChangeFlag)
                    {
                        Para.radiChangeFlag = false;
                        FRIZY_LOG(LOG_DEBUG, "fix Dir.%d", Para.radiIncrDecDir);
                        //半径处理
                        if(Para.radiIncrDecDir == INCREASE) //半径递增
                        {
                            Para.currFixedRadius++;
                            FRIZY_LOG(LOG_DEBUG, "currFixedRadius.%d", Para.currFixedRadius);
                            if(Para.currFixedRadius >= FIXED_MAX_RADIUS)
                            {
                                //超过最大清扫半径之后，开始递减清扫
                                Para.radiIncrDecDir = DECRE;
                                Para.fixedState = FIXED_IDLE;
                                break;
                            }
                        }
                        else //半径递减
                        {
                            FRIZY_LOG(LOG_DEBUG, "currFixedRadius.%d", Para.currFixedRadius);
                            if(Para.currFixedRadius > 0)
                                Para.currFixedRadius--;
                            else //半径减为0停止定点清扫
                            {
                                Para.fixedState = FIXED_END;
                                break;
                            }
                        }
                        //计算半径
                        tempRadius = 2*StepValue+Para.currFixedRadius;
                        //计算速度
                        Para.currLowSpeed = FIXED_HIGH_SP*Para.currFixedRadius/tempRadius;
                        Para.currFixedTurnNum++;
                        Para.fixedState = FIXED_IDLE;
                        FRIZY_LOG(LOG_DEBUG, "lowSp.%d turnNum.%d curR.%d dir.%d",
                        Para.currLowSpeed,Para.currFixedTurnNum,Para.currFixedRadius,Para.currFixedDir);
                    }
                }
                break;

                case FIXED_END:
                {
                    chassis_.chassisSpeed(0, 0, 1);
                    Para.fixedState = FIXED_IDLE;
                    memset(&Para,0,sizeof(fixdPara_t));
                    Para.currHighSpeed = FIXED_HIGH_SP;
                    Para.currLowSpeed = FIXED_LOW_SP;
                    Para.currFixedDir = FIXED_CW;//定点清扫方向，顺时针
                    Para.intoFixedAngle = fix_sensor_.addAngle;
                    Para.radiIncrDecDir = INCREASE;
                    Para.behindState = FIXED_BEHIND_IDLE;
                    FRIZY_LOG(LOG_DEBUG, "fixed cleaning complete");
                    return true;
                }
                break;

                case FIXED_WAIT:
                {
                }
                break;
            }
            usleep(20 * 1000);
        }
        chassis_.chassisSpeed(0, 0, 1);
        return false;
    }

    void InternalSpiralPlanning::randomClean()
    {
        while(1)
        {
            if(randomClean_start_index == true)
            {
        
                _motioncontrol.robotchassiscontrol.GetSensor(&current_sensor);
                if(current_sensor.bump ==0 && current_sensor.obs == 0 && current_sensor.magnVirWall == 0)
                // if(current_sensor.obs == 0)
                {
                    FRIZY_LOG(LOG_DEBUG, "START TO WALK STRAIGHT IN RANDOM MODE");
                    _motioncontrol.robotchassiscontrol.chassisSpeed(250,250,1);
                }
                else
                {
                    if((current_sensor.leftw != 0||current_sensor.rightw != 0)&&current_sensor.obs == 1)
                    {
                        _motioncontrol.robotchassiscontrol.chassisSpeed(0,0,1);
                        current_sensor.obs = 0;
                    }
                    if((current_sensor.leftw != 0||current_sensor.rightw != 0)&&current_sensor.magnVirWall == 1)
                    {
                        _motioncontrol.robotchassiscontrol.chassisSpeed(0,0,1);
                        current_sensor.magnVirWall = 0;
                    }                
                    _motioncontrol.robotchassiscontrol.chassisSpeed(100,-100,1);
                    int rotate_time = 1000+rand()%2000+1;
                    FRIZY_LOG(LOG_DEBUG, "THE ROTATE TIME IS %d",rotate_time);
                    usleep(1000*rotate_time);
                }
                usleep(50*1000);
            }
            else if(randomClean_start_index == false)
            {
                usleep(100*1000);
                continue;
            }
            
        }
    }
}

