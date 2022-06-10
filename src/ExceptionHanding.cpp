/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-27 11:41:01
 * @Project      : UM_path_planning
 */

#include <iostream>
#include "common_function/ExceptionHanding.h"
#include "navigation_algorithm/AlongWall.h"

#define _Pi 3.1415926
/*
*  边轮 电机参数
*/
//π
#define USER_PI (3.14159f)

//齿轮比

#define WHEEL_GEAR_RATIO (46.852f)

//电机转一圈脉冲数
#define WHEEL_CYCLE_PULSES_NUM (20)

//边轮直径
#define WHEEL_DIAMETER (60) //(64.4f)//mm

//两轮间距  左边轮中间到右边轮中间的距离
#define WHEEL_SPACING (193.5f) //mm

//边轮可以达到的最大速度 mm/s
#define WHEEL_TURN_MAX_SPD (340)

using namespace useerobot;
// extern float gyo_angle_;

current_pose_t* current_pose;
namespace useerobot
{
    /**** 架空脱困识别 end ********/
    // static int16_t angVelOutCnt = 0,abnrCurrOutCntL = 0,abnrCurrOutCntR = 0;
    //脱困检测变量
    stuckCheckPara_t escCkeckPara;
    // 脱困参数定义
    escapeActionPara_t escActPara;
    //记录脱困前的沿墙方向
    WallFollowDir_t alongWallDir;
    //吧台椅识别变量结构体
    espIdentPitRol_t espIdentPR;

//能转动的角度范围定义
#define ENABLE_MOVE_ANGLE_H (40)
#define ENABLE_MOVE_ANGLE_L (10)
#define WHEEL_LEFT_CURRENT_LIMIT (550)  //左轮电流限制
#define WHEEL_RIGHT_CURRENT_LIMIT (550) //右轮电流限制
#define WHEEL_CURRENT_ABNORMAL (30)     //电流异常差值倍数 放大了10倍
//标识宏定义
#define CUR_ABNORMAL (0x01)    //电流异常标识
#define RATE_ABNORMAL (0x02)   //角速度异常标识
#define PITROL_ABNORMAL (0x04) //俯仰横滚异常标识
#define ACC_ABNORMAL (0x08)    //加速度异常标识
//数组内存宏定义
#define STUCK_ACC_ARRY_NUM (5)          //卡死判定加速度滑动平均值数组内存
#define PITROL_AVG_NUM (5)              //俯仰和横滚滑动平均值数组内存
#define PITROL_LAST_AVG_NUM (10)        //1.6s前俯仰和横滚滑动平均值数组内存
#define AVG_WHEEL_CURARRY_NUM (5)       //电流滑动平均值数组内存
#define AVG_LAST_WHEEL_CURARRY_NUM (10) //1.6s前电流滑动平均值数组内存
#define POSITION_CHECK_ARRY_NUM (6)     //1s

    /*滑动数据存储数组定义*/
    static int escLastLeftSideBrushCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastRightSideBrushCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastLeftCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastRightCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLeftCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int escRightCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int RightSideBrushCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int LeftSideBrushCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int mainbrushcurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int LeftWheelcurArry[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int RightWheelcurArry[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int PositionDiffArry[POSITION_CHECK_ARRY_NUM] = {0};
    // static int
    static long long escEnterTimeInterval = 0; //记录进入脱困的时间间隔
    static int16_t escEnterIntervalCnt = 0;    //记录进入短时间脱困的次数

    static int gyroAccBuf[STUCK_ACC_ARRY_NUM] = {0};
    static float pitRolAvgArry[PITROL_AVG_NUM] = {0};
    static float pitRolLastAvg[PITROL_LAST_AVG_NUM] = {0};
    static int escPreActionArry[3] = {0}; //记录脱困前的前三个动作

    static int preTimeCiffAvgF = 0;
    static int preTimeCiffAvgL = 0;
    static int preTimeCiffAvgR = 0;

    static int16_t angVelOutCnt = 0, abnrCurrOutCntL = 0, abnrCurrOutCntR = 0;

    static int act = 0;                //脱困动作状态机
    static long long actStartTime = 0; //自旋开始时间

    static float succesAimforward = 0.0f;

    static int16_t escpCiffSpinCnt = 0;

    static uint8_t saveCiffExitEscpFlag = 0;//记录探地 退出脱困标志

    static float espCTPrePitVal = 999.999f;//记录触发探地前的俯仰角度
    static bumpEscCheck_t bumpEscDet;
    static gyroEscapeCheck_t gyroEscDet;
    //碰撞、探地脱困状态机
    static escapBumpState_t escapeBumpState = ESCAP_BUMP_IDLE;
    /*
    * 碰撞、探地脱困报警标识 
    * 0：无，1碰撞，2探地 3磁条
    */
    static uint8_t fallBumpCliffSign = 0;

    //模式切换结构体
    typedef struct ESCAP_CHECK_STRUCT
    {
        float lastPitRolRcov = 0; //记录前一段时间的俯仰河横滚数据
        int pitRolOutCnt = 0;     //俯仰和横滚超出范围计数
        int pitRolOutOffCnt = 0;  //俯仰和横滚未超出范围计数
    } escapeCheckPara_t;
    escapeCheckPara_t escCheckPara;

    SmallArea smallArea;

    EscapePlan::EscapePlan(/* args */)
    {
    }

    EscapePlan::~EscapePlan()
    {
    }

    void EscapePlan::Init()
    {
        FRIZY_LOG(LOG_DEBUG, "tuokun init");
        trouble.aimPoint.x = 1000, trouble.aimPoint.y = 1000;
        smallArea.smallWall = 0;
        trouble.type = nothing;
        wallPoint.clear();
        wallAgg.clear();
        rollsign = 0;
    }

    Trouble EscapePlan::EscapeRecognition(Sensor sensor, Grid cur)
    {
        printf("wwwccc1.%d.%d.%d\n", sensor.leftw, sensor.rightw, IsWall());

        //   if (IsWall() != 0)
        //   {
        //     printf("step1\n");
        //     stuckTime3 ++;
        //     if (stuckTime3 == STUCK_WALL_TIME)
        //     {
        //       if (abs(cur.x - stuck.x) + abs(cur.y - stuck.y) < 3
        //           && (abs(cur.forward - stuck.forward) < 45 || abs(cur.forward - stuck.forward) > 315))
        //       {
        //         printf("kasi3\n");
        //         stuckTime3 = 0;
        //         trouble.type = stucks;
        //         return trouble;
        //       }

        //       stuck.x = cur.x,stuck.y = cur.y;
        //       stuckTime3 = 0;
        //     }
        //     printf("step2\n");
        //   }
        //   else
        //   {

        //     stuckTime3 = 0;
        //     if (sensor.leftw > 0 && sensor.rightw > 0)
        //     {
        //       stuckTime2 = 0;

        //       stuckTime1 ++;
        //       if (stuckTime1 == STUCK_GO_TIME)
        //       {
        //         if (abs(cur.x - stuck.x) + abs(cur.y - stuck.y) < 3)
        //         {
        //           printf("kasi1\n");
        //           stuckTime1 = 0;
        //           trouble.type = stucks;
        //           return trouble;
        //         }
        //         stuck.x = cur.x,stuck.y = cur.y;
        //         stuckTime1 = 0;
        //       }
        //     }
        //     else if (sensor.leftw * sensor.rightw < 0)
        //     {
        //       stuckTime1 = 0;

        //       stuckTime2 ++;
        //       if (stuckTime2 == STUCK_TURN_TIME)
        //       {
        //         printf("kasi2\n");
        //         stuckTime2 = 0;
        //         trouble.type = stucks;
        //         return trouble;
        //       }

        //     }
        //     else
        //     {
        //       printf("stay\n");
        //       stuckTime1 = 0;
        //       stuckTime2 = 0;
        //     }
        //   }

        //
        trouble.type = nothing;
        if (IsWall() == 2 && wallPoint.size() < 1000 && wallAgg.size() < 1000)
        {

            if (smallArea.smallWall && abs(smallArea.smallPoint.x - cur.x) + abs(smallArea.smallPoint.y - cur.y) > 8)
            {
                FRIZY_LOG(LOG_DEBUG, "huifu");
                smallArea.smallWall = 0;
            }

            FRIZY_LOG(LOG_DEBUG, "step3.%d.%d", wallPoint.size(), wallAgg.size());

            if (wallAgg.size() != 0)
            {

                auto it = wallAgg.end() - 1;
                for (;; it--)
                {
                    if (wallAgg.back().addAngle - it->addAngle > 330)
                    {
                        FRIZY_LOG(LOG_DEBUG, "roll1.%d.%d", wallAgg.back().addAngle, it->addAngle);
                        rollsign = 1;

                        //break;
                    }
                    if (wallAgg.back().addAngle - it->addAngle > 450)
                    {

                        FRIZY_LOG(LOG_DEBUG, "4504501");

                        if (process == BOUND && IsWall() == 2)
                        {

                            int8_t tempro = 1;
                            for (; it < wallAgg.end(); it++)
                            {
                                if (abs(wallAgg.back().x - it->x) + abs(wallAgg.back().y - it->y > 5))
                                {
                                    tempro = 0;
                                    break;
                                }
                            }

                            if (tempro == 1)
                            {
                                Init();
                                smallArea.smallPoint = cur;
                                //smallArea.smallWall = 1;
                                FRIZY_LOG(LOG_DEBUG, "raozhu2");
                                trouble.type = windcolumn;
                                return trouble;
                            }
                        }
                        rollsign = 1;
                        break;
                    }

                    if (wallAgg.back().addAngle - it->addAngle < -360)
                    {
                        FRIZY_LOG(LOG_DEBUG, "rollsign = 2.%d.%d", wallAgg.back().addAngle, it->addAngle);
                        rollsign = 2;
                    }

                    if (wallAgg.back().addAngle - it->addAngle < -450)
                    {

                        FRIZY_LOG(LOG_DEBUG, "4504502 %d %d",it->x,it->y);

                        if (process == BOUND && IsWall() == 2)
                        {

                            int8_t tempro = 1;
                            for (; it < wallAgg.end(); it++)
                            {
                                if (abs(wallAgg.back().x - it->x) + abs(wallAgg.back().y - it->y) > 5)
                                {
                                    tempro = 0;
                                    break;
                                }
                            }

                            if (tempro == 1)
                            {
                                Init();
                                smallArea.smallPoint = cur;
                                //smallArea.smallWall = 1;
                                FRIZY_LOG(LOG_DEBUG, "xiaofanwei2");
                                trouble.type = smallarea;
                                return trouble;
                            }
                        }
                        rollsign = 2;
                        break;
                    }

                    if (it == wallAgg.begin())
                        break;
                }
            }

            if (wallPoint.size() == 0 || cur.x != wallPoint.back().x || cur.y != wallPoint.back().y)
            {
                FRIZY_LOG(LOG_DEBUG, "step4.%d.%d.%d.%d", wallPoint.size(), wallAgg.size(), trouble.aimPoint.x, trouble.aimPoint.y);
                if (abs(trouble.aimPoint.x - cur.x) + abs(trouble.aimPoint.y - cur.y) < sensor.size)
                {
                    FRIZY_LOG(LOG_DEBUG, "maybe.%d", rollsign);

                    if (rollsign == 1 && trouble.rollArray.size() > 3)
                    {
                        FRIZY_LOG(LOG_DEBUG, "raozhu1");
                        trouble.type = windcolumn;
                        return trouble;
                    }
                    else if (rollsign == 2)
                    {
                        if (process != BOUND)
                        {
                            Init();
                            smallArea.smallPoint = cur;
                            //smallArea.smallWall = 1;
                            FRIZY_LOG(LOG_DEBUG, "xiaofanwei1");
                            trouble.type = smallarea;
                            return trouble;
                        }
                        else
                        {
                            FRIZY_LOG(LOG_DEBUG, "dafanwei1");
                        }
                    }
                }
                else
                {
                    int tempN = wallPoint.size() - sensor.size;
                    for (int i = 0; i < tempN; i++)
                    {
                        //
                        if (abs(wallPoint[i].x - cur.x) + abs(wallPoint[i].y - cur.y) < 1)
                        {
                            trouble.rollArray.clear();
                            trouble._maprange.xmax = -1000;
                            trouble._maprange.xmin = 1000;
                            trouble._maprange.ymax = -1000;
                            trouble._maprange.ymin = 1000;
                            auto it = wallPoint.end() - 1;
                            for (;; it--)
                            {
                                if (it->x < trouble._maprange.xmin)
                                    trouble._maprange.xmin = it->x;
                                if (it->x > trouble._maprange.xmax)
                                    trouble._maprange.xmax = it->x;
                                if (it->y < trouble._maprange.ymin)
                                    trouble._maprange.ymin = it->y;
                                if (it->y > trouble._maprange.ymax)
                                    trouble._maprange.ymax = it->y;
                                FRIZY_LOG(LOG_DEBUG, "yiquan.%d.%d", it->x, it->y);
                                trouble.rollArray.push_back(*it);
                                if (cur.x == it->x && cur.y == it->y)
                                    break;
                            }

                            FRIZY_LOG(LOG_DEBUG, "aimPoint.%d.%d", wallPoint[i + sensor.size].x, wallPoint[i + sensor.size].y);
                            trouble.aimPoint = wallPoint[i + sensor.size];
                            break;
                        }
                        else
                        {
                        }
                    }
                }

                FRIZY_LOG(LOG_DEBUG, "input1.%d.%d", cur.x, cur.y);
                wallPoint.push_back(cur);
            }

            if (wallAgg.size() == 0 || abs(cur.addAngle - wallAgg.back().addAngle) > 30)
            {
                FRIZY_LOG(LOG_DEBUG, "input2.%d", cur.addAngle);

                wallAgg.push_back(cur);
            }
        }
        else
        {
            if (wallPoint.size() || wallAgg.size())
            {
                FRIZY_LOG(LOG_DEBUG, "clear all");
                smallArea.smallWall = 0;
                trouble.aimPoint.x = 1000, trouble.aimPoint.y = 1000;

                wallPoint.clear();
                wallAgg.clear();
            }
            FRIZY_LOG(LOG_DEBUG, "nothing");
            trouble.type = nothing;
            rollsign = 0;
        }

        return trouble;
    }

    /**************************************分割线************************************************/
    //获取当前时间  单位ms
    long long getTime()
    {
        struct timeval t;
        gettimeofday(&t, NULL);
        return (long long)t.tv_sec * 1000 + (long long)t.tv_usec / 1000;
    }

    //获取两个轮子状态
    wheelState_t EscapePlan::getWheelControlState()
    {
        //0:停止 1:前进 2:后退
        wheelState_t tmp_wheelsta;
        chassisEscape.GetSensor(&escapSensor);
        if (escapSensor.leftw < 0)
            tmp_wheelsta.wheelLeftState = 2;
        else if (escapSensor.leftw == 0)
            tmp_wheelsta.wheelLeftState = 0;
        else
            tmp_wheelsta.wheelLeftState = 1;
        if (escapSensor.rightw < 0)
            tmp_wheelsta.wheelRightState = 2;
        else if (escapSensor.rightw == 0)
            tmp_wheelsta.wheelRightState = 0;
        else
            tmp_wheelsta.wheelRightState = 1;
        tmp_wheelsta.wheelState = chassisEscape.getWheelState();
        FRIZY_LOG(LOG_DEBUG, "tmp_wheelsta.wheelState:%d", tmp_wheelsta.wheelState);
        return tmp_wheelsta;
    }

    //获取俯仰横滚值 num 0:俯仰    1:横滚
    float EscapePlan::getPitRol(int num)
    {
        chassisEscape.GetSensor(&escapSensor);
        if (num == 0)
            return escapSensor.XAngle;
        else if (num == 1)
            return escapSensor.YAngle;
        else
            return 0;
    }

    //获取加速度
    int EscapePlan::getAcc(int num)
    {
        chassisEscape.GetSensor(&escapSensor);
        if (num == 0)
            return escapSensor.XAcc;
        else if (num == 1)
            return escapSensor.YAcc;
        else if (num == 2)
            return escapSensor.ZAcc;
        else
            return 0;
    }

    //获取角速度
    int EscapePlan::getAngleRate()
    {
        chassisEscape.GetSensor(&escapSensor);
        return escapSensor.zGyroOriginal;
    }

    //获取累加角度
    int EscapePlan::getAddAngle()
    {
        // FRIZY_LOG(LOG_DEBUG, "(360-gyo_angle_*180/_Pi)*10 = %d ",(360-gyo_angle_*180/_Pi)*10);
        chassisEscape.GetSensor(&escapSensor);
        return escapSensor.addAngle;

        // chassisEscape.GridPoint(&escapGrid);
        // return escapGrid.forward*10;
    }

    /*
    *   数组入队 (滑动存入一定的数据)
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    void SlidingArrayAddF(float value_buf[], int n, float ADNum)
    {
        for (int i = 0; i < n - 1; i++)
        {
            memcpy(&value_buf[i], &value_buf[i + 1], sizeof(value_buf[i]));
        }
        value_buf[n - 1] = ADNum;
    }

    /*
    *   数组入队 (滑动存入一定的数据)
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    void SlidingArrayAddI(int value_buf[], int n, int ADNum)
    {
        for (int i = 0; i < n - 1; i++)
        {
            memcpy(&value_buf[i], &value_buf[i + 1], sizeof(value_buf[i]));
        }
        value_buf[n - 1] = ADNum;
    }

    //滑动平均滤波算法（递推平均滤波法）

    /*
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    int GlideFilterAD(int value_buf[], int n, int ADNum)
    {
        int sum = 0;
        SlidingArrayAddI(value_buf, n, ADNum);

        for (int count = 0; count < n; count++)
            sum += value_buf[count];
        return (int)(sum / n);
    }

    /*
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    float GlideFilterADf(float value_buf[], int n, float ADNum)
    {
        float sum = 0.0f;
        SlidingArrayAddF(value_buf, n, ADNum);
        for (int count = 0; count < n; count++)
            sum += value_buf[count];
        return (float)(sum / n);
    }

    //加速度滑动平均值
    int EscapePlan::gyroAccSlidingAvg()
    {
        static long long lastCycTimeGyr = 0;
        static int lastFiltVal = 0;

        if (getTime() - lastCycTimeGyr >= 20) //防止一个周期内被多次调用,导致相同数据,被重复计算
        {
            int accY = getAcc(1);
            lastFiltVal = GlideFilterAD(gyroAccBuf, STUCK_ACC_ARRY_NUM, accY) * 10;
            lastCycTimeGyr = getTime();
        }

        return lastFiltVal; //除以1024
    }

    /*
    * 俯仰和横滚滑动平均值
    * 机器   pit 前面抬起 负数 后面抬起 正数
    *        rol 右边抬起 负数 左边抬起 正数
    */
    float EscapePlan::gyroPitRolSlidingAvg()
    {
        static long long lastCycTime = 0;
        static float lastFiltVal = 0.0f;

        if (getTime() - lastCycTime >= 20) //防止一个周期内被多次调用,导致相同数据,被重复计算
        {
            chassisEscape.GetSensor(&escapSensor);
            float anglePitEsc = escapSensor.XAngle; //俯仰
            float angleRolEsc = escapSensor.YAngle; //横滚
            float pitRolSqrtf = 0;
            pitRolSqrtf = sqrtf(anglePitEsc * anglePitEsc + angleRolEsc * angleRolEsc);
            // FRIZY_LOG(LOG_INFO,"gyroPitRolSlidingAvg Pit:%f,Rol:%f, PitRol:%f",anglePitEsc, angleRolEsc, pitRolSqrtf);

            lastFiltVal = GlideFilterADf(pitRolAvgArry, PITROL_AVG_NUM, pitRolSqrtf);
            lastCycTime = getTime();
        }

        return lastFiltVal;
    }

    void EscpIdentPR_Init()
    {
        memset(&espIdentPR, 0, sizeof(espIdentPitRol_t));
    }

    /*
    * 针对机器在沿墙时吧台椅机器无法前进的识别
    * 只针对沿墙模式识别
    */
    bool EscapePlan::EscpBarChairDete(void)
    {
        // #if (0 == ESCP_BAR_CHAIR_EN)
        //     return false;
        // #endif

        static int espWheelLast = 0;
        static int espOffCnt = 0;
        //    static int16 espIdentPROoutCnt = 0;
        static int espIdentPROutCnt = 0;
        static int espFixActSum = 0;
        if (IsWall() == EXIT_WALL) //这个识别只在沿墙中有效
        {
            EscpIdentPR_Init();
            espFixActSum = 0;
            return false;
        }
        //趋势识别
        EscpIdentPitRolTrends();
        // FRIZY_LOG(LOG_DEBUG, "wheelState:%d", chassisEscape.getWheelState());
        if (chassisEscape.getWheelState() == WHEELFIXSPEED)
        {
            espFixActSum++;
        }
        if (espWheelLast != chassisEscape.getWheelState())
        {
            if (espWheelLast == WHEELFIXSPEED)
            {
                float upWidTmp = 0.0f;
                int ptgeTmp = 0;

                //上升幅度
                upWidTmp = espIdentPR.endUpPitRol - espIdentPR.startUpPitRol;
                // FRIZY_LOG(LOG_DEBUG, "end=%0.1f start=%0.1f %d", espIdentPR.endUpPitRol, espIdentPR.startUpPitRol, espIdentPROutCnt);
                if (espFixActSum != 0)
                {
                    //上升时间占动作的总时间的百分比
                    ptgeTmp = (100 * espIdentPR.espUpIdentCnt / espFixActSum);
                }
                // FRIZY_LOG(LOG_DEBUG, "upWidTmp:%f", upWidTmp);
                if (upWidTmp > 3.5f) //上升幅度大于3.5度
                {
                    // FRIZY_LOG(LOG_DEBUG, "ptgeTmp:%d", ptgeTmp);
                    if (ptgeTmp > 50 /*75*/) //上升百分比大于50%
                    {
                        espOffCnt = 0;
                        espIdentPROutCnt++;
                        FRIZY_LOG(LOG_DEBUG, "espIdentPROutCnt:%d", espIdentPROutCnt);
                        if (espIdentPROutCnt > 5)
                        {
                            printf("xxxxEscpIdentxxxx\n");
                            espIdentPROutCnt = 0;
                            stuckType = ESCAPE_EVENT_TYPE_LEFT_OFF;
                            return true;
                        }
                    }
                    else
                    {
                        if (espIdentPROutCnt > 0)
                            espIdentPROutCnt--;
                        // printf("inavild %0.2f %d\n\n", upWidTmp, ptgeTmp);
                    }
                }
                else
                {
                    if (ptgeTmp < 40 || upWidTmp < 0.5f) //上升百分比小于40%或者上升幅度小于0.5f三次则清除计数
                    {
                        if (espIdentPROutCnt > 0)
                            espIdentPROutCnt--;
                        espOffCnt++;
                        if (espOffCnt > 3)
                        {
                            espOffCnt = 0;
                            espIdentPROutCnt = 0;
                            // printf("cleanOutCnt\n");
                        }
                    }
                }

                // printf("actEsp w=%d Up[%0.1f %0.1f {%d %d %d}(%0.1f)]%d\n", espWheelLast, espIdentPR.startUpPitRol, espIdentPR.endUpPitRol, espIdentPR.espUpIdentCnt, espFixActSum, ptgeTmp, espIdentPR.endUpPitRol - espIdentPR.startUpPitRol, espIdentPROutCnt);
            }
            //        printf("espFixAct %d %d\n",espFixActSum,espIdentPR.espUpIdentCnt);
            EscpIdentPR_Init();
            espFixActSum = 0;
        }

        espWheelLast = chassisEscape.getWheelState();
        return false;
    }

    // 处理的俯仰和横滚值
    float EscapePlan::escpPitRolIdentGet()
    {
        static float pitRolSqrtf = 0.0f;

        float anglePitEsc = getPitRol(0); //俯仰
        float angleRolEsc = getPitRol(1); //横滚

        pitRolSqrtf = sqrtf((anglePitEsc * anglePitEsc) + (angleRolEsc * angleRolEsc)) * 0.4f + pitRolSqrtf * 0.6f;

        return pitRolSqrtf;
    }

    /*
    * 识别俯仰横滚的趋势
    */
    void EscapePlan::EscpIdentPitRolTrends()
    {
        float pitRolEscTmp = 0.0f;
        chassisEscape.GetSensor(&escapSensor);
        pitRolEscTmp = escpPitRolIdentGet();
        //识别上升趋势
        if ((pitRolEscTmp - espIdentPR.lastPitRol) > 0.0f)
        {
            if (espIdentPR.dwOpenCnt > 0)
                espIdentPR.dwOpenCnt--;
            if (espIdentPR.upOpenCnt > 2)
            {
                espIdentPR.dwPRFlag = false;
                if (!espIdentPR.upPRFlag)
                {
                    espIdentPR.upPRFlag = true;
                    // FRIZY_LOG(LOG_DEBUG, "startUpPitRol=%0.2f", pitRolEscTmp);
                    espIdentPR.startUpPitRol = pitRolEscTmp;
                }
                espIdentPR.dwOpenCnt = 0;
                espIdentPR.espDwIdentCnt = 0;
                espIdentPR.espUpIdentCnt++; //上升趋势计数
                espIdentPR.startDwPitRol = 0;
                //            espIdentPR.endDwPitRol = 0;
                //            espIdentPR.identPRDwCnt = 0;
                espIdentPR.endUpPitRol = pitRolEscTmp;
            }
            else
            {
                espIdentPR.upOpenCnt++;
            }
        }
        else
        {
            float tmpDiff = 0;

            tmpDiff = espIdentPR.endUpPitRol - pitRolEscTmp;

            //顶墙时，幅度会下降一点，这里把这里滤掉
            if (tmpDiff > 0)
            {
                if (tmpDiff < 1.0f)
                {
                    espIdentPR.espUpIdentCnt++;
                }
                else if (tmpDiff > 5.0f)
                {
                    // FRIZY_LOG(LOG_DEBUG, "espUpIdentCnt clean");
                    espIdentPR.espUpIdentCnt = 0;
                }
            }
        }
#if 0
        //识别下降趋势
        if((espIdentPR.lastPitRol-pitRolEscTmp)>0.0f)
        {
            if(espIdentPR.upOpenCnt>0)
                espIdentPR.upOpenCnt--;
            if(espIdentPR.dwOpenCnt>2)
            {
                espIdentPR.upPRFlag = FALSE;
                
                if(!espIdentPR.dwPRFlag)
                {
                    espIdentPR.dwPRFlag = TRUE;
                    if(espIdentPR.espUpIdentCnt>25)
                    {
                        printf("EspPRUp[%0.2f->%0.2f]%d\n",espIdentPR.startUpPitRol,pitRolEscTmp,espIdentPR.espUpIdentCnt);
                    }
                    espIdentPR.startDwPitRol = pitRolEscTmp;
                }
    //            espIdentPR.upOpenCnt = 0;
    //            espIdentPR.espUpIdentCnt = 0;//上升降趋势计数清零
                espIdentPR.espDwIdentCnt++;//下降趋势计数
    //            espIdentPR.startUpPitRol = 0;
    //            espIdentPR.endUpPitRol = 0;
    //            espIdentPR.identPRUpCnt = 0;
                espIdentPR.endDwPitRol = pitRolEscTmp;
            }
            else
            {
                espIdentPR.dwOpenCnt++;
            }
        }
        else
        {
            
        }
#endif
        //    printf("lSN %0.1f %0.1f [%d %d] [%d %d]\n",espIdentPR.lastPitRol,pitRolEscTmp
        //    ,espIdentPR.upPRFlag,espIdentPR.espUpIdentCnt
        //    ,espIdentPR.dwPRFlag,espIdentPR.espDwIdentCnt);

        espIdentPR.lastPitRol = pitRolEscTmp;
    }

    //机器卡死判定      return true 卡死  false 正常
    bool EscapePlan::ESCAPE_StuckTrigCheck_My_Test()
    {
        // FRIZY_LOG(LOG_INFO, "机器卡死判定");
        uint8_t stuckRet = 0;
        static int8_t lastescWheelLSta = 0;
        static int8_t lastescWheelRSta = 0;
        static int8_t PitRolCheckCnt = 0;

        wheelState_t wheelSta_tmp;
        wheelSta_tmp = getWheelControlState();
        // FRIZY_LOG(LOG_DEBUG,"10SPit:%f,10SRol:%f,10SPitRol:%f",escActPara.escBasePit, escActPara.escBaseRol, escActPara.escBasePitRol);
        // FRIZY_LOG(LOG_DEBUG,"wheelstate:%d, leftwheelstate:%d, rightwheelstate:%d",wheelSta_tmp.wheelState,wheelSta_tmp.wheelLeftState,wheelSta_tmp.wheelRightState);
        if (lastescWheelLSta != wheelSta_tmp.wheelLeftState ||
            lastescWheelRSta != wheelSta_tmp.wheelRightState)
        {
            PitRolCheckCnt = 0;
            stuckRet &= ~CUR_ABNORMAL;    //电流异常标识
            stuckRet &= ~PITROL_ABNORMAL; //俯仰异常标识
        }
        lastescWheelLSta = wheelSta_tmp.wheelLeftState;
        lastescWheelRSta = wheelSta_tmp.wheelRightState;

        //电流检测
        if (abnormalElectricCheck())
        {
            FRIZY_LOG(LOG_INFO, "cur check:abnormal");
            stuckRet |= CUR_ABNORMAL;
        }
        else
        {
            // FRIZY_LOG(LOG_DEBUG,"cur check:normal");
            stuckRet &= ~CUR_ABNORMAL;
        }
        //俯仰和横滚检测
        if (abnormalPitRolCheck())
        {
            FRIZY_LOG(LOG_INFO, "pitrol check:abnormal");
            stuckRet |= PITROL_ABNORMAL;
            PitRolCheckCnt = 500 / 20;
        }
        else
        {
            // FRIZY_LOG(LOG_DEBUG,"pitrol check:normal");
            if (PitRolCheckCnt > 0)
            {
                PitRolCheckCnt--;
                stuckRet &= ~PITROL_ABNORMAL;
            }
        }

        if ((stuckRet & CUR_ABNORMAL) && (stuckRet & PITROL_ABNORMAL)) //俯仰横滚和电流同时异常判定为困住
        {

            angVelOutCnt = 0;
            abnrCurrOutCntL = 0;
            abnrCurrOutCntR = 0;
            // printf("esc xxx myTest stuck[%d]\n",stuckRet);
            stuckType = ESCAPE_EVENT_TYPE_STUCK;
            return true;
        }
        return false;
    }

    static int addAngleCnt = 0;
    //架空离地判定初始化
    void EscapePlan::ESCAPE_LeftOffCheckInit()
    {
        chassisEscape.GridPoint(&escapGrid);
        LeftOffDistance = 0;
        LeftOffStartAngle = getAddAngle() / 10;
        LeftOffStartX = escapGrid.realx * 15 / 100;
        LeftOffStartY = escapGrid.realy * 15 / 100;
        addAngleCnt = 0;
    }

    //架空检测
    bool EscapePlan::ESCAPE_LeftFloorCheck()
    {
        int checkFlag = 0;
        int retTmp = 0;
        static int retOverheadSta = 0;
        float pitRolEscTmp = 0.0f;
        pitRolEscTmp = gyroPitRolSlidingAvg();
        recordPitRol_10s(pitRolEscTmp);
        // FRIZY_LOG(LOG_DEBUG, "pitRolEscTmp:%f, pitRolLastAvg[0]:%f", pitRolEscTmp, pitRolLastAvg[0]);
        // FRIZY_LOG(LOG_DEBUG, "pitRolEscTmp - pitRolLastAvg[0]:%f", pitRolEscTmp - pitRolLastAvg[0]);
        if (fabsf(pitRolEscTmp - pitRolLastAvg[0]) >= 3.5f)
        {
            if (0 == escCheckPara.lastPitRolRcov)
            {
                escCheckPara.lastPitRolRcov = pitRolLastAvg[0];
                FRIZY_LOG(LOG_DEBUG, "lastPitRolRcov=%0.2f", escCheckPara.lastPitRolRcov);
            }
        }
        if (fabsf(pitRolEscTmp - escCheckPara.lastPitRolRcov) >= 3.5f && (getMEMSRelatAngle(0) < -5.0f))
        {
            escCheckPara.pitRolOutCnt++;
            // FRIZY_LOG(LOG_DEBUG, "escCheckPara.pitRolOutCnt:%d", escCheckPara.pitRolOutCnt);
            if (escCheckPara.pitRolOutCnt > 60 /*3000/20*/)
            {
                escCheckPara.lastPitRolRcov = 0;
                escCheckPara.pitRolOutCnt = 0;
                FRIZY_LOG(LOG_DEBUG, "xxx esc Overhead");
                retOverheadSta |= 0x02;
            }
        }
        else
        {
            if ((0 != preTimeCiffAvgF || 0 != preTimeCiffAvgL || 0 != preTimeCiffAvgR) && ((++escCheckPara.pitRolOutOffCnt) > 60 /*3000/20*/))
            {
                escCheckPara.pitRolOutOffCnt = 0;
                preTimeCiffAvgF = 0;
                preTimeCiffAvgL = 0;
                preTimeCiffAvgR = 0;
                FRIZY_LOG(LOG_DEBUG, "retOverheadSta &=~0x02");
            }
            escCheckPara.pitRolOutCnt = 0;
            escCheckPara.lastPitRolRcov = 0;
            retOverheadSta &= ~0x02;
        }
        SlidingArrayAddF(pitRolLastAvg, PITROL_LAST_AVG_NUM, pitRolEscTmp);

        retTmp = groundAssistCkeck();

        if (retTmp)
        {
            retOverheadSta |= retTmp;
        }
        else
        {
            retOverheadSta &= ~0x01;
        }

        if (retOverheadSta == 0x03)
        {
            retOverheadSta = 0x0;
            // FRIZY_LOG(LOG_DEBUG, "overHead trig esccap");
            stuckType = ESCAPE_EVENT_TYPE_LEFT_OFF;
            return true;
        }
        return false;
    }

/**** 架空脱困识别 start ********/
#define CIFF_ARRY_NUM (10)
#define LAST_CIFF_ARRY_NUM (4)
    static int ciffFEscBuf[CIFF_ARRY_NUM] = {0};
    static int ciffLEscBuf[CIFF_ARRY_NUM] = {0};
    static int ciffREscBuf[CIFF_ARRY_NUM] = {0};

    static int ciffLastFEscBuf[LAST_CIFF_ARRY_NUM] = {0};
    static int ciffLastLEscBuf[LAST_CIFF_ARRY_NUM] = {0};
    static int ciffLastREscBuf[LAST_CIFF_ARRY_NUM] = {0};
    static int ciffSlidingAvgCnt = 0; //记录数组储存计数

    static void GroundAssistCkeckInit(void)
    {
        ciffSlidingAvgCnt = 0; //记录数组储存计数
    }
    int EscapePlan::groundAssistCkeck()
    {
        static int outCiffFCnt = 0, outCiffLCnt = 0, outCiffRCnt = 0, ciffDiffCnt = 0;
        static long long lastCiffTime = 0;
        chassisEscape.GetSensor(&escapSensor);
        int ciffFSlidAvgTmp = 0;
        int ciffLSlidAvgTmp = 0;
        int ciffRSlidAvgTmp = 0;
        if (getTime() - lastCiffTime >= 60)
        {
            lastCiffTime = getTime();
            ciffFSlidAvgTmp = GlideFilterAD(ciffFEscBuf, CIFF_ARRY_NUM, escapSensor.midRightCliffValue); //600ms数据
            ciffLSlidAvgTmp = GlideFilterAD(ciffLEscBuf, CIFF_ARRY_NUM, escapSensor.leftCliffValue);
            ciffRSlidAvgTmp = GlideFilterAD(ciffREscBuf, CIFF_ARRY_NUM, escapSensor.rightCliffValue);
            SlidingArrayAddI(ciffLastFEscBuf, LAST_CIFF_ARRY_NUM, ciffFSlidAvgTmp); //2.4s滑动数据记录
            SlidingArrayAddI(ciffLastLEscBuf, LAST_CIFF_ARRY_NUM, ciffLSlidAvgTmp);
            SlidingArrayAddI(ciffLastREscBuf, LAST_CIFF_ARRY_NUM, ciffRSlidAvgTmp);
            //填充满数值之后再进行判定
            if (ciffSlidingAvgCnt <= LAST_CIFF_ARRY_NUM * CIFF_ARRY_NUM)
            {
                ciffSlidingAvgCnt++;
                ciffDiffCnt = 0;
                outCiffFCnt = 0;
                outCiffLCnt = 0;
                outCiffRCnt = 0;
                preTimeCiffAvgF = 0;
                preTimeCiffAvgL = 0;
                preTimeCiffAvgR = 0;
                escActPara.groundPeneDiff = 0;
                return 0x0;
            }
            if (abs(ciffFSlidAvgTmp - ciffLSlidAvgTmp) < 50 && abs(ciffFSlidAvgTmp - ciffRSlidAvgTmp) < 50)
            {
                escActPara.groundPeneDiff = 1;
            }
            else
            {
                escActPara.groundPeneDiff = 0;
            }
            //当前的探地值与2.4s前的探地值比较
            if (abs(ciffLastFEscBuf[0] - ciffFSlidAvgTmp) >= 600)
            {
                if (0 == preTimeCiffAvgF)
                {
                    preTimeCiffAvgF = ciffLastFEscBuf[0];
                    FRIZY_LOG(LOG_DEBUG, "preTimeCiffAvgF=%d,%d", preTimeCiffAvgF, ciffFSlidAvgTmp);
                }
            }
            if (abs(ciffLastLEscBuf[0] - ciffLSlidAvgTmp) >= 600)
            {
                if (0 == preTimeCiffAvgL)
                {
                    preTimeCiffAvgL = ciffLastLEscBuf[0];
                    FRIZY_LOG(LOG_DEBUG, "preTimeCiffAvgL=%d %d", preTimeCiffAvgL, ciffLSlidAvgTmp);
                }
            }

            if (abs(ciffLastREscBuf[0] - ciffRSlidAvgTmp) >= 600)
            {
                if (0 == preTimeCiffAvgR)
                {
                    preTimeCiffAvgR = ciffLastREscBuf[0];
                    FRIZY_LOG(LOG_DEBUG, "preTimeCiffAvgR=%d %d", preTimeCiffAvgR, preTimeCiffAvgR);
                }
            }

            //超出第一级阈值之后，持续判断后续是否超出
            if (0 != preTimeCiffAvgF && abs(preTimeCiffAvgF - ciffFSlidAvgTmp) >= 500)
            {
                outCiffFCnt++;
                // FRIZY_LOG(LOG_DEBUG, "ESCPDiff F=%d", abs(preTimeCiffAvgF - ciffFSlidAvgTmp));
            }
            else
            {
                if (outCiffFCnt > 0)
                    outCiffFCnt--;
                else
                    preTimeCiffAvgF = 0;
            }

            if (0 != preTimeCiffAvgL && abs(preTimeCiffAvgL - ciffLSlidAvgTmp) >= 500)
            {
                outCiffLCnt++;
                // FRIZY_LOG(LOG_DEBUG, "ESCPDiff L=%d", abs(preTimeCiffAvgL - ciffLSlidAvgTmp));
            }
            else
            {
                if (outCiffLCnt > 0)
                    outCiffLCnt--;
                else
                    preTimeCiffAvgL = 0;
            }

            if (0 != preTimeCiffAvgR && abs(preTimeCiffAvgR - ciffRSlidAvgTmp) >= 500)
            {
                outCiffRCnt++;
                // FRIZY_LOG(LOG_DEBUG, "ESCPDiff R=%d", abs(preTimeCiffAvgR - ciffRSlidAvgTmp));
            }
            else
            {
                if (outCiffRCnt > 0)
                    outCiffRCnt--;
                else
                    preTimeCiffAvgR = 0;
            }

            if (outCiffFCnt > 4 || outCiffLCnt > 4 || outCiffRCnt > 4)
            {
                //中间的探地与两边的探地右一定的差值
                if (abs(ciffFSlidAvgTmp - ciffLSlidAvgTmp) >= 500 || abs(ciffFSlidAvgTmp - ciffRSlidAvgTmp) >= 500 || abs(ciffLSlidAvgTmp - ciffRSlidAvgTmp) >= 500)
                {
                    FRIZY_LOG(LOG_DEBUG, "FL=%d,FR=%d,LR=%d %d %d %d %d", abs(ciffFSlidAvgTmp - ciffLSlidAvgTmp), abs(ciffFSlidAvgTmp - ciffRSlidAvgTmp), abs(ciffLSlidAvgTmp - ciffRSlidAvgTmp), ciffDiffCnt, outCiffFCnt, outCiffLCnt, outCiffRCnt);
                    ciffDiffCnt++;
                    if (ciffDiffCnt > 50 && (outCiffFCnt > 45 || outCiffLCnt > 45 || outCiffRCnt > 45))
                    {
                        ciffDiffCnt = 0;
                        outCiffFCnt = 0;
                        outCiffLCnt = 0;
                        outCiffRCnt = 0;
                        preTimeCiffAvgF = 0;
                        preTimeCiffAvgL = 0;
                        preTimeCiffAvgR = 0;
                        FRIZY_LOG(LOG_DEBUG, "esc ciffDiffCnt 0x01");
                        return 0x1;
                    }
                }
                else
                {
                    if (ciffDiffCnt > 0)
                        ciffDiffCnt--;
                    // if(outCiffFCnt>0)
                    //     outCiffFCnt--;

                    // if(outCiffLCnt>0)
                    //     outCiffLCnt--;

                    // if(outCiffRCnt>0)
                    //     outCiffRCnt--;
                }
            }
            else
            {
                ciffDiffCnt = 0;
            }
        }
        return 0x0;
    }

    bool EscapePlan::cliffCheck()
    {
        static int8_t lastDetcbehdSta = 0;
        static int16_t offCtMagCnt = 0;
        static int16_t ciffTrigPreCnt = 0;
        static int16_t alertDelayCnt = 0;
        static int32_t startBehDis = 0;
        
        int32_t backDistan = 0;
        int32_t currDistan = 0;
        
        uint16_t retTrig = 0;
        int8_t bPcTtmp = 0;

        static long long lastSysTemTim = 0;
        long long timeDiff = getTime() - lastSysTemTim;
        if(timeDiff < 100)
            return false;
        // FRIZY_LOG(LOG_DEBUG, "long long timeDiff = %lld %lld %lld", getTime(), lastSysTemTim, timeDiff);
        
        static float roughlyDisTmp = 0.0f;
        int aimSpdL = chassisEscape.getSendSpeed().first;
        int aimSpdR = chassisEscape.getSendSpeed().second;

        int aimLineVel = (aimSpdL + aimSpdR) / 2;                      //目标线速度
        int fdblineVel = (escapSensor.rightw + escapSensor.leftw) / 2; //反馈线速度  mm/s
        //大致的位移
        roughlyDisTmp += fabs(fdblineVel * timeDiff / 1000.0); //mm
        lastSysTemTim = getTime();

        // FRIZY_LOG(LOG_DEBUG, "lr aim spd  [%d,%d] [%d,%d]", aimSpdL, aimSpdR, escapSensor.leftw, escapSensor.rightw);
        // FRIZY_LOG(LOG_DEBUG, "aimLineVel fdblineVel DisTmp %d %d %f", aimLineVel, fdblineVel, roughlyDisTmp);
        if(lastDetcbehdSta != bumpEscDet.detcbehindState)
        {
            FRIZY_LOG(LOG_DEBUG, "detBehSta[%d->%d]",lastDetcbehdSta,bumpEscDet.detcbehindState);
            lastDetcbehdSta = bumpEscDet.detcbehindState;
        }
        switch(bumpEscDet.detcbehindState)
        {
            case DETC_BEHIND_IDLE:
                //三个探地同时触发，在2.5s内不进行探地脱困
                if(escapSensor.mcuLeftCliff && escapSensor.mcuRightCliff &&
                    escapSensor.mcuLeftMidCliff && escapSensor.mcuRightMidCliff)
                {
                    if(++bumpEscDet.ciffAllCnt < 2500/50)
                    {
                        break;
                    }
                }
                else
                {
                    if(bumpEscDet.ciffAllCnt > 3)
                    {
                        bumpEscDet.ciffAllCnt-=3;
                    }
                    if(bumpEscDet.ciffAllOnCnt > 500/50)
                    {
                        bumpEscDet.ciffAllCnt = 0;
                        bumpEscDet.ciffAllOnCnt = 0;
                    }
                }
                
                if(escapSensor.cliff)
                {
                    bumpEscDet.detcbehindState = DETC_BEHIND_ATC;
                    //左右两边触发时只后退半个机身
                    if(escapSensor.mcuLeftCliff || escapSensor.leftCliff)
                    {
                        bumpEscDet.behindEndF = 1;
                    }
                    else if(escapSensor.mcuRightCliff || escapSensor.rightCliff)
                    {
                        bumpEscDet.behindEndF = 2;
                    }
                    else
                    {
                        bumpEscDet.behindEndF = 0;
                    }
                    FRIZY_LOG(LOG_DEBUG, "atcSta %d %d", escapSensor.cliff, bumpEscDet.behindEndF);
                }
                else
                {
                    escapeBumpState = ESCAP_BUMP_IDLE;
                    bumpEscDet.behindCount = 0;
                    // escapeFrontCount = 0;
                    fallBumpCliffSign = 0;
                    if(bumpEscDet.ciffAllCnt > 0)
                        bumpEscDet.ciffAllOnCnt++;
                    if(espCTPrePitVal>=999.998f)
                    {
                        espCTPrePitVal = getPitRol(0);
                    }
                    if(!escapSensor.cliff)
                    {
                        if(ciffTrigPreCnt>2000/20)
                        {
                            ciffTrigPreCnt = 0;
                            espCTPrePitVal = getPitRol(0);
                        }
                        else
                        {
                            ciffTrigPreCnt++;
                        }
                    }
                }
            break;

            case DETC_BEHIND_ATC: //1 后退动作检测
                if(chassisEscape.getWheelState() == WHEELBEHIND)
                {
                    //记录后退前的距离与时间
                    bumpEscDet.behindpreTime = getTime();
                    bumpEscDet.behindpreDistan = roughlyDisTmp;
                    bumpEscDet.behindCount++;
                    FRIZY_LOG(LOG_DEBUG, "behindCount.%d %lld %d", bumpEscDet.behindCount,bumpEscDet.behindpreTime,bumpEscDet.behindpreDistan);
                    bumpEscDet.detcbehindState = DETC_BEHIND_STOP;
                }
                else if(chassisEscape.getWheelState() != WHEELSTOP)//不是后退动作则清零
                {
                    bumpEscDet.behindCount = 0;
                    bumpEscDet.detcbehindState = DETC_BEHIND_IDLE;
                    FRIZY_LOG(LOG_DEBUG, "idleSta %d", escapSensor.cliff);
                }
            break;

            case DETC_BEHIND_STOP: //2 等待一次后退的停止
                if(chassisEscape.getWheelState() != WHEELBEHIND)
                {
                    if(!escapSensor.cliff || chassisEscape.getWheelState() != WHEELSTOP)
                    {
                        bumpEscDet.behindpreTime = getTime();
                        bumpEscDet.behindpreDistan = roughlyDisTmp;
                    }
                    if(chassisEscape.getWheelState() != WHEELSTOP)//不是执行后退动作时，重新开始检测
                    {
                        bumpEscDet.detcbehindState = DETC_BEHIND_IDLE;
                        break;
                    }
                }
                long long behindTimeTmp = 0;
                currDistan = roughlyDisTmp;
                behindTimeTmp = getTime() - bumpEscDet.behindpreTime;
                FRIZY_LOG(LOG_DEBUG, "behindTimeTmp %d", behindTimeTmp);
                //计算后退的距离
                backDistan = abs(bumpEscDet.behindpreDistan - currDistan);
                FRIZY_LOG(LOG_DEBUG, "behindEndF %d %d %d %d", bumpEscDet.behindEndF, bumpEscDet.behindpreDistan, currDistan, backDistan);
                if(bumpEscDet.behindEndF)//左/右触发探地
                {
                    
                    //后退超过半个机身，进入360旋转
                    if(behindTimeTmp>3*1000||backDistan >= 150)//触发探地脱困
                    {
                        FRIZY_LOG(LOG_DEBUG, "bPcTtmp %lld %d", behindTimeTmp, backDistan);
                        bPcTtmp = 1;
                    }
                }
                else//前面触发探地
                {
                    //后退超过3s
                    if(behindTimeTmp>3*1000||backDistan >= 200)
                    {
                        FRIZY_LOG(LOG_DEBUG, "bPcTtmp %lld %d", behindTimeTmp, backDistan);
                        bPcTtmp = 1;
                    }
                }
                if(bPcTtmp)
                {
                    if(escapSensor.cliff)
                    {
                        float diffPitTmp = 0.0f;
                        int diffFTmp = 0;
                        int diffLTmp = 0;
                        int diffRTmp = 0;
                        diffPitTmp = escActPara.escBasePit - getPitRol(0);
                        int frontCliff = escapSensor.midLeftCliffValue < escapSensor.midRightCliffValue ? escapSensor.midLeftCliffValue : escapSensor.midRightCliffValue;
                        diffFTmp = frontCliff - escActPara.escCiffFBase;
					    diffRTmp = escapSensor.rightCliffValue - escActPara.escCiffRBase;
					    diffLTmp = escapSensor.leftCliffValue - escActPara.escCiffLBase;
                        FRIZY_LOG(LOG_DEBUG, "CIFF ECP %0.1f %0.1f %d %d %d TU=%d %d", diffPitTmp, getPitRol(0),
                        diffFTmp,diffRTmp,diffLTmp,escActPara.parkTubeCnt,escActPara.saveUphillCnt);
                        bumpEscDet.detcbehindState = DETC_BEHIND_IDLE;
                        bumpEscDet.cTScnSpinF = false;
                        if(diffPitTmp>3.0f && (abs(diffRTmp-diffLTmp)>800 || escActPara.parkTubeCnt>=3))//saveUphillCnt 识别上下坡计数
                        {
                            // retTrig = ESCAPE_EVENT_LEFT_OFF;   //进入架空脱困
                            stuckType = ESCAPE_EVENT_TYPE_LEFT_OFF;
						    FRIZY_LOG(LOG_DEBUG, "enter OFF EVENT");
                            return true;
                        }
                        else
                        {
                            // retTrig = ESCAPE_EVENT_BUMPER;   //进入碰撞、探地脱困
                            stuckType = ESCAPE_EVENT_TYPE_CLIFF;
                            FRIZY_LOG(LOG_DEBUG, "enter CLIFF EVENT");
                            return true;
                        }
                    }
                }
            break;
        }
        return false;
    }

    int EscapePlan::cliffEscape()
    {
        static int espcTSpinCnt = 0;
        static int lastEspBmpSta = 0;
        static int spinDirSave = 0;
        static int ciffInvalidCnt = 0;
        static int cTOffCnt = 0;
        static int spinDetcPreAngle = 0;
        static int tmpAglRolPit = 0;
        int ret = 0;
        while(GLOBAL_CONTROL == WHEEL_RUN)
        {
            chassisEscape.GetSensor(&escapSensor);
            if(lastEspBmpSta != escapeBumpState)
            {
                FRIZY_LOG(LOG_DEBUG, "EspBmpSta[%d->%d]", lastEspBmpSta, escapeBumpState);
                lastEspBmpSta = escapeBumpState;
            }
            switch(escapeBumpState)
            {
                case ESCAP_BUMP_IDLE:
                {
                    if(escapSensor.cliff)
                    {
                        escpCiffSpinCnt = 0;
                        espcTSpinCnt = 0;
                        ciffInvalidCnt = 0;
                        spinDetcPreAngle = 0;
                        escapeBumpState = ESCAP_BUMP_SPIN;
                    }
                    else //没有异常，退出脱困
                    {
                        fallBumpCliffSign = 0;
                        // escapeFrontCount = 0;
                        ret = 2;
                        FRIZY_LOG(LOG_DEBUG, "ESCAPE CLIFF no cliff");
                    }
                    spinDirSave = 0;
                }
                break;
                
                case ESCAP_BUMP_SPIN:      //3自旋
                {   
                    //记录自旋前的角度
                    spinDetcPreAngle = escapSensor.addAngle;
                    if(spinDirSave == 0)
                    {
                        if(escapSensor.mcuRightMidCliff || escapSensor.rightCliff)//右
                        {
                            bumpEscDet.cTspinLastDir = WHEELLEFTSPIN;
                            act = 39;
                        }
                        else if(escapSensor.mcuLeftMidCliff || escapSensor.leftCliff)//左
                        {
                            bumpEscDet.cTspinLastDir = WHEELRIGHTSPIN;
                            act = 40;
                        }
                        else//中
                        {
                            //根据累加角度来旋转，减小陀螺仪角度累加
                            if(escapSensor.addAngle > 0)
                            {
                                bumpEscDet.cTspinLastDir = WHEELLEFTSPIN;
                                act = 39;
                            }
                            else
                            {
                                bumpEscDet.cTspinLastDir = WHEELRIGHTSPIN;
                                act = 40;
                            }
                        }
                    }
                    else
                    {
                        if(spinDirSave & 0x01)//第二次自旋
                        {
                            bumpEscDet.cTspinLastDir = WHEELLEFTSPIN;
                            act = 39;
                        }
                        else
                        {
                            bumpEscDet.cTspinLastDir = WHEELRIGHTSPIN;
                            act = 40;
                        }
                        
                    }
                    //如果自旋角度大于25°，直接后退
                    if(!bumpEscDet.cTScnSpinF || espcTSpinCnt < 2)//机器前倾，万向轮可能掉下去了
                    {
                        tmpAglRolPit = 200;
                    }
                    else
                    {
                        tmpAglRolPit = 600;
                    }
                    escapeBumpState = ESCAP_BUMP_SPIN_WAIT;
                    FRIZY_LOG(LOG_DEBUG, "bumpSpin %X %d %0.2f %d ect=%d", 
                    spinDirSave, bumpEscDet.cTspinLastDir, escapSensor.XAngle, tmpAglRolPit, espcTSpinCnt);
                    ciffInvalidCnt = 0;
                }
                break;
                
                case ESCAP_BUMP_SPIN_WAIT://4自旋等待
                {
                    //全方位触发有碰撞、探地信号
                    if(!escapSensor.cliff)
                    {
                        if(++ciffInvalidCnt > 10)
                        {
                            escpCiffSpinCnt = 0;
                            ciffInvalidCnt = 0;
                            FRIZY_LOG(LOG_DEBUG, "spin 360...,ciff Invalid");
                            if(0 == bumpEscDet.behindEndF)
                                escapeBumpState = ESCAP_BUMP_BEHIND;
                            else
                            {
                                escapeBumpState = ESCAP_BUMP_IDLE;
                                ret = 2;
                                saveCiffExitEscpFlag = 0x2;
                            }
                            FRIZY_LOG(LOG_DEBUG, "behindEndF=%d", bumpEscDet.behindEndF);
                        }
                        if(bumpEscDet.behindEndF)
                            break;
                    }
                    else
                    {
                        if(escapSensor.cliff)
                            escpCiffSpinCnt++;
                        ciffInvalidCnt = 0;
                        if(escpCiffSpinCnt > (20*1000/20))//探地超过20s直接判定为脱困失败
                        {
                            FRIZY_LOG(LOG_DEBUG, "ciffSpin outTime 20s,enter AngleEscape");
                            escpCiffSpinCnt = 0;
                            escapeBumpState = ESCAP_BUMP_IDLE;
                            // memset(&AngleEscape,0,sizeof(AngleEscape));//复位脱困参数    ??
                            fallBumpCliffSign = 2;
                            ret = -1;
                        }
                        if(escapSensor.mcuLeftMidCliff || escapSensor.mcuRightMidCliff || escapSensor.midCliff)//前面触发探地
                        {
                            if(abs(spinDetcPreAngle - escapSensor.addAngle) > tmpAglRolPit)
                            {
                                cTOffCnt = 0;
                                ciffInvalidCnt = 0;
                                act = 32;
                                escapeBumpState = ESCAP_BUMP_BEHIND;
                                FRIZY_LOG(LOG_DEBUG, "xxtmpAgl=%d %0.2f %d", tmpAglRolPit, escapSensor.XAngle, escapSensor.cliff);
                            }
                            else
                            {
                                if(++cTOffCnt>3*1000/20)
                                {
                                    ciffInvalidCnt = 0;
                                    cTOffCnt = 0;
                                    act = 32;
                                    escapeBumpState = ESCAP_BUMP_BEHIND;
                                }
                            }
                            
                        }
                        else  //左边 或者右边 有探地触发
                        {
                            if(abs(spinDetcPreAngle - escapSensor.addAngle) > 600 || ++cTOffCnt > 4*1000/20)//自旋超过90°还有探地，或者4s,直接报警
                            {
                                FRIZY_LOG(LOG_DEBUG, "lr cliff %d %d",spinDetcPreAngle - escapSensor.addAngle, cTOffCnt);
                                ciffInvalidCnt = 0;
                                cTOffCnt = 0;
                                act = 32;
                                escpCiffSpinCnt = 0;
                                escapeBumpState = ESCAP_BUMP_IDLE;
                                // memset(&AngleEscape,0,sizeof(AngleEscape));//复位脱困参数
                                fallBumpCliffSign = 2;
                                ret = -1;
                            }
                        }
                    }
                }
                break;

                case ESCAP_BUMP_BEHIND://5后退
                {
                    if(chassisEscape.getWheelState() == WHEELSTOP)
                    {
                        ciffInvalidCnt = 0;
                        // chassisEscape.chassisSpeed(-120, -120, 1);
                        act = 41;
                        escapeBumpState = ESCAP_BUMP_BEHIND_WAIT;
                    }
                    else if(++ciffInvalidCnt > 3*1000/20)   
                    {
                        ciffInvalidCnt = 0;
                        act = 32;
                    }
                }
                break;

                case ESCAP_BUMP_BEHIND_WAIT://6后退等待
                {
                    if(chassisEscape.getWheelState() == WHEELSTOP)
                    {
                        if(!escapSensor.cliff)
                        {
                            escapeBumpState = ESCAP_BUMP_IDLE;
                            ret = 2;
                            saveCiffExitEscpFlag = 0x2;
                            FRIZY_LOG(LOG_DEBUG, "bumpIdle Cif_wait %d", escapSensor.cliff);
                        }
                        else
                        {
                            if((spinDirSave&0xf0) == 0x30)
                            {   
                                espcTSpinCnt++;
                                FRIZY_LOG(LOG_DEBUG, "cT esp %d", espcTSpinCnt);
                                escpCiffSpinCnt = 0;
                                ciffInvalidCnt = 0;
                                spinDetcPreAngle = 0;
                                //前面的探地触发了，并且是机器头部向地面倾斜
                                if((escapSensor.XAngle - espCTPrePitVal>3.0f) &&
                                    escapSensor.cliff)
                                {
                                    if(espcTSpinCnt>=3)//循环动作两次之后就判定为失败
                                    {
                                        bumpEscDet.cTScnSpinF = false;
                                        espcTSpinCnt = 0;
                                        escapeBumpState = ESCAP_BUMP_IDLE;
                                        fallBumpCliffSign = 2;
                                        ret = -1;
                                        FRIZY_LOG(LOG_DEBUG, "espCT aler1");
                                        break;
                                    }
                                    else
                                        escapeBumpState = ESCAP_BUMP_SPIN;
                                }
                                else
                                {
                                    bumpEscDet.cTScnSpinF = true;
                                    if(espcTSpinCnt>=5)//循环动作两次之后就判定为失败
                                    {
                                        espcTSpinCnt = 0;
                                        escapeBumpState = ESCAP_BUMP_IDLE;
                                        fallBumpCliffSign = 2;
                                        ret = -1;
                                        FRIZY_LOG(LOG_DEBUG, "espCT aler2");
                                        break;
                                    }
                                    else
                                    {
                                        escapeBumpState = ESCAP_BUMP_SPIN;
                                    }
                                }
                            }
                        }
                        //上一次是左自旋
                        if(bumpEscDet.cTspinLastDir == WHEELLEFTSPIN)
                        {
                            spinDirSave = 0x02;//下一次就右自旋
                            FRIZY_LOG(LOG_DEBUG, "spnNext 2");
                        }
                        else
                        {
                            spinDirSave = 0x01;
                            FRIZY_LOG(LOG_DEBUG, "spnNext 1");
                        }
                        spinDirSave |= 0x30;
                        FRIZY_LOG(LOG_DEBUG, "spinDirSave=%X %d,pit=%0.2f preP=%0.2f scf=%d\n", 
                        spinDirSave, bumpEscDet.cTspinLastDir, escapSensor.XAngle, espCTPrePitVal, bumpEscDet.cTScnSpinF);
                        escapeBumpState = ESCAP_BUMP_SPIN;
                    }
                    else
                    {
                        if(!escapSensor.cliff)
                        {
                            ciffInvalidCnt++;
                            if(ciffInvalidCnt>15)
                            {
                                ciffInvalidCnt = 0;
                                act = 32;
                            }
                        }
                        else 
                        {
                            cTOffCnt++;
                            if(cTOffCnt > 2*1000/20)
                            {
                                cTOffCnt = 0;
                                act = 32;
                            }
                        }
                    }
                }
                break;
            }
            if(ret != 0)
            {
                FRIZY_LOG(LOG_DEBUG, "return ret:%d", ret);
                return ret;
                // escapeEvent &= ~ESCAPE_EVENT_BUMPER;
            }
            usleep(20 * 1000);
        }
    }


    //监测陀螺仪数据用于判定是否脱困成功           未完成
    int EscapePlan::monitorIsEscapeSuccess()
    {
        int aimSpdL = chassisEscape.getSendSpeed().first;
        int aimSpdR = chassisEscape.getSendSpeed().second;
        int aimBaeGyro = 0;
        int fdbBaeGyro = 0;
        escAimSpdCalAngle = calcAimAglVelocity(); //目标速度差计算角速度 单位 0.1dps
        escFdbSpdCalAngle = calcFdbAglVeloctiy(); //反馈速度差计算角速度
        escGyroAngle = escapSensor.zGyroOriginal; //陀螺仪反馈的角速度
        if (aimSpdL != 0 || aimSpdR != 0)
        {

            if (0 != escAimSpdCalAngle)
            {
                aimBaeGyro = abs(100 * escGyroAngle / escAimSpdCalAngle);
            }

            if (0 != escFdbSpdCalAngle)
            {
                fdbBaeGyro = abs(100 * escGyroAngle / escFdbSpdCalAngle);
            }

            FRIZY_LOG(LOG_DEBUG, "EspAct a=%d,f=%d,g=%d,ag=%d,fg=%d,cnt=%d,%d", escAimSpdCalAngle, escFdbSpdCalAngle, escGyroAngle, aimBaeGyro, fdbBaeGyro, escActPara.aglVelEffecCnt, escActPara.escAglDiffCntSpin);
            if (escGyroAngle > 2200)
            {
                if (escActPara.escpAglVeloEffcCnt > 800 / 20)
                {
                    FRIZY_LOG(LOG_DEBUG, "escapAct success %d\n", __LINE__);
                    escActPara.escpAglVeloEffcCnt = 0;
                    return 1;
                }
                else
                {
                    escActPara.escpAglVeloEffcCnt++;
                }
            }
            else
            {
                escActPara.escpAglVeloEffcCnt = 0;
            }

            if (((aimSpdL + aimSpdR) == 0) || (abs(aimSpdL - aimSpdR) > 70))
            {
                if (aimBaeGyro < 30) //判定为动作无效
                {
                    escActPara.escAglDiffCntSpin++;
                    if (escActPara.escAglDiffCntSpin > 800 / 20)
                    {
                        escActPara.escAglDiffCntSpin = 0;
                        return -1;
                    }
                }
                else
                {
                    escActPara.escAglDiffCntSpin = 0;
                }

                if (aimBaeGyro >= 70) //判定为动作有效
                {
                    if (escActPara.aglVelEffecCnt >= 600 / 20)
                    {
                        escActPara.aglVelEffecCnt = 0;
                        FRIZY_LOG(LOG_DEBUG, "escapAct success %d\n", __LINE__);
                        return 1;
                    }
                    else
                    {
                        escActPara.aglVelEffecCnt++;
                    }
                }
                else
                {
                    if (escActPara.aglVelEffecCnt > 0)
                    {
                        escActPara.aglVelEffecCnt--;
                    }
                }
            }
            else
            {
                if (aimBaeGyro < 10) //判定为动作无效
                {
                    escActPara.escAglDiffCntSpin++;
                    if (escActPara.escAglDiffCntSpin > 800 / 20)
                    {
                        escActPara.escAglDiffCntSpin = 0;
                        return -1;
                    }
                }
                else
                {
                    escActPara.escAglDiffCntSpin = 0;
                }
            }
            FRIZY_LOG(LOG_DEBUG, "EspAct a=%d,f=%d,g=%d,ag=%d,fg=%d,cnt=%d,%d[%d,%d]", escAimSpdCalAngle, escFdbSpdCalAngle, escGyroAngle, aimBaeGyro, fdbBaeGyro, escActPara.aglVelEffecCnt, escActPara.escAglDiffCntSpin, aimSpdL, aimSpdR);
        }
        else
        {
            escActPara.escpAglVeloEffcCnt = 0;
            escActPara.escAglDiffCntSpin = 0;
            escActPara.aglVelEffecCnt = 0;
        }

        return 0;
    }

    //脱困动作电流和角速度监控              未完成
    void EscapePlan::escActCanMoveCheck()
    {
        static int8_t lastWheelSta = WHEELSTOP;
        static int8_t effectRateCnt = 0;
        static int8_t effectCurCnt = 0;
        static int8_t lastWheelAct = 0;
        static int16_t checkDelayCnt = 0; //开始动作后400ms，开始记录
        //从停止到运动 lastWheelSta == WHEEL_STOP
        if (WHEELSTOP != chassisEscape.getWheelState() && lastWheelSta != chassisEscape.getWheelState())
        {
            checkDelayCnt = 20; //400/20
        }
        lastWheelSta = chassisEscape.getWheelState();
        if (checkDelayCnt > 0)
        {
            checkDelayCnt--;
            return;
        }
        if (WHEELSTOP == chassisEscape.getWheelState())
            return;
    }

    //角速度异常判定    未完成
    bool EscapePlan::abnormalAngVelocCheck(int mode_t)
    {
        bool angVelRet = false;
        static uint8_t lastEscWheelSta = WHEELSTOP;
        static int16_t actionChangCnt = 0;
        int32_t spdDiff = 0;
        int32_t spdDiffThresh = 0;
        int32_t rateAngle = 0;
        wheelState_t wheel_sta;

        rateAngle;

        wheel_sta = getWheelControlState();
        //动作变更500ms之后再开始判定
        if (lastEscWheelSta != wheel_sta.wheelState)
        {
            actionChangCnt = 25; //500/20
        }
        lastEscWheelSta = wheel_sta.wheelState;
        //累加左右自旋和左后右后的角速度
        if (wheel_sta.wheelLeftState == WHEELBEHIND)
        {
            if (wheel_sta.wheelRightState == WHEELFRONT || wheel_sta.wheelRightState == WHEELSTOP)
            {
                escActPara.leftAngVelAdd += abs(rateAngle);
            }
        }
        else if (wheel_sta.wheelRightState == WHEELBEHIND)
        {
            if (wheel_sta.wheelLeftState == WHEELFRONT || wheel_sta.wheelRightState == WHEELSTOP)
            {
                escActPara.rightAngVelAdd += abs(rateAngle);
            }
        }
        //动作变更500ms之后再开始判定
        // if(actionChangCnt > 0 && )
    }
    //俯仰和横滚异常判定  return true 异常 false 正常
    bool EscapePlan::abnormalPitRolCheck()
    {
        // FRIZY_LOG(LOG_DEBUG,"进入俯仰和横滚异常判定");
        static bool lastOutAvgFlag = false;
        static float lastOutAvg = 0.0f;
        bool pitRolFailRet = false;
        float pitRolEscTmp = 0.0f; //俯仰和横滚滑动平均值
        pitRolEscTmp = gyroPitRolSlidingAvg();
        // FRIZY_LOG(LOG_DEBUG,"pitRolEscTmp:%f",pitRolEscTmp);
        // FRIZY_LOG(LOG_DEBUG,"lastOutAvgFlag:%d, pitRolLastAvg[0]:%f, fabsf(pitRolEscTmp - pitRolLastAvg[0]:%f",
        // lastOutAvgFlag, pitRolLastAvg[0], fabsf(pitRolEscTmp - pitRolLastAvg[0]));
        if (false == lastOutAvgFlag && fabsf(pitRolEscTmp - pitRolLastAvg[0]) >= 2.0f)
        // if(false == lastOutAvgFlag && fabsf(pitRolEscTmp - pitRolLastAvg[0]) >= 6.0f)
        {
            if (!lastOutAvgFlag)
            {
                angVelOutCnt = 0;
                // FRIZY_LOG(LOG_INFO,"lastOutAvgFlag is false");
            }
            lastOutAvgFlag = true;
            // FRIZY_LOG(LOG_INFO,"lastOutAvgFlag is true");
            lastOutAvg = pitRolLastAvg[0];
        }
        // FRIZY_LOG(LOG_DEBUG,"lastOutAvgFlag:%d, lastOutAvg:%f", lastOutAvgFlag, lastOutAvg);
        // FRIZY_LOG(LOG_DEBUG,"fabsf(pitRolEscTmp - lastOutAvg):%f",fabsf(pitRolEscTmp - lastOutAvg));
        if (lastOutAvgFlag && fabsf(pitRolEscTmp - lastOutAvg) >= 2.0f)
        // if(lastOutAvgFlag && fabsf(pitRolEscTmp - lastOutAvg) >= 6.0f)
        {
            chassisEscape.GetSensor(&escapSensor);
            // FRIZY_LOG(LOG_DEBUG, "check Pit:%d", escapSensor.XAngle);
            // FRIZY_LOG(LOG_DEBUG, "angVelOutCnt:%d", angVelOutCnt);
            if (abs(escapSensor.XAngle) > 2.5f)
            // if(escapSensor.XAngle < -70.0f || escapSensor.XAngle > 40.0f)
            {
                // FRIZY_LOG(LOG_DEBUG,"PIT ABNORMAL");
                if (angVelOutCnt > 2000 / 50)
                // if(angVelOutCnt > 10)
                {
                    //这里面不清零
                    FRIZY_LOG(LOG_DEBUG, "esc pitRolFail[%0.1f %0.1f %f %0.1f]", pitRolEscTmp, pitRolLastAvg[0], escapSensor.XAngle, lastOutAvg);
                    // FRIZY_LOG(LOG_DEBUG,"pitRolFailRet = true");
                    pitRolFailRet = true;
                }
                else
                {
                    angVelOutCnt++;
                }
            }
            else
            {
                // FRIZY_LOG(LOG_DEBUG,"< 2.5f");
                angVelOutCnt = 0;
                lastOutAvgFlag = false;
                pitRolFailRet = false;
            }
        }
        else
        {
            // FRIZY_LOG(LOG_DEBUG,"< 2.0f");
            angVelOutCnt = 0;
            lastOutAvgFlag = false;
            pitRolFailRet = false;
        }
        SlidingArrayAddF(pitRolLastAvg, PITROL_LAST_AVG_NUM, pitRolEscTmp);
        return pitRolFailRet;
    }
    // 边耍缠绕检测
    void EscapePlan::SideBrushElectricCheck()
    {
        int32_t LeftSideBrushCurAvg = 0;
        int32_t RightSideBrushCurAvg = 0;
        //边耍电流采集与判定
        // chassisEscape.GetSensor(&escapSensor);

        if (escapSensor.rightSideBrushElectricity && escapSensor.leftSideBrushElectricity)
        {
            //边刷异常检测
            if (escapSensor.rightSideBrushElectricity > SIDEBRUSHELECTRICITYINDEX || escapSensor.leftSideBrushElectricity > SIDEBRUSHELECTRICITYINDEX)
            {
                RightSideBrushCurAvg = GlideFilterAD(RightSideBrushCurArry, AVG_WHEEL_CURARRY_NUM, escapSensor.rightSideBrushElectricity);
                sideCheck.rightSideCurrentNow = RightSideBrushCurAvg;
                LeftSideBrushCurAvg = GlideFilterAD(LeftSideBrushCurArry, AVG_WHEEL_CURARRY_NUM, escapSensor.leftSideBrushElectricity);
                sideCheck.leftSideCurrentNow = LeftSideBrushCurAvg;
                FRIZY_LOG(LOG_DEBUG, "RightSideBrushCurAvg: %d  LeftSideBrushCurAvg : %d", RightSideBrushCurAvg, LeftSideBrushCurAvg);
                //边刷电流过大直接报警
                if (RightSideBrushCurAvg > SIDEBRUSHELECTRICTYMAX * 1.35 || LeftSideBrushCurAvg > SIDEBRUSHELECTRICTYMAX * 1.35)
                {
                    sideCheck.sideBrushAlertsum++;
                    if (sideCheck.sideBrushAlertsum > 20)
                    {
                        side_brush_alert = true;
                        sideCheck.sideBrushAlertsum = 0;
                    }
                }
                if (sideCheck.leftSideCurrentNow > SIDEBRUSHELECTRICTYMAX || sideCheck.rightSideCurrentNow > SIDEBRUSHELECTRICTYMAX)
                {
                    sideCheck.sideCurrentCountOne++;   //边刷一级阈值计数 ++
                    sideCheck.sideCurrentCountTwo = 0; //边刷二级阈值计数清零
                    if (sideCheck.sideCurrentCountOne >= 20)
                    {
                        sideCheck.sideCurrentCountThree = 0; //边刷三级阈值计数清零
                        sideCheck.sideCurrentCountOne = 0;   //边刷一级阈值计数清零
                        sideCheck.sideCurrentErr++;          //边刷错误阈值 ++；
                        side_abnomal_index = true;
                        FRIZY_LOG(LOG_DEBUG, "sideCheck.sideCurrentErr : %d", sideCheck.sideCurrentErr);
                        // stuckType = ESCAPE_EVENT_TYPE_STUCK;
                        // UMAPI_CtrlSideBrush(0);
                    }
                }
                //边刷回复正常
                else if (side_abnomal_index == true && sideCheck.leftSideCurrentNow <= SIDEBRUSHELECTRICTYMAX && sideCheck.leftSideCurrentNow <= SIDEBRUSHELECTRICTYMAX)
                {
                    sideCheck.sideCurrentCountTwo++; //边刷二级阈值计数 ++
                    if (sideCheck.sideCurrentCountTwo > 20)
                    {
                        sideCheck.sideCurrentCountOne = 0;   //边刷一级阈值计数清零
                        sideCheck.sideCurrentCountTwo = 0;   //边刷二级阈值计数清零
                        sideCheck.sideCurrentCountThree = 0; //边刷三级阈值计数清零
                        sideCheck.sideCurrentErr = 0;        //边刷错误阈值清零
                        side_abnomal_index = false;          //消除边刷脱困标志
                        escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                        FRIZY_LOG(LOG_DEBUG, "sideCheck refresh normal");
                    }
                }

                //边刷被缠绕脱困失败
                if (sideCheck.sideCurrentErr >= 15) //边刷超过阈值一级警报
                {
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                    FRIZY_LOG(LOG_DEBUG, "sideCheck.sideCurrentErr >= 15");
                }
            }
        }
    }
    //电流异常检测
    bool EscapePlan::abnormalElectricCheck()
    {
        static bool curOutFlagL = false;
        static bool curOutFlagR = false;
        bool judRetL = false;
        bool judRetR = false;
        bool curSumRetL = false;
        bool curSumRetR = false;
        int32_t escLeftCurAvg = 0;
        int32_t escRightCurAvg = 0;

        wheelState_t wheelSta_tmp;
        wheelSta_tmp = getWheelControlState();
        //电流采集与判定
        chassisEscape.GetSensor(&escapSensor);
        // FRIZY_LOG(LOG_DEBUG, "left cur:%f,right cur:%f", escapSensor.leftWheelElec, escapSensor.rightWheelElec);
        if (escapSensor.leftWheelElec > 0 && escapSensor.rightWheelElec > 0)
        {
            //左轮
            if (WHEELSTOP != wheelSta_tmp.wheelLeftState && escapSensor.leftWheelElec > 50)
            {
                // FRIZY_LOG(LOG_DEBUG,"left cur:%f", escapSensor.leftWheelElec);
                //求滑动平均值
                escLeftCurAvg = GlideFilterAD(escLeftCurArry, AVG_WHEEL_CURARRY_NUM, escapSensor.leftWheelElec);
                // FRIZY_LOG(LOG_DEBUG,"leftCurSlidAvg:%d", escLeftCurAvg);
                // 数组中填满数据之后才开始判定
                // FRIZY_LOG(LOG_DEBUG,"leftCurSlidAvg num:%d", escCkeckPara.arryAvgCurCountL);
                if (escCkeckPara.arryAvgCurCountL > AVG_WHEEL_CURARRY_NUM * AVG_LAST_WHEEL_CURARRY_NUM)
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组中填满数据之后才开始判定");
                    // FRIZY_LOG(LOG_DEBUG,"10*escLeftCurAvg:%d,escLastLeftCurAvg[0]:%d", 10*escLeftCurAvg, escLastLeftCurAvg[0]);

                    int tmpl = 0;
                    if (!escLastRightCurAvg[0])
                    {
                        // FRIZY_LOG(LOG_DEBUG, "wuyushijian1 %d", escLastRightCurAvg[0]);
                    }
                    else
                    {
                        tmpl = 10 * escRightCurAvg / escLastRightCurAvg[0];
                    }

                    // FRIZY_LOG(LOG_DEBUG, "10*escLeftCurAvg)/escLastLeftCurAvg[0]:%d", tmpl);
                    //当前的值和1.6s 前的滑动平均值变化大于1.5倍
                    if (tmpl >= WHEEL_CURRENT_ABNORMAL)
                    {
                        if (!curOutFlagL)
                        {
                            abnrCurrOutCntL = 0;
                        }
                        curOutFlagL = true;
                    }
                    // FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntL:%d",abnrCurrOutCntL);
                    if (curOutFlagL)
                    {
                        // FRIZY_LOG(LOG_DEBUG, "left cur:%f", escapSensor.leftWheelElec);
                        if (escapSensor.leftWheelElec > WHEEL_LEFT_CURRENT_LIMIT * 3 / 4)
                        {
                            // FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntL:%d", abnrCurrOutCntL);
                            if (abnrCurrOutCntL > 800 / 20)
                            {
                                //这里面不清零
                                judRetL = true;
                                curOutFlagL = false;
                            }
                            else
                            {
                                abnrCurrOutCntL++;
                            }
                        }
                        else
                        {
                            judRetL = false;
                            abnrCurrOutCntL = 0;
                        }
                    }
                    else
                    {
                        abnrCurrOutCntL = 0;
                        judRetL = false;
                    }
                }
                else
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组元素+1");
                    escCkeckPara.arryAvgCurCountL++;
                }
                SlidingArrayAddI(escLastLeftCurAvg, AVG_LAST_WHEEL_CURARRY_NUM, escLeftCurAvg);
            }
            else
            {
                abnrCurrOutCntL = 0;
                curOutFlagL = false;
                judRetL = false;
            }
            //右轮
            if (WHEELSTOP != wheelSta_tmp.wheelRightState && escapSensor.rightWheelElec > 50)
            {
                // FRIZY_LOG(LOG_DEBUG,"right cur:%f", escapSensor.rightWheelElec);
                //求滑动平均值
                escRightCurAvg = GlideFilterAD(escRightCurArry, AVG_WHEEL_CURARRY_NUM, escapSensor.rightWheelElec);
                // FRIZY_LOG(LOG_DEBUG,"rightCurSlidAvg:%d", escRightCurAvg);
                // FRIZY_LOG(LOG_DEBUG,"rightCurSlidAvg num:%d", escCkeckPara.arryAvgCurCountR);
                if (escCkeckPara.arryAvgCurCountR > AVG_WHEEL_CURARRY_NUM * AVG_LAST_WHEEL_CURARRY_NUM)
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组中填满数据之后才开始判定");
                    //当前的值 1.6s 前的滑动平均值变化大于1.5倍
                    // FRIZY_LOG(LOG_DEBUG,"10*escRightCurAvg:%d,escLastRightCurAvg[0]:%d", 10*escRightCurAvg, escLastRightCurAvg[0]);

                    int tmpr = 0;
                    if (!escLastRightCurAvg[0])
                    {
                        // FRIZY_LOG(LOG_DEBUG, "wuyushijian2 %d", escLastRightCurAvg[0]);
                    }
                    else
                    {
                        tmpr = 10 * escRightCurAvg / escLastRightCurAvg[0];
                    }

                    // FRIZY_LOG(LOG_DEBUG, "10*escRightCurAvg/escLastRightCurAvg[0]:%d", tmpr);
                    if (tmpr >= WHEEL_CURRENT_ABNORMAL)
                    {
                        if (!curOutFlagR)
                        {
                            abnrCurrOutCntR = 0;
                        }
                        curOutFlagR = true;
                    }

                    if (curOutFlagR)
                    {
                        // FRIZY_LOG(LOG_DEBUG, "right cur:%f", escapSensor.rightWheelElec);
                        if (escapSensor.rightWheelElec > WHEEL_RIGHT_CURRENT_LIMIT * 3 / 4)
                        {
                            // FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntR:%d", abnrCurrOutCntR);
                            if (abnrCurrOutCntR > 800 / 20)
                            {
                                //这里面不清零
                                judRetR = true;
                                curOutFlagR = false;
                            }
                            else
                            {
                                abnrCurrOutCntR++;
                            }
                        }
                        else
                        {
                            abnrCurrOutCntR = 0;
                            judRetR = false;
                        }
                    }
                    else
                    {
                        abnrCurrOutCntR = 0;
                        judRetR = false;
                    }
                }
                else
                {
                    escCkeckPara.arryAvgCurCountR++;
                }
                SlidingArrayAddI(escLastRightCurAvg, AVG_LAST_WHEEL_CURARRY_NUM, escRightCurAvg);
            }
            else
            {
                abnrCurrOutCntR = 0;
                curOutFlagR = false;
                judRetR = false;
            }
        }
        else
        {
            curOutFlagL = false;
            curOutFlagR = false;
            abnrCurrOutCntR = 0;
            abnrCurrOutCntL = 0;
        }
        if (judRetR || judRetL || curSumRetR || curSumRetL)
        {
            abnrCurrOutCntR = 0;
            abnrCurrOutCntL = 0;
            // escCkeckPara.arryAvgCurCountR = 0;
            // escCkeckPara.arryAvgCurCountL = 0;
            return true;
        }
        else
        {
            return false;
        }
    }

    //lds位置信息 机器速度反馈  电流 综合判定
    int EscapePlan::machinLocationInfoCheck(void)
    {
        static long long lastSysTemTim = 0;
        static long long lastkCheckTemTim = 0;

        static float_t lastAngle = 0;
        static int dirvPosiChangSum = 0;
        static int ldsPosiChangSum = 0;

        float_t angle = 0.0f, angleTmp = 0.0f;
        float_t roughlyDisTmp = 0.0f;
        float_t currX = 0.0f, currY = 0.0f;

        int ret = 0;

        long long timeDiff = getTime() - lastSysTemTim;

        chassisEscape.GetSensor(&escapSensor);

        // FRIZY_LOG(LOG_DEBUG, "WHEEL CUR L:%0.1f R:%0.1f", escapSensor.leftWheelElec,escapSensor.rightWheelElec);

        chassisEscape.GridPoint(&escapGrid);
        int aimSpdL = chassisEscape.getSendSpeed().first;
        int aimSpdR = chassisEscape.getSendSpeed().second;

        int aimLineVel = (aimSpdL + aimSpdR) / 2;                      //目标线速度
        int fdblineVel = (escapSensor.rightw + escapSensor.leftw) / 2; //反馈线速度  mm/s
        //-------- ------用底层反馈回来的速度 转成坐标系上两轴的分量(不准确的,只作脱困识别)-----------------

        //大致的位移
        roughlyDisTmp = fdblineVel * timeDiff / 1000; //mm

        angle = escapGrid.forward; //航向角0~360

        //上次和当前角度的平均值
        angleTmp = _PI * (360.0f - ((angle + lastAngle) / 2.0f)) / 180.0f;

        lastAngle = angle;

        //计算x轴和y轴方向的分量
        currX = (float)roughlyDisTmp * cos(angleTmp); //mm
        currY = (float)roughlyDisTmp * sin(angleTmp); //mm

        //在0° 左右10°时将分量置为负的
        if (((angle > 350) && (lastAngle < 10)) || ((angle < 10) && (lastAngle > 350)))
        {
            currX = 0.0f - currX;
            currY = 0.0f - currY;
        }

        //  FRIZY_LOG(LOG_DEBUG,"CalcCorrd %0.1f,%0.1f,%0.1f,aglT=%0.1f agl=%0.1f,%0.1f"
        //     ,currX,currY,roughlyDisTmp,angleTmp,escapGrid.forward,escapSensor.ZAngle);

        escActPara.fdbRoughlyIncX += (currX / 10.0f); //cm
        escActPara.fdbRoughlyIncY += (currY / 10.0f); //cm

        // escAimSpdCalAngle = calcAimAglVelocity();//目标速度差计算角速度
        static float lastLocX = 0;
        static float lastLocY = 0;

        float grXTmp = escapGrid.realx * 15; //cm
        float grYTmp = escapGrid.realy * 15; //cm

        //lds 给出的增量
        escActPara.ldsIncX = (grXTmp - escActPara.startCheckLdsX);
        escActPara.ldsIncY = (grYTmp - escActPara.startCheckLdsY);

        lastLocX = grXTmp;
        lastLocY = grYTmp;

        lastSysTemTim = getTime();

        if (aimSpdL || aimSpdR)
        {
            if ((getTime() - lastkCheckTemTim) >= 1000) //每隔一秒对比一次轮子反馈的和lds反馈的坐标增量
            {
                lastkCheckTemTim = getTime();

                float diffTmpX = escActPara.fdbRoughlyIncX - escActPara.ldsIncX;
                float diffTmpY = escActPara.fdbRoughlyIncY - escActPara.ldsIncY;

                //两种坐标的差值
                int disDiffTmp = (int)sqrtf(diffTmpX * diffTmpX + diffTmpY * diffTmpY);

                SlidingArrayAddI(PositionDiffArry, POSITION_CHECK_ARRY_NUM, disDiffTmp);

                if (escActPara.fdbRoughlyIncX > 100000 || escActPara.fdbRoughlyLastIncX > 100000 || escActPara.fdbRoughlyIncY > 100000 || escActPara.fdbRoughlyLastIncY > 100000)
                {
                    escActPara.fdbRoughlyIncX = 0;
                    escActPara.fdbRoughlyLastIncX = 0;
                    escActPara.fdbRoughlyIncY = 0;
                    escActPara.fdbRoughlyLastIncY = 0;
                }
                //1s内的位置差
                float fdbPosiTmpX = escActPara.fdbRoughlyIncX - escActPara.fdbRoughlyLastIncX;
                float fdbPosiTmpY = escActPara.fdbRoughlyIncY - escActPara.fdbRoughlyLastIncY;

                //底层速度得出的位置变化
                escActPara.dirvPosiChangSum += sqrtf(fdbPosiTmpX * fdbPosiTmpX + fdbPosiTmpY * fdbPosiTmpY);

                if (escActPara.dirvPosiChangSum > 10000)
                {
                    FRIZY_LOG(LOG_DEBUG, "errDate %0.1f,%0.1f,%0.1f[%0.1f,%0.1f]", escActPara.fdbRoughlyIncX, escActPara.fdbRoughlyLastIncX, escActPara.fdbRoughlyIncY, escActPara.fdbRoughlyLastIncY, currX, currY);
                }
                float ldsPosiTmpX = escActPara.ldsIncX - escActPara.ldsLastIncX;
                float ldsPosiTmpY = escActPara.ldsIncY - escActPara.ldsLastIncY;

                //lds的位置变化
                escActPara.ldsPosiChangSum += sqrtf(ldsPosiTmpX * ldsPosiTmpX + ldsPosiTmpY * ldsPosiTmpY);

                FRIZY_LOG(LOG_DEBUG, "escp tmpDiff %0.1f,%0.1f,%0.1f,%0.1f", fdbPosiTmpX, fdbPosiTmpY, ldsPosiTmpX, ldsPosiTmpY);
                FRIZY_LOG(LOG_DEBUG, "diffChangSum fdb=%0.1f lds=%0.1f", escActPara.dirvPosiChangSum, escActPara.ldsPosiChangSum);

                //6s 内差值大于60cm
                int posiTimeDiff = 0;

                posiTimeDiff = PositionDiffArry[POSITION_CHECK_ARRY_NUM - 1] - PositionDiffArry[0];

                if (posiTimeDiff > 60 && (escActPara.dirvPosiChangSum - escActPara.ldsPosiChangSum) > 60)
                {
                    ret = 1;
                    FRIZY_LOG(LOG_DEBUG, "LOCP DIFF DISOUT %d,%d\n", PositionDiffArry[POSITION_CHECK_ARRY_NUM - 1], PositionDiffArry[0]);

                    escActPara.positDiffMinCnt = 0;
                    //需要重新开始计算差异
                    escActPara.fdbRoughlyIncX = 0.0f;
                    escActPara.fdbRoughlyIncY = 0.0f;
                    escActPara.ldsIncX = 0.0f;
                    escActPara.ldsIncY = 0.0f;
                    memset(PositionDiffArry, 0, sizeof(PositionDiffArry));
                    escActPara.startCheckLdsX = grXTmp;
                    escActPara.startCheckLdsY = grYTmp;
                    escActPara.dirvPosiChangSum = 0.0f;
                    escActPara.ldsPosiChangSum = 0.0f;
                }
                else
                {
                    if (posiTimeDiff < 15 || abs(escActPara.dirvPosiChangSum - escActPara.ldsPosiChangSum) < 20)
                    {
                        escActPara.positDiffMinCnt++;
                        if (escActPara.positDiffMinCnt >= 12)
                        {
                            escActPara.positDiffMinCnt = 0;
                            //需要重新开始计算差异
                            escActPara.fdbRoughlyIncX = 0.0f;
                            escActPara.fdbRoughlyIncY = 0.0f;
                            escActPara.ldsIncX = 0.0f;
                            escActPara.ldsIncY = 0.0f;
                            memset(PositionDiffArry, 0, sizeof(PositionDiffArry));
                            escActPara.startCheckLdsX = grXTmp;
                            escActPara.startCheckLdsY = grYTmp;
                            escActPara.dirvPosiChangSum = 0.0f;
                            escActPara.ldsPosiChangSum = 0.0f;
                            FRIZY_LOG(LOG_DEBUG, "LOCP clean");
                        }
                    }
                    else
                    {
                        if (escActPara.positDiffMinCnt > 0)
                        {
                            escActPara.positDiffMinCnt--;
                        }
                    }
                }

                FRIZY_LOG(LOG_DEBUG, "LOCP(%0.1f %0.1f),(%0.1f,%0.1f),%0.1f,%0.1f,DF=%d,%d", escActPara.fdbRoughlyIncX, escActPara.fdbRoughlyIncY, escActPara.ldsIncX, escActPara.ldsIncY, diffTmpX, diffTmpY, disDiffTmp, posiTimeDiff);

                escActPara.fdbRoughlyLastIncX = escActPara.fdbRoughlyIncX;
                escActPara.fdbRoughlyLastIncY = escActPara.fdbRoughlyIncY;

                escActPara.ldsLastIncX = escActPara.ldsIncX;
                escActPara.ldsLastIncY = escActPara.ldsIncY;
            }
        }
        return ret;
    }

    static int b_time = 0;
    static int behindSumSpd = 0;
    static int back_dis = 0, back_time = 0, spin_flag = 0, rotate_flag = 0;
    //按给定距离后退
    bool EscapePlan::wheelBackDist(int speed, int dis) //速度 距离mm
    {
        int currSpdTmp = 0;

        currSpdTmp = (escapSensor.rightw + escapSensor.leftw) / 2;

        behindSumSpd += currSpdTmp;
        b_time++;

        FRIZY_LOG(LOG_DEBUG, "b_time:%d,%d", abs(behindSumSpd * b_time * 10), (dis - 5) * 1000);
        chassisEscape.chassisSpeed(speed, speed, 1);

        if (abs(behindSumSpd * b_time * 10) >= (dis - 5) * 1000) //mm/s ms
        {
            chassisEscape.chassisSpeed(0, 0, 1);
            b_time = 0;
            behindSumSpd = 0;
            return true;
        }
        else
        {
            return false;
        }

#if 0
        if(!back_dis)
        {
            b_time = dis / abs(speed);//ms
            back_dis = 1;
        }
        FRIZY_LOG(LOG_DEBUG, "b_time:%d", b_time);
        chassisEscape.chassisSpeed(speed, speed, 1);
        b_time --;
        usleep(100 * 1000);
        if(b_time)
            return false;
        else
        {
            chassisEscape.chassisSpeed(0, 0, 1);
            back_dis = 0;
            return true;
        }
#endif
    }

    //按给定时间后退
    bool EscapePlan::wheelCtrlStraight(int speed, int walkTime) //速度  时间ms
    {
        static int s_stime;
        if (walkTime == 0) {
            chassisEscape.chassisSpeed(speed, speed, 1);
            return false;
        }
        if (!back_time)
        {
            s_stime = walkTime / 10;
            back_time = 1;
        }
        s_stime--;
        FRIZY_LOG(LOG_DEBUG, "s_stime:%d", s_stime);
        chassisEscape.chassisSpeed(speed, speed, 1);
        // usleep(100 * 1000);
        if (s_stime > 0)
            return false;
        else
        {
            chassisEscape.chassisSpeed(0, 0, 1);
            back_time = 0;
            return true;
        }
    }

    bool EscapePlan::wheelCtrlLidar(int speed, int walkTime)
    {
        static int times;
        if (!back_time)
        {
            times = walkTime / 10;
            back_time = 1;
        }
        times--;
        if(escapSensor.bump || escapSensor.cliff)
        {
            chassisEscape.chassisSpeed(0, 0, 1);
            back_time = 0;
            return true;
        }
        chassisEscape.chassisSpeed(speed, speed, 1);
        if (times > 0)
            return false;
        else
        {
            chassisEscape.chassisSpeed(0, 0, 1);
            back_time = 0;
            return true;
        }
    }

    //脱困成功后的自旋                          dir 0:左自旋 1:右自旋
    bool EscapePlan::spinSuccess(int speed, int dir, float angle)
    {
        if (!dir)
        {
            FRIZY_LOG(LOG_INFO, "left aimforward:%f, gyo_angle_:%f", angle, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(-speed, speed, 1);
            chassisEscape.GetSensor(&escapSensor);
            FRIZY_LOG(LOG_DEBUG, "spinSuccess SPEED L:%d R:%d", escapSensor.leftw, escapSensor.rightw);
            if (fabs(angle - (360 - gyo_angle_ * 180 / _Pi)) < 4 || fabs(angle - (360 - gyo_angle_ * 180 / _Pi)) > 356)
            {
                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                return true;
            }
        }
        else
        {
            FRIZY_LOG(LOG_INFO, "right aimforward:%f, gyo_angle_:%f", angle, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(speed, -speed, 1);
            chassisEscape.GetSensor(&escapSensor);
            FRIZY_LOG(LOG_DEBUG, "spinSuccess SPEED L:%d R:%d", escapSensor.leftw, escapSensor.rightw);
            if (fabs(angle - (360 - gyo_angle_ * 180 / _Pi)) < 4 || fabs(angle - (360 - gyo_angle_ * 180 / _Pi)) > 356)
            {
                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                return true;
            }
        }
        return false;
    }

    //自旋                                                  //0:相对角度 1:绝对角度
    bool EscapePlan::escSpin(int speed, int dir, float angle, int relatAbs)
    {
        static float spin_aimforward = 0.0f;
        if (!dir)
        {
            if (!spin_flag)
            {
                if (!relatAbs)
                {
                    spin_aimforward = (360 - gyo_angle_ * 180 / _Pi) - angle;
                    if (spin_aimforward <= 0)
                    {
                        spin_aimforward = 360 + spin_aimforward;
                    }
                }
                else
                {
                    spin_aimforward = angle;
                }
                spin_flag = 1;
            }

            FRIZY_LOG(LOG_INFO, "spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(-speed, speed, 1);
            if (fabs(spin_aimforward - (360 - gyo_angle_ * 180 / _Pi)) < 5 || fabs(spin_aimforward - (360 - gyo_angle_ * 180 / _Pi)) > 355)
            {
                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                spin_flag = 0;
                return true;
            }
            else
                return false;
        }
        else
        {
            if (!spin_flag)
            {
                if (!relatAbs)
                {
                    spin_aimforward = (360 - gyo_angle_ * 180 / _Pi) + angle;
                    if (spin_aimforward >= 360)
                    {
                        spin_aimforward = spin_aimforward - 360;
                    }
                }
                else
                {
                    spin_aimforward = angle;
                }
                spin_flag = 1;
            }
            FRIZY_LOG(LOG_INFO, "spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(speed, -speed, 1);
            if (fabs(spin_aimforward - (360 - gyo_angle_ * 180 / _Pi)) < 5 || fabs(spin_aimforward - (360 - gyo_angle_ * 180 / _Pi)) > 355)
            {
                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                spin_flag = 0;
                return true;
            }
            else
                return false;
        }
    }
    //单边旋                                                          //0:相对角度 1:绝对角度
    bool EscapePlan::singleRotate(int speedL, int speedR, float angle, int relatAbs)
    {
        static float rotate_aimforward = 0.0f;
        if (speedL < speedR)
        {
            if (!rotate_flag)
            {

                if (!relatAbs)
                {
                    rotate_aimforward = (360 - gyo_angle_ * 180 / _Pi) - angle;
                    if (rotate_aimforward <= 0)
                    {
                        rotate_aimforward = 360 + rotate_aimforward;
                    }
                }
                else
                {
                    rotate_aimforward = angle;
                }
                rotate_flag = 1;
            }
            // if(getTime - actStartTime >= )
            // {
            //     FRIZY_LOG(LOG_DEBUG, "spin timeout");
            //     return true;
            // }
            FRIZY_LOG(LOG_INFO, "rotate_aimforward:%f, gyo_angle_:%f", rotate_aimforward, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(speedL, speedR, 1);
            if (fabs(rotate_aimforward - (360 - gyo_angle_ * 180 / _Pi)) < 5 || fabs(rotate_aimforward - (360 - gyo_angle_ * 180 / _Pi)) > 355)
            {
                FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                rotate_flag = 0;
                return true;
            }
            else
                return false;
        }
        else
        {
            if (!rotate_flag)
            {
                if (!relatAbs)
                {
                    rotate_aimforward = (360 - gyo_angle_ * 180 / _Pi) + angle;
                    if (rotate_aimforward >= 360)
                    {
                        rotate_aimforward = rotate_aimforward - 360;
                    }
                }
                else
                {
                    rotate_aimforward = angle;
                }
                rotate_flag = 1;
            }
            FRIZY_LOG(LOG_INFO, "aimforward:%f, gyo_angle_:%f", rotate_aimforward, (360 - gyo_angle_ * 180 / _Pi));
            chassisEscape.chassisSpeed(speedL, speedR, 1);
            // chassisEscape.GetSensor(&escapSensor);
            if (fabs(rotate_aimforward - (360 - gyo_angle_ * 180 / _Pi)) < 5 || fabs(rotate_aimforward - (360 - gyo_angle_ * 180 / _Pi)) > 355)
            {
                FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                chassisEscape.chassisSpeed(0, 0, 1);
                rotate_flag = 0;
                return true;
            }
            else
                return false;
        }
    }
    // //自旋                                                  //0:相对角度 1:绝对角度
    // bool EscapePlan::escSpin(int speed, int dir, float angle, int relatAbs)
    // {
    //     static float spin_aimforward = 0.0f;
    //     float nowAngle = escapSensor.addAngle;
    //     if(!dir)
    //     {
    //         if(!spin_flag)
    //         {
    //             if(!relatAbs)
    //             {
    //                 // spin_aimforward = (360-gyo_angle_*180/_Pi) - angle;
    //                 // if(spin_aimforward <= 0)
    //                 // {
    //                 //     spin_aimforward = 360 + spin_aimforward;
    //                 // }
    //                 spin_aimforward = nowAngle - angle;
    //             }
    //             else
    //             {
    //                 spin_aimforward = angle;
    //             }
    //             spin_flag = 1;
    //         }

    //         // FRIZY_LOG(LOG_INFO,"spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, (360-gyo_angle_*180/_Pi));
    //         FRIZY_LOG(LOG_INFO,"spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, nowAngle);
    //         chassisEscape.chassisSpeed(-speed,speed,1);
    //         // if(fabs(spin_aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(spin_aimforward - (360-gyo_angle_*180/_Pi)) > 350)
    //         if(fabs(spin_aimforward - (nowAngle)) < 10)
    //         {
    //             FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
    //             chassisEscape.chassisSpeed(0, 0, 1);
    //             spin_flag = 0;
    //             return true;
    //         }

    //     }
    //     else
    //     {
    //         if(!spin_flag)
    //         {
    //             if(!relatAbs)
    //             {
    //                 // spin_aimforward = (360-gyo_angle_*180/_Pi) + angle;
    //                 // if(spin_aimforward >= 360)
    //                 // {
    //                 //     spin_aimforward = spin_aimforward - 360;
    //                 // }
    //                 spin_aimforward = nowAngle + angle;
    //             }
    //             else
    //             {
    //                 spin_aimforward = angle;
    //             }
    //             spin_flag = 1;
    //         }
    //         // FRIZY_LOG(LOG_INFO,"spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, (360-gyo_angle_*180/_Pi));
    //         FRIZY_LOG(LOG_INFO,"spin_aimforward:%f, gyo_angle_:%f", spin_aimforward, nowAngle);
    //         chassisEscape.chassisSpeed(-speed,speed,1);
    //         // if(fabs(spin_aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(spin_aimforward - (360-gyo_angle_*180/_Pi)) > 350)
    //         if(fabs(spin_aimforward - (nowAngle)) < 10)
    //         {
    //             FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
    //             chassisEscape.chassisSpeed(0, 0, 1);
    //             spin_flag = 0;
    //             return true;
    //         }
    //     }
    //     return false;
    // }
    // //单边旋                                                          //0:相对角度 1:绝对角度
    // bool EscapePlan::singleRotate(int speedL, int speedR, float angle, int relatAbs)
    // {
    //     static float rotate_aimforward = 0.0f;
    //     float nowAngle = escapSensor.addAngle;
    //     if(abs(speedL) > abs(speedR))
    //     {
    //         if(!rotate_flag)
    //         {
    //             // if(!relatAbs)
    //             // {
    //             //     rotate_aimforward = (360-gyo_angle_*180/_Pi) - angle;
    //             //     if(rotate_aimforward <= 0)
    //             //     {
    //             //         rotate_aimforward = 360 + rotate_aimforward;
    //             //     }
    //             // }
    //             if(!relatAbs)
    //             {
    //                 // rotate_aimforward = (360-gyo_angle_*180/_Pi) + angle;
    //                 // FRIZY_LOG(LOG_INFO,"rotate_aimforward:%f, gyo_angle_:%f", rotate_aimforward, (360-gyo_angle_*180/_Pi));
    //                 // if(rotate_aimforward >= 360)
    //                 // {
    //                 //     rotate_aimforward = rotate_aimforward - 360;
    //                 // }
    //                 rotate_aimforward = nowAngle + angle;
    //             }
    //             else
    //             {
    //                 rotate_aimforward = angle;
    //             }
    //             rotate_flag = 1;
    //         }
    //         // FRIZY_LOG(LOG_INFO,"rotate_aimforward:%f, gyo_angle_:%f", rotate_aimforward, (360-gyo_angle_*180/_Pi));
    //         FRIZY_LOG(LOG_INFO,"rotate_aimforward:%f, gyo_angle_:%f", rotate_aimforward, nowAngle);
    //         // if(fabs(rotate_aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(rotate_aimforward - (360-gyo_angle_*180/_Pi)) > 350)
    //         chassisEscape.chassisSpeed(speedL, speedR, 1);
    //         if(fabs(rotate_aimforward - nowAngle) < 10)
    //         {
    //             FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
    //             chassisEscape.chassisSpeed(0, 0, 1);
    //             rotate_flag = 0;
    //             return true;
    //         }
    //     }
    //     else
    //     {
    //         if(!rotate_flag)
    //         {
    //             // if(!relatAbs)
    //             // {
    //             //     rotate_aimforward = (360-gyo_angle_*180/_Pi) + angle;
    //             //     if(rotate_aimforward >= 360)
    //             //     {
    //             //         rotate_aimforward = rotate_aimforward - 360;
    //             //     }
    //             // }
    //             if(!relatAbs)
    //             {
    //                 // rotate_aimforward = (360-gyo_angle_*180/_Pi) - angle;
    //                 // if(rotate_aimforward <= 0)
    //                 // {
    //                 //     rotate_aimforward = 360 + rotate_aimforward;
    //                 // }
    //                 rotate_aimforward = nowAngle - angle;
    //             }
    //             else
    //             {
    //                 rotate_aimforward = angle;
    //             }
    //             rotate_flag = 1;
    //         }
    //         FRIZY_LOG(LOG_INFO,"aimforward:%f, gyo_angle_:%f", rotate_aimforward, (360-gyo_angle_*180/_Pi));
    //         FRIZY_LOG(LOG_INFO,"rotate_aimforward:%f, gyo_angle_:%f", rotate_aimforward, nowAngle);
    //         chassisEscape.chassisSpeed(speedL, speedR, 1);
    //         // if(fabs(rotate_aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(rotate_aimforward - (360-gyo_angle_*180/_Pi)) > 350)
    //         if(fabs(rotate_aimforward - nowAngle) < 10)
    //         {
    //             FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
    //             chassisEscape.chassisSpeed(0, 0, 1);
    //             rotate_flag = 0;
    //             return true;
    //         }
    //     }
    //     return false;
    // }
    #define UPHILL_DATA_NUM (1500/100)
    static long long  escUpdataTime = 0;
    //用于记录记录10s前的机器姿态
    int EscapePlan::recordPitRol_10s(float val)
    {
        
 
        float tmpPitEsc = getPitRol(0);//俯仰
        float tmpRolEsc =  getPitRol(1);//横滚
        
        float retTmpGet = 0.0f;
        // driveAdcData_t escAdc;
        long long timeDiff = 0;
        int cliffF = escapSensor.midLeftCliffValue < escapSensor.midRightCliffValue ? escapSensor.midLeftCliffValue : escapSensor.midRightCliffValue;
        if(0 == escUpdataTime)//开始运行时记录一次
        {
            
            // DRIV_GetAdcData(&escAdc);s
            escActPara.saveUphillCnt = 0;
            escActPara.escBasePitRol = val;
            escActPara.escBasePit = tmpPitEsc;
            escActPara.escBaseRol = tmpRolEsc;
            escActPara.escCiffFBase = cliffF;
            escActPara.escCiffLBase = escapSensor.leftCliffValue;
            escActPara.escCiffRBase = escapSensor.rightCliffValue;
        }
        
        //每10s记录一次机器姿态
        timeDiff = getTime() - escUpdataTime;
    
        if(timeDiff>=100)
        {
            // DRIV_GetAdcData(&escAdc);
            escUpdataTime = getTime();
            
            if(fabsf(val-escActPara.escBasePitRol)>0.5f)
            {

            }
            else
            {
                escActPara.escBasePitRol = val;
            }
            
            if(fabsf(tmpPitEsc-escActPara.escBasePit)>0.6f)
            {

            }
            else
            {
                escActPara.escBasePit = tmpPitEsc;
            }
            
            if(fabsf(tmpRolEsc-escActPara.escBaseRol)>0.6f)
            {

            }
            else
            {
                escActPara.escBaseRol = tmpRolEsc;
            }
            
            int16_t diffFTmp = 0;
            int16_t diffLTmp = 0;
            int16_t diffRTmp = 0;
    
            diffFTmp = cliffF-escActPara.escCiffFBase;
            diffRTmp = escapSensor.rightCliffValue-escActPara.escCiffRBase;
            diffLTmp = escapSensor.leftCliffValue-escActPara.escCiffLBase;
            
            
            //检测上坡 下坡并更正基础值
            if((escActPara.escBasePit - tmpPitEsc)< -3.0f || (escActPara.escBasePit - tmpPitEsc) > 3.0f)
            {
                FRIZY_LOG(LOG_DEBUG, "uphill %d %d %d %d",diffFTmp,diffLTmp,diffRTmp,escActPara.saveUphillCnt);
                if(abs((diffFTmp - diffRTmp))<100 && abs((diffFTmp-diffLTmp))<100)//平面
                {
                    if(escActPara.saveUphillCnt>=UPHILL_DATA_NUM)
                    {
                        if(escActPara.parkTubeCnt>0)
                            escActPara.parkTubeCnt--;
                        escActPara.saveUphillCnt = 0;
                        escActPara.escBasePitRol = val;
                        escActPara.escBasePit = tmpPitEsc;
                        escActPara.escBaseRol = tmpRolEsc;
                    }
                    else
                    {
                        escActPara.saveUphillCnt++;
                    }
                }
                else
                {
                    escActPara.saveUphillCnt = 0;
                }
                
                if((escActPara.escBasePit - tmpPitEsc) < -3.0f)
                {
                    //识别上U椅或者圆管
                    if(abs(diffFTmp - diffRTmp)>1000 || abs(diffFTmp-diffLTmp)<1000)
                    {
                        escActPara.parkTubeCnt++;
                    }
                }
            }
            else
            {
                escActPara.saveUphillCnt = 0;
            }
            
            //更正探地基础值
            if((diffFTmp - diffRTmp)<150 && (diffFTmp-diffLTmp)<150
                && cliffF>3800 && escapSensor.leftCliffValue>3800  && escapSensor.rightCliffValue>3800 )
            {
                escActPara.escCiffFBase = cliffF;
                escActPara.escCiffLBase = escapSensor.leftCliffValue;
                escActPara.escCiffRBase = escapSensor.rightCliffValue;
            }
            
            
            if(abs(cliffF-escActPara.escCiffFBase)>1000)
            {

            }
            else
            {
                escActPara.escCiffFBase = cliffF;
            }
            if(abs(escapSensor.rightCliffValue-escActPara.escCiffRBase)>1000)
            {

            }
            else
            {
                escActPara.escCiffRBase = escapSensor.rightCliffValue;
            }
            if(abs(escapSensor.leftCliffValue-escActPara.escCiffLBase)>1000)
            {

            }
            else
            {
                escActPara.escCiffLBase = escapSensor.leftCliffValue;
            }
        }
    #if 0
        static float savePitRol = 0.0f;
        static float savePit = 0.0f;
        static float saveRol = 0.0f;
        float tmpRec = 0.0f;
        float tmp = 0.0f;
        float tmpPitEsc = getPitRol(0); //俯仰
        float tmpRolEsc = getPitRol(1); //横滚

        if (record_time == 0) //开始运行时记录一次
        {
            escActPara.escBasePit = tmpPitEsc;
            escActPara.escBaseRol = tmpRolEsc;
            escActPara.escBasePitRol = val;
            escActPara.escCiffFBase = cliffF;
            escActPara.escCiffLBase = escapSensor.leftCliffValue;
            escActPara.escCiffRBase = escapSensor.rightCliffValue;
            savePit = tmpPitEsc;
            saveRol = tmpRolEsc;
            savePitRol = val;
            // escActPara.escBasePitRol = sqrtf(escapSensor.XAngle * escapSensor.XAngle + escapSensor.YAngle * escapSensor.YAngle);
            FRIZY_LOG(LOG_DEBUG, "0s pre pit:%f,rol:%f,pitrol:%f", escActPara.escBasePit, escActPara.escBaseRol, escActPara.escBasePitRol);
        }

        static float lastTimePitVal = 0.0f;
        static float lastTimeRolVal = 0.0f;

        if (getTime() - record_time >= 1500)
        {
            if (fabsf(tmpPitEsc - lastTimePitVal) > 1.0f)
            {
                escActPara.escBasePit = lastTimePitVal;
                if (escActPara.escapPrePitChangCnt >= 7)
                {
                    escActPara.escapPrePitChangCnt = 0;
                    lastTimePitVal = tmpPitEsc;
                }
                else
                {
                    escActPara.escapPrePitChangCnt++;
                }
            }
            else
            {
                escActPara.escBasePit = tmpPitEsc;
                lastTimePitVal = tmpPitEsc;
                escActPara.escapPrePitChangCnt = 0;
            }
            if (fabsf(tmpRolEsc - lastTimeRolVal) > 1.0f)
            {
                escActPara.escBaseRol = lastTimeRolVal;
                if (escActPara.escapPreRolChangCnt >= 7)
                {
                    escActPara.escapPreRolChangCnt = 0;
                    lastTimeRolVal = tmpRolEsc;
                }
                else
                {
                    escActPara.escapPreRolChangCnt++;
                }
            }
            else
            {
                escActPara.escBaseRol = tmpRolEsc;
                lastTimeRolVal = tmpRolEsc;
                escActPara.escapPreRolChangCnt = 0;
            }

            if ((0 == escActPara.escapPrePitChangCnt) &&
                (0 == escActPara.escapPreRolChangCnt))
            {
                escActPara.escBasePitRol = val;
            }

            FRIZY_LOG(LOG_DEBUG, "record pit=%0.1f,rol=%0.1f[%0.1f,%0.1f]%0.1f,cnt=%d,%d", tmpPitEsc, tmpRolEsc, escActPara.escBasePit, escActPara.escBaseRol, escActPara.escBasePitRol, escActPara.escapPrePitChangCnt, escActPara.escapPreRolChangCnt);
            record_time = getTime();
        }
    #endif
    }

    // 脱困获取相对变化俯仰横滚角度
    float EscapePlan::getMEMSRelatAngle(int index)
    {
        float temp_pit, temp_rol;
        if (index == 0)
        {
            temp_pit = getPitRol(0);
            // FRIZY_LOG(LOG_DEBUG, "current pit:%f,10s pre pit:%f", temp_pit, escActPara.escBasePit);
            // FRIZY_LOG(LOG_DEBUG, "getMEMSRelatAngle[0]%f", fabsf(temp_pit- escActPara.escBasePit));
            return (temp_pit - escActPara.escBasePit);
        }
        else if (index == 1)
        {
            temp_rol = getPitRol(1);
            // FRIZY_LOG(LOG_DEBUG, "current rol:%f,10s pre rol:%f", temp_rol, escActPara.escBaseRol);
            // FRIZY_LOG(LOG_DEBUG, "getMEMSRelatAngle[1]%f", fabsf(temp_rol - escActPara.escBaseRol));
            return (temp_rol - escActPara.escBaseRol);
        }
        else
            return 0.0f;
    }
    
    //进入脱困前和执行脱困动作后的差值
    float EscapePlan::getMEMSActChangAngle(int index)
    {
        if (index == 0)
        {
            // FRIZY_LOG(LOG_DEBUG, "current pit:%f, before escape pit:%f", getPitRol(0), escActPara.escActionPrePit);
            // FRIZY_LOG(LOG_DEBUG, "getMEMSActChangAngle[0]%f", fabsf(getPitRol(0) - escActPara.escActionPrePit));
            return getPitRol(0) - escActPara.escActionPrePit;
        }
        else if (index == 1)
        {
            // FRIZY_LOG(LOG_DEBUG, "current rol:%f, before escape rol:%f", getPitRol(1), escActPara.escActionPreRol);
            // FRIZY_LOG(LOG_DEBUG, "getMEMSActChangAngle[1]%f", fabsf(getPitRol(1) - escActPara.escActionPreRol));
            return getPitRol(1) - escActPara.escActionPreRol;
        }
        else
            return 0.0f;
    }

    // 针对架空脱困和尾部翘起卡死脱困，如果数据正常直接切入脱困
    bool EscapePlan::escActStepBehind()
    {
        static int8_t pitRolBCnt = 0;
        if (escActPara.escPreTypeRec & 0x40) //机器前面卡住，后面翘起(欧式家具卡)
        {
            if (fabsf(getMEMSRelatAngle(0)) < 1.5f && fabsf(getMEMSRelatAngle(1)) < 1.5f)
            {
                if (++pitRolBCnt > 100 / 20)
                {
                    pitRolBCnt = 0;
                    FRIZY_LOG(LOG_DEBUG, "escPreTypeRec 0x40 ok");
                    return true;
                }
                else
                    pitRolBCnt = 0;
            }
        }
        else if (escActPara.escPreTypeRec & 0x20) //机器前面翘起(风扇座架起)
        {
            if (fabsf(getMEMSRelatAngle(0)) < 1.5f && fabsf(getMEMSRelatAngle(1)) < 1.5f)
            {
                if (++pitRolBCnt > 100 / 20)
                {
                    FRIZY_LOG(LOG_DEBUG, "escPreTypeRec 0x20 ok");
                    pitRolBCnt = 0;
                    return true;
                }
            }
            else
            {
                pitRolBCnt = 0;
            }
        }
        //监控进入脱困后机器移动的角度
        int16_t angleChangTme = 0;
        angleChangTme = abs(enterAddAngle - getAddAngle());
        // FRIZY_LOG(LOG_DEBUG, "angleChangTme:%d, escAngleMaxChange:%d", angleChangTme, escActPara.escAngleMaxChange);
        if (angleChangTme > escActPara.escAngleMaxChange)
        {
            escActPara.escAngleMaxChange = angleChangTme;
        }
        return false;
    }

    //执行脱困动作前对陀螺仪数据分析
    //分析是哪种卡住，进入不同的脱困动作
    void EscapePlan::escStepPreGyroAnal()
    {
        // FRIZY_LOG(LOG_DEBUG,"into escStepPreGyroAnal");
        static int8_t escPreStopTime = 0;
        float tmpEscPrePR = 0.0f;
        if (WHEELSTOP == chassisEscape.getWheelState())
        {
            if (++escPreStopTime > 10)
            {
                escPreStopTime = 0;
                if (escActPara.escInterval > 8000) //进入脱困的时间大于8s
                {
                    escEnterIntervalCnt = 0;
                    escActPara.escActStepRecord = ESC_STEP_0;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_DEBUG, "esc > 8s");
                }
                else //进入脱困的时间小于8s   短时进入脱困
                {
                    escEnterIntervalCnt++;
                    escActPara.escActStepRecord = ESC_STEP_1;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_DEBUG, "esc < 8s");
                }
                if (escEnterIntervalCnt > 8) //短时进入脱困超过8次，直接判定为脱困失败
                {
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_INFO, "escEnterIntervalCnt:%d", escEnterIntervalCnt);
                }
                escActPara.escStopPrePitRol = gyroPitRolSlidingAvg(); //记录机器陀螺仪姿态
                escActPara.escActionPrePit = getPitRol(0);            //俯仰
                escActPara.escActionPreRol = getPitRol(1);            //横滚
                FRIZY_LOG(LOG_DEBUG, "before act pit:%f,rol:%f,pitrol:%f", escActPara.escActionPrePit, escActPara.escActionPreRol, escActPara.escStopPrePitRol);
                //是靠墙动作进入的脱困，并且姿态相差较大，这里判定为上了风扇座或者U型椅子的脚
                tmpEscPrePR = fabsf(escActPara.escStopPrePitRol - escActPara.escBasePitRol); //当前姿态与卡住前10s的机器姿态
                FRIZY_LOG(LOG_DEBUG, "tmpEscPrePR:%f = escStopPrePitRol:%f - escBasePitRol:%f", tmpEscPrePR, escActPara.escStopPrePitRol, escActPara.escBasePitRol);
                FRIZY_LOG(LOG_DEBUG, "10s esc pit:%f,rol:%f,pitrol:%f", escActPara.escBasePit, escActPara.escBaseRol, escActPara.escBasePitRol);
                if ((tmpEscPrePR > 3.5f || fabsf(getMEMSRelatAngle(0)) > 3.0f || fabsf(getMEMSRelatAngle(1)) > 3.0f) && stuckType != ESCAPE_EVENT_TYPE_LIDAR_STUCK)
                {
                    if (getMEMSRelatAngle(0) < -3.5f) //俯仰，前面翘起
                    {
                        escActPara.escPreTypeRec = 0x20;
                        FRIZY_LOG(LOG_DEBUG, "escActPara.escPreTypeRec = 0x20");
                    }
                    else if (getMEMSRelatAngle(0) > 3.5f) //俯仰，前面压下
                    {
                        if (getMEMSRelatAngle(1) >= 3.5f) //右前被压下
                        {
                            escActPara.escPreTypeRec = 0x41;
                            FRIZY_LOG(LOG_DEBUG, "escActPara.escPreTypeRec = 0x41");
                        }
                        else if (getMEMSRelatAngle(1) <= -3.5f) //左前被压下
                        {
                            escActPara.escPreTypeRec = 0x42;
                            FRIZY_LOG(LOG_DEBUG, "escActPara.escPreTypeRec = 0x42");
                        }
                        else //前被压下
                        {
                            escActPara.escPreTypeRec = 0x40;
                            FRIZY_LOG(LOG_DEBUG, "escActPara.escPreTypeRec = 0x40");
                        }
                    }
                    // saveEnterEscapMode =  TR_BASIS_PICTCH;
                }
                if (escActPara.escPreTypeRec & 0x20 || stuckType == ESCAPE_EVENT_TYPE_LEFT_OFF) //架空脱困
                {
                    FRIZY_LOG(LOG_DEBUG, "ESCAPE TYEP : ESCAPE_EVENT_TYPE_LEFT_OFF");
                    escActPara.escActStepRecord = ESC_STEP_OVERHEAD; //先尝试架空脱困
                    escActPara.enterStep0PreStep = ESCACT_START;
                }
                else if (escActPara.escPreTypeRec & 0x40 || stuckType == ESCAPE_EVENT_TYPE_STUCK) //底部卡住
                {
                    FRIZY_LOG(LOG_DEBUG, "ESCAPE TYEP : ESC_STEP_STUCK_HEAD");
                    escActPara.escActStepRecord = ESC_STEP_STUCK_HEAD;
                    escActPara.enterStep0PreStep = ESCACT_START;
                }
                else if (stuckType == ESCAPE_EVENT_TYPE_LIDAR_STUCK)
                {
                    FRIZY_LOG(LOG_DEBUG, "ESCAPE TYEP : ESC_STEP_LIDAR");
                    escActPara.escActStepRecord = ESC_STEP_LIDAR; //雷达遮挡脱困
                    escActPara.enterStep0PreStep = ESCACT_START;
                }
            }
            else if (escPreStopTime == 5)
            {
                FRIZY_LOG(LOG_INFO, "sliding average:%f,before stick 10s:%f", escActPara.escStopPrePitRol, escActPara.escBasePitRol);
            }
        }
        else
        {
            if (getTime() - escActPara.escActExecuTime >= 2000)
            {
                FRIZY_LOG(LOG_DEBUG, "TIME OUT");
                chassisEscape.chassisSpeed(0, 0, 1);
                act = 32;
            }
            escPreStopTime = 0;
        }
    }

    //记录脱困前的动作
    void EscapePlan::recordActBeforeEsc()
    {
        int wheelSta = WHEELSTOP;
        wheelSta = chassisEscape.getWheelState();
        if (wheelSta != WHEELSTOP)
            SlidingArrayAddI(escPreActionArry, 3, wheelSta);
    }

    /*
    * 脱困步骤0 后退
    */
    void EscapePlan::escActStep_0()
    {
        static int8_t escBeStep3Cnt = 0;
        switch (escActPara.escActState)
        {
        case 0: //1后退
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                //如果是动作有效，进入后退动作
                if (1 == escActPara.enterStep0Sta)
                {
                    act = 13;
                    // wheelBackDist(-200, 80);//正常的速度后退
                    escActPara.escUnmovableRateCnt = 0;
                }
                else
                {
                    if (escActPara.enterStep0PreStep == ESC_STEP_ABERRANT)
                    {
                        // wheelBackDist(-200, 80);//正常的速度后退
                        act = 13;
                    }
                    else if (escActPara.enterStep0PreStep == ESC_STEP_3)
                    {
                        act = 14;
                        // wheelCtrlStraight(-300, 800);//后退
                        escBeStep3Cnt++;
                    }
                    else if (escActPara.enterStep0PreStep == ESC_STEP_1 || escActPara.enterStep0PreStep == ESC_STEP_2) //由第一二步到这里的
                    {
                        act = 15;
                        // wheelCtrlStraight(-240, 800);//后退
                    }
                    else
                    {
                        escBeStep3Cnt = 0;
                        // wheelBackDist(-200, 80);//正常的速度后退
                        act = 13;
                    }
                }
                escActPara.enterStep0Sta = 0;
                escActPara.escActState = ESCACT_BEHIND_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                {
                    FRIZY_LOG(LOG_DEBUG, "TIME OUT");
                    chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
        }
        break;

        case ESCACT_BEHIND_WAIT: //2 后退等待
        {
            if (-1 == escActPara.actionRetDelay || (WHEELSTOP == chassisEscape.getWheelState())) //后退动作无效
            {
                if (escActPara.enterStep0PreStep == ESC_STEP_3) //由第三步到这里的
                {
                    escActPara.escActStepRecord = ESC_STEP_4;
                }
                else if (escActPara.enterStep0PreStep == ESC_STEP_2) //由第二步到这里的
                {
                    escActPara.escActStepRecord = ESC_STEP_3;
                }
                else if (escActPara.enterStep0PreStep == ESC_STEP_1) //由第一步到这里的
                {
                    escActPara.escActStepRecord = ESC_STEP_2;
                }
                else
                {
                    escActPara.escActStepRecord = ESC_STEP_1;
                }
                if (escActPara.escActValidCnt >= 4) //动作3次都成功了，后退失败，也判定为脱困成功
                {
                    escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                    escActPara.enterSuccPreStep = ESC_STEP_1;

                    FRIZY_LOG(LOG_DEBUG, "ValidCnt>4 success");
                }
                escActPara.escActState = ESCACT_START;
            }
            else if (1 == escActPara.actionRetDelay) //后退动作有效
            {
                escActPara.enterSuccPreStep = ESC_STEP_0;
                escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
                escActPara.escActState = ESCACT_START;
            }
            if (escBeStep3Cnt >= 3)
            {
                escBeStep3Cnt = 0;
                FRIZY_LOG(LOG_DEBUG, "escBeStep3Cnt>3 faild\n");
            }
        }
        break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*
    * 动作1之后的判定中间操作
    * cnt 动作次数
    * lastAct 上一个动作 
    */
    void EscapePlan::escActStep1WaitTmp(int cnt, int lastAct)
    {
        if (escActPara.actionRetDelay == -1) //自旋无效
        {
            if (getTime() - escActPara.escActExecuTime >= 1500)
            {
                if (cnt <= 1) //尝试自旋两次
                {
                    escActPara.escActState = (escapeAction_t)lastAct;
                    chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
                else //自旋两次都没有成功
                {
                    escActPara.escActStepRecord = ESC_STEP_2; //进入脱困第二步骤
                    escActPara.escActState = ESCACT_START;
                }
            }
        }
        else if (escActPara.actionRetDelay == 1 || chassisEscape.getWheelState() == WHEELSTOP)
        {
            escActPara.escActStepRecord = ESC_STEP_0; //切入后退
            escActPara.enterStep0PreStep = ESC_STEP_1;
            escActPara.escActState = ESCACT_START;
            escActPara.enterStep0Sta = 1;
            escActPara.escActValidCnt++;
        }
    }

    /*          
    * 脱困步骤1 根据脱困前的动作进行尝试动作
    * actResu：动作的结果 -1表示无效动作 1表示有效动作
    */
    void EscapePlan::escActStep_1()
    {
        switch (escActPara.escActState)
        {
        case 0:
        {
            //根据脱困前的动作来决定之后的动作
            if (escPreActionArry[2] == WHEELLEFTSPIN)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if (escPreActionArry[2] == WHEELRIGHTSPIN)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if (escPreActionArry[2] == WHEELFRONT)
            {
                if (RIGHTAW == getAlongWallDir()) //之前是右沿墙
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
            }
            else //尝试左后 右后
            {
                escActPara.escActState = ESCACT_NUILROT_LEFT;
            }
        }
        break;

        case ESCACT_SPIN_RIGHT: //5右自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                // escSpin(300, 1, 30, 0);
                act = 16;
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
                escActPara.rightSpinCnt++;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                {
                    act = 32;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;

        case ESCACT_SPIN_RIGHT_WAIT: //6右自旋等待
        {
            escActStep1WaitTmp(escActPara.rightSpinCnt, ESCACT_SPIN_RIGHT);
        }
        break;

        case ESCACT_SPIN_LEFT: //3左自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                // escSpin(300, 0, 30, 0);
                act = 17;
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
                escActPara.leftSpinCnt++;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                {
                    act = 32;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;

        case ESCACT_SPIN_LEFT_WAIT: //4左自旋等待
        {
            escActStep1WaitTmp(escActPara.leftSpinCnt, ESCACT_SPIN_LEFT);
        }
        break;

        case ESCACT_NUILROT_LEFT: //7 左单边旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.leftBehindCnt++;
                // singleRotate(-300, -60, 30, 0);
                act = 18;
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                {
                    act = 32;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;

        case ESCACT_NUILROT_LEFT_WAIT: // 8左单边旋等待
        {
            escActStep1WaitTmp(escActPara.leftBehindCnt, ESCACT_NUILROT_LEFT);
        }
        break;

        case ESCACT_NUILROT_RIGHT: //右单边旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.leftBehindCnt++;
                // singleRotate(-60, -300, 30, 0);
                act = 19;
                escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                {
                    act = 32;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;

        case ESCACT_NUILROT_RIGHT_WAIT: //右单边旋等待
        {
            escActStep1WaitTmp(escActPara.rightBehindCnt, ESCACT_NUILROT_RIGHT);
        }
        break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*
    * 动作2 之后的判定中间操作
    * faildAct 失败之后要切换的动作
    */
    void EscapePlan::escActStep2WaitTmp(int failedAct)
    {
        int timeDiffStep2 = 0;
        int angleActDiffStep2 = 0;
        timeDiffStep2 = getTime() - escActPara.escActExecuTime;
        angleActDiffStep2 = abs(escActPara.escActPreAngle - getAddAngle()) / 10;
        if (-1 == escActPara.actionRetDelay || (timeDiffStep2 >= 2000 && angleActDiffStep2 < 5)) //动作无效
        {
            if (timeDiffStep2 >= 1500)
            {
                if (ESCACT_SPIN_LEFT == failedAct) //下一步发出的是左自旋指令
                {
                    if (escActPara.leftSpinCnt >= 1) //左自旋 动作次数计数
                    {
                        failedAct = ESCACT_NUILROT_LEFT;
                    }
                }
                else if (ESCACT_SPIN_RIGHT == failedAct) //下一步发出的是右自旋指令
                {
                    if (escActPara.rightSpinCnt >= 1) //右自旋 动作次数计数
                    {
                        failedAct = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if (ESCACT_NUILROT_LEFT == failedAct) //下一步发出的是左后旋指令
                {
                    if (escActPara.leftBehindCnt >= 1) //左后 动作次数计数
                    {
                        failedAct = ESCACT_SPIN_LEFT;
                    }
                }
                else if (ESCACT_NUILROT_RIGHT == failedAct) //下一步发出的是右后旋指令
                {
                    if (escActPara.rightBehindCnt >= 1) //右后 动作次数计数
                    {
                        failedAct = ESCACT_SPIN_RIGHT;
                    }
                }
                //所有动作都尝试了一次
                if (escActPara.leftSpinCnt && escActPara.rightSpinCnt && escActPara.leftBehindCnt && escActPara.rightBehindCnt)
                {
                    escActPara.escActStepRecord = ESC_STEP_0;
                    escActPara.escActState = ESCACT_START;
                    escActPara.enterStep0PreStep = ESC_STEP_2;
                    escActPara.enterStep0Sta = -1; //标记是动作失败切回的后退
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                    FRIZY_LOG(LOG_DEBUG, "escStep2 4 dir %d %d", escActPara.escUnmovableRateCnt, escActPara.escUnmovableCurCnt);
                }
                else
                {
                    escActPara.escActState = (escapeAction_t)failedAct;
                }
                escActPara.escActPreAngle = getAddAngle();
                FRIZY_LOG(LOG_DEBUG, "escActStep2 outTime->%d", escActPara.escActState);
            }
            escActPara.escActValidCnt = 0;
        }
        else if (escActPara.actionRetDelay == 1 || chassisEscape.getWheelState() == WHEELSTOP) //动作有效
        {
            FRIZY_LOG(LOG_DEBUG, "step2 enter succ,%d %d", escActPara.actionRetDelay, chassisEscape.getWheelState());
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;
            FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
            escActPara.enterSuccPreStep = ESC_STEP_2;
            escActPara.escActState = ESCACT_START;
            escActPara.escActValidCnt++;
            if (escActPara.stepContinuNum >= 4)
            {
                escActPara.escActStepRecord = ESC_STEP_0;
                escActPara.escActState = ESCACT_START;
                escActPara.enterStep0PreStep = ESC_STEP_2;
                escActPara.enterStep0Sta = -1; //标记是动作失败切回的后退
                escActPara.stepContinuNum = 0;
            }
            escActPara.stepContinuNum++;
        }
    }

    /*
    * 脱困步骤2 第一步动失败后，根据第一步的动作，换一个方向转动
    * actResu：动作的结果 -1表示无效动作 1表示有效动作
    */
    void EscapePlan::escActStep_2()
    {
        static int lastStep2Sta = -1;
        if (lastStep2Sta != escActPara.escActState)
            FRIZY_LOG(LOG_DEBUG, "escStep2[%d->%d]", lastStep2Sta, escActPara.escActState);
        lastStep2Sta = escActPara.escActState;
        switch (escActPara.escActState)
        {
        case 0:
        {
            if (0 != escActPara.rightSpinCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if (0 != escActPara.leftSpinCnt)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if (0 != escActPara.rightBehindCnt)
            {
                escActPara.escActState = ESCACT_NUILROT_LEFT;
            }
            else if (0 != escActPara.leftBehindCnt)
            {
                escActPara.escActState = ESCACT_NUILROT_RIGHT;
            }
            else
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }

            escActPara.leftSpinCnt = 0;    //左自旋 动作次数计数
            escActPara.rightSpinCnt = 0;   //右自旋 动作次数计数
            escActPara.leftBehindCnt = 0;  //左后 动作次数计数
            escActPara.rightBehindCnt = 0; //右后 动作次数计数
        }
        break;

        case ESCACT_SPIN_LEFT: //3 左自旋
        {
            escActPara.leftSpinCnt++;
            // escSpin(300, 0, 60, 0);
            act = 20;
            escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
        }
        break;
        case ESCACT_SPIN_LEFT_WAIT: //4 左自旋等待
        {
            escActStep2WaitTmp(ESCACT_NUILROT_RIGHT);
        }
        break;

        case ESCACT_SPIN_RIGHT: //5 右自旋
        {
            escActPara.rightSpinCnt++;
            // escSpin(300, 1, 60, 0);
            act = 21;
            escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
        }
        break;
        case ESCACT_SPIN_RIGHT_WAIT: //6 右自旋等待
        {
            escActStep2WaitTmp(ESCACT_NUILROT_LEFT);
        }
        break;

        case ESCACT_NUILROT_LEFT: //7 左单边旋
        {
            escActPara.leftBehindCnt++;
            // singleRotate(-400, -80, 35, 0);
            act = 22;
            escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
        }
        break;
        case ESCACT_NUILROT_LEFT_WAIT: //8 左单边旋等待
        {
            escActStep2WaitTmp(ESCACT_SPIN_RIGHT);
        }
        break;
        case ESCACT_NUILROT_RIGHT: //9 右单边旋
        {
            escActPara.rightBehindCnt++;
            // singleRotate(-80, -400, 35, 0);
            act = 23;
            escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
        }
        break;
        case ESCACT_NUILROT_RIGHT_WAIT: //10 右单边旋等待
        {
            escActStep2WaitTmp(ESCACT_SPIN_LEFT);
        }
        break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*
    * 动作3之后的判定中间操作
    * cnt 动作次数
    * lastAct 下一个动作 
    */
    void EscapePlan::escActStep3WaitTmp(int cnt, int lastAct)
    {
        if (-1 == escActPara.actionRetDelay) //自旋无效
        {
            //持续2s,切换动作
            if (getTime() - escActPara.escActExecuTime >= 2000)
            {
                if (cnt <= 1) //自旋尝试两次
                {
                    escActPara.escActState = (escapeAction_t)lastAct;
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
                else //自旋两次都没有成功
                {
                    if (escActPara.rightSpinCnt <= 1 || escActPara.leftSpinCnt <= 1)
                    {
                        if (lastAct == ESCACT_SPIN_LEFT)
                        {
                            if (escActPara.leftSpinCnt >= 2)
                            {
                                escActPara.escActState = ESCACT_SPIN_RIGHT;
                            }
                        }
                        else
                        {
                            if (escActPara.rightSpinCnt >= 2)
                            {
                                escActPara.escActState = ESCACT_SPIN_LEFT;
                            }
                        }
                    }
                    else
                    {
                        //进入第四步
                        escActPara.escActStepRecord = ESC_STEP_4;
                        escActPara.escActState = ESCACT_START;
                    }
                }
                FRIZY_LOG(LOG_DEBUG, "cntTmpStep3 %d %d\n", cnt, escActPara.escActState);
            }
        }
        else if (escActPara.actionRetDelay == 1 || chassisEscape.getWheelState() == WHEELSTOP)
        {
            escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
            FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
            escActPara.enterSuccPreStep = ESC_STEP_3;
            escActPara.escActState = ESCACT_START;
            if (escActPara.stepContinuNum >= 3)
            {
                //进入第四步
                escActPara.escActStepRecord = ESC_STEP_4;
                escActPara.escActState = ESCACT_START;
                escActPara.stepContinuNum = 0;
            }
            escActPara.stepContinuNum++;
            FRIZY_LOG(LOG_DEBUG, "Step3 enter back:%d %d", cnt, escActPara.escActState);
        }
    }

    /*
    * 脱困步骤3 左右大力自旋各两次
    */
    void EscapePlan::escActStep_3()
    {
        switch (escActPara.escActState)
        {
        case 0:
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                //左边角速度累加大于右边,这里就认为左边比右边可移动空间更大
                if (escActPara.leftAngVelAdd > escActPara.rightAngVelAdd)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                escActPara.leftSpinCnt = 0;    //左自旋 动作次数计数
                escActPara.rightSpinCnt = 0;   //右自旋 动作次数计数
                escActPara.leftBehindCnt = 0;  //左后 动作次数计数
                escActPara.rightBehindCnt = 0; //右后 动作次数计数
                escActPara.frontCnt = 0;
                escActPara.behindCnt = 0;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2000)
                {
                    act = 32;
                    // chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;

        case ESCACT_SPIN_LEFT: //3 左自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.leftSpinCnt++;
                // escSpin(500, 0, 30, 0);
                act = 24;
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2000)
                {
                    act = 32;
                    // chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        case ESCACT_SPIN_LEFT_WAIT: //4 左自旋等待
        {
            escActStep3WaitTmp(escActPara.leftSpinCnt, ESCACT_SPIN_LEFT);
        }

        case ESCACT_SPIN_RIGHT: //5 右自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.rightSpinCnt++;
                // escSpin(500, 1, 30, 0);
                act = 25;
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2000)
                {
                    act = 32;
                    // chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
        }
        break;
        case ESCACT_SPIN_RIGHT_WAIT: //6 右自旋等待
        {
            escActStep3WaitTmp(escActPara.rightSpinCnt, ESCACT_SPIN_RIGHT);
        }
        break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*
    * 动作4之后的判定中间操作 
    */
    void EscapePlan::escActStep4WaitTmp()
    {
        static int step4LastAct = 0;
        static int angleRateCnt = 0;
        static int step4ActWaitTime = 0;
        int timeExecuDiff = 0;
        int angleDiff = 0;
        int angleRate = 0;
        timeExecuDiff = getTime() - escActPara.escActExecuTime;
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle()) / 10;
        angleRate = getAngleRate();
        if (angleRate > 1500)
        {
            angleRateCnt++;
        }
        else
        {
            angleRateCnt = 0;
        }
        if (step4LastAct != ESCACT_BEHIND && step4LastAct != ESCACT_FRONT)
        {
            if (timeExecuDiff <= 1400 && timeExecuDiff >= 1000 && chassisEscape.getWheelState() == WHEELSTOP)
            {
                angleRateCnt = 100;
            }
        }
        if (angleRateCnt > 8 || (escActPara.actionRetDelay == 1))
        {
            angleRateCnt = 0;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;
            FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
            escActPara.enterSuccPreStep = ESC_STEP_4;
            escActPara.escActState = ESCACT_START;
            FRIZY_LOG(LOG_DEBUG, "step4 enter succ %d %d", escActPara.escActionTryCnt, timeExecuDiff);
            return;
        }
        if (chassisEscape.getWheelState() == WHEELSTOP)
        {
            if (0 == escActPara.escActionTryCnt)
            {
                angleRateCnt = 0;
                if (escActPara.escAngleMaxChange > 600)
                    escActPara.escActState = ESCACT_FRONT;
                else
                    escActPara.escActState = ESCACT_BEHIND;
            }
            else if (1 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if (2 == escActPara.escActionTryCnt)
            {
                if (angleDiff > 5 && angleDiff < 70)
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                else if (angleDiff >= 70)
                {
                    escActPara.escActState = ESCACT_FRONT;
                } //不需要考虑其他情况
            }
            else if (3 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if (4 == escActPara.escActionTryCnt)
            {
                if (angleDiff > 5 && angleDiff < 70)
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                else if (angleDiff >= 70)
                {
                    escActPara.escActState = ESCACT_FRONT;
                } //不需要考虑其他情况
            }
            else if (5 <= escActPara.escActionTryCnt && escActPara.escActionTryCnt <= 7) //左右摇摆
            {
                if ((escActPara.escActionTryCnt % 2) == 0)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }

                step4ActWaitTime = 500;
            }
            else if (8 == escActPara.escActionTryCnt) //后退
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if (9 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if (10 <= escActPara.escActionTryCnt && escActPara.escActionTryCnt <= 12) //左右摇摆
            {
                if ((escActPara.escActionTryCnt % 2) == 0)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }

                step4ActWaitTime = 500;
            }
            else if (13 == escActPara.escActionTryCnt) //后退
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if (14 == escActPara.escActionTryCnt) //尝试向右旋转
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
                step4ActWaitTime = 4000;
            }
            else if (15 == escActPara.escActionTryCnt)
            {
                if (angleDiff > 5 && angleDiff <= 50)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                    step4ActWaitTime = 3000;
                }
                else if (angleDiff > 50 && angleDiff <= 80)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if (angleDiff > 80)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
            }
            else if (16 == escActPara.escActionTryCnt) //尝试向左旋转
            {
                if (ESCACT_SPIN_RIGHT == step4LastAct) //第二次右转了
                {
                    if (angleDiff > 5 && angleDiff <= 50)
                    {
                        escActPara.escActState = ESCACT_SPIN_RIGHT;
                        step4ActWaitTime = 3000;
                    }
                    else if (angleDiff > 50 && angleDiff <= 80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else if (angleDiff > 80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT; //尝试向左旋转
                        step4ActWaitTime = 4000;
                    }
                }
                else if (ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_LEFT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT; //尝试向左旋转
                }
            }
            else if (17 == escActPara.escActionTryCnt)
            {
                if (ESCACT_SPIN_LEFT == step4LastAct)
                {
                    if (angleDiff > 5 && angleDiff <= 50)
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                        step4ActWaitTime = 3000;
                    }
                    else if (angleDiff > 50 && angleDiff <= 80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else if (angleDiff > 80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                }
                else if (ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_RIGHT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
            else if (18 == escActPara.escActionTryCnt)
            {
                if (ESCACT_SPIN_LEFT == step4LastAct)
                {
                    if (angleDiff > 5 && angleDiff <= 50)
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                        step4ActWaitTime = 3000;
                    }
                    else if (angleDiff > 50 && angleDiff <= 80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else if (angleDiff > 80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                }
                else if (ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_RIGHT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
            else
            {
                escActPara.escActionTryCnt = 0;

                if (escActPara.escUnmovableRateCnt > 1100 || escActPara.escUnmovableCurCnt > 900 || escActPara.escAngleMaxChange < 300)
                {
                    FRIZY_LOG(LOG_DEBUG, "MachUnVable %d %d %d", escActPara.escUnmovableRateCnt, escActPara.escUnmovableCurCnt, escActPara.escAngleMaxChange);
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                }
                else if (escActPara.stepContinuNum >= 3)
                {
                    escActPara.escActStepRecord = ESC_STEP_1;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    escActPara.stepContinuNum = 0;
                    FRIZY_LOG(LOG_DEBUG, "step4->step1");
                }
                escActPara.stepContinuNum++;
            }
            step4LastAct = (escapeAction_t)escActPara.escActState;
            escActPara.escActionTryCnt++;
            FRIZY_LOG(LOG_DEBUG, "step4 try=%d %d", escActPara.escActionTryCnt, escActPara.escActState);
            escActPara.escActPreAngle = getAddAngle();
        }
        else
        {
            if (step4ActWaitTime < 450)
                step4ActWaitTime = 3000;

            if (timeExecuDiff >= step4ActWaitTime)
            {
                // chassisEscape.chassisSpeed(0, 0, 1);
                act = 32;
            }
        }
    }

    /*
    * 脱困步骤4
    * 前进，然后快速摆动
    */
    void EscapePlan::escActStep_4()
    {
        switch (escActPara.escActState)
        {
        case ESCACT_START:
        {
            escActPara.frontCnt = 0;        //前进 动作次数计数
            escActPara.behindCnt = 0;       //后退 动作次数计数
            escActPara.leftSpinCnt = 0;     //左自旋 动作次数计数
            escActPara.rightSpinCnt = 0;    //右自旋 动作次数计数
            escActPara.leftBehindCnt = 0;   //左后 动作次数计数
            escActPara.rightBehindCnt = 0;  //右后 动作次数计数
            escActPara.escActionTryCnt = 0; //动作尝试次数计数
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_NUILROT_LEFT: //左单边旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.leftBehindCnt++;
                // singleRotate(-400, -100, 50, 0);
                act = 26;
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
        }
        break;
        case ESCACT_NUILROT_LEFT_WAIT: //左单边旋等待
        {
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_NUILROT_RIGHT: //右单边旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.rightBehindCnt++;
                // singleRotate(-100, -400, 50, 0);
                act = 27;
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
        }
        break;
        case ESCACT_NUILROT_RIGHT_WAIT: //右单边旋等待
        {
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_SPIN_RIGHT: //5 右自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.rightSpinCnt++;
                // escSpin(500, 1, 90, 0);
                act = 29;
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
            }
            if (getTime() - escActPara.escActExecuTime >= 2500)
            {
                // chassisEscape.chassisSpeed(0, 0, 1);
                act = 32;
            }
        }
        break;
        case ESCACT_SPIN_RIGHT_WAIT: //6 右自旋等待
        {
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_SPIN_LEFT: //3 左自旋
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.leftSpinCnt++;
                // escSpin(500, 0, 90, 0);
                act = 28;
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
            }
            if (getTime() - escActPara.escActExecuTime >= 2500)
            {
                // chassisEscape.chassisSpeed(0, 0, 1);
                act = 32;
            }
        }
        break;
        case ESCACT_SPIN_LEFT_WAIT: //4 左自旋等待
        {
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_FRONT: //14前进
        {
            escActPara.frontCnt++;
            // wheelCtrlStraight(160, 600);
            act = 30;
            escActPara.escActState = ESCACT_FRONT_WAIT;
        }
        break;
        case ESCACT_FRONT_WAIT: //15前进等待
        {
            escActStep4WaitTmp();
        }
        break;

        case ESCACT_BEHIND: //1 后退
        {
            escActPara.behindCnt++;
            // wheelBackDist(-400,80);//正常的速度后退
            act = 31;
            escActPara.escActState = ESCACT_BEHIND_WAIT;
        }
        break;
        case ESCACT_BEHIND_WAIT: //2 后退等待
        {
            escActStep4WaitTmp();
        }
        break;
        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*      
    *在架空脱困 卡死中监控机器的俯仰横滚数据     
    * 10成功下来 2x前 3x后 4x左 5x右 6x 中
    */
    int8_t EscapePlan::overheadCheck(float tmpDiff)
    {
        bool actChangGraFlag = false;
        int8_t retTmp = 0;
        float currPitTmp = getPitRol(0);
        float currRolTmp = getPitRol(1);
        float overPit = getMEMSRelatAngle(0);
        float overRol = getMEMSRelatAngle(1);
        float actChangPitTmp = fabsf(getMEMSActChangAngle(0));
        float actChangRolTmp = fabsf(getMEMSActChangAngle(1));
        int addAngleError = abs((getAddAngle() / 10) - (enterAddAngle / 10));
        FRIZY_LOG(LOG_DEBUG, "tmpDiff:%f", tmpDiff);
        FRIZY_LOG(LOG_DEBUG, "addAngleError:%d", addAngleError);
        FRIZY_LOG(LOG_DEBUG, "currPitTmp:%f, currRolTmp:%f", currPitTmp, currRolTmp);
        FRIZY_LOG(LOG_DEBUG, "overPit:%f, overRol:%f", overPit, overRol);
        FRIZY_LOG(LOG_DEBUG, "actChangPitTmp:%f, actChangRolTmp:%f", actChangPitTmp, actChangRolTmp);
        FRIZY_LOG(LOG_DEBUG, "cliff:%d", escapSensor.cliff);
        //俯仰和衡滚变化角度大
        if (actChangPitTmp >= 5.0f || actChangRolTmp >= 5.0f)
        {
            actChangGraFlag = true;
            FRIZY_LOG(LOG_DEBUG, "actChangGraFlag = true");
        }
        // 判定为架空脱成功了
        if (((tmpDiff < 2.5f && fabsf(overPit) < 3.5f && fabsf(overRol) < 3.5f && actChangGraFlag) || (tmpDiff < 1.5f && fabsf(overPit) < 1.5f && fabsf(overRol) < 1.5f) || (actChangGraFlag && (fabsf(currPitTmp) < 3.2f && fabsf(currRolTmp) < 3.2f)) || (1 == escActPara.groundPeneDiff && actChangGraFlag)))
        {
            retTmp = 10;
            FRIZY_LOG(LOG_DEBUG, "overhead escape sucess!");
        }
        else
        {
            if (overPit < -5.0f) //前面翘起
            {
                retTmp = 21;
            }
            else if (overPit > 5.0f) //后面翘起
            {
                retTmp = 31;
            }
            else if (overRol > 5.0f) //左边翘起
            {
                retTmp = 41;
            }
            else if (overRol < -5.0f) //右边翘起
            {
                retTmp = 51;
            }
            else
            {
                if (fabsf(overRol) - fabsf(overPit) > 1.5f) //偏左右
                {
                    if (overRol) //偏左
                    {
                        retTmp = 42;
                    }
                    else //偏右
                    {
                        retTmp = 52;
                    }
                }
                else if (fabsf(overPit) - fabsf(overRol) > 1.5f) //偏前后
                {
                    if (overPit > 0) //偏后
                    {
                        retTmp = 32;
                    }
                    else //偏前
                    {
                        retTmp = 22;
                    }
                }

                else
                {
                    retTmp = 60;
                }
            }
        }
        // FRIZY_LOG(LOG_DEBUG, "retTmp:%d", retTmp);
        return retTmp;
    }
    static int8_t leftTryOverhCnt = 0, rightTryOverhCnt = 0;
    static int8_t frontTryOverhCnt = 0, behindTryOverhCnt = 0, interTryOverhCnt = 0;

    //架空做完动作等待判断,确定下一步动作
    int EscapePlan::judgeActionOverHead(float tmpDiff)
    {
        static int lastOverActSta = 0;
        static int errEacOVerActCnt = 0;
        static int nextActWaitTime = 0;
        int8_t actPitRol = 0;
        int8_t actPitRolTmp = 0;
        int32_t angleDiff = 0;
        actPitRol = overheadCheck(tmpDiff);
        actPitRolTmp = (int8_t)(actPitRol / 10);
        FRIZY_LOG(LOG_DEBUG, "actPitRolTmp:%d", actPitRolTmp);
        if (actPitRolTmp == 1)
        {
            FRIZY_LOG(LOG_INFO, "success head");
            // chassisEscape.chassisSpeed(0, 0, 1);
            act = 32;
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
            return 0;
        }
        if (actPitRolTmp == 3 && ESCACT_FRONT_WAIT != escActPara.escActState)
        {
            nextActWaitTime = 1000;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle()) / 10;
        FRIZY_LOG(LOG_DEBUG, "angleDiff:%d = (escActPara.escActPreAngle:%d - getAddAngle():%d)/10", angleDiff, escActPara.escActPreAngle, getAddAngle());
        if (chassisEscape.getWheelState() == WHEELSTOP)
        // if(act == 0)
        {
            FRIZY_LOG(LOG_DEBUG, "action over, next action");
            if (actPitRolTmp == 2)
            {
                if (0 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (1 == frontTryOverhCnt)
                {
                    alongWallDir = getAlongWallDir();
                    if (alongWallDir == RIGHTAW)
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    lastOverActSta = escActPara.escActState;
                }
                else
                {
                    nextActWaitTime = StepOverWaitRepeat(angleDiff, &lastOverActSta, frontTryOverhCnt);
                }
                behindTryOverhCnt = 0;
                frontTryOverhCnt++;
            }
            else if (actPitRolTmp == 3) //后
            {
                if (0 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
                else if (1 == behindTryOverhCnt)
                {
                    if (getPitRol(1) > 0) //左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    lastOverActSta = escActPara.escActState;
                }
                else
                {
                    if (5 == behindTryOverhCnt || 10 == behindTryOverhCnt)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                    else
                    {
                        nextActWaitTime = StepOverWaitRepeat(angleDiff, &lastOverActSta, behindTryOverhCnt);
                    }
                }
                behindTryOverhCnt++;
            }
            else if (actPitRolTmp == 4) //左
            {
                if (0 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if (1 == leftTryOverhCnt)
                {
                    if (getPitRol(1) > 0) //左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    lastOverActSta = escActPara.escActState;
                }
                else
                {
                    nextActWaitTime = StepOverWaitRepeat(angleDiff, &lastOverActSta, leftTryOverhCnt);
                }
                behindTryOverhCnt = 0;
                leftTryOverhCnt++;
            }
            else if (actPitRolTmp == 5) //右
            {
                if (0 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                if (1 == rightTryOverhCnt)
                {
                    if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
                    {
                        escActPara.escActState = (escapeAction_t)lastOverActSta;
                    }
                    else
                        escActPara.escActState = ESCACT_SPIN_RIGHT;
                    lastOverActSta = escActPara.escActState;
                }
                else
                {
                    nextActWaitTime = StepOverWaitRepeat(angleDiff, &lastOverActSta, rightTryOverhCnt);
                }
                behindTryOverhCnt = 0;
                rightTryOverhCnt++;
            }
            else if (actPitRolTmp == 6) //中
            {
                if (0 == interTryOverhCnt)
                {
                    alongWallDir = getAlongWallDir();
                    if (alongWallDir == RIGHTAW)
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if (1 == interTryOverhCnt)
                {
                    if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
                    {
                        escActPara.escActState = (escapeAction_t)lastOverActSta;
                    }
                    else
                    {
                        if (getPitRol(1) > 0) //左边翘起多
                        {
                            escActPara.escActState = ESCACT_NUILROT_LEFT;
                        }
                        else
                        {
                            escActPara.escActState = ESCACT_NUILROT_RIGHT;
                        }
                        lastOverActSta = escActPara.escActState;
                    }
                }
                else
                {
                    nextActWaitTime = StepOverWaitRepeat(angleDiff, &lastOverActSta, interTryOverhCnt);
                }

                behindTryOverhCnt = 0;
                interTryOverhCnt++;
            }
            else //正常情况不会走到这里
            {

                if (escActPara.leftSpinCnt > escActPara.rightSpinCnt + 1)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                errEacOVerActCnt++;
            }
            if (leftTryOverhCnt > 15 || rightTryOverhCnt > 15 || frontTryOverhCnt > 15 || behindTryOverhCnt > 15 || interTryOverhCnt > 15 || errEacOVerActCnt > 3)
            {
                leftTryOverhCnt = 0;
                rightTryOverhCnt = 0;
                frontTryOverhCnt = 0;
                behindTryOverhCnt = 0;
                interTryOverhCnt = 0;
                errEacOVerActCnt = 0;
                escActPara.escActStepRecord = ESC_STEP_0; //切入后退
                escActPara.enterStep0PreStep = ESC_STEP_0;
                escActPara.escActState = ESCACT_START;
                escActPara.enterStep0Sta = 1;
            }
            escActPara.escActPreAngle = getAddAngle();
            FRIZY_LOG(LOG_DEBUG, "overWait:%d %d %d %d %d %d %d %d\n", escActPara.escActState, frontTryOverhCnt, behindTryOverhCnt, leftTryOverhCnt, rightTryOverhCnt, interTryOverhCnt, angleDiff, actPitRolTmp);
        }
        else
        {
            if (nextActWaitTime < 500)
                nextActWaitTime = 3000;
            cout << "now action use time:" << getTime() - escActPara.escActExecuTime << " "
                 << "ms" << endl;
            if (getTime() - escActPara.escActExecuTime >= 1500)
            {
                if (angleDiff < 5)
                    act = 32;
            }
            if (getTime() - escActPara.escActExecuTime >= nextActWaitTime)
            {
                FRIZY_LOG(LOG_DEBUG, "nextActWaitTime:%d", nextActWaitTime);
                act = 32;
            }
        }
        return 0;
    }

    int EscapePlan::StepOverWaitRepeat(int angleDiff, int *lastStaTmp, int tryCntTmp)
    {
        int waitTimeRet = 0;
        if (2 == tryCntTmp)
        {
            if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
            {
                if (*lastStaTmp == ESCACT_NUILROT_LEFT)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
            }
            else
            {
                if (*lastStaTmp == ESCACT_NUILROT_LEFT)
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
            }
            *lastStaTmp = escActPara.escActState;
        }
        else if (3 == tryCntTmp)
        {
            if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
            {
                if (*lastStaTmp == ESCACT_NUILROT_LEFT)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
            }
            else //开始尝试自旋
            {
                if (RIGHTAW == getAlongWallDir()) //之前是右沿墙
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
            }
            *lastStaTmp = escActPara.escActState;
        }
        else if (4 == tryCntTmp)
        {
            if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
            {
                if (*lastStaTmp == ESCACT_SPIN_LEFT)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
            }
            else
            {
                if (*lastStaTmp == ESCACT_SPIN_LEFT)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
        }
        else if (5 == tryCntTmp)
        {
            if (angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else //开始尝试快速左右摆动
            {
                waitTimeRet = 600;
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            *lastStaTmp = escActPara.escActState;
        }
        else if (6 == tryCntTmp)
        {
            if (ESCACT_SPIN_LEFT == *lastStaTmp)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else
            {
                escActPara.escActState = ESCACT_NUILROT_LEFT;
            }
            waitTimeRet = 600;
            *lastStaTmp = escActPara.escActState;
        }
        else if (7 == tryCntTmp)
        {
            if (ESCACT_SPIN_LEFT == *lastStaTmp)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else
            {
                waitTimeRet = 600;
                escActPara.escActState = ESCACT_NUILROT_LEFT;
            }
            *lastStaTmp = escActPara.escActState;
        }
        else if (8 == tryCntTmp)
        {
            if (ESCACT_BEHIND == *lastStaTmp)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            waitTimeRet = 600;
            *lastStaTmp = escActPara.escActState;
        }
        else if (9 == tryCntTmp)
        {
            if (ESCACT_SPIN_RIGHT == *lastStaTmp)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else
            {
                escActPara.escActState = ESCACT_NUILROT_RIGHT;
            }
            waitTimeRet = 600;
            *lastStaTmp = escActPara.escActState;
        }
        else if (10 == tryCntTmp)
        {
            if (ESCACT_SPIN_RIGHT == *lastStaTmp)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else
            {
                escActPara.escActState = ESCACT_NUILROT_RIGHT;
            }
            waitTimeRet = 600;
            *lastStaTmp = escActPara.escActState;
        }
        else if (11 == tryCntTmp)
        {
            escActPara.escActState = ESCACT_NUILROT_LEFT;
        }
        else if (12 == tryCntTmp)
        {
            escActPara.escActState = ESCACT_SPIN_RIGHT;
        }
        else if (13 == tryCntTmp)
        {
            escActPara.escActState = ESCACT_SPIN_RIGHT;
        }
        else if (14 == tryCntTmp)
        {
            escActPara.escActState = ESCACT_BEHIND;
        }
        else
        {
            if (17 == tryCntTmp)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else
            {
                if (escActPara.leftSpinCnt > escActPara.rightSpinCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
        }
        FRIZY_LOG(LOG_DEBUG, "tryCntTmp %d,%d", tryCntTmp, escActPara.escActState);
        return waitTimeRet;
    }

    //卡死做完动作等待判断,确定下一步动作
    int EscapePlan::judgeActionStuckHead(float tmpDiff)
    {
        static int lastHeadSta = 0;
        static int actExeTime = 0;
        int8_t actPitRol = 0;
        int8_t actPitRolTmp = 0;
        int32_t angleDiff = 0;
        //2x前 3x后 4x左 5x右 6x 中
        actPitRol = overheadCheck(tmpDiff);
        actPitRolTmp = (int8_t)(actPitRol / 10);
        FRIZY_LOG(LOG_DEBUG, "actPitRolTmp:%d %d", actPitRolTmp, actPitRol);
        if (actPitRolTmp == 1)
        {
            FRIZY_LOG(LOG_INFO, "success stuck");
            // chassisEscape.chassisSpeed(0, 0, 1);
            act = 32;
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
            return 0;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle()) / 10;
        FRIZY_LOG(LOG_DEBUG, "angleDiff:%d = (escActPara.escActPreAngle:%d - getAddAngle():%d)/10", angleDiff, escActPara.escActPreAngle, getAddAngle());
        if (chassisEscape.getWheelState() == WHEELSTOP)
        // if(act == 0)
        {
            if (actPitRolTmp != 2) //前面翘起//2 == actPirRolTmp
            {
                if (0 == frontTryOverhCnt)
                {
                    if (getPitRol(1) > 0) //左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if (1 <= frontTryOverhCnt && frontTryOverhCnt <= 4)
                {
                    if (angleDiff > 5 && angleDiff < 20)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_NUILROT_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_NUILROT_LEFT;
                        }
                    }
                }
                else if (5 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (6 <= frontTryOverhCnt && frontTryOverhCnt <= 9)
                {
                    if (angleDiff > 5)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_SPIN_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_SPIN_LEFT;
                        }
                    }
                }
                else if (10 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (11 <= frontTryOverhCnt && frontTryOverhCnt <= 14)
                {
                    if (angleDiff > 5 && angleDiff < 20)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_SPIN_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_SPIN_LEFT;
                        }
                    }
                    actExeTime = 500;
                }
                else if (15 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
                else if (16 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (17 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else
                {
                    actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, frontTryOverhCnt);
                }

                frontTryOverhCnt++;
            }
            else if (actPitRolTmp == 2)
            {
                if (0 == interTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                    frontTryOverhCnt = 0;
                }
                else if (1 <= interTryOverhCnt && interTryOverhCnt <= 2)
                {
                    if (angleDiff > 5 && angleDiff < 20)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_SPIN_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_SPIN_LEFT;
                        }
                    }
                }
                else
                {
                    actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, interTryOverhCnt);
                }
                interTryOverhCnt++;
            }
            else if (actPitRolTmp == 3 || actPitRolTmp == 6)
            {
                if (0 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
                else if (1 == behindTryOverhCnt)
                {
                    if (getPitRol(1) > 0) //左边翘起多
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_SPIN_RIGHT;
                    }
                }
                else if (1 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (2 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (3 == behindTryOverhCnt)
                {
                    if (escActPara.leftSpinCnt > escActPara.rightSpinCnt)
                    {
                        escActPara.escActState = ESCACT_SPIN_RIGHT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                    }
                }
                else if (4 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (5 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (6 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
                else
                {
                    if (12 == behindTryOverhCnt)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                    else
                    {
                        actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, behindTryOverhCnt);
                    }
                }
                behindTryOverhCnt++;
            }
            else if (actPitRolTmp == 4)
            {
                if (0 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else if (1 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (2 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (3 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else if (4 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (5 == leftTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else
                {
                    actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, leftTryOverhCnt);
                }

                leftTryOverhCnt++;
            }
            else if (actPitRolTmp == 5)
            {
                if (0 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if (1 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (2 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (3 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if (4 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (5 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, rightTryOverhCnt);
                }

                rightTryOverhCnt++;
            }
            else //一版情况不会到这里
            {
                if (0 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if (1 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (2 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (3 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if (4 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (5 == rightTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else
                {
                    actExeTime = StepOverWaitRepeat(angleDiff, &lastHeadSta, rightTryOverhCnt);
                }
                rightTryOverhCnt++;
            }
            if (leftTryOverhCnt > 17 || rightTryOverhCnt > 17 || frontTryOverhCnt > 17 || behindTryOverhCnt > 17 || interTryOverhCnt > 17)
            {
                leftTryOverhCnt = 0;
                rightTryOverhCnt = 0;
                frontTryOverhCnt = 0;
                behindTryOverhCnt = 0;
                interTryOverhCnt = 0;
                escActPara.escActStepRecord = ESC_STEP_0; //切入后退
                escActPara.enterStep0PreStep = ESC_STEP_0;
                escActPara.escActState = ESCACT_START;
                escActPara.enterStep0Sta = 1;
            }
            escActPara.escActPreAngle = getAddAngle();
            lastHeadSta = (escapeAction_t)escActPara.escActState;
            FRIZY_LOG(LOG_DEBUG, "ActHead:%d %d %d,cnt r=%d b=%d c=%d L=%d in=%d\n",
             actPitRol, actPitRolTmp, escActPara.escActState, 
             rightTryOverhCnt, behindTryOverhCnt, interTryOverhCnt, leftTryOverhCnt, frontTryOverhCnt);
        }
        else
        {
            if (actExeTime < 500)
                actExeTime = 3000;
            if (getTime() - escActPara.escActExecuTime >= 1500)
            {
                if (angleDiff < 5)
                    act = 32;
            }
            FRIZY_LOG(LOG_DEBUG, "actExeTime:%d", actExeTime);
            if (getTime() - escActPara.escActExecuTime >= actExeTime)
                act = 32;
        }

        return 0;
    }

    //架空脱困
    void EscapePlan::escStepOverHead()
    {
        float currPitRol = 0.0f, diffPitRol = 0.0f;
        currPitRol = gyroPitRolSlidingAvg();                           //记录机器陀螺仪姿态
        diffPitRol = fabsf(escActPara.escBasePitRol - currPitRol); //和10s前的做对比
        switch (escActPara.escActState)
        {
        case ESCACT_START:                 //判断下一步执行什么动作
            escActPara.frontCnt = 0;       //前进 动作次数计数
            escActPara.behindCnt = 0;      //后退 动作次数计数
            escActPara.leftSpinCnt = 0;    //左自旋 动作次数计数
            escActPara.rightSpinCnt = 0;   //右自旋 动作次数计数
            escActPara.leftBehindCnt = 0;  //左后 动作次数计数
            escActPara.rightBehindCnt = 0; //右后 动作次数计数
            leftTryOverhCnt = 0;
            rightTryOverhCnt = 0;
            frontTryOverhCnt = 0;
            behindTryOverhCnt = 0;
            interTryOverhCnt = 0;
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_NUILROT_LEFT:
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                // singleRotate(-300, 10, 90, 0);
                act = 1;
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            else
            {

                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
            break;

        case ESCACT_NUILROT_LEFT_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_NUILROT_RIGHT:
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                act = 2;
                // singleRotate(10, -300, 90, 0);
                escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
            break;

        case ESCACT_NUILROT_RIGHT_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_SPIN_LEFT:
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_LEFT");
                act = 3;
                // escSpin(300, 0, 90, 0);
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
            break;

        case ESCACT_SPIN_LEFT_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_SPIN_RIGHT:
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_RIGHT");
                act = 4;
                // escSpin(300, 1, 90, 0);
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }

            break;

        case ESCACT_SPIN_RIGHT_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_BEHIND:
            FRIZY_LOG(LOG_DEBUG, "ESCACT_BEHIND");
            act = 5;
            // wheelBackDist(-300, 150);   //正常的速度后退
            escActPara.escActState = ESCACT_BEHIND_WAIT;

            break;

        case ESCACT_BEHIND_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        case ESCACT_FRONT:
            FRIZY_LOG(LOG_DEBUG, "ESCACT_FRONT");
            act = 6;
            // wheelCtrlStraight(160, 1000);
            escActPara.escActState = ESCACT_FRONT_WAIT;

            break;

        case ESCACT_FRONT_WAIT:
            judgeActionOverHead(diffPitRol);
            break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    //头部卡死
    void EscapePlan::escActStepStuckTheHead()
    {
        float currPitRol = 0.0f, diffPitRol = 0.0f;
        currPitRol = gyroPitRolSlidingAvg(); //记录机器陀螺仪姿态
        // FRIZY_LOG(LOG_INFO, "compare with 10s pitrol:%f, %f", escActPara.escBasePit, escActPara.escBaseRol);
        diffPitRol = fabsf(escActPara.escBasePitRol - currPitRol); //和10s前的做对比
        switch (escActPara.escActState)
        {
        case ESCACT_START:
            escActPara.frontCnt = 0;       //前进 动作次数计数
            escActPara.behindCnt = 0;      //后退 动作次数计数
            escActPara.leftSpinCnt = 0;    //左自旋 动作次数计数
            escActPara.rightSpinCnt = 0;   //右自旋 动作次数计数
            escActPara.leftBehindCnt = 0;  //左后 动作次数计数
            escActPara.rightBehindCnt = 0; //右后 动作次数计数
            leftTryOverhCnt = 0;
            rightTryOverhCnt = 0;
            frontTryOverhCnt = 0;
            behindTryOverhCnt = 0;
            interTryOverhCnt = 0;
            escActPara.escActState = ESCACT_BEHIND;
            break;

        case ESCACT_NUILROT_LEFT: //左单边旋
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_LEFT");
                act = 7;
                // singleRotate(-400, 10, 60, 0);
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
            break;

        case ESCACT_NUILROT_LEFT_WAIT: //左单边旋等待
            judgeActionStuckHead(diffPitRol);
            break;

        case ESCACT_NUILROT_RIGHT: //右单边旋
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                act = 8;
                // singleRotate(10, -400, 60, 0);
                escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }

            break;

        case ESCACT_NUILROT_RIGHT_WAIT:
            judgeActionStuckHead(diffPitRol);
            break;

        case ESCACT_SPIN_RIGHT: //5 右自旋
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_RIGHT");
                act = 10;
                // escSpin(300, 1, 60, 0);
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    // chassisEscape.chassisSpeed(0, 0, 1);
                    act = 32;
                }
            }
            break;

        case ESCACT_SPIN_RIGHT_WAIT:
            judgeActionStuckHead(diffPitRol);
            break;

        case ESCACT_SPIN_LEFT: //3 左自旋
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_LEFT");
                act = 9;
                // escSpin(300, 0, 60, 0);
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 2500)
                {
                    FRIZY_LOG(LOG_DEBUG, "Action time out");
                    act = 32;
                    // chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

        case ESCACT_SPIN_LEFT_WAIT:
            judgeActionStuckHead(diffPitRol);
            break;

        case ESCACT_BEHIND: //1 后退
            FRIZY_LOG(LOG_DEBUG, "ESCACT_BEHIND");
            act = 11;
            // wheelCtrlStraight(-300, 600);
            escActPara.escActState = ESCACT_BEHIND_WAIT;
            break;

        case ESCACT_BEHIND_WAIT: //2
            judgeActionStuckHead(diffPitRol);
            break;

        case ESCACT_FRONT: //14前进
            FRIZY_LOG(LOG_DEBUG, "ESCACT_FRONT");
            act = 12;
            // wheelCtrlStraight(160, 600);
            escActPara.escActState = ESCACT_FRONT_WAIT;
            break;

        case ESCACT_FRONT_WAIT:
            judgeActionStuckHead(diffPitRol);
            break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    static int last_state = -1;
    static int front_bump_cnt = 0;
    static long long start_front_time;
    int EscapePlan::judgeActionLidar()
    {
        int rc = 0;

        if (current_pose->exceptional_case == 0)
        {
            escActPara.lidarNormalCnt++;
            if (escActPara.lidarNormalCnt > 1000 / 20)
            {
                FRIZY_LOG(LOG_INFO, "success Lidar");
                act = 32;
                escActPara.escActState = ESCACT_START;
                escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
                return 0;
            }
        }
        else if (current_pose->exceptional_case == 2)
        {
            if(escActPara.lidarNormalCnt > 0)
                escActPara.lidarNormalCnt--;
        }
        if(front_bump_cnt > 2)
        {
            FRIZY_LOG(LOG_DEBUG, "switch alongwall escape");
            act = 32;
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            escActPara.escActState = ESCACT_AW_WAIT;//切入沿墙
        }
        if (getTime() - escStartTime > 30000)//30秒雷达未恢复正常则报警
        {
            act = 32;
            FRIZY_LOG(LOG_INFO, "failed Lidar");
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_FAILD; //切入失败                
            return 0;
        }
        if (chassisEscape.getWheelState() == WHEELSTOP)
        {
            if(getTime() - start_front_time < 2000)//前进时间不足两秒被打断
                front_bump_cnt++;
            if (last_state == ESCACT_BEHIND)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if(last_state == ESCACT_SPIN_LEFT)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
            else if(last_state == ESCACT_FRONT)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if(last_state == ESCACT_SPIN_RIGHT)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
        }
        else
        {
            if (getTime() - escActPara.escActExecuTime >= 2500)
                act = 32;
        }
    }

    
    void EscapePlan::lidarEscape()
    {
        switch (escActPara.escActState)
        {
            case ESCACT_START:
                last_state = -1;
                front_bump_cnt = 0;
                FRIZY_LOG(LOG_DEBUG, "lidar escape init");
                escActPara.escActState = ESCACT_BEHIND;
                break;

            case ESCACT_BEHIND:
                act = 35;
                FRIZY_LOG(LOG_DEBUG, "ESCACT_BEHIND");
                last_state = escActPara.escActState;
                escActPara.escActState = ESCACT_BEHIND_WAIT;
                break;

            case ESCACT_BEHIND_WAIT:
                judgeActionLidar();
                break;

            case ESCACT_SPIN_LEFT:
                if (chassisEscape.getWheelState() == WHEELSTOP)
                {
                    act = 37;
                    last_state = escActPara.escActState;
                    FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_LEFT");
                    escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
                }
                else
                {
                    if (getTime() - escActPara.escActExecuTime >= 2500)
                    {
                        FRIZY_LOG(LOG_DEBUG, "Action time out");
                        act = 32;
                    }
                }

                break;

            case ESCACT_SPIN_LEFT_WAIT:
                judgeActionLidar();
                break;

            case ESCACT_SPIN_RIGHT:
                if (chassisEscape.getWheelState() == WHEELSTOP)
                {
                    act = 38;
                    last_state = escActPara.escActState;
                    FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_RIGHT");
                    escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
                }
                else
                {
                    if (getTime() - escActPara.escActExecuTime >= 2500)
                    {
                        FRIZY_LOG(LOG_DEBUG, "Action time out");
                        act = 32;
                    }
                }

                break;

            case ESCACT_SPIN_RIGHT_WAIT:
                judgeActionLidar();
                break;

            case ESCACT_FRONT:
                act = 36;
                start_front_time = getTime();
                last_state = escActPara.escActState;
                FRIZY_LOG(LOG_DEBUG, "ESCACT_FRONT");
                escActPara.escActState = ESCACT_FRONT_WAIT;
                break;

            case ESCACT_FRONT_WAIT:
                judgeActionLidar();
                break;
            

            case ESCACT_AW_WAIT:
                if (current_pose->exceptional_case == 0)
                {
                    escActPara.lidarNormalCnt++;
                    if (escActPara.lidarNormalCnt > 1000 / 20)
                    {
                        FRIZY_LOG(LOG_INFO, "success Lidar");
                        StopWallFollow();
                        act = 32;
                        escActPara.escActState = ESCACT_START;
                        escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
                        break;
                    }
                }
                else if (current_pose->exceptional_case == 2)
                {
                    if(escActPara.lidarNormalCnt > 0)
                        escActPara.lidarNormalCnt--;
                }
                break;
            }
    }

    //头部卡住动作执行等待操作
    void EscapePlan::stuckActHeadWait(float tmpDiff)
    {
        static int8_t lastHeadSta = 0;
        static int32_t actExeTime = 0;
        int8_t actPirRol = 0;
        int8_t actPirRolTmp = 0;
        int32_t angleDiff = 0;
        //2x前 3x后 4x左 5x右 6x中
        actPirRol = overheadCheck(tmpDiff);
        actPirRolTmp = (int8_t)(actPirRol / 10);

        if (1 == actPirRolTmp)
        {
            FRIZY_LOG(LOG_INFO, "success head");
            // chassisEscape.chassisSpeed(0, 0, 1);
            act = 32;
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS; //切入成功
            return;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle());
        if (chassisEscape.getWheelState() == WHEELSTOP)
        {
            if (actPirRolTmp != 2)
            {
                if (0 == frontTryOverhCnt)
                {
                    if (getPitRol(1) > 0) //左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if (1 <= frontTryOverhCnt && frontTryOverhCnt <= 4)
                {
                    if (angleDiff > 5 && angleDiff < 20)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_NUILROT_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_NUILROT_LEFT;
                        }
                    }
                }
                else if (5 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if (6 <= frontTryOverhCnt && frontTryOverhCnt <= 9)
                {
                    if (angleDiff > 5)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if (ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_SPIN_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_SPIN_LEFT;
                        }
                    }
                }
                else if (10 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
            }
        }
    }

    // 架空动作 之后的判定中间操作  不用
    // int EscapePlan::escStepOverheadWait(float tmpDiff)
    // {
    //     static int8_t lastOverActSta = 0;
    //     static int8_t errEacOVerActCnt = 0;
    //     static int32_t nextActWaitTime = 0;//下一个动作等待的时间
    //     int8_t actPirRol = 0;
    //     int8_t actPirRolTmp = 0;
    //     int16_t angleRet = 0;
    //     int32_t angleDiff = 0;
    //     //2x前 3x后 4x左 5x右 6x 中
    //     actPirRol = overheadCheck(tmpDiff);
    //     actPirRolTmp = (int8_t)(actPirRol/10);
    //     if(1 == actPirRolTmp)
    //     {
    //         chassisEscape.chassisSpeed(0, 0, 0);
    //         escActPara.escActState = ESCACT_START;
    //         escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
    //         FRIZY_LOG(LOG_INFO,"enter step success, actPirRol:%d, actPirRolTmp:%d", actPirRol, actPirRolTmp);
    //     }
    //     if(actPirRolTmp == 3 && ESCACT_FRONT_WAIT != escActPara.escActState)
    //     {
    //         nextActWaitTime = 1000;
    //     }
    //     angleDiff = abs(escActPara.escActPreAngle - getAddAngle())/10;
    //     if(chassisEscape.getWheelState() == WHEELSTOP)
    //     {

    //     }

    // }

    //利用鲁大师进行识别
    bool EscapePlan::GridMacinCheck(void)
    {
        // chassisEscape.GridPoint(&escapGrid);
        // escapGrid.realy * 15;//cm
        // escapGrid.realx * 15;//cm
        // escapGrid.
    }

    //脱困失败处理
    void EscapePlan::escActStepFailed()
    {
        // memset(escLastLeftCurAvg,0,sizeof(escLastLeftCurAvg));
        // memset(escLastRightCurAvg,0,sizeof(escLastRightCurAvg));
        // memset(escLeftCurArry,0,sizeof(escLeftCurArry));
        // memset(escRightCurArry,0,sizeof(escRightCurArry));
        // memset(pitRolLastAvg,0,sizeof(pitRolLastAvg));
        // memset(pitRolAvgArry,0,sizeof(pitRolAvgArry));
        // memset(gyroAccBuf,0,sizeof(gyroAccBuf));
        // memset(escPreActionArry,0,sizeof(escPreActionArry));
        // memset(&escActPara,0,sizeof(escActPara));
        // ESCAPE_LeftOffCheckInit();
        // stuckType = NOTHTING;
        // escCkeckPara.arryAvgCurCountR = 0;
        // escCkeckPara.arryAvgCurCountL = 0;
        // record_time = 0;
        // recordPit.clear();
        // recordRol.clear();
        // recordPitRol.clear();
        // act = 0;
        act = 32;
        escapModeInit();
    }

    //脱困成功
    void EscapePlan::escActStepSuccess()
    {
        switch (escActPara.escActState)
        {
        case 0:
        {
            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                escActPara.escActState = ESCACT_SUCCESS;
            }
            else
            {
                if (getTime() - escActPara.escActExecuTime >= 1500)
                    act = 32;
            }
        }
        break;

        case ESCACT_SUCCESS: //11脱困结束成功
        {
            if (chassisEscape.getWheelState() != WHEELSTOP)
            {
                act = 32;
                break;
            }
            //脱困后,根据进入脱困前的状况旋转角度
            if (beforeEscState == 1) //之前处于沿墙状态下进入的脱困
            {
                if (getAlongWallDir() == LEFTAW)
                {
                    succesAimforward = enterGyroAngle + 90;
                    if (succesAimforward >= 360)
                    {
                        succesAimforward = succesAimforward - 360;
                    }
                    // spinSuccess(120, 1, succesAimforward);
                    act = 33;
                    FRIZY_LOG(LOG_DEBUG, "spin:%f -> %f", enterGyroAngle, succesAimforward);
                }
                else
                {
                    succesAimforward = enterGyroAngle - 90;
                    if (succesAimforward <= 0)
                    {
                        succesAimforward = 360 + succesAimforward;
                    }
                    // spinSuccess(120, 0, succesAimforward);
                    act = 34;
                    FRIZY_LOG(LOG_DEBUG, "spin:%f -> %f", enterGyroAngle, succesAimforward);
                }
            }
            else //非沿墙状态下进入的脱困
            {
                succesAimforward = enterGyroAngle + 180;
                if (succesAimforward >= 360)
                {
                    succesAimforward = succesAimforward - 360;
                }
                act = 33;
                // spinSuccess(120, 1, succesAimforward);
                FRIZY_LOG(LOG_DEBUG, "spin:%f -> %f", enterGyroAngle, succesAimforward);
            }
            escActPara.escActState = ESCACT_END;
        }
        break;

        case ESCACT_END: //13脱困结束
        {
            long long actOutDiff = 0;
            actOutDiff = getTime() - escActPara.escActExecuTime;
            if ((-1 == escActPara.actionRetDelay || actOutDiff >= 5000) && stuckType != ESCAPE_EVENT_TYPE_LIDAR_STUCK) //自旋无效
            {
                act = 32;
                if (escActPara.enterSuccessCnt > 10)
                {
                    FRIZY_LOG(LOG_DEBUG, "faild SuccessCnt=%d\n", escActPara.enterSuccessCnt);
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                    escActPara.escActState = ESCACT_START;
                    escActPara.enterSuccessCnt = 0;
                }
                else if (escActPara.enterSuccessCnt <= 3)
                {
                    //由第四步切换到脱困成功的，仍然切换到第四步
                    if (ESC_STEP_4 == escActPara.enterSuccPreStep)
                    {
                        escActPara.escActStepRecord = ESC_STEP_4;
                    }
                    else if (ESC_STEP_3 == escActPara.enterSuccPreStep)
                    {
                        escActPara.escActStepRecord = ESC_STEP_3;
                    }
                    else if (ESC_STEP_2 == escActPara.enterSuccPreStep)
                    {
                        escActPara.escActStepRecord = ESC_STEP_2;
                    }
                    else if (ESC_STEP_STUCK_HEAD == escActPara.enterSuccPreStep)
                    {
                        escActPara.escActStepRecord = ESC_STEP_STUCK_HEAD;
                    }
                    else if (ESC_STEP_OVERHEAD == escActPara.enterSuccPreStep)
                    {
                        escActPara.escActStepRecord = ESC_STEP_OVERHEAD;
                    }
                    else
                    {
                        escActPara.escActStepRecord = ESC_STEP_1;
                        escActPara.escActState = ESCACT_START;
                        escActPara.enterStep0PreStep = ESC_STEP_SUCCESS;
                        FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
                    }
                    escActPara.escActState = ESCACT_START;
                }
                else if (escActPara.enterSuccessCnt < 5)
                {
                    escActPara.escActStepRecord = ESC_STEP_2;
                    escActPara.escActState = ESCACT_START;
                }
                else if (escActPara.enterSuccessCnt < 6)
                {
                    escActPara.escActStepRecord = ESC_STEP_1;
                    escActPara.escActState = ESCACT_START;
                }
                else
                {
                    if (escActPara.enterSuccessCnt % 2 == 0)
                    {
                        escActPara.escActStepRecord = ESC_STEP_4;
                    }
                    else
                    {
                        escActPara.escActStepRecord = ESC_STEP_3;
                    }

                    escActPara.escActState = ESCACT_START;
                }
                escActPara.enterSuccessCnt++;
                FRIZY_LOG(LOG_INFO, "succStep enter faild %d %d,W=%d T=%d", escActPara.enterSuccPreStep, escActPara.escActStepRecord, escActPara.actionRetDelay, actOutDiff);
                break;
            }

            if (chassisEscape.getWheelState() == WHEELSTOP)
            {
                isSuccess = 1;
                FRIZY_LOG(LOG_DEBUG, "isSuccess == 1,LINE:%d", __LINE__);
            }
        }
        break;

        default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    //脱困处理
    int EscapePlan::escDeal(int state)
    {
        FRIZY_LOG(LOG_INFO, "start escape!");
        chassisEscape.GridPoint(&escapGrid);
        /*static*/ int lastAct = -1;
        int lastAction = -1;
        escStartTime = getTime(); //记录脱困开始时间
        FRIZY_LOG(LOG_DEBUG, "enter esc time:%lld", escStartTime);
        enterGyroAngle = (360 - gyo_angle_ * 180 / _Pi); //记录脱困前的角度
        FRIZY_LOG(LOG_DEBUG, "enterGyroAngle:%f", enterGyroAngle);
        enterAddAngle = getAddAngle(); //记录脱困前的陀螺仪累加角度
        beforeEscState = state;        //记录脱困前的扫地机状态
        int addAngle = enterAddAngle / 10;
        int cnt = 0;
        int escpRecordLast = -1;                                   //记录大状态机
        escActPara.escInterval = getTime() - escEnterTimeInterval; //计算进入脱困的时间间隔
        FRIZY_LOG(LOG_DEBUG, "GLOBAL_CONTROL:%d", GLOBAL_CONTROL);
        isSuccess = 0;
        chassisEscape.chassisSpeed(0, 0, 1);
        while (GLOBAL_CONTROL == WHEEL_RUN)
        {
            //执行动作改变,更新动作执行时间
            chassisEscape.GetSensor(&escapSensor);
            chassisEscape.GridPoint(&escapGrid);
            FRIZY_LOG(LOG_DEBUG, "WHEEL SPEED LEFT:%d, RIGHT:%d", escapSensor.leftw, escapSensor.rightw);

            if (lastAct != escActPara.escActState)
            {
                FRIZY_LOG(LOG_DEBUG, "actstate:%d->%d", lastAct, escActPara.escActState);
            }

            if (lastAction != act)
            {
                FRIZY_LOG(LOG_DEBUG, "act:%d -> %d", lastAction, act);
                lastAction = act;
            }
            // //脱困动作 -1无效 1有效
            escActPara.escActExecSta = monitorIsEscapeSuccess();
            // //同一个动作把结果延长直到下一个动作
            if (-1 == escActPara.escActExecSta || 1 == escActPara.escActExecSta)
            {
                escActPara.actionRetDelay = escActPara.escActExecSta;
            }
            if (lastAct != escActPara.escActState)
            {
                FRIZY_LOG(LOG_INFO, "ACTION CHANGE %lld", getTime());
                //有动作变更时更新时间
                escActPara.escActExecuTime = getTime();
                cout << "escActExecuTime:" << escActPara.escActExecuTime << endl;
                escActPara.actionRetDelay = 0;
            }
            lastAct = escActPara.escActState;
            //一个状态停留超过5s，直接判定为动作无效
            if (getTime() - escActPara.escActExecuTime >= 8000)
            {
                if (stuckType != ESCAPE_EVENT_TYPE_LIDAR_STUCK)
                {
                    FRIZY_LOG(LOG_INFO, "ACTION INVALID");
                    escActPara.enterStep0PreStep = ESC_STEP_0;
                    escActPara.escActState = ESCACT_START;
                    FRIZY_LOG(LOG_DEBUG, "act out 8s escActState:%d escActStepRecord:%d", escActPara.escActState, escActPara.escActStepRecord);
                }
            }
            else if (getTime() - escActPara.escActExecuTime >= 5000)
            {
                escActPara.actionRetDelay = -1;
                FRIZY_LOG(LOG_DEBUG, "act out 5s escActPara:%d escActStepRecord:%d", escActPara.escActState, escActPara.escActStepRecord);
            }
            if ((getTime() - escStartTime >= (60000 * 2)) && escActPara.escActStepRecord != ESC_STEP_SUCCESS)
            {
                FRIZY_LOG(LOG_DEBUG, "now - start = %lld %lld %lld", escStartTime, getTime(), getTime()-escStartTime);
                if(stuckType == ESCAPE_EVENT_TYPE_LIDAR_STUCK)
                {
                    FRIZY_LOG(LOG_DEBUG, "lidar along wall time out");
                    StopWallFollow();
                }
                escActPara.escActStepRecord = ESC_STEP_FAILD;
                escActPara.escActState = ESCACT_START;
                FRIZY_LOG(LOG_INFO, "TIME OUT, ESCAPE FAILED");
            }
            //针对架空脱困和尾部翘起卡死脱困，如果数据正常直接切入脱困
            if (escActPara.escActStepRecord != ESC_STEP_SUCCESS &&
                escActStepBehind() && (escActPara.enterSuccessCnt < 2))
            {
                // chassisEscape.chassisSpeed(0, 0, 1);
                act = 32;
                escActPara.enterSuccPreStep = escActPara.escActStepRecord;
                escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                FRIZY_LOG(LOG_DEBUG, "LINE:%d", __LINE__);
                escActPara.escActState = ESCACT_START;
            }

            if (escpRecordLast != escActPara.escActStepRecord)
            {
                FRIZY_LOG(LOG_INFO, "ActStepRecord %d->%d", escpRecordLast, escActPara.escActStepRecord);
                escpRecordLast = escActPara.escActStepRecord;
            }
            switch (escActPara.escActStepRecord)
            {
            case ESC_STEP_START:
                //脱困前停止10次，采集停止的加速数据
                escStepPreGyroAnal();
                break;
            case ESC_STEP_0: //1
                // FRIZY_LOG(LOG_INFO,"ESC_STEP_0");
                escActStep_0();
                break;
            case ESC_STEP_1: //2
                // FRIZY_LOG(LOG_INFO,"ESC_STEP_1");
                escActStep_1();
                break;
            case ESC_STEP_2: //3
                // FRIZY_LOG(LOG_INFO,"ESC_STEP_2");
                escActStep_2();
                break;
            case ESC_STEP_3: //4
                // FRIZY_LOG(LOG_INFO,"ESC_STEP_3");
                escActStep_3();
                break;
            case ESC_STEP_4: //5
                // FRIZY_LOG(LOG_INFO,"ESC_STEP_4");
                escActStep_4();
                break;
            case ESC_STEP_FAILD:
                // FRIZY_LOG(LOG_INFO,"ESCAPE FAILED");
                if(stuckType == ESCAPE_EVENT_TYPE_LIDAR_STUCK)
                    isSuccess = 3;
                else
                    isSuccess = 2;
                escActStepFailed();
                break;
            case ESC_STEP_SUCCESS:
                // FRIZY_LOG(LOG_INFO,"ESCAPE SUCCESSFUL");
                escActStepSuccess();
                break;
            case ESC_STEP_OVERHEAD: //架空
                escStepOverHead();
                break;

            case ESC_STEP_STUCK_HEAD: //头部卡死(欧式家具)
                escActStepStuckTheHead();
                break;

            case ESC_STEP_LIDAR: //雷达遮挡
                lidarEscape();
                break;

            default:
                escActPara.escActStepRecord = ESC_STEP_START;
                break;
            }

            if(isSuccess == 1)
            {
                escEnterTimeInterval = getTime();
                escapModeInit();
                return 1;
            }
            else if(isSuccess == 2)
            {
                return 2;
            }
            else if(isSuccess == 3)
            {
                return 3;
            }
            usleep(20 * 1000);
        } //while(1) end
        FRIZY_LOG(LOG_DEBUG, "escape not run");
        return 0;
    }

    void EscapePlan::EscapeSmallArea()
    {
        FRIZY_LOG(LOG_INFO, " EscapeSmallArea ");
        for (int i = 0; i < 10; i++)
        {
            chassisEscape.GetSensor(&escapSensor);
            if (escapSensor.bump != 0)
            {
                chassisEscape.chassisSpeed(100, -100, 1);
                usleep(50 * 1000);
            }

            else
            {
                wheelCtrlStraight(-200, 50);
            }
        }
    }

    void EscapePlan::escAct()
    {
        while (1)
        {
            if (GLOBAL_CONTROL != WHEEL_RUN)
            {
                // FRIZY_LOG(LOG_DEBUG, "GLOBAL_CONTROL:%d", GLOBAL_CONTROL);
                usleep(200 * 1000);
                continue;
            }
            else
            {
                static int lastAct = 0;
                if (lastAct != act)
                {
                    FRIZY_LOG(LOG_DEBUG, "actRun %d -> %d", lastAct, act);
                    lastAct = act;
                }
                switch (act)
                {
                case 0: //
                    //usleep(50 * 1000);
                    break;

                case 1: //架空左单边旋
                    if (singleRotate(-300, 10, 90, 0))
                        act = 0;
                    break;

                case 2: //架空右单边旋
                    if (singleRotate(10, -300, 90, 0))
                        act = 0;
                    break;

                case 3: //架空左自旋
                    if (escSpin(300, 0, 90, 0))
                        act = 0;
                    break;

                case 4: //架空右自旋
                    if (escSpin(300, 1, 90, 0))
                        act = 0;
                    break;

                case 5: //架空后退
                    if (wheelBackDist(-300, 150))
                        act = 0;
                    break;

                case 6: //架空前进
                    if (wheelCtrlStraight(200, 1000))
                        act = 0;
                    break;

                case 7: //卡死左单边旋
                    if (singleRotate(-400, 10, 60, 0))
                        act = 0;
                    break;

                case 8: //卡死右单边旋
                    if (singleRotate(10, -400, 60, 0))
                        act = 0;
                    break;

                case 9: //卡死左自旋
                    if (escSpin(300, 0, 60, 0))
                        act = 0;
                    break;

                case 10: //卡死右自旋
                    if (escSpin(300, 1, 60, 0))
                        act = 0;
                    break;

                case 11: //卡死后退
                    if (wheelCtrlStraight(-300, 600))
                        act = 0;
                    break;

                case 12: //卡死前进
                    if (wheelCtrlStraight(200, 600))
                        act = 0;
                    break;

                case 13:
                    if (wheelBackDist(-300, 80)) //正常的速度后退 step0
                        act = 0;
                    break;

                case 14:
                    if (wheelCtrlStraight(-300, 800)) //后退 step0
                        act = 0;
                    break;

                case 15:
                    if (wheelCtrlStraight(-300, 800)) //后退 step0
                        act = 0;
                    break;

                case 16:
                    if (escSpin(300, 1, 30, 0)) //右自旋 step1
                        act = 0;
                    break;

                case 17:
                    if (escSpin(300, 0, 30, 0)) //左自旋 step1
                        act = 0;
                    break;

                case 18:
                    if (singleRotate(-300, -60, 30, 0)) //左单边旋 step1
                        act = 0;
                    break;
                case 19:
                    if (singleRotate(-60, -300, 30, 0)) //右单边旋 step1
                        act = 0;
                    break;
                case 20:
                    if (escSpin(300, 0, 60, 0)) //左自旋 step2
                        act = 0;
                    break;

                case 21:
                    if (escSpin(300, 1, 60, 0)) //右自旋 step2
                        act = 0;
                    break;

                case 22:
                    if (singleRotate(-400, -80, 35, 0)) //左单边旋 step2
                        act = 0;
                    break;

                case 23:
                    if (singleRotate(-80, -400, 35, 0)) //右单边 step2
                        act = 0;
                    break;

                case 24:
                    if (escSpin(400, 0, 30, 0)) //左自旋 step3
                        act = 0;
                    break;

                case 25:
                    if (escSpin(400, 1, 30, 0)) //右单边旋 step3
                        act = 0;
                    break;

                case 26:
                    if (singleRotate(-400, -100, 50, 0)) //左单边旋 step4
                        act = 0;
                    break;

                case 27:
                    if (singleRotate(-100, -400, 50, 0)) //右单边旋 step4
                        act = 0;
                    break;

                case 28:
                    if (escSpin(400, 0, 90, 0)) //左自旋 step4
                        act = 0;
                    break;

                case 29:
                    if (escSpin(400, 1, 90, 0)) //右自旋 step4
                        act = 0;
                    break;

                case 30:
                    if (wheelCtrlStraight(200, 600)) //前进 step4
                        act = 0;
                    break;

                case 31:
                    if (wheelBackDist(-400, 80)) //后退 step4
                        act = 0;
                    break;

                case 32:
                    chassisEscape.chassisSpeed(0, 0, 1); //打断当前的动作
                    FRIZY_LOG(LOG_DEBUG, "change espcape action, stop now action");
                    back_dis = 0, back_time = 0, spin_flag = 0, rotate_flag = 0;
                    b_time = 0;
                    behindSumSpd = 0;
                    act = 0;
                    break;

                case 33:
                    if (spinSuccess(120, 1, succesAimforward)) //脱困成功右自旋
                        act = 0;
                    break;

                case 34:
                    if (spinSuccess(120, 0, succesAimforward)) //脱困成功左自旋
                        act = 0;
                    break;

                case 35:                                //雷达脱困动作
                    if (wheelCtrlLidar(-120, 1200)) //后退
                        act = 0;
                    break;

                case 36:
                    if (wheelCtrlLidar(120, 4000)) //前进
                        act = 0;
                    break;

                case 37:
                    if (escSpin(120, 0, 180, 0)) //左自旋
                        act = 0;
                    break;

                case 38:
                    if (escSpin(120, 1, 90, 0)) //右自旋
                        act = 0;
                    break;
                case 39:
                    if(escSpin(120, 0, 110, 0))//左自旋
                        act = 0;
                    break;

                case 40:
                    if(escSpin(120, 1, 110, 0))//右自旋
                        act = 0;
                    break;
                    // default:
                    //     act = 0;
                    // break;
                
                case 41:
                    if (wheelCtrlStraight(-120, 0)) //后退 step4
                        act = 0;
                    break;
                }
                usleep(10 * 1000);
            }
        }
    }

    void EscapePlan::espThreadStart()
    {
        FRIZY_LOG(LOG_DEBUG, "escape thread run");
        escThread_ = std::make_shared<std::thread>(&EscapePlan::escAct, this);
    }

    void EscapePlan::espThreadStop()
    {
        escThread_->join();
        escThread_ = nullptr;
    }

    bool EscapePlan::escapeCheck()
    {
        static int lastControl = -1;
        if (lastControl != WHEEL_RUN && GLOBAL_CONTROL == WHEEL_RUN)
        {
            escapModeInit();
        }

        lastControl = GLOBAL_CONTROL;

        if (GLOBAL_CONTROL != WHEEL_RUN)
        {

            return false;
        }

        recordActBeforeEsc();

        // if (machinLocationInfoCheck()) //test
        // {
        //     FRIZY_LOG(LOG_DEBUG, "machinLocat abnormal");
        //     // return true ;
        // }
        if (ESCAPE_LeftFloorCheck()) //架空检测
        {
            FRIZY_LOG(LOG_DEBUG, "leftFloorCheck");
            return true;
        }
        else if (ESCAPE_StuckTrigCheck_My_Test()) //卡死检测
        {
            FRIZY_LOG(LOG_DEBUG, "stuckCheck");
            return true;
        }
        else if (EscpBarChairDete()) //吧台椅检测
        {
            FRIZY_LOG(LOG_DEBUG, "on barChair");
            return true;
        }
        else if (wheelToGyroAngleCheck())
        {
            FRIZY_LOG(LOG_DEBUG, "wheelToGyroAngle abnormal");
            return true;
        }
        else if(cliffCheck())   //探地脱困
        {
            FRIZY_LOG(LOG_DEBUG, "cliff abnormal");
            return true;
        }
        else if (lidarCheck())
        {
            FRIZY_LOG(LOG_DEBUG, "lidar abnormal");
            return true;
        }
        else
            return false;
    }

    int EscapePlan::escapeProcess(int robot_state)
    {
        FRIZY_LOG(LOG_INFO, "stucktype:%d", stuckType);
        int wallstate = robot_state;
        int rc = 0;
        switch(stuckType)
        {
            case NOTHING:
                rc = escDeal(wallstate);
            break;

            case ESCAPE_EVENT_TYPE_LEFT_OFF:
                rc = escDeal(wallstate);
            break;
            
            case ESCAPE_EVENT_TYPE_STUCK:
                rc = escDeal(wallstate);
            break;

            case ESCAPE_EVENT_TYPE_LIDAR_STUCK:
                rc = escDeal(wallstate);
            break;
            
            case ESCAPE_EVENT_TYPE_CLIFF:
                rc = cliffEscape();
                if(rc == -1)
                    rc = 2;
                else
                    rc = 1;
            break;
        }
        FRIZY_LOG(LOG_DEBUG, "esc process return rc:%d", rc);
        return rc;
    }

    // bool EscapePlan::wheelToGyroAngleCheck()
    // {
    //     static int16_t lastGyroAngle = 0;
    //     int16_t gyroAngleNow;
    //     static int16_t calcWheelAngleRato = 0;
    //     static int16_t encoderCalAngleData = 0;
    //     static int16_t encoderAngleData = 0;
    //     int16_t tmpCalAngleData = 0;
    //     int16_t tmpcalcWheelAngle = 0;
    //     static int callbacktimes = 30;
    //     static long long lastTime;
    //     static long long curtime;
    //     static long long interveltime;
    //     FRIZY_LOG(LOG_DEBUG, "check the wheelToGyroAngle");
    //     curtime = getTime();
    //     if(escapSensor.leftw !=0 || escapSensor.rightw !=0)
    //     {
    //         interveltime = curtime - lastTime;
    //         FRIZY_LOG(LOG_DEBUG, "interveltime: %d ",interveltime);
    //         calcWheelAngleRato = interveltime*(escapSensor.leftw - escapSensor.rightw)*180/(0.22*_Pi); //周期可能会从在问题
    //         gyroAngleNow = 360-gyo_angle_*180/_Pi;
    //         FRIZY_LOG(LOG_DEBUG, "calcWheelAngleRato: %d , gyroAngleNow: %d",calcWheelAngleRato,gyroAngleNow);
    //         if(callbacktimes)
    //         {
    //             encoderCalAngleData += calcWheelAngleRato;

    //             calWheelAngleList.push_back(calcWheelAngleRato);
    //             calAngleList.push_back(gyroAngleNow);
    //             callbacktimes --;
    //         }
    //         else
    //         {
    //             auto delete_data =calWheelAngleList.begin();
    //             encoderCalAngleData =  encoderCalAngleData- *delete_data;
    //             auto anlge_front = calAngleList.begin();
    //             auto anlge_end = calAngleList.end();
    //             encoderAngleData = *anlge_end - *anlge_front;
    //             FRIZY_LOG(LOG_DEBUG, "encoderCalAngleData = %d , encoderAngleData = %d",encoderCalAngleData,encoderAngleData);
    //             calWheelAngleList.pop_front();
    //             calWheelAngleList.push_back(calcWheelAngleRato);
    //             calAngleList.pop_front();
    //             calAngleList.push_back(gyroAngleNow);
    //             if (encoderCalAngleData - encoderAngleData >20)
    //             {
    //                 return true;
    //                 FRIZY_LOG(LOG_DEBUG, "check the wheelToGyroAngle true");
    //             }
    //             else
    //             {
    //                 return false;
    //             }

    //         }
    //         lastTime = getTime();
    //     }

    // }

    bool EscapePlan::lidarCheck()
    {
        static int lidar_cnt_ = 0;
        // chassisEscape.GridPoint(&escapGrid);
        // share_mem_sync sync(current_pose);
        if (current_pose->exceptional_case == 2)
        {
            ++lidar_cnt_;
            FRIZY_LOG(LOG_DEBUG, "lidarcnt:%d", lidar_cnt_);
            if (lidar_cnt_ > 1000 / 50)
            {
                stuckType = ESCAPE_EVENT_TYPE_LIDAR_STUCK;
                lidar_cnt_ = 0;
                return true;
            }
        }
        else
        {
            if(lidar_cnt_ > 0)
            {
                lidar_cnt_--;
            }
        }
        return false;
    }

    bool EscapePlan::wheelToGyroAngleCheck()
    {
        bool retTmp = false;

        static int16_t lastGyroAngle = 0;
        int16_t gyroAngleNow;
        static int16_t calcWheelAngleRato = 0;
        int16_t tmpCalAngleData = 0;
        int16_t tmpcalcWheelAngle = 0;
        static int spdApproCnt = 0;
        static long long lastTime;
        static long long curtime;
        static long long interveltime;

        static long long normalStartTime;
        int aimSpdL = chassisEscape.getSendSpeed().first;
        int aimSpdR = chassisEscape.getSendSpeed().second;

        // FRIZY_LOG(LOG_DEBUG, "check the wheelToGyroAngle");
        FRIZY_LOG(LOG_INFO, "EAPaim[%d,%d]fdb[%d,%d]", aimSpdL, aimSpdR, escapSensor.leftw, escapSensor.rightw);
        curtime = getTime();
        int aimBaeGyro = 0;
        int fdbBaeGyro = 0;
        int fdbBaeAim = 0;

        if (aimSpdR != 0 || aimSpdL != 0)
        {
            interveltime = curtime - lastTime;
            int tmpAglSpd = 0;

            aimSpdCalAngleData = calcAimAglVelocity(); //目标速度差计算角速度 单位 0.1dps

            fdbSpdCalAngleData = calcFdbAglVeloctiy(); //反馈速度差计算角速度

            gyroAngleData = escapSensor.zGyroOriginal; //陀螺仪反馈的角速度

            if (0 != aimSpdCalAngleData)
            {
                aimBaeGyro = abs(100 * gyroAngleData / aimSpdCalAngleData);
                fdbBaeAim = abs(100 * fdbSpdCalAngleData / aimSpdCalAngleData);
            }

            if (0 != fdbSpdCalAngleData)
            {
                fdbBaeGyro = abs(100 * gyroAngleData / fdbSpdCalAngleData);
            }

            int diffValTmp = 0;

            if (abs(gyroAngleData) < abs(aimSpdCalAngleData))
            {
                diffValTmp = abs(abs(gyroAngleData) - abs(aimSpdCalAngleData)) / 10;
            }

            FRIZY_LOG(LOG_DEBUG, "aimAglSpd=%d,fdbAglSpd=%d,gySpd=%d,ag=%d,fa=%d,fg=%d,cnt=%d,DIFF=%d", aimSpdCalAngleData, fdbSpdCalAngleData, escapSensor.zGyroOriginal, aimBaeGyro, fdbBaeAim, fdbBaeGyro, escActPara.aglDiffCntSpin, diffValTmp);

            if (((aimSpdL + aimSpdR) == 0) || (abs(aimSpdL - aimSpdR) > 90))
            {
                if (fdbBaeGyro < 40 && aimBaeGyro < 55)
                {
                    escActPara.aglDiffCntSpin++;
                    if (escActPara.aglDiffCntSpin > 3000 / 50)
                    {
                        escActPara.aglDiffCntSpin = 0;
                        FRIZY_LOG(LOG_DEBUG, "spin aglReto err");
                        retTmp = true;
                    }
                }
                else
                {
                    if (escActPara.aglDiffCntSpin > 0)
                    {
                        escActPara.aglDiffCntSpin--;
                    }
                }
            }
            else //other wheel mode
            {
                spdApproCnt = 0;
                escActPara.ciffDealCnt = 0;
                if ((fdbBaeGyro < 40 && aimBaeGyro < 45 && diffValTmp > 50) || (fdbBaeGyro < 10 && aimBaeGyro < 10 && diffValTmp > 10))
                {
                    escActPara.g_AngleSpeedErrRate += diffValTmp;
                    escActPara.g_AngleSpeedErrCnt++;
                    if (escActPara.g_AngleSpeedErrCnt > 4000 / 50 && escActPara.g_AngleSpeedErrRate > 4000)
                    {
                        FRIZY_LOG(LOG_DEBUG, "g_AngleSpeedErr %d %d", escActPara.g_AngleSpeedErrCnt, escActPara.g_AngleSpeedErrRate);
                        escActPara.g_AngleSpeedErrRate = 0;
                        escActPara.g_AngleSpeedErrCnt = 0;
                        retTmp = true;
                    }
                    escActPara.retoNormalCnt = 0;
                }
                else
                {

                    if (escActPara.retoNormalCnt > 5000 / 50)
                    {
                        escActPara.g_AngleSpeedErrRate = 0;
                        escActPara.retoNormalCnt = 0;
                        escActPara.g_AngleSpeedErrRate = 0;
                        FRIZY_LOG(LOG_DEBUG, "clean g_AngleSpeedErr %d %d", escActPara.g_AngleSpeedErrCnt, escActPara.g_AngleSpeedErrRate);
                    }
                    else if (escActPara.retoNormalCnt > 2500 / 50)
                    {
                        if (escActPara.g_AngleSpeedErrRate > 200)
                        {
                            escActPara.g_AngleSpeedErrRate -= 200;
                        }
                        else
                        {
                            escActPara.g_AngleSpeedErrRate = 0;
                        }
                    }
                    else
                    {
                        escActPara.retoNormalCnt++;
                    }

                    if (escActPara.g_AngleSpeedErrCnt > 0)
                    {
                        escActPara.g_AngleSpeedErrCnt--;
                    }
                }
            }
            lastTime = getTime();
        }
        else
        {
            escActPara.ciffDealCnt = 0;
        }

        return retTmp;
    } //fun
    //警报检测
    Alert_type EscapePlan::alert_detect()
    {
        int32_t MainBrushCurAvg = 0;
        int32_t LeftWheelCurAvg = 0;
        int32_t RightWheelCurAvg = 0;
        int32_t LiftSignAvg = 0;

        MainBrushCurAvg = GlideFilterAD(mainbrushcurArry, AVG_WHEEL_CURARRY_NUM, escapSensor.rollBrushElec);
        LeftWheelCurAvg = GlideFilterAD(LeftWheelcurArry, AVG_LAST_WHEEL_CURARRY_NUM, escapSensor.leftWheelElec);
        RightWheelCurAvg = GlideFilterAD(RightWheelcurArry, AVG_LAST_WHEEL_CURARRY_NUM, escapSensor.rightWheelElec);
    }
    //脱困模式初始化
    bool EscapePlan::escapModeInit()
    {

        memset(escLastLeftCurAvg, 0, sizeof(escLastLeftCurAvg));
        memset(escLastRightCurAvg, 0, sizeof(escLastRightCurAvg));
        memset(escLeftCurArry, 0, sizeof(escLeftCurArry));
        memset(escRightCurArry, 0, sizeof(escRightCurArry));
        memset(pitRolLastAvg, 0, sizeof(pitRolLastAvg));
        memset(pitRolAvgArry, 0, sizeof(pitRolAvgArry));
        memset(gyroAccBuf, 0, sizeof(gyroAccBuf));
        memset(escPreActionArry, 0, sizeof(escPreActionArry));

        // memset(RightSideBrushCurArry, 0,sizeof(RightSideBrushCurArry));
        // memset(mainbrushcurArry, 0,sizeof(mainbrushcurArry));
        // memset(LeftWheelcurArry, 0,sizeof(LeftWheelcurArry));
        // memset(RightWheelcurArry, 0,sizeof(RightWheelcurArry));
        memset(PositionDiffArry, 0, sizeof(PositionDiffArry));

        memset(&bumpEscDet,0,sizeof(bumpEscCheck_t));
        memset(&escCheckPara, 0, sizeof(escapeCheckPara_t));
        memset(&escCkeckPara, 0, sizeof(stuckCheckPara_t));
        memset(&escActPara, 0, sizeof(escapeActionPara_t));
        memset(&sideCheck, 0, sizeof(sideCheck_t));

        ESCAPE_LeftOffCheckInit();
        stuckType = NOTHING;
        angVelOutCnt = 0;
        abnrCurrOutCntL = 0;
        abnrCurrOutCntR = 0;
        GroundAssistCkeckInit();
        callbacktimes = 0;
        recordPit.clear();
        recordRol.clear();
        recordPitRol.clear();
        act = 0;
        back_dis = 0, back_time = 0, spin_flag = 0, rotate_flag = 0;
        side_abnomal_index = false;
        side_brush_alert = false;
        chassisEscape.GridPoint(&escapGrid);
        escActPara.startCheckLdsX = escapGrid.realx * 15;
        escActPara.startCheckLdsY = escapGrid.realy * 15;
        escUpdataTime = 0;
    }

    //计算目标角速度 //单位 0.01dps
    int EscapePlan::calcAimAglVelocity(void)
    {
        int aimSpdL = chassisEscape.getSendSpeed().first;
        int aimSpdR = chassisEscape.getSendSpeed().second;

        if (abs(aimSpdL) > WHEEL_TURN_MAX_SPD) //防止超出最大速度,计算出一个很大的无法到达的目标线速度
        {
            if (aimSpdL > 0)
            {
                aimSpdL = WHEEL_TURN_MAX_SPD;
            }
            else
            {
                aimSpdL = 0 - WHEEL_TURN_MAX_SPD;
            }
        }

        if (aimSpdR > WHEEL_TURN_MAX_SPD) //timeV/防止超出最大速度,计算出一个很大的无法到达的目标线速度
        {
            if (aimSpdR > 0)
            {
                aimSpdR = WHEEL_TURN_MAX_SPD;
            }
            else
            {
                aimSpdR = 0 - WHEEL_TURN_MAX_SPD;
            }
        }

        return ((aimSpdR - aimSpdL) * WHEEL_SPACING * 180 / _Pi / 1000);
    }

    //计算反馈角速度 //单位 0.01dps
    int EscapePlan::calcFdbAglVeloctiy(void)
    {
        return ((escapSensor.rightw - escapSensor.leftw) * WHEEL_SPACING * 180 / _Pi / 1000);
    }
} // namespace useerobot