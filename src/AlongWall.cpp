/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-19 19:27:58
 * @Project      : UM_path_planning
 */


#include "navigation_algorithm/AlongWall.h"
#include "common_function/ExceptionHanding.h"

#define WALL_SLOWSPEED 150
#define WALL_STARTSPEED 180
#define WALL_NORMALSPEED 260
#define WALL_FASTSPEED 350

#define WALL_PIDSLOWSPEED 280
#define WALL_PIDSTARTSPEED 340
#define WALL_PIDNORMALSPEED 500
#define WALL_PIDFASTSPEED 660

#define WALL_RATATESPEED 120

#define WHEEL_SLOW_INDEX 0.4 
#define error_index 4
#define WALL_NORMALVALUE 3600
#define BLACK_WALL  1200
#define BACK_TIME 20
#define BACK_TIME_SMALLPLACE 15
#define BACK_TIME_FORBID 20
#define SPEED_COEFFIC 2.48333f

using namespace useerobot;
EscapePlan escape;
extern useerobot::Maze _maze;
extern float gyo_angle_;
extern int fanFlag;
static const float BOUND_X = 15.0;
static const float BOUND_Y = 15.0;
namespace useerobot
{

    extern SmallArea smallArea;

    bool alongwalk_run_index =false;
    static int backTime = BACK_TIME;
    bool first_flag = true;
    bool escapeVir = false;
    bool escapeFor = false;
    chassisBase tmpchass;
    bool firstAlongWall = false;;
    static Alongwallstate_t wallstate_t;
    static int lastValue = 0;
    static int minValueCnt = 0;
    static int maxValueCnt = 0;
    static int adjustTurnCnt = 0;
    static int lastObs = 0;
    static WHEELSTATE wheelSta;
    static int maxValue = 0;
    static int maxObs = 0;
    static int smallPlaceStartAngle = 0;
    static int smallPlaceFlag = 0, smallPlaceTurnCnt = 0; 
    static int enterSmallPlaceFlag = 0;
    static long long smallPlaceCnt = 0;
    WallFallowPara_t WallFallowPara;
    static long long exitTime;
    static int WALL_SPEED ;
    static int WALL_PIDSPEED ;
    static int forbid_flag = 0;
    AlongWall::AlongWall(/* args */)
    {
    }
    
    AlongWall::~AlongWall()
    {
    }

    bool AlongWall::wallBack()
    {
        while(backTime &&(GLOBAL_CONTROL != WHEEL_STOP))
        {
            FRIZY_LOG(LOG_DEBUG, "backTime: %d", backTime);
            backTime --;   
            chassis.chassisSpeed(-160,-160,1);
            return 0;
        }
        chassis.chassisSpeed(0,0,1);
        return 1;
    }

    long long getCurrentTime()
    {
        struct timeval time;

        gettimeofday(&time,NULL);

        return (long long)time.tv_sec*1000 + (long long)time.tv_usec/1000;
    }

    void StartWallFollow(WallFollowModel_t Wall_Follow_model, WallFollowDir_t dir, SPEED_t speed)
    {
        if((WallFallowPara.Model == Wall_Follow_model)&&(WallFallowPara.WallFollowRunState == WallFollow_Run))//防止反复调用
        {
            WallFallowPara.Speed = speed;
            return;
        }
        FRIZY_LOG(LOG_INFO,"StartWallFollow");
        

        WallFallowPara.TurnErrCnt = 0;
        memset(&WallFallowPara,0,sizeof(WallFallowPara_t));//复位沿墙参数
        // VTurnFlag = 0;
        // First5253Flag = 0;
        WallFallowPara.Speed = speed;

        WallFallowPara.Model = Wall_Follow_model;          //沿墙模式赋值
        WallFallowPara.Dir = dir;
        WallFallowPara.WallFollowRunState = WallFollow_Run;
        WallFallowPara.State = WallFollowAlong;
        WallFallowPara.RestrictedZone = 0;
        WallFallowPara.BoundaryZone = 0;
        if(WallFallowPara.Dir == LEFTAW)
            wallstate_t = LEFT_WALL;
        else if(WallFallowPara.Dir == RIGHTAW)
            wallstate_t = RIGHT_WALL;
        WallFallowPara.Dis = DEFALUT_WALL_VAL;
        WallFallowPara.BumpFlag = 0;
        if(fanFlag) {
            WallFallowPara.BumpState = BumpNo;
            WallFallowPara.BumpRecord = BumpNo;
        }
        WallFallowPara.BumpDealState = BumpNoAction;
        firstAlongWall = true;
        first_flag = true;
        backTime = BACK_TIME;
        escapeVir = false;
        escapeFor = false;
        alongwalk_run_index = true;
        forbid_flag = 0;
        // if(FAST_QUICK == WallFallowPara.Speed)
        // {
        //     smallPlaceFlag = 1;
        // }
        // else
        // {
        //     smallPlaceFlag = 0;
        // }
        
        // WallFallowPara.LastAddAngle = DRIV_AddAngleGetNoReset()/10;
        // WallFallowPara.Dis = DEFALUT_WALL_VAL;
        // WallFallowPara.ShiJian = xTaskGetTickCount();
    }

    Alongwallstate_t IsWall()
    {
        if (wallstate_t == EXIT_WALL)
            return EXIT_WALL;
        else if (wallstate_t == LEFT_WALL)
            return LEFT_WALL;
        else if (wallstate_t == RIGHT_WALL)
            return RIGHT_WALL;
    }

    WallFollowDir_t getAlongWallDir()
    {
        return WallFallowPara.Dir;
    }

    //停止沿墙
    void StopWallFollow()
    {

        // if(WallFallowPara.WallFollowRunState == WallFollow_Stop)//防止反复调用
        // {

        //     return;
        // }
        tmpchass.chassisSpeed(0,0,1);
        wallstate_t = EXIT_WALL;
        alongwalk_run_index = false;
        WallFallowPara.WallFollowRunState = WallFollow_Stop;
        WallFallowPara.StopFlag = 1;
        if(fanFlag != 0)
            fanFlag = 0;
        exitTime = getCurrentTime();
        FRIZY_LOG(LOG_INFO, "Stop Wall Follow");
    }

    //继续沿墙
    void ContinueWallFollow(void)
    {
        if(WallFollow_Pause != WallFallowPara.WallFollowRunState)//如果前面没调用暂停，这个程序就不执行
        {
            return;
        }
        WallFallowPara.WallFollowRunState = WallFollow_Run;
        WallFallowPara.State = WallFallowPara.PauseState;
    }
    //暂停沿墙
    void PauseWallFollow(void)
    {
        if(WallFollow_Run != WallFallowPara.WallFollowRunState)     //暂停状态和非沿墙状态下
        {
            return;
        }
        WallFallowPara.PauseState = WallFallowPara.State;
        WallFallowPara.PauseFlag = 1;
    }

    int AlongWall::getObsFront()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.midOmnibearingOn - currentSensor.midOmnibearingOff;
    }

    int AlongWall::getObsLeft()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.leftOmnibearingOn - currentSensor.leftOmnibearingOff;
    }
    
    int AlongWall::getObsRight()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.rightOmnibearingOn - currentSensor.rightOmnibearingOff;
    }

    int AlongWall::getAddAngle()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.addAngle;
    }

    int AlongWall::recognizeBlackWall()
    {
        adjustTurnCnt ++;
        uint8_t rightWallCheckFlag = 0;
        // chassis.GetSensor(&currentSensor);
        int angle = currentSensor.addAngle;
        // FRIZY_LOG(LOG_DEBUG, "currentSensor.addAngle:%d",currentSensor.addAngle);
        // if(WallFallowPara.Dir == RIGHTAW)
        // {
        //     if(WallFallowPara.BumpDealState == Turn)
        //     {
        //         if(rightWallCheck())
        //         {
        //             chassis.chassisSpeed(0, 0, 1);
        //             FRIZY_LOG(LOG_DEBUG, "ANGLE SUCCESSFUL");
        //             rightWallCheckFlag = 1;
        //         }
        //     }
        // }
        static uint32_t radarTrunCount = 0,obsTrunCount = 0;
        uint8_t obsFlag = 0,radarFlag = 0;
        if(WallFallowPara.BumpDealState == Turn)
        {
            if(WallFallowPara.BumpRecord == RadarLeft || WallFallowPara.BumpRecord == RadarRight || WallFallowPara.BumpRecord == RadarFront)
            {
                radarFlag |= 1;
                if(!currentSensor.wallData.wallObsLeft && !currentSensor.wallData.wallObsRight && !currentSensor.wallData.wallObsFront)
                {
                    radarTrunCount ++;
                    if(radarTrunCount > 200 / 20)
                    {
                        chassis.chassisSpeed(0, 0, 1);
                        radarTrunCount = 0;
                        FRIZY_LOG(LOG_DEBUG, "Radar SUCCESSFUL");
                        radarFlag |= 2;
                        // return 1;
                    }
                }
                else
                {
                    radarTrunCount = 0;
                }
            }
            else if(WallFallowPara.BumpRecord == OBSLeft || WallFallowPara.BumpRecord == OBSRight || WallFallowPara.BumpRecord == OBSFront)
            {
                obsFlag |= 1;
                if(!currentSensor.leftOmnibearingTurn && !currentSensor.rightOmnibearingTurn && !currentSensor.midOmnibearingTurn)
                {
                    obsTrunCount ++;
                    if(obsTrunCount > 200 / 20)
                    {
                        chassis.chassisSpeed(0, 0, 1);
                        obsTrunCount = 0;
                        FRIZY_LOG(LOG_DEBUG, "OBS SUCCESSFUL");
                        obsFlag |= 2;
                        // return 1;
                    }
                }
                else 
                {
                    obsTrunCount = 0;
                }
            }
            else
            {
                obsFlag = 0;
                radarFlag = 0;
            }
        }
        else
        {
            obsFlag = 0;
            radarFlag = 0;
            radarTrunCount = 0;
            obsTrunCount = 0;
        }
        if((WallFallowPara.BumpRecord == BumpInter && !recognize) || obsFlag || radarFlag)
        {
            if(WallFallowPara.Dir == LEFTAW)
            {
                if(currentSensor.leftAlongWall > lastValue && currentSensor.leftAlongWall > 100)
                {
                    if(minValueCnt)
                        minValueCnt--;
                    if(currentSensor.leftAlongWall > maxValue)
                    {
                        maxValueCnt++;
                        maxValue = currentSensor.leftAlongWall;
                    }
                }
                else 
                    minValueCnt++;
                lastValue = currentSensor.leftAlongWall;
                int leftObs = getObsLeft();
                if(leftObs > lastObs && leftObs > 200)
                {
                    if(leftObs > maxObs)
                        maxObs = leftObs;
                    // FRIZY_LOG(LOG_DEBUG, "leftobs:%d, lastobs:%d, maxobs:%d", leftObs, lastObs, maxObs);
                }
                lastObs = leftObs;
            }
            else if(WallFallowPara.Dir == RIGHTAW)
            {
                if(rightWallCheck())
                {
                    FRIZY_LOG(LOG_DEBUG, "ANGLE SUCCESSFUL");
                    rightWallCheckFlag = 1;
                }
                // FRIZY_LOG(LOG_DEBUG, "currentSensor.rightAlongWall:%d, lastvalue:%d, maxVal:%d", currentSensor.rightAlongWall, lastValue, maxValue);
                if(currentSensor.rightAlongWall > lastValue && currentSensor.rightAlongWall > 100)
                {
                    if(minValueCnt)
                        minValueCnt--;
                    if(currentSensor.rightAlongWall > maxValue)
                    {
                        maxValueCnt++;
                        maxValue = currentSensor.rightAlongWall;
                    }
                    FRIZY_LOG(LOG_DEBUG, "maxVal:%d", maxValue);
                }
                else 
                    minValueCnt++;
                lastValue = currentSensor.rightAlongWall;
                int rightObs = getObsRight();
                // FRIZY_LOG(LOG_DEBUG, "1.rightobs:%d, lastobs:%d, maxobs:%d", rightObs, lastObs, maxObs);
                if(rightObs > lastObs && rightObs > 200)
                {
                    if(rightObs > maxObs)
                        maxObs = rightObs;
                }
                lastObs = rightObs;
                // FRIZY_LOG(LOG_DEBUG, "2.rightobs:%d, lastobs:%d, maxobs:%d", rightObs, lastObs, maxObs);
            }
            // FRIZY_LOG(LOG_DEBUG, "maxValueCnt:%d, minValueCnt:%d, adjustTurnCnt:%d", maxValueCnt, minValueCnt, adjustTurnCnt);
            // FRIZY_LOG(LOG_DEBUG, "angle/10:%d, wallpara.starturn:%d, 1-2:%d",angle/10, WallFallowPara.StartTrunAngle, (angle / 10) - WallFallowPara.StartTrunAngle);
            if((maxValueCnt > 3/*minValueCnt > 2*/ && adjustTurnCnt >= 10 &&
            ((WallFallowPara.Dir == LEFTAW && ((angle / 10) - WallFallowPara.StartTrunAngle >= 30)) || \
            (WallFallowPara.Dir == RIGHTAW && (((angle / 10) - WallFallowPara.StartTrunAngle <= -30) /*&& rightWallCheckFlag*/)))) || \
            ( WallFallowPara.Dir == RIGHTAW && ( (radarFlag & 2) || (obsFlag & 2) ) && ((angle / 10) - WallFallowPara.StartTrunAngle <= -20) ) || \
            ( WallFallowPara.Dir == LEFTAW && ( (radarFlag & 2) || (obsFlag & 2) ) && ((angle / 10) - WallFallowPara.StartTrunAngle >= 20) ) )
            {
                FRIZY_LOG(LOG_DEBUG, "radarFlag:%d, obsFlag:%d angle:%d  %d", radarFlag, obsFlag, (int32_t)(angle / 10),(int32_t)WallFallowPara.StartTrunAngle);
                WallFallowPara.BumpCnt = 0;
                maxValueCnt = 0;
                // minValueCnt = 0;
                FRIZY_LOG(LOG_DEBUG, "maxValue:%d, maxObs:%d", maxValue, maxObs);
                FRIZY_LOG(LOG_DEBUG, "maxValue * 95 /100:%d", maxValue * 95 /100);
                if(maxValue * 95 /100 >= 1000)
                {
                    if(maxObs < 1600 && maxObs > 200/* && maxValue < 2000*/)
                    {
                        // isBlackWall = 1;
                        WallFallowPara.Dis = BLACK_WALL;
                    }
                    else
                    {
                        // isBlackWall = 0;
                        // WallFallowPara.Dis = maxValue * 95 / 100;
                        if(maxValue * 90 / 100 > 3700)
                        {
                            WallFallowPara.Dis = 3700;
                        }
                        else if(maxValue * 90 / 100 < 2700)
                        {
                            WallFallowPara.Dis = 2700;
                        }
                        else 
                        {
                            // WallFallowPara.Dis = WALL_NORMALVALUE;

                            WallFallowPara.Dis = maxValue * 90 / 100;
                        }
                    }
                    FRIZY_LOG(LOG_DEBUG, "WALL DIS:%d", WallFallowPara.Dis);
                }
                else
                {
                    WallFallowPara.Dis = WALL_NORMALVALUE;
                    FRIZY_LOG(LOG_DEBUG, "WALL DIS:%d", WALL_NORMALVALUE);
                    WallFallowPara.ValueErrStartTrunAngle = angle / 10;
                }
                adjustTurnCnt = 0;
                // WallFallowPara.BumpDealState = WaitTurn;
                WallFallowPara.BumpCnt = 0;
                WallFallowPara.TurnErrCnt = 0;
                // if(rightWallCheckFlag == 1)
                // {
                //     return 3;
                // }
                return 1;
            }
        }
        return 0;
    }

    int AlongWall::wallObsFront()
    {
        static float lastObsFront = 0;
        static float obsFront = 0;
        static uint32_t hshObsFrontCount = 0;
        float slopeObs = 0;
        static float lastSlopeObs = 0;
        static uint16_t slopeObsCount = 0;
        static uint8_t slopeObsFlag = 0;
        static uint8_t whiteFlag = 0;
        static uint8_t whiteCount = 0;
        
        // chassis.GetSensor(&currentSensor);
        if(WallFallowPara.State != WallFollowAlong)
        {
            hshObsFrontCount = 0;
            lastObsFront = 0;
            obsFront = 0;
            slopeObsCount = 0;
            slopeObsFlag = 0;
            lastSlopeObs = 0;
            whiteFlag = 0;
            whiteCount = 0;
            return 0;
        }
        hshObsFrontCount ++;
        obsFront += getObsFront();
        // FRIZY_LOG(LOG_DEBUG,"hshObsFrontCount:%d,obsFront += getObsFront():%f",hshObsFrontCount,obsFront);
        if(hshObsFrontCount > 20 / 20)
        {
            hshObsFrontCount = 0;
            if(lastObsFront != 0 && whiteFlag == 0)
            {
                obsFront /= (20 / 20) + 1;
                slopeObs = obsFront / lastObsFront;
                // FRIZY_LOG(LOG_DEBUG,"obsFront /= (20 / 20) + 1:%f,slopeObs:%f",obsFront,slopeObs);
                if(slopeObs > 1.02f)
                {
                    slopeObsCount ++;
                    // FRIZY_LOG(LOG_DEBUG,"slopeObsCount:%u",slopeObsCount);
                    if(slopeObsCount > 20)
                    {
                        slopeObsCount = 20;
                        slopeObsFlag = 1;
                    }
                }
                else
                {
                    if(slopeObsCount > 0)
                        slopeObsCount --;
                }
                if(slopeObsFlag == 1)
                {
                    // FRIZY_LOG(LOG_DEBUG,"slopeObsFlag == 1");
                    if(currentSensor.midOmnibearingTurn == 1)
                    {
                        // FRIZY_LOG(LOG_DEBUG,"currentSensor.midOmnibearingTurn");
                        if(slopeObs > 1.2f)
                        {
                            // FRIZY_LOG(LOG_DEBUG,"slopeObs > 1.2f");
                            slopeObsCount = 0;
                            slopeObsFlag = 0;
                            // FRIZY_LOG(LOG_DEBUG, "OBSFRONT OK--1");
                            // FRIZY_LOG(LOG_DEBUG, "slopeObs:%f,obsFront:%f,lastObsFront:%f,slopeObsCount:%d,slopeObsFlag:%d",slopeObs,obsFront,lastObsFront,slopeObsCount,slopeObsFlag);
                            return 1;
                        }
                        else if(lastSlopeObs > slopeObs && whiteFlag == 0)
                        {
                            // FRIZY_LOG(LOG_DEBUG,"lastSlopeObs > slopeObs && whiteFlag == 0");
                            whiteFlag = 1;
                            whiteCount = 0;
                        }
                    }
                    else
                    {
                        whiteFlag = 0;
                        whiteCount = 0;
                    }

                }
                lastSlopeObs = slopeObs;
            }
            else if(whiteFlag == 1)
            {
                whiteCount ++;
                // FRIZY_LOG(LOG_DEBUG,"whiteCount:%d\n",whiteCount);
                if(whiteCount > 1)
                {
                    whiteCount = 0;
                    whiteFlag = 0;
                    slopeObsCount = 0;
                    slopeObsFlag = 0;
                    FRIZY_LOG(LOG_DEBUG,"OBSFRONT OK--1");
                    return 1;
                }
            }
            
            lastObsFront = obsFront;
            obsFront = 0;
        }
        return 0;
    }

    Grid caculateAngle(Grid cur,Grid aim)
	{
		//0
		if (aim.x - cur.x > 0 && aim.y - cur.y == 0)
		{			
				aim.forward = 0;	     
		}
		//90
		else if (aim.x - cur.x == 0 && aim.y - cur.y < 0)
		{
				aim.forward = 90;
		}
		//270
		else if (aim.x - cur.x == 0 && aim.y - cur.y > 0)
		{
				aim.forward = 270;
		}		
		//180					
		else if (aim.x - cur.x < 0 && aim.y - cur.y == 0)
		{
				aim.forward = 180;
		}
		//other angle
		else if (abs(aim.x - cur.x) != 0 && abs(aim.y - cur.y) != 0)
		{
				float tempcan = float(abs(aim.y - cur.y))/abs(aim.x - cur.x);
				
				if (aim.y - cur.y > 0 && aim.x - cur.x > 0)
				{
						aim.forward = 360 - atan(tempcan) * 180 / 3.14;
				}
				if (aim.y - cur.y > 0 && aim.x - cur.x < 0)
				{
						aim.forward = 180 + atan(tempcan) * 180 / 3.14;
				}
				if (aim.y - cur.y < 0 && aim.x - cur.x < 0)
				{
						aim.forward = 180 - atan(tempcan) * 180 / 3.14;
				}
				if (aim.y - cur.y < 0 && aim.x - cur.x > 0)
				{
						aim.forward = atan(tempcan) * 180 / 3.14;
				}		
				//printf("jizhi.%f,%f\n",tempcan,atan(tempcan));						
		}		
		else
		{
				FRIZY_LOG(LOG_DEBUG,"goup");
				aim.forward = cur.forward;
		}

		return aim;
	}

    //是否在回充座禁区内
    bool AlongWall::isVirArea()
    {
        float x = current_pos.realx * 15 / 100;
        float y = current_pos.realy * 15 / 100;
        int cnt = 0;
        for(auto p : _maze.seatPoint)
        {
            if((sqrtf(((p.first - x) * (p.first - x)) + ((p.second - y) * (p.second - y))) < 0.1))
            {
                cnt ++;
                if(cnt > 1)
                {
                    FRIZY_LOG(LOG_DEBUG, "isVirArea");
                    return true;
                }
            }
        }
        return false;
    }

    //是否在设置的禁区内
    bool AlongWall::isForArea()
    {
        float x = current_pos.realx * 15 / 100;
        float y = current_pos.realy * 15 / 100;
        int cnt = 0;
        for(auto p : _maze.forbidenPoint)
        {
            if((sqrtf(((p.first - x) * (p.first - x)) + ((p.second - y) * (p.second - y))) < 0.1))
            {
                cnt ++;
                if(cnt > 1)
                {
                    FRIZY_LOG(LOG_DEBUG, "isForArea");
                    return true;
                }
            }
        }
        return false;
    }

    //远离
    bool AlongWall::isAwayVirArea()
    {
        float x = current_pos.realx * 15 / 100;
        float y = current_pos.realy * 15 / 100;
        float re_dis = 10000.0;
        for(auto p : _maze.seatPoint)
        {
            float tmp = sqrtf(((p.first - x) * (p.first - x)) + ((p.second - y) * (p.second - y)));
            if(re_dis > tmp)
                re_dis = tmp;
        }
        if(re_dis > 0.3)
            return true;
        else
            return false;
    }

    bool AlongWall::isAwayForArea()
    {
        float x = current_pos.realx * 15 / 100;
        float y = current_pos.realy * 15 / 100;
        float re_dis = 10000.0;
        for(auto p : _maze.forbidenPoint)
        {
            float tmp = sqrtf(((p.first - x) * (p.first - x)) + ((p.second - y) * (p.second - y)));
            if(re_dis > tmp)
                re_dis = tmp;
        }
        if(re_dis > 0.3)
            return true;
        else
            return false;
    }

    void AlongWall::bumpScan()
    {
        if(WallFallowPara.WallFollowRunState != WallFollow_Run)//不沿墙不做动作
        {
            return;
        }
        uint8_t ObsState = 0;
        float frontForDis = 0.0f;
        float frontVirDis = 0.0f;
        bool isInVirArea = isVirArea();//是否在回充座禁区内
        bool isInForArea = isForArea();//是否在禁区内
        uint8_t rightCheckFlag = 0;
        static uint8_t pidCount = 0;
        if(!isInVirArea)
        {
            if(escapeVir) {
                if(isAwayVirArea()) {
                    FRIZY_LOG(LOG_DEBUG, "away vir area");
                    escapeVir = false;
                }
            }
        }
        if(!isInForArea)
        {
            if(escapeFor) {
                if(isAwayForArea()) {
                    FRIZY_LOG(LOG_DEBUG, "away for area");
                    escapeFor = false;
                }
            }
        }
        WHEELSTATE wheel_sta;
        ObsState = wallObsFront();
        wheel_sta = chassis.getWheelState();

        if((currentSensor.wallData.rightWallDistance < 0.25f || WallFallowPara.Value > 1500) && (!currentSensor.wallData.wallObsFront && !currentSensor.midOmnibearingTurn))
        {
            if(currentSensor.wallData.wallObsRight || currentSensor.rightOmnibearingTurn)
            {
                // FRIZY_LOG(LOG_INFO,"rightCheckFlag--1:%d  %d\n",currentSensor.wallData.wallObsRight,currentSensor.rightOmnibearingTurn);
                // FRIZY_LOG(LOG_INFO,"rightCheckFlag--2:%d  %f %d %d %d", rightCheckFlag, \
                currentSensor.wallData.rightWallDistance,WallFallowPara.Value,currentSensor.wallData.wallObsFront,currentSensor.midOmnibearingTurn);
                currentSensor.wallData.wallObsRight = 0;
                currentSensor.rightOmnibearingTurn = 0;
            }
            rightCheckFlag = 0;
        }
        else
        {
            rightCheckFlag == 1;
        }
        bool isForEmpty = _maze.forbidenPoint.empty();
        if(!isForEmpty)
        {
            frontForDis = getBanDis(0, 0);
        }
        bool isVirEmpty = _maze.seatPoint.empty();
        frontVirDis = getBanDis(0, 1);
        if(isInForArea && !escapeFor && forbid_flag)   //进入禁区
        {
            escapeFor = true;
        }
        if(!forbid_flag)
            forbid_flag = 1;
        if(!isVirEmpty && isInVirArea && !escapeVir)    //进入回充座禁区
        {
            WallFallowPara.BumpRecord = IntoVir;
            WallFallowPara.BumpState = IntoVir;
            escapeVir = true;
        }
        //回充座虚拟墙
        else if(!isVirEmpty && !escapeVir && WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == RIGHTAW 
        && (frontVirDis < 0.25 || getBanDis(4, 1) < 0.15 || getBanDis(3, 1) < 0.15))
        {
            WallFallowPara.BumpRecord = ForbidChargeFront;
            WallFallowPara.BumpState = ForbidChargeFront;
            forOrVir = 1;
        }
        else if(!isVirEmpty && !escapeVir && WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == LEFTAW 
        && (frontVirDis < 0.25 || getBanDis(3, 1) < 0.15 || getBanDis(4, 1) < 0.15))
        {
            WallFallowPara.BumpRecord = ForbidChargeFront;
            WallFallowPara.BumpState = ForbidChargeFront;
            forOrVir = 1;
        }
        else if(!isVirEmpty && !escapeVir && WallFallowPara.State == WallFollowForbid && WallFallowPara.Dir == RIGHTAW 
        && (frontVirDis < 0.2 || getBanDis(3, 1) < 0.2))
        {
            WallFallowPara.BumpRecord = ForbidChargeRight;
            WallFallowPara.BumpState = ForbidChargeRight;
            forOrVir = 1;
        }
        else if(!isVirEmpty && !escapeVir && WallFallowPara.State == WallFollowForbid && WallFallowPara.Dir == LEFTAW 
        && (frontVirDis < 0.2 || getBanDis(4, 1) < 0.2))
        {
            WallFallowPara.BumpRecord = ForbidChargeLeft;
            WallFallowPara.BumpState = ForbidChargeLeft;
            forOrVir = 1;
        }
        //边界
        else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == RIGHTAW && (getBoudaryDis(0) < 0.25 || getBoudaryDis(4) < 0.3)
        && !WallFallowPara.BoundaryZone)
        {
            WallFallowPara.BumpRecord = BoundaryFront;
            WallFallowPara.BumpState = BoundaryFront;
        }
        else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == LEFTAW && (getBoudaryDis(0) < 0.25 || getBoudaryDis(3) < 0.3)
        && !WallFallowPara.BoundaryZone)
        {
            WallFallowPara.BumpRecord = BoundaryFront;
            WallFallowPara.BumpState = BoundaryFront;
        }
        else if(WallFallowPara.Dir == RIGHTAW && (getBoudaryDis(0) < 0.25 || getBoudaryDis(3) < 0.25) && WallFallowPara.BoundaryZone)
        {   
            WallFallowPara.BumpRecord = BoundaryRight;
            WallFallowPara.BumpState = BoundaryRight;
        }
        else if(WallFallowPara.Dir == LEFTAW && (getBoudaryDis(0) < 0.25 || getBoudaryDis(4) < 0.25) && WallFallowPara.BoundaryZone)
        {
            WallFallowPara.BumpRecord = BoundaryLeft;
            WallFallowPara.BumpState = BoundaryLeft;
        }
        //禁区
        else if(!isForEmpty && !escapeFor && !fanFlag && !WallFallowPara.RestrictedZone && WallFallowPara.Dir == RIGHTAW 
        && (frontForDis < 0.25 || getBanDis(4, 0) < 0.3))
        {
            WallFallowPara.BumpRecord = ForbidFront;
            WallFallowPara.BumpState = ForbidFront;
            forOrVir = 0;
        }
        else if(!isForEmpty && !escapeFor && !fanFlag && !WallFallowPara.RestrictedZone && WallFallowPara.Dir == LEFTAW 
        && (frontForDis < 0.25 || getBanDis(3, 0) < 0.3))
        {
            WallFallowPara.BumpRecord = ForbidFront;
            WallFallowPara.BumpState = ForbidFront;
            forOrVir = 0;
        }
        else if(!isForEmpty && !escapeFor && !fanFlag && WallFallowPara.RestrictedZone && WallFallowPara.Dir == RIGHTAW 
        && (frontForDis < 0.25 || getBanDis(3, 0) < 0.25))
        {
            WallFallowPara.BumpRecord = ForbidRight;
            WallFallowPara.BumpState = ForbidRight;
            forOrVir = 0;
        }
        else if(!isForEmpty && !escapeFor && !fanFlag && WallFallowPara.RestrictedZone && WallFallowPara.Dir == LEFTAW 
        && (frontForDis < 0.25 || getBanDis(4, 0) < 0.25))
        {
            WallFallowPara.BumpRecord = ForbidLeft;
            WallFallowPara.BumpState = ForbidLeft;
            forOrVir = 0;
        }
        //雷达信号
        else if(!smallPlaceFlag && currentSensor.wallData.wallObsFront/* && currentSensor.wallData.rightWallDistance < 0.27f*/
            /*&& (currentSensor.leftw > 120 && currentSensor.rightw > 120 && abs(currentSensor.leftw - currentSensor.rightw) <= 100)*/)
        {
            // if(currentSensor.leftw > 120 && currentSensor.rightw > 120)
            {
                WallFallowPara.BumpRecord = RadarFront;
                WallFallowPara.BumpState = RadarFront;
            }
            // else
            // {
            //     if(WallFallowPara.Dir == LEFTAW)
            //     {
            //         /*if(currentSensor.wallData.wallObsLeft)
            //         {
            //             WallFallowPara.BumpRecord = RadarLeft;
            //             WallFallowPara.BumpState = RadarLeft;
            //         }
            //         else */if(currentSensor.wallData.wallObsRight)
            //         {
            //             WallFallowPara.BumpRecord = RadarRight;
            //             WallFallowPara.BumpState = RadarRight;
            //         }
            //         else
            //         {
            //             WallFallowPara.BumpRecord = RadarFront;
            //             WallFallowPara.BumpState = RadarFront;
            //         }
            //     }
            //     else
            //     {
            //         /*if(currentSensor.wallData.wallObsRight)
            //         {
            //             WallFallowPara.BumpRecord = RadarRight;
            //             WallFallowPara.BumpState = RadarRight;
            //         }
            //         else */if(currentSensor.wallData.wallObsLeft)
            //         {
            //             WallFallowPara.BumpRecord = RadarLeft;
            //             WallFallowPara.BumpState = RadarLeft;
            //         }
            //         else
            //         {
            //             WallFallowPara.BumpRecord = RadarFront;
            //             WallFallowPara.BumpState = RadarFront;
            //         }
            //     }
            // }
        }
        // else if(currentSensor.wallData.wallObsLeft && !smallPlaceFlag && WallFallowPara.Dir != LEFTAW/* && currentSensor.wallData.rightWallDistance < 0.27f*/
        //         /*&& currentSensor.leftw > 50 && currentSensor.rightw > 50*/
        //     /*&& (currentSensor.leftw > 120 && currentSensor.rightw > 120 && abs(currentSensor.leftw - currentSensor.rightw) <= 20)*/)
        // {
        //     WallFallowPara.BumpRecord = RadarLeft;
        //     WallFallowPara.BumpState = RadarLeft;
        // }
        
        else if(currentSensor.wallData.wallObsRight && !smallPlaceFlag && WallFallowPara.Dir != RIGHTAW /*&& currentSensor.wallData.rightWallDistance < 0.27f*/
                /*&& currentSensor.leftw > 50 && currentSensor.rightw > 50*/
            /*&& (currentSensor.leftw > 120 && currentSensor.rightw > 120 && abs(currentSensor.leftw - currentSensor.rightw) <= 20)*/)
        {
            WallFallowPara.BumpRecord = RadarRight;
            WallFallowPara.BumpState = RadarRight;
        }
        //全方位中
        else if(WallFallowPara.State == WallFollowAlong && !smallPlaceFlag && /*currentSensor.midOmnibearingTurn*/ObsState == 1 
            /*&& (currentSensor.leftw > 120 && currentSensor.rightw > 120 && abs(currentSensor.leftw - currentSensor.rightw) <= 20)*/)
        {
            // if(currentSensor.leftw > 120 && currentSensor.rightw > 120)
            {
                WallFallowPara.BumpRecord = OBSFront;
                WallFallowPara.BumpState = OBSFront;
            }
            // else
            // {
            //     if(WallFallowPara.Dir == LEFTAW)
            //     {
            //         /*if(currentSensor.leftOmnibearingTurn)
            //         {
            //             WallFallowPara.BumpRecord = OBSLeft;
            //             WallFallowPara.BumpState = OBSLeft;
            //         }
            //         else */if(currentSensor.rightOmnibearingTurn)
            //         {
            //             WallFallowPara.BumpRecord = OBSRight;
            //             WallFallowPara.BumpState = OBSRight;
            //         }
            //         else
            //         {
            //             WallFallowPara.BumpRecord = OBSFront;
            //             WallFallowPara.BumpState = OBSFront;
            //         }
            //     }
            //     else
            //     {
            //         if(currentSensor.rightOmnibearingTurn)
            //         {
            //             WallFallowPara.BumpRecord = OBSRight;
            //             WallFallowPara.BumpState = OBSRight;
            //         }
            //         else if(currentSensor.leftOmnibearingTurn)
            //         {
            //             WallFallowPara.BumpRecord = OBSLeft;
            //             WallFallowPara.BumpState = OBSLeft;
            //         }
            //         else
            //         {
            //             WallFallowPara.BumpRecord = OBSFront;
            //             WallFallowPara.BumpState = OBSFront;
            //         }
            //     }
            // }
        }
        //全方位左
        // else if(/*WallFallowPara.State == WallFollowAlong &&*/!smallPlaceFlag &&  WallFallowPara.Dir != LEFTAW && currentSensor.leftOmnibearingTurn
        //         && currentSensor.leftw > 50 && currentSensor.rightw > 50
        //     /*&&  getObsLeft() > getObsFront() && getObsLeft() > getObsRight()*/)
        // {
        //     WallFallowPara.BumpRecord = OBSLeft;
        //     WallFallowPara.BumpState = OBSLeft;
        // }
        //全方位右
        else if(/*WallFallowPara.State == WallFollowAlong &&*/ !smallPlaceFlag && WallFallowPara.Dir != RIGHTAW && currentSensor.rightOmnibearingTurn
                && currentSensor.leftw > 50 && currentSensor.rightw > 50
            /*&&  getObsRight() > getObsFront() && getObsRight() > getObsLeft()*/)
        {
            WallFallowPara.BumpRecord = OBSRight;
            WallFallowPara.BumpState = OBSRight;
        }
        else if(currentSensor.leftCliff || currentSensor.mcuLeftCliff)
        {
            WallFallowPara.BumpRecord = CliffLeft;
            WallFallowPara.BumpState = CliffLeft;
        }
        else if(currentSensor.rightCliff || currentSensor.mcuRightCliff)
        {
            WallFallowPara.BumpRecord = CliffRight;
            WallFallowPara.BumpState = CliffRight;
        }
        else if(currentSensor.midCliff || currentSensor.mcuLeftMidCliff || currentSensor.mcuRightMidCliff)
        {
            WallFallowPara.BumpRecord = CliffInter;
            WallFallowPara.BumpState = CliffInter;
        }
        else if(currentSensor.bump)
        {
            WallFallowPara.BumpRecord = BumpInter;
            WallFallowPara.BumpState = BumpInter;
        }
        // //虚拟墙右
        // else if(currentSensor.rightVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualRight;
        //     WallFallowPara.BumpState = MagVirtualRight;
        // }
        // //虚拟墙左
        // else if(currentSensor.leftVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualLeft;
        //     WallFallowPara.BumpState = MagVirtualLeft;
        // }
        // //虚拟墙中
        // else if(currentSensor.leftFrontVir || currentSensor.rightFrontVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualInter;
        //     WallFallowPara.BumpState = MagVirtualInter;
        // }
        // //全方位左
        // else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir != LEFTAW && currentSensor.obs/*全方位转向*/
        //     &&  getObsLeft() > getObsFront() && getObsLeft() > getObsRight())
        // {
        //     WallFallowPara.BumpRecord = OBSLeft;
        //     WallFallowPara.BumpState = OBSLeft;
        // }
        // //全方位右
        // else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir != RIGHTAW && currentSensor.obs
        //     &&  getObsRight() > getObsFront() && getObsRight() > getObsLeft())
        // {
        //     WallFallowPara.BumpRecord = OBSRight;
        //     WallFallowPara.BumpState = OBSRight;
        // }
        
        else if (((WallFallowPara.BumpDealState < BumpSignalBack) || (WallFallowPara.BumpDealState > Virtual5253Turn)) &&
                 (WallFallowPara.Model != DockWallFollow) &&
                 !WallFallowPara.VTurnCnt &&
                  (isVirEmpty || (!isVirEmpty && 
                  sqrtf((((current_pos.realx * 15 / 100) - rechargeX) * ((current_pos.realx * 15 / 100) - rechargeX)) + 
                  (((current_pos.realy * 15 / 100) - rechargeY) * ((current_pos.realy * 15 / 100) - rechargeY))) > 2)))
        {
            // FRIZY_LOG(LOG_DEBUG, "check 41 singal");
            if(currentSensor.leftInfrared)//左侧检测出回充虚拟墙信号
            {
                // FRIZY_LOG(LOG_DEBUG, "leftInfrared signal");
                if(WallFallowPara.Dir != RIGHTAW)   //右沿墙不处理左侧回充座虚拟墙
                {
                    // WallFallowPara.Dir = LEFTAW;                      //包含无墙可能性
                    WallFallowPara.LastAddAngle = getAddAngle() / 10;
                    WallFallowPara.BumpRecord = InsideVirtual;
                    WallFallowPara.BumpState = InsideVirtual;
                    FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord = InsideVirtual");
                }
                else 
                {
                    WallFallowPara.BumpState = BumpNo;
                }
            }
            else if(currentSensor.rightInfrared)
            {
                // FRIZY_LOG(LOG_DEBUG, "rightInfrared signal");
                if(WallFallowPara.Dir != LEFTAW)    //左侧沿墙不处理右侧回充座虚拟墙  
                {
                    WallFallowPara.BumpRecord = InsideVirtual;
                    WallFallowPara.BumpState = InsideVirtual;
                    FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord = InsideVirtual");
                }
                else 
                {
                    WallFallowPara.BumpState = BumpNo;
                }
            }
            else if(currentSensor.leftFrontInfrared || currentSensor.rightFrontInfrared)    //前方检测出回充虚拟墙信号
            {
                FRIZY_LOG(LOG_DEBUG, "frontInfrared signal");
                WallFallowPara.BumpRecord = FrontVirtual;
                WallFallowPara.BumpState = FrontVirtual;
            }
            else 
            {
                WallFallowPara.BumpState = BumpNo;
            }
        }
        else 
        {
            WallFallowPara.BumpState = BumpNo;
        }
        
        if(smallPlaceFlag == 2 && WallFallowPara.BumpRecord == BumpInter)
        {
            WallFallowPara.FastWall_Count ++;
        }
        if(WallFallowPara.FastWall_Count && smallPlaceFlag == 2)
        {
            if(++ WallFallowPara.FastWall_Count> 1000 / 20)
            {
                WallFallowPara.FastWall_Count = 1000 / 20;
            }
            else
            {
                if(WallFallowPara.BumpRecord == BumpInter)
                {
                    WallFallowPara.BumpState = BumpNo;
                    //FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpState = BumpInter -> BumpNo");
                }
            }
        }
        
        if(smallPlaceFlag == 2)
        {
            smallPlaceFlag = 0;
        }
        static uint8_t wallBumpState = 0;
        if(wallBumpState != WallFallowPara.BumpState)
        {
            FRIZY_LOG(LOG_DEBUG, "Wallbump:%d,  %d\n",wallBumpState,WallFallowPara.BumpState);
            wallBumpState = WallFallowPara.BumpState;
        }
            // FRIZY_LOG(LOG_DEBUG, "wallData:%d,%d,%d  %f %d %d\n",\
            // currentSensor.wallData.wallObsFront,currentSensor.wallData.wallObsLeft , \
            // currentSensor.wallData.wallObsRight,currentSensor.wallData.rightWallDistance,currentSensor.leftw,currentSensor.rightw);
        if(WallFallowPara.BumpState == IntoVir)
        {
            chassis.chassisSpeed(0, 0, 1);
            WallFallowPara.State = WallFollowForbid;
            WallFallowPara.RestrictedZone = 1;
            WallFallowPara.BumpFlag = 1;
            WallFallowPara.BumpDealState = Turn;
            // FRIZY_LOG(LOG_DEBUG, "recharge x y:%d, %d", (int)round(rechargeX*100/15), (int)round(rechargeY*100/15));
            // FRIZY_LOG(LOG_DEBUG, "curPoint x y:%d, %d", current_pos.x, current_pos.y);
            Grid rechargeSeat = {(int)round(rechargeX*100/15), (int)round(rechargeY*100/15)};
            Grid curPoint = {current_pos.x, current_pos.y};
            Grid tmp = caculateAngle(curPoint, rechargeSeat);
            // FRIZY_LOG(LOG_DEBUG, "tmp.forward:%f", tmp.forward);
            virRotateAngle = tmp.forward - 110;
            if(virRotateAngle <= 0)
            {
                virRotateAngle = 360 + virRotateAngle;
            }
            pidCount = 0;
            FRIZY_LOG(LOG_DEBUG, "virRotateAngle:%d", virRotateAngle);
        }
        else if(WallFallowPara.BumpState == ForbidChargeFront || WallFallowPara.BumpState == ForbidChargeLeft || WallFallowPara.BumpState == ForbidChargeRight)
        {
            chassis.chassisSpeed(0, 0, 1);
            if(WallFallowPara.BumpState == ForbidChargeFront)
            {
                WallFallowPara.State = WallFollowForbid;
                WallFallowPara.BumpDealState = Turn;
            }
            else if(WallFallowPara.BumpState == ForbidChargeRight || WallFallowPara.BumpState == ForbidChargeLeft)
                WallFallowPara.BumpDealState = Turn;
            backTime = BACK_TIME_FORBID;
            WallFallowPara.BumpFlag = 1;
            pidCount = 0;            
            if(WallFallowPara.RestrictedZone == 1)
            {
                WallFallowPara.RestrictedZone = 0;
            }
            WallFallowPara.BoundaryZone = 0;
            FRIZY_LOG(LOG_DEBUG, "RestrictedZone--2:%d", WallFallowPara.BumpState);
        }
        else if(WallFallowPara.BumpState == BoundaryFront || WallFallowPara.BumpState == BoundaryLeft || WallFallowPara.BumpState == BoundaryRight)
        {
            chassis.chassisSpeed(0, 0, 1);
            if (WallFallowPara.BumpState == BoundaryFront)
            {
                WallFallowPara.BumpDealState = Turn;
            }
            else if(WallFallowPara.BumpState == BoundaryLeft || WallFallowPara.BumpState == BoundaryRight)
            {
                WallFallowPara.BumpDealState = BumpSignalBack;
            }
            backTime = BACK_TIME_FORBID;
            WallFallowPara.BumpFlag = 1;
            pidCount = 0;
            WallFallowPara.BoundaryZone = 1;
            WallFallowPara.State = WallFollowAlong;
            FRIZY_LOG(LOG_DEBUG, "BoundaryZone--3:%d, dealsta:%d", WallFallowPara.BumpState, WallFallowPara.BumpDealState);
            
        }
        else if(WallFallowPara.BumpState == ForbidFront || WallFallowPara.BumpState == ForbidLeft || WallFallowPara.BumpState == ForbidRight)
        {
            chassis.chassisSpeed(0, 0, 1);
            if(WallFallowPara.BumpState == ForbidFront)
            {
                // WallFallowPara.State = WallFollowForbid;
                WallFallowPara.BumpDealState = Turn;
            }
            else if(WallFallowPara.BumpState == ForbidRight || WallFallowPara.BumpState == ForbidLeft)                
                WallFallowPara.BumpDealState = BumpSignalBack;
            backTime = BACK_TIME_FORBID;
            WallFallowPara.BumpFlag = 1;
            pidCount = 0;
            WallFallowPara.RestrictedZone = 1;
            WallFallowPara.State = WallFollowAlong;
            FRIZY_LOG(LOG_DEBUG, "RestrictedZone--3:%d", WallFallowPara.BumpState);
        }
        else if(WallFallowPara.BumpState == OBSFront || WallFallowPara.BumpState == OBSLeft || WallFallowPara.BumpState == OBSRight \
         || WallFallowPara.BumpState == RadarFront || WallFallowPara.BumpState == RadarLeft || WallFallowPara.BumpState == RadarRight)
        {
            // if((WallFallowPara.BumpDealState > WaitBack)||(WallFallowPara.BumpDealState < BumpSignalBack))
            // {               
                if(currentSensor.Infrared || currentSensor.cliff)
                    chassis.chassisSpeed(0, 0, 1);
                else 
                {
                    // FRIZY_LOG(LOG_DEBUG, "wheel stop");
                    chassis.wheelCtrlStop();
                }
                fanFlag = 0;
                WallFallowPara.Dis = DEFALUT_WALL_VAL;
                WallFallowPara.BumpDealState = Turn;
                WallFallowPara.BumpFlag = 1;
                if(WallFallowPara.State == WallFollowVIRTUAL)
                {
                    WallFallowPara.VTurnCnt = 0;
                }
                if(WallFallowPara.BumpState == RadarFront && WallFallowPara.Dir == RIGHTAW \
                    && (currentSensor.wallData.rightWallDistance < 0.22f))          
                {      
                    FRIZY_LOG(LOG_DEBUG, "RadarFront Back");
                    WallFallowPara.BumpDealState = BumpSignalBack;
                    backTime = BACK_TIME_SMALLPLACE;
                }
                
            // }
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.BumpCnt = 0;
            WallFallowPara.NoWallCnt = 0; 
            pidCount = 0;
            if(WallFallowPara.RestrictedZone == 1)
            {
                WallFallowPara.RestrictedZone = 0;
            }
            WallFallowPara.BoundaryZone = 0;
        }
        else if(currentSensor.bump || currentSensor.cliff)
        {
            // if((WallFallowPara.BumpDealState > WaitBack)||(WallFallowPara.BumpDealState < BumpSignalBack))
            // {
            if(currentSensor.cliff)
            {
                FRIZY_LOG(LOG_DEBUG, "catch cliff signal");
                chassis.chassisSpeed(0, 0 ,1);
                WallFallowPara.BumpDealState = BumpSignalBack;
                WallFallowPara.BumpFlag = 1;
            }
            else if(/*currentSensor.magnVirWall ||*/ currentSensor.bump)          
            {      
                FRIZY_LOG(LOG_DEBUG, "catch bump signal");
                chassis.chassisSpeed(0, 0 ,1);
                WallFallowPara.BumpDealState = BumpSignalBack;
                WallFallowPara.BumpFlag = 1;
            }
            // }
            fanFlag = 0;
            if(smallPlaceFlag)
                backTime = BACK_TIME_SMALLPLACE;
            else
            {
                if(currentSensor.cliff)
                    backTime = 40;
                else
                    backTime = BACK_TIME;
            }
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.BumpCnt = 0;
            WallFallowPara.NoWallCnt = 0;
            WallFallowPara.FastWall_Count = 0;
            pidCount = 0;
            WallFallowPara.BoundaryZone = 0;
            if(currentSensor.bump && WallFallowPara.State == WallFollowForbid && isInVirArea)
            {
                WallFallowPara.BumpDealState = BumpSignalBack;
                WallFallowPara.BumpFlag = 1;
                backTime = BACK_TIME_FORBID;
            }
            else if(currentSensor.bump && WallFallowPara.State == WallFollowForbid)
            {
                if(WallFallowPara.Dir == LEFTAW)
                {
                    if(!forOrVir)
                    {
                        if(getBanDis(2, 0) > 0.3 && frontForDis > 0.3)
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            WallFallowPara.State = WallFollowAlong;
                        	WallFallowPara.RestrictedZone = 0;
                            backTime = BACK_TIME;
                        }
                        else
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            backTime = BACK_TIME_FORBID;
                        }
                    }
                    else
                    {
                        if(getBanDis(2, 1) > 0.25 && frontVirDis > 0.25)
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            WallFallowPara.State = WallFollowAlong;
                        	WallFallowPara.RestrictedZone = 0;
                            backTime = BACK_TIME;
                        }
                        else
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            backTime = BACK_TIME_FORBID;
                        }
                    }
                }
                else if(WallFallowPara.Dir == RIGHTAW)
                {
                    if(!forOrVir)
                    {
                        if(getBanDis(1, 0) > 0.3 && getBanDis(0, 0) > 0.3)
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            WallFallowPara.State = WallFollowAlong;
                        	WallFallowPara.RestrictedZone = 0;
                            backTime = BACK_TIME;
                        }
                        else
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            backTime = BACK_TIME_FORBID;
                        }
                    }
                    else
                    {
                        if(getBanDis(1, 1) > 0.25 && frontVirDis > 0.25)
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            WallFallowPara.State = WallFollowAlong;
                        	WallFallowPara.RestrictedZone = 0;
                            backTime = BACK_TIME;
                        }
                        else
                        {
                            WallFallowPara.BumpDealState = BumpSignalBack;
                            WallFallowPara.BumpFlag = 1;
                            backTime = BACK_TIME_FORBID;
                        }
                    }
                }
            }
            // if(currentSensor.magnVirWall)
            // {
            //     // FRIZY_LOG(LOG_DEBUG, "识别虚拟墙信号");
            //     WallFallowPara.State = WallFollowVIRTUAL;
            //     WallFallowPara.BumpDealState = TurnWithOutVirtual;
            //     WallFallowPara.BumpFlag = 1;
            //     FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpDealState = TurnWithOutVirtual");
            // }
            if(currentSensor.bump && (!currentSensor.Infrared) && WallFallowPara.State == WallFollowVIRTUAL)
            {
                chassis.chassisSpeed(0, 0 ,1);
                WallFallowPara.BumpDealState = BumpSignalBack;
                WallFallowPara.BumpFlag = 1;
                WallFallowPara.State = WallFollowAlong;
                backTime = BACK_TIME;
                FRIZY_LOG(LOG_DEBUG, "along virtual wall finished");
            }
            if(WallFallowPara.State == WallFollowVIRTUAL)
            {
                WallFallowPara.VTurnCnt = 0;
                // WallFallowPara.BumpDealState = BumpSignalBack;
            }
            // printf("WallFallowPara.State:%d %d\n",WallFallowPara.State,currentSensor.magnVirWall);
        }
        else if((WallFallowPara.BumpState == FrontVirtual || WallFallowPara.BumpState == InsideVirtual)
        /*&&((WallFallowPara.BumpDealState < BumpSignalBack)||(WallFallowPara.BumpDealState > WaitBack))*/)//后退不做处理
        {
            chassis.chassisSpeed(0, 0 ,1);
            WallFallowPara.BumpDealState = TurnWithOutVirtual;
            WallFallowPara.BumpFlag = 1;
            WallFallowPara.Dis = DEFALUT_WALL_VAL;
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.NoWallCnt = 0; 
            WallFallowPara.BumpCnt = 0;
            fanFlag = 0;
            pidCount = 0;
            if(WallFallowPara.RestrictedZone == 1)
            {
                WallFallowPara.RestrictedZone = 0;
            }
            WallFallowPara.BoundaryZone = 0;
        }
        else if(FAST_QUICK == WallFallowPara.Speed && (currentSensor.wallData.rightWallDistance > 0.25f && WallFallowPara.Value < 1500)/* && currentSensor.XAngle < -0.5f*/)
        {
            //UMAPI_CtrlBuzzer(2, 2700, 0, 0, 200);
            smallPlaceFlag = 2;
            WALL_SPEED = WALL_FASTSPEED;
            WALL_PIDSPEED = WALL_PIDFASTSPEED;
            pidCount = 0;
        }
        else if(    (currentSensor.midOmnibearingSlow == 1          && currentSensor.midOmnibearingTurn    == 0)                                    \
                 || (currentSensor.leftOmnibearingSlow == 1         && currentSensor.leftOmnibearingTurn   == 0 && WallFallowPara.Dir == RIGHTAW)   \
                 || (currentSensor.rightOmnibearingSlow == 1        && currentSensor.rightOmnibearingTurn  == 0 && WallFallowPara.Dir == LEFTAW)    \
                 || (currentSensor.wallData.wallObsFrontSlow == 1   && currentSensor.wallData.wallObsFront == 0)                                    \
                 || (currentSensor.wallData.wallObsLeftSlow  == 1   && currentSensor.wallData.wallObsLeft  == 0 && WallFallowPara.Dir == RIGHTAW)   \
                 || (currentSensor.wallData.wallObsRightSlow == 1   && currentSensor.wallData.wallObsRight == 0 && WallFallowPara.Dir == LEFTAW))     // 前全方位减速处理
        {
            pidCount = 0;
            WALL_SPEED = WALL_SLOWSPEED;
            WALL_PIDSPEED = WALL_PIDSLOWSPEED;
            // printf("WallSlow: %d-%d %d-%d %d-%d %d-%d %d-%d %d-%d",currentSensor.midOmnibearingSlow,currentSensor.midOmnibearingTurn,\
            // currentSensor.leftOmnibearingSlow,currentSensor.leftOmnibearingTurn,currentSensor.rightOmnibearingSlow,currentSensor.rightOmnibearingTurn,\
            // currentSensor.wallData.wallObsFrontSlow,currentSensor.wallData.wallObsFront,currentSensor.wallData.wallObsLeftSlow,currentSensor.wallData.wallObsLeft,\
            // currentSensor.wallData.wallObsRightSlow,currentSensor.wallData.wallObsRight);
            // chassis.chassisSpeed(currentSensor.leftw * WHEEL_SLOW_INDEX, currentSensor.rightw * WHEEL_SLOW_INDEX, 1);
        }
        else
        {
            if(pidCount ++ >= 4000 / 20 && currentSensor.leftw > 100 && currentSensor.rightw > 100 && abs(currentSensor.leftw - currentSensor.rightw) < 120
            && WallFallowPara.RestrictedZone == 0 && WallFallowPara.BoundaryZone == 0)
            {
                pidCount = 4000 / 20;
                WALL_SPEED = WALL_NORMALSPEED;
                WALL_PIDSPEED = WALL_PIDNORMALSPEED; 
            }
            else
            {
                if(pidCount)
                    pidCount --;
                WALL_SPEED = WALL_STARTSPEED;
                WALL_PIDSPEED = WALL_PIDSTARTSPEED;
            }
            // printf("wallSpwwd:%d  %d-%d %d %d\n",pidCount,currentSensor.leftw,currentSensor.rightw,WALL_SPEED,WALL_PIDSPEED);
        }
        // FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord:%d bumpstate:%d",WallFallowPara.BumpRecord, WallFallowPara.BumpState);
    }

    int AlongWall::bumpDeal()
    {
        static int turnCnt = 0; /*adjustTurnCnt = 0, lastGyroState = 0;*/
        static int lastBumpDealState;
        // chassis.GetSensor(&currentSensor);
        if(lastBumpDealState != WallFallowPara.BumpDealState)
        {
            FRIZY_LOG(LOG_DEBUG, "pBumpDealSta[%d->%d]\n",lastBumpDealState,WallFallowPara.BumpDealState);
        }
        lastBumpDealState = WallFallowPara.BumpDealState;
        // FRIZY_LOG(LOG_DEBUG, "bumpDeaState:%d", WallFallowPara.BumpDealState);
        switch(WallFallowPara.BumpDealState)
        {
            case BumpSignalBack:
                if(WallFallowPara.BumpRecord > BumpInter && WallFallowPara.BumpRecord < ForbidFront)
                {
                    if(chassis.getWheelState() == WHEELSTOP && WallFallowPara.WallFollowRunState == WallFollow_Run)
                    {
                        WallFallowPara.BumpCnt = 0;
                        chassis.chassisSpeed(-160, -160, 1);
                        // FRIZY_LOG(LOG_DEBUG, "send back speed");
                        WallFallowPara.BumpDealState = BackWithoutSignal;
                        // backTime = BACK_TIME;
                        return 0;
                    }
                    else
                    {
                        // FRIZY_LOG(LOG_DEBUG, "wheel not stop");
                        chassis.chassisSpeed(0, 0, 1);
                        return 0;
                    }
                }
                else 
                {
                    if(!wallBack())
                        return 0;
                    // FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord < CliffRight");
                    // backTime = BACK_TIME;
                    WallFallowPara.BumpDealState = Turn;
                    return 0;
                }
            break;

            case BackWithoutSignal:
                if(!currentSensor.cliff)
                {
                    WallFallowPara.BumpCnt ++;
                    if(WallFallowPara.BumpCnt > 10)
                    {
                        FRIZY_LOG(LOG_DEBUG, "no cliff signal, next step turn");
                        WallFallowPara.BumpCnt = 0;
                        chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpDealState = Turn;
                        return 0;
                    }
                    else
                        return 0;
                }
                else 
                {
                    FRIZY_LOG(LOG_DEBUG, "still has cliff signal, back continue");
                    WallFallowPara.BumpCnt = 0;
                    chassis.chassisSpeed(-160, -160, 1);
                    return 0;
                    
                }
            break;

            case WaitBack:

                // FRIZY_LOG(LOG_DEBUG, "WaitBack, bump:%d",currentSensor.bump);
                if(chassis.getWheelState() == WHEELSTOP)
                {
                    WallFallowPara.BumpCnt = 0;
                    if(!currentSensor.bump)
                    {
                        // backTime = BACK_TIME;
                        turnCnt ++;
                        if(turnCnt >= 3)//后退到旋转中间要间隔60ms，为路径做准备
                        {
                            turnCnt = 0;
                            WallFallowPara.BumpDealState = Turn;
                            return 0;
                        }
                        else
                            return 0;
                    }
                    // else 
                    // {
                    //     if(!wallBack())
                    //     {
                    //         return 0;
                    //     }
                    //     backTime = BACK_TIME;
                    // }
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"WaitBack chassisSpeed(0, 0, 1)");
                    chassis.chassisSpeed(0, 0, 1);
                    return 0;
                }    
            break;

            case Turn:
            {
                if(WallFallowPara.BumpRecord == IntoVir)
                {
                    if(!rotateSlam(virRotateAngle))
                        return 0; 
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--IntoVir");
                } 
                else if(WallFallowPara.BumpRecord == ForbidChargeFront)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(!rotaterobot(recordForward, 60))
                        return 0; 
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--ForbidChargeFront");
                }
                else if(WallFallowPara.BumpRecord == ForbidChargeLeft || WallFallowPara.BumpRecord == ForbidChargeRight)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                    }
                    if(!rotaterobot(recordForward, 35))
                        return 0;
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--ForbidChargeLeft ForbidChargeRight");
                }
                else if(WallFallowPara.BumpRecord == ForbidFront || WallFallowPara.BumpRecord == BoundaryFront)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(!rotaterobot(recordForward, 60))
                        return 0; 
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--ForbidFront BoundaryFront %d", WallFallowPara.BumpRecord);
                }
                else if(WallFallowPara.BumpRecord == ForbidLeft || WallFallowPara.BumpRecord == ForbidRight ||
                        WallFallowPara.BumpRecord == BoundaryLeft || WallFallowPara.BumpRecord == BoundaryRight)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                    }
                    if(!rotaterobot(recordForward, 35))
                        return 0;
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    if((current_pos.forward < 40 || current_pos.forward > 320) && WallFallowPara.WallForbidden.wallForward != 181)
                    {
                        if(++ WallFallowPara.WallForbidden.wallForward0Count > 2)
                        {
                            WallFallowPara.WallForbidden.wallForward = 1;
                        }
                        else
                        {
                            WallFallowPara.WallForbidden.wallForward = 400;
                        }
                        WallFallowPara.WallForbidden.wallForward90Count = 0;
                        WallFallowPara.WallForbidden.wallForward180Count = 0;
                        WallFallowPara.WallForbidden.wallForward270Count = 0;
                    }
                    else if(current_pos.forward < 130 && current_pos.forward > 50 && WallFallowPara.WallForbidden.wallForward != 271)
                    {
                        if(++ WallFallowPara.WallForbidden.wallForward90Count > 2)
                        {
                            WallFallowPara.WallForbidden.wallForward = 91;
                        }
                        else
                        {
                            WallFallowPara.WallForbidden.wallForward = 400;
                        }
                        WallFallowPara.WallForbidden.wallForward0Count = 0;
                        WallFallowPara.WallForbidden.wallForward180Count = 0;
                        WallFallowPara.WallForbidden.wallForward270Count = 0;
                    }
                    else if(current_pos.forward < 220 && current_pos.forward > 140 && WallFallowPara.WallForbidden.wallForward != 1)
                    {
                        if(++ WallFallowPara.WallForbidden.wallForward180Count > 2)
                        {
                            WallFallowPara.WallForbidden.wallForward = 181;
                        }
                        else
                        {
                            WallFallowPara.WallForbidden.wallForward = 400;
                        }
                        WallFallowPara.WallForbidden.wallForward0Count = 0;
                        WallFallowPara.WallForbidden.wallForward90Count = 0;
                        WallFallowPara.WallForbidden.wallForward270Count = 0;
                    }
                    else if(current_pos.forward < 310 && current_pos.forward > 230 && WallFallowPara.WallForbidden.wallForward != 91)
                    {
                        if(++ WallFallowPara.WallForbidden.wallForward270Count > 2)
                        {
                            WallFallowPara.WallForbidden.wallForward = 271;
                        }
                        else
                        {
                            WallFallowPara.WallForbidden.wallForward = 400;
                        }
                        WallFallowPara.WallForbidden.wallForward0Count = 0;
                        WallFallowPara.WallForbidden.wallForward90Count = 0;
                        WallFallowPara.WallForbidden.wallForward180Count = 0;
                    }
                    else
                    {
                        memset(&WallFallowPara.WallForbidden,0,sizeof(WallFallowForbiddenPara_t));
                        WallFallowPara.WallForbidden.wallForward = 400;
                    }
                    FRIZY_LOG(LOG_DEBUG, "wallForward = %f forward = %f", WallFallowPara.WallForbidden.wallForward,current_pos.forward);
                    FRIZY_LOG(LOG_DEBUG, "Turn--ForbidLeft ForbidRight BoundaryLeft BoundaryRight:%d", WallFallowPara.BumpRecord);
                }
                else if(WallFallowPara.BumpRecord == OBSFront || WallFallowPara.BumpState == RadarFront)
                {   
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 60))
                            return 0;
                    }
                    // chassis.chassisSpeed(0, 0, 1);
                    // WallFallowPara.Dis = DEFALUT_WALL_VAL;
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--OBSFront");
                }
                else if(WallFallowPara.BumpRecord == RadarLeft)
                {   
                    if(!rotateFlag)
                    {
                        recordForward = (360-gyo_angle_*180/_Pi);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(WallFallowPara.Dir == RIGHTAW)
                        {
                            if(!rotaterobot(recordForward, 135))
                                return 0;
                        }
                        else
                        {
                            if(!rotaterobot(recordForward, 35))
                                return 0;
                        }
                    }
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--RadarLeft");
                }
                else if(WallFallowPara.BumpRecord == RadarRight)
                {   
                    if(!rotateFlag)
                    {
                        recordForward = (360-gyo_angle_*180/_Pi);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(WallFallowPara.Dir == LEFTAW)
                        {
                            if(!rotaterobot(recordForward, 135))
                                return 0;
                        }
                        else
                        {
                            if(!rotaterobot(recordForward, 35))
                                return 0;
                        }
                    }
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--RadarRight");
                }
                else if(WallFallowPara.BumpRecord == OBSRight)
                {
                    // if(WallFallowPara.Dir == LEFTAW)
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            recordForward = (360-gyo_angle_*180/_Pi);
                            WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            rotateFlag = 1;
                            lastValue = 0;
                            minValueCnt = 0;
                            maxValueCnt = 0; 
                            adjustTurnCnt = 0;
                            lastObs = 0;
                            maxValue = 0;
                            maxObs = 0;
                        }
                        if(smallPlaceFlag)
                        {
                            if(!rotaterobot(recordForward, 20))
                                return 0;
                        }
                        else
                        {
                            if(WallFallowPara.Dir == LEFTAW)
                            {
                                if(!rotaterobot(recordForward, 135))
                                    return 0;
                            }
                            else
                            {
                                if(!rotaterobot(recordForward, 35))
                                    return 0;
                            }
                        }
                        // chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        FRIZY_LOG(LOG_DEBUG, "Turn--OBSRight");
                    }
                }
                else if(WallFallowPara.BumpRecord == OBSLeft)
                {
                    // if(WallFallowPara.Dir == RIGHTAW)
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            recordForward = (360-gyo_angle_*180/_Pi);
                            WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            rotateFlag = 1;
                            lastValue = 0;
                            minValueCnt = 0;
                            maxValueCnt = 0; 
                            adjustTurnCnt = 0;
                            lastObs = 0;
                            maxValue = 0;
                            maxObs = 0;
                        }
                        if(smallPlaceFlag)
                        {
                            if(!rotaterobot(recordForward, 20))
                                return 0;
                        }
                        else
                        {
                            if(WallFallowPara.Dir == RIGHTAW)
                            {
                                if(!rotaterobot(recordForward, 135))
                                    return 0;
                            }
                            else
                            {
                                if(!rotaterobot(recordForward, 35))
                                    return 0;
                            }
                        }
                        // chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        FRIZY_LOG(LOG_DEBUG, "Turn--OBSLeft");
                    }
                }
                else if(WallFallowPara.BumpRecord == BumpInter)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;     
                        recordForward = (360-gyo_angle_*180/_Pi);   //gyro
                        recordTime = getCurrentTime();
                        FRIZY_LOG(LOG_DEBUG, "rotate start recordTime:%lld\n", recordTime);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                        FRIZY_LOG(LOG_DEBUG, "Turn--BumpInter");
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                    }
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    // backTime = BACK_TIME;
                    recognize = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn BumpInter");
                }
                else if(WallFallowPara.BumpRecord == CliffInter || WallFallowPara.BumpRecord == CliffRight || WallFallowPara.BumpRecord == CliffLeft
                    ||  WallFallowPara.BumpRecord == MagVirtualInter || WallFallowPara.BumpRecord == MagVirtualRight || WallFallowPara.BumpRecord == MagVirtualLeft)
                {
                    if(!rotateFlag)
                    {
                        // chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                    }
                    // chassis.chassisSpeed(0, 0, 1);
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn--cliff:%d", WallFallowPara.BumpRecord);
                }
                FRIZY_LOG(LOG_DEBUG, "turn state finished:%d", WallFallowPara.BumpRecord);
                wallCheckInit();
                WallFallowPara.BumpCnt = 0;
                WallFallowPara.BumpRecord = BumpNo;
                WallFallowPara.BumpDealState = BumpNoAction;
                WallFallowPara.BumpFlag = 0;
            }
            break;
            
            case WaitTurn:
                if(chassis.getWheelState() == WHEELSTOP)
                {
                    WallFallowPara.BumpCnt = 0;
                    WallFallowPara.BumpDealState = BumpNoAction;
                    WallFallowPara.BumpFlag = 0;
                    WallFallowPara.BumpRecord = BumpNo;
                }
                else
                {
                    chassis.chassisSpeed(0, 0, 1);
                    return 0;
                }
                    
            break;

            case TurnWithOutVirtual:    // 41信号处理
            {   
                if(WallFallowPara.State != WallFollowVIRTUAL)
                {
                    WallFallowPara.State = WallFollowVIRTUAL;
                    if(WallFallowPara.BumpState == InsideVirtual)
                    // if(WallFallowPara.BumpRecord == MagVirtualLeft || WallFallowPara.BumpRecord == MagVirtualRight)
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 35))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        // backTime = BACK_TIME;
                        WallFallowPara.State = WallFollowVIRTUAL;
                    }
                    else 
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 55))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        // backTime = BACK_TIME;
                    }
                    WallFallowPara.BumpDealState = WaitTurn;
                }  
                else
                {
                    if(WallFallowPara.BumpRecord == InsideVirtual)
                    // if(WallFallowPara.BumpRecord == MagVirtualLeft || WallFallowPara.BumpRecord == MagVirtualRight)
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 35))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        // backTime = BACK_TIME;
                    }
                    else 
                    {
                        if(!rotateFlag)
                        {
                            // chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        // backTime = BACK_TIME;
                    }
                }
                // WallFallowPara.BumpDealState = WaitTurnOrEscape;
                WallFallowPara.VTurnCnt = 50;          
                WallFallowPara.BumpDealState = BumpNoAction;
                WallFallowPara.BumpFlag = 0;
                WallFallowPara.BumpCnt = 0;
                WallFallowPara.BumpRecord = BumpNo;
            }
            break;
        }
        // }
        //FRIZY_LOG(LOG_DEBUG,"BumpDealState:%d finished", WallFallowPara.BumpDealState);
        return 1;
    }

    void AlongWall::wallFollowDeal(void)
    {
        
        int16_t CinDSpeed = 0;
        int16_t wallLeftWheel = 0,wallRightWheel = 0;
        if(firstAlongWall)
        {
            if(first_flag)
            {
                first_flag = false;
                rotateFlag = 0;
                deal = 0;
                recognize = 0;
                escapeFlag = 0;
                enterForward = 360-gyo_angle_*180/_Pi;
                recordForward = 360-gyo_angle_*180/_Pi;
                if(!fanFlag && getCurrentTime() - exitTime > 10000)
                {
                    enterSmallPlaceFlag = 0;
                    smallPlaceStartAngle = getAddAngle() / 10;
                }
            }
            if(!fanFlag)
            {
                if(FAST_QUICK == WallFallowPara.Speed)
                {
                    if(!rotaterobot(recordForward, 20))
                        return;
                }
                else if(smallPlaceFlag)
                {
                    if(!rotaterobot(recordForward, 20))
                        return;
                }
                else
                {
                    if(!rotaterobot(recordForward, 45))
                        return;
                }
            }
            firstAlongWall = false;
            return;
        }
        // FRIZY_LOG(LOG_INFO,"WallFallowPara.State: %d",WallFallowPara.State);
        // else                                 //无碰撞，处理沿墙动作
        // {
            if(WallFallowPara.PauseFlag)
            {
                WallFallowPara.BumpDealState = BumpNoAction;
                chassis.chassisSpeed(0, 0, 1);
                WallFallowPara.BumpFlag = 0;
                WallFallowPara.PauseState = WallFallowPara.State; //记录暂停前机子的状态
                WallFallowPara.State = WallFollowPause;
            }
            lastWallState = WallFallowPara.State;
            switch (WallFallowPara.State)
            {
                case WallFollowStart:
                    if(!WallFallowPara.Model)
                        break;
                break;

                case WallFollowAlong:
                    // chassis.GetSensor(&currentSensor);
                    if(fanFlag || escapeFor)
                    {
                        if(WallFallowPara.Dir == LEFTAW)
                        {
                            chassis.chassisSpeed(WALL_SPEED * 0.55, WALL_SPEED, 1);
                        }
                        else
                        {
                            chassis.chassisSpeed(WALL_SPEED, WALL_SPEED * 0.55, 1);
                        }
                        break;
                    }
                    if(WallFallowPara.Dir == LEFTAW)            //通过沿墙方向读取对应的沿墙值
                    {
                        WallFallowPara.Value = currentSensor.leftAlongWall;
                        // FRIZY_LOG(LOG_DEBUG,"LEFT_WallFallowPara.Value:%d", WallFallowPara.Value);
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        WallFallowPara.Value = currentSensor.rightAlongWall;
                        // FRIZY_LOG(LOG_DEBUG,"RIGHT_WallFallowPara.Value:%d", WallFallowPara.Value);
                    }
                    if(smallPlaceFlag)
                    {
                        WallFallowPara.Dis = 3900;
                        //FRIZY_LOG(LOG_DEBUG, "smallPlaceFlag = 1, WallFallowPara.Dis = 3900;");
                    }
                    CinDSpeed = wallPid(WallFallowPara.Value,WallFallowPara.Dis);//进行pid调速
                    if(WallFallowPara.Dir == LEFTAW)//左沿墙
                    {
                        wallLeftWheel = WALL_SPEED - (CinDSpeed / 2);
                        wallRightWheel = WALL_SPEED + (CinDSpeed / 2);
                        if(wallRightWheel > 260/*WALL_SPEED + 20*/)
                        {
                            wallRightWheel = 260/*WALL_SPEED + 20*/;
                        }
                        if(wallLeftWheel <= 0)
                        {
                            wallLeftWheel = 1;
                        }
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)//右沿墙
                    {
                        wallLeftWheel = WALL_SPEED + (CinDSpeed / 2);
                        wallRightWheel = WALL_SPEED - (CinDSpeed / 2);
                        if(wallLeftWheel > WALL_SPEED + 20)
                        {
                            wallLeftWheel = WALL_SPEED + 20;
                        }
                        else if(wallLeftWheel < -(WALL_SPEED + 20))
                        {
                            wallLeftWheel = -(WALL_SPEED + 20);
                        }
                        if(wallRightWheel <= 0)
                        {
                            wallRightWheel = 1;
                        }
                        else if(wallRightWheel > WALL_SPEED + 20)
                        {
                            wallRightWheel = WALL_SPEED + 20;
                        }
                    }

                    chassis.chassisSpeed(wallLeftWheel, wallRightWheel, 1);
                    FRIZY_LOG(LOG_DEBUG,"send pid wall speed finished:%d  %d, %d", WallFallowPara.Dir,wallLeftWheel, wallRightWheel);
                    recognizeSmallPlace();   

                break;

                case WallFollowEnd:      
                    WallFallowPara.WallFollowRunState = WallFollow_Stop;
                    chassis.chassisSpeed(0, 0, 1);
                    memset(&WallFallowPara, 0, sizeof(WallFallowPara_t));//复位沿墙参数
                    FRIZY_LOG(LOG_INFO, "Stop AlongWall");
                break;

                case WallFollowPause:
                    if(WallFallowPara.PauseFlag)
                    {
                        FRIZY_LOG(LOG_INFO, "WallFollow Pause");
                    }
                    WallFallowPara.PauseFlag = 0;
                    WallFallowPara.WallFollowRunState = WallFollow_Pause;
                break;  

                case WallFollowVIRTUAL:  
                    FRIZY_LOG(LOG_DEBUG, "WallFollowVIRTUAL");
                    if(WallFallowPara.VTurnCnt)
                    {
                        WallFallowPara.VTurnCnt --;
                    }
                    if(WallFallowPara.Dir == LEFTAW)
                    {
                        chassis.chassisSpeed(WALL_SPEED * 0.6, WALL_SPEED, 1);
                        break;
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        chassis.chassisSpeed(WALL_SPEED, WALL_SPEED * 0.6, 1);
                        break;
                    }
                    // FRIZY_LOG(LOG_DEBUG, "along the virtual wall");
                break;

                case WallFollowForbid:
                    if(WallFallowPara.Dir == LEFTAW)
                    {
                        chassis.chassisSpeed(WALL_SPEED * 0.6, WALL_SPEED, 1);
                        break;
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        chassis.chassisSpeed(WALL_SPEED, WALL_SPEED * 0.6, 1);
                        break;
                    }
                break;
            }
        // }

    }

    void AlongWall::recognizeSmallPlace()
    {
        //获取陀螺仪累加角度
        int addAngle = getAddAngle() / 10;
        FRIZY_LOG(LOG_DEBUG, "addAngle:%d, smallPlaceStartAngle:%d, 1-2:%d", addAngle, smallPlaceStartAngle, addAngle - smallPlaceStartAngle);
        //小范围检测
        if(((addAngle - smallPlaceStartAngle >= 450 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle <= -450 && WallFallowPara.Dir == RIGHTAW)) &&
           !enterSmallPlaceFlag)
        {
            FRIZY_LOG(LOG_DEBUG, "check addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceCnt = getCurrentTime();
            enterSmallPlaceFlag = 1;
            smallPlaceStartAngle = addAngle;
        }
        //小范围判断
        if(((addAngle - smallPlaceStartAngle > 720 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle < -720 && WallFallowPara.Dir == RIGHTAW)) && enterSmallPlaceFlag == 1)
        {
            FRIZY_LOG(LOG_DEBUG, "judge addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceTurnCnt = 0;
            smallPlaceCnt = getCurrentTime();
            enterSmallPlaceFlag = 2;    
            FRIZY_LOG(LOG_DEBUG, "start smallplace mode");
            smallPlaceFlag = 1;
            smallPlaceStartAngle = addAngle;
            small_place_x = current_pos.x * 0.15;
            small_place_y = current_pos.y * 0.15;
        }
        else if(getCurrentTime() - smallPlaceCnt > 60000 && enterSmallPlaceFlag == 1)
        {
            FRIZY_LOG(LOG_DEBUG, "smallPlaceCnt > 60000");
            smallPlaceCnt = getCurrentTime();
            if((addAngle - smallPlaceStartAngle < 450 && WallFallowPara.Dir == LEFTAW) ||
               (addAngle - smallPlaceStartAngle > -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
                smallPlaceFlag = 0;
                enterSmallPlaceFlag = 0;
            }
            else if((addAngle - smallPlaceStartAngle > 450 && WallFallowPara.Dir == LEFTAW) ||
                    (addAngle - smallPlaceStartAngle < -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "start smallplace mode");
                smallPlaceTurnCnt = 0;
                enterSmallPlaceFlag = 2;
                smallPlaceFlag = 1;
                small_place_x = current_pos.x * 0.15;
                small_place_y = current_pos.y * 0.15;
            }
            smallPlaceStartAngle = addAngle;
        }
        if(((addAngle - smallPlaceStartAngle > 1080 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle < -1080 && WallFallowPara.Dir == RIGHTAW)) && enterSmallPlaceFlag == 2)
        {
            FRIZY_LOG(LOG_DEBUG, "judge addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceCnt = getCurrentTime();
            smallPlaceTurnCnt = 0;
            FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
            smallPlaceFlag = 0;
            enterSmallPlaceFlag = 0;
            smallPlaceStartAngle = addAngle;
        }
        else if(getCurrentTime() - smallPlaceCnt > 90000 && enterSmallPlaceFlag == 2)   //退出小范围
        {
            FRIZY_LOG(LOG_DEBUG, "smallPlaceCnt > 90000");
            smallPlaceCnt = getCurrentTime();
            if((addAngle - smallPlaceStartAngle <= 450 && WallFallowPara.Dir == LEFTAW) ||
               (addAngle - smallPlaceStartAngle >= -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
                smallPlaceFlag = 0;
                enterSmallPlaceFlag = 0;
                smallPlaceTurnCnt = 0;
            }
            smallPlaceStartAngle = addAngle;
        }   
    }

    void AlongWall::findWallByValue()
    {
        // chassis.GetSensor(&currentSensor);
        if(currentSensor.leftAlongWall > IR_BEGIN_ALONG && !WallFallowPara.Dir)
        {
            if(WallFallowPara.Model == DockWallFollow)  //暂时不管
            {
                // FUN_WheelCtrlStop();
                // if(NoDir == WallFallowPara.Dir)
                //     WallFallowPara.Dir = RIGHT;
                // WallFallowPara.BumpFlag = 1;
                // WallFallowPara.BumpDealState = WallValueTurn;
            }
            else
                WallFallowPara.Dir = LEFTAW;
        }
        else if(currentSensor.rightAlongWall > IR_BEGIN_ALONG && !WallFallowPara.Dir)
        {
            if(WallFallowPara.Model == UncleanWallFollow)   //暂时不管
            {
                // FUN_WheelCtrlStop();
                // WallFallowPara.Dir = LEFT;
                // WallFallowPara.BumpFlag = 1;
                // WallFallowPara.BumpDealState = WallValueTurn;
            }
            else
                WallFallowPara.Dir = RIGHTAW;
        }
    }

    void AlongWall::straightFindWall()
    {
        chassis.chassisSpeed(200, 200, 1);
        // chassis.GetSensor(&currentSensor);
        if(WallFallowPara.Dir == LEFTAW)            //通过沿墙方向读取对应的沿墙值
        {
            WallFallowPara.Value = currentSensor.leftAlongWall;
        }
        else if(WallFallowPara.Dir == RIGHTAW)
        {
            WallFallowPara.Value = currentSensor.rightAlongWall;
        }
        if(WallFallowPara.Value > 3000)
            chassis.chassisSpeed(0, 0, 1);
        
    }

    void AlongWall::chassisAlongWall()
    {
        FRIZY_LOG(LOG_INFO, "start to chassisWallAlong");
        bool tmp_index;
        // StartWallFollow(RandomWallFollow, LEFTAW, SLOW);
        while(1)
        {
            
            if(GLOBAL_CONTROL != WHEEL_RUN)
            {
                // FRIZY_LOG(LOG_DEBUG, "GLOBAL_CONTROL:%d", GLOBAL_CONTROL);
                if (GLOBAL_CONTROL == WHEEL_PAUSE){
                    chassis.chassisSpeed(0,0,1);
                }
                usleep(200 * 1000);
                continue;
            }
            else
            {
                // FRIZY_LOG(LOG_DEBUG, "alongwall task running");
                usleep(20 * 1000);
                //遥控器事件不沿墙
                if(alongwalk_run_index == true)
                {   
                    //FRIZY_LOG(LOG_DEBUG, "smallPlaceFlag:%d, WallFallowPara.BumpFlag:%d", smallPlaceFlag, WallFallowPara.BumpFlag);
                    chassis.GetSensor(&currentSensor);          //获取底盘数据
                    chassis.GridPoint(&current_pos);
                    //FRIZY_LOG(LOG_DEBUG, "along wall pos:%f %f", current_pos.realx*15/100, current_pos.realy*15/100);
                    //FRIZY_LOG(LOG_DEBUG, "along wall speed L:%d, R:%d", currentSensor.leftw, currentSensor.rightw);
                    
                    chassis.getPlanningInfo(&current_planning_info);
                    rechargeX = current_planning_info.charger_seat_position.x;
                    rechargeY = current_planning_info.charger_seat_position.y;
                    // FRIZY_LOG(LOG_DEBUG, "yuanshi shuju x y = %f %f", current_planning_info.charger_seat_position.x, current_planning_info.charger_seat_position.y);
                    // FRIZY_LOG(LOG_DEBUG, "recharge x y = %0.2f %0.2f ", rechargeX, rechargeY); 
                    if(currentSensor.leftInfrared || currentSensor.rightInfrared || 
                        currentSensor.leftFrontInfrared || currentSensor.rightFrontInfrared)
                    {
                        //FRIZY_LOG(LOG_DEBUG, "WallFallowPara.State:%d, VTurnCnt:%d", WallFallowPara.State, WallFallowPara.VTurnCnt);//2:正常沿墙 3:沿回充坐虚拟墙
                        //FRIZY_LOG(LOG_DEBUG, "Infrared L:%d R:%d FL:%d FR:%d", currentSensor.leftInfrared, currentSensor.rightInfrared,
                        //currentSensor.leftFrontInfrared, currentSensor.rightFrontInfrared);
                    }
                    if(smallPlaceFlag)
                    {
                        float tmp_dis = sqrtf((((current_pos.realx * 0.15) - small_place_x) * ((current_pos.realx * 0.15) - small_place_x)) + 
                        (((current_pos.realy * 0.15) - small_place_y) * ((current_pos.realy * 0.15) - small_place_y)));
                        if(tmp_dis > 1.5)
                        {
                            smallPlaceFlag = 0;
                            enterSmallPlaceFlag = 0;
                            smallPlaceStartAngle = currentSensor.addAngle / 10;
                            FRIZY_LOG(LOG_DEBUG, "away smallplace, quit slamplace mode");
                        }
                    }
                    if(!WallFallowPara.BumpFlag && !firstAlongWall)                            //处理机器类碰撞，不做扫描碰撞信息
                        bumpScan();
                    if(WallFallowPara.BumpFlag == 1)          //处理类碰撞动作，不处理沿墙动作
                    {
                        initPid();
                        bumpDeal();//机器 类碰撞  自旋处理
                    }
                    else
                    {
                        wallFollowDeal();    
                    }
                }
            }
            tmp_index = alongwalk_run_index;
        }
    }
    void AlongWall::initPid(void)
    {
        memset(&WallFallowPara.PID, 0 ,sizeof(PID_t));//复位沿墙参数
    }

    //NowValue 当前沿墙值， AimValue 目标沿墙值
    int16_t AlongWall::wallPid(uint16_t NowValue,uint16_t AimValue)
    {
        static int lastValue = 0;
        const static float index = 0.6;
        int16_t out;
        static uint32_t noWallCnt = 0;
        static uint8_t noWallFlag = 0;
        uint8_t radaWallFlag = 0;
        float nowAngle = 0.0f;
        static uint32_t radarWallCount = 0;
        // NowValue = NowValue * (1 - index) + lastValue * index;
        
		double distance = 0.0f;
        static double lastDistance = 0.0f;
        if(WallFallowPara.RestrictedZone == 1 || WallFallowPara.BoundaryZone == 1)
        {
            if(WallFallowPara.RestrictedZone == 1) {
                if(WallFallowPara.Dir == LEFTAW)
                {
                    distance = getwallBanDis(2, 0);
                }
                else
                {
                    distance = getwallBanDis(1, 0);
                }
            } else 
            {
                if(WallFallowPara.Dir == LEFTAW)
                {
                    distance = getwallBanDis(2, 1);
                }
                else
                {
                    distance = getwallBanDis(1, 1);
                }
            }
            distance *= 1000;        //转换为mm单位
            if(distance > 260)
            {
                distance -= 260;
            }
            else
            {
                distance = 0;
            }
            if(distance >= 50)
            {
                NowValue = 41209 * pow(distance, -1.027f) * 3300 / 4096;
            }
            else
            {
                NowValue = (47474 - 767.25 * distance) * 330 / 4096;//47474
            }
            AimValue = 3000;
            distance = lastDistance * 0.5f + distance * 0.5f;
            // WallFallowPara.PID.P = (AimValue - NowValue) / 8;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            // WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 6;
            // WallFallowPara.PID.Last_P = WallFallowPara.PID.P;
            // out = WallFallowPara.PID.P + WallFallowPara.PID.PD;
            // if(out > WALL_PIDSPEED)
            // {
            //     out = WALL_PIDSPEED;
            // }
            // else if(out < -WALL_PIDSPEED)
            // {
            //     out = -WALL_PIDSPEED;
            // }
            //当目标值等于0时，处理反馈值大于270°的情况
            nowAngle = current_pos.forward;
            if (nowAngle > 270 && 1 == WallFallowPara.WallForbidden.wallForward)
            {
                nowAngle = nowAngle - 360;
            }
            if(WallFallowPara.WallForbidden.wallForward != 400 && WallFallowPara.WallForbidden.wallForward != 0 && distance < 60)
            {
                WallFallowPara.PID.P = ((WallFallowPara.WallForbidden.wallForward - 1) - nowAngle) * 4;
                // WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
                WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 2;
                WallFallowPara.PID.Last_P = WallFallowPara.PID.P;
                out = WallFallowPara.PID.P + WallFallowPara.PID.PD;
                if(out > WALL_PIDSPEED)
                {
                    out = WALL_PIDSPEED;
                }
                else if(out < -WALL_PIDSPEED)
                {
                    out = -WALL_PIDSPEED;
                }
                //FRIZY_LOG(LOG_DEBUG, "JIN-1: P:%d I:%d D:%d distance:%d nowAngle:%f wallForward:%f out:%d", \
                //WallFallowPara.PID.P,WallFallowPara.PID.PI,WallFallowPara.PID.PD,(uint32_t)distance,nowAngle,WallFallowPara.WallForbidden.wallForward,out);
                return out;
            }
            // if(distance < 30)
            //     WallFallowPara.PID.P = (distance - 40) * 4;
            // else
            WallFallowPara.PID.P = (distance - 10) * 2;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 2;
            WallFallowPara.PID.Last_P = WallFallowPara.PID.P;
            out = WallFallowPara.PID.P + WallFallowPara.PID.PD;
            if(out > WALL_PIDSPEED)
            {
                out = WALL_PIDSPEED;
            }
            else if(out < -WALL_PIDSPEED)
            {
                out = -WALL_PIDSPEED;
            }
            lastDistance = distance;
            //FRIZY_LOG(LOG_DEBUG, "JIN-2: P:%d I:%d D:%d distance:%d NowValue:%d AimValue:%d out:%d", \
            //WallFallowPara.PID.P,WallFallowPara.PID.PI,WallFallowPara.PID.PD,(uint32_t)distance,NowValue,AimValue,out);
            return WALL_PIDSPEED - 60;
        }
        if(NowValue < 500 && currentSensor.wallData.rightWallDistance * 1000 < 1000)
        {
            radarWallCount ++;
            if(radarWallCount > 400 / 20)
            {
                radarWallCount = 400 / 20;
            }
        }
        else
        {
            // if(radarWallCount)
            //     radarWallCount --;
            radarWallCount = 0;
        }
        if(radarWallCount)
        {
            NowValue = currentSensor.wallData.rightWallDistance * 1000;
            if(NowValue > 160)
                NowValue -= 160;
            else 
                NowValue = 0;
    //        if(data.AdcToDmaData.RightWall < 1000)
            {
                if(NowValue >= 50)
                {
                    NowValue = 41209 * pow(NowValue, -1.027f) * 3300 / 4096;
                }
                else
                {
                    NowValue = (47474 - 767.25 * NowValue) * 330 / 4096;//47474
                }
                AimValue = 3700;
                radaWallFlag = 1;
                //FRIZY_LOG(LOG_DEBUG,"radarNowValue:%f %d",currentSensor.wallData.rightWallDistance * 1000,NowValue);
                
            }
        }
        if(NowValue < 500)
        {
            AimValue = 3700;
            noWallCnt ++;
            if(noWallCnt > 100 / 20)
            {
                noWallCnt = 200 / 20;
                noWallFlag = 1;
            }

        }
        else
        {
            if(noWallCnt)
            {
                noWallCnt --;
            }
            else
            {
                noWallFlag = 0;
            }
            
        }
        if(AimValue == BLACK_WALL)
        {
            // if(NowValue < 500)
            // {
            //     WallFallowPara.PID.P = (AimValue - NowValue) / 5;
            // }
            // else
            // {
            //     WallFallowPara.PID.P = (AimValue - NowValue) / 16;
            // }
            if(noWallFlag == 1)
            {
                AimValue = WALL_NORMALVALUE;
                WallFallowPara.PID.P = (AimValue - NowValue) / 8;
            }
            // {
            //     WallFallowPara.PID.P = (AimValue - NowValue) / 3;
            // }
            else
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 12;
            }
            
            WallFallowPara.PID.PI += WallFallowPara.PID.P;
            WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 6;
            // WallFallowPara.PID.P = (AimValue - NowValue)/11;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P;
            // WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P)*4;
        }
        else
        {
            if(radaWallFlag)
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 8;
            }
            else
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 12;
            }
            WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 6;
            // if(NowValue > 3100)
            //     WallFallowPara.PID.P = (AimValue - NowValue);
            // else
            //     WallFallowPara.PID.P = (AimValue - NowValue)/8;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            // WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P)*8;
        }
        if(WallFallowPara.PID.PI > 30)
        {
            WallFallowPara.PID.PI = 30;
        }
        else if(WallFallowPara.PID.PI < -30)
        {
            WallFallowPara.PID.PI = -30;
        }
        WallFallowPara.PID.Last_P = WallFallowPara.PID.P;
        out = WallFallowPara.PID.P + WallFallowPara.PID.PD;
        //FRIZY_LOG(LOG_DEBUG, "out:%d", out);
        // if(AimValue == BLACK_WALL)
        // {
        //     if(out > 180)
        //     {
        //         out = 180;
        //     }
        //     else if(out < -180)
        //     {
        //         out = -180;
        //     }
        // }
        // else
        // if(radaWallFlag == 1)
        {
            if(out > WALL_PIDSPEED)
            {
                out = WALL_PIDSPEED;
            }
            else if(out < -WALL_PIDSPEED)
            {
                out = -WALL_PIDSPEED;
            }
        }
        // else
        // {
        //     if(out > WALL_PIDSPEED)
        //     {
        //         out = WALL_PIDSPEED;
        //     }
        //     else if(out < -WALL_PIDSPEED)
        //     {
        //         out = -WALL_PIDSPEED;
        //     }
        // }
        //FRIZY_LOG(LOG_DEBUG, "P:%d I:%d D:%d NowValue:%d AimValue:%d out:%d", WallFallowPara.PID.P,WallFallowPara.PID.PI,WallFallowPara.PID.PD,NowValue,AimValue,out);
        //FRIZY_LOG(LOG_DEBUG, "XAngle-1:%f",currentSensor.XAngle);
        lastValue = NowValue;
        return out;
    }

    static uint8_t lWallType = 0,rWallType = 0; //沿墙类型 1：黑墙 2：灰墙 白墙
    static uint16_t lWallMaxValue = 0,rWallMaxValue = 0;//求斜率时 存储沿墙最大值
    static double leftWallDiff = 0,lastLeftWallDiff = 0,lastLeftWall = 0,rightWallDiff = 0,lastRightWallDiff = 0,lastRightWall = 0;//沿墙差值
    static double leftWallSlope = 0,rightWallSlope = 0;//沿墙值斜率
    static uint8_t leftWallFlag = 0,rightWallFlag = 0;//是否满足沿墙标志

    void AlongWall::wallCheckInit(void)
    {
        FRIZY_LOG(LOG_DEBUG, "wall check Init\n");
        lWallType = 0;
        rWallType = 0;
        lWallMaxValue = 0;
        rWallMaxValue = 0;
        leftWallDiff = 0;
        lastLeftWallDiff = 0;
        lastLeftWall = 0;
        rightWallDiff = 0;
        lastRightWallDiff = 0;
        lastRightWall = 0;
        leftWallSlope = 0;
        rightWallSlope = 0;
        leftWallFlag = 0;
        rightWallFlag = 0;
    }

    //右沿墙斜率判断
    int8_t AlongWall::rightWallCheck(void)
    {
        static uint8_t rWallFlagSum = 0;
        static uint16_t rWallCheckCount = 0;
        rWallCheckCount ++;
        if(rWallCheckCount >= 40 / 20)
        {
            rightWallDiff = currentSensor.rightAlongWall - lastRightWall;//计算当前沿墙与上次沿墙的差值

            if(lastRightWallDiff)
                rightWallSlope = rightWallDiff / lastRightWallDiff;//当前差值与上次差值 求斜率
            else
                rightWallSlope = 1;

            if(currentSensor.rightAlongWall > 600)//沿墙大于800
            {
                if(currentSensor.rightAlongWall > lastRightWall)//沿墙值比上一次值大
                {
                    if(currentSensor.rightAlongWall > rWallMaxValue)//若当前沿墙值比最大记录沿墙值还大 则记录最大沿墙值
                    {
                        rWallMaxValue = currentSensor.rightAlongWall;
                    }
    //                printf("senR .%d .%d\n",rWallMaxValue,currentSensor.rightAlongWall);
                }
                else
                {
                    if(rWallMaxValue > 600 && rWallMaxValue < 2800)//若最大沿墙值为600-2000之间  则判断为黑墙
                    {
                        rWallType = 1;
                    }
                    else if(rWallMaxValue >= 2800)//则判断为其他墙
                    {
                        rWallType = 2;
                    }
                    else
                    {
                        rWallType = 0;
                    }
                }
            }
            
            
            if(rWallType == 1)
            {
                if(rightWallSlope < -2 && lastRightWall > currentSensor.rightAlongWall) //求得斜率>1 同时上次的沿墙值比当前的大
                {
                    rWallFlagSum ++;
                }
                else if(rightWallSlope > 0.6f && lastRightWall > currentSensor.rightAlongWall) //求得斜率>1 同时上次的沿墙值比当前的大
                {
                    rWallFlagSum ++;
                    if(rWallFlagSum >= 2)
                    {
                        rWallFlagSum = 0;
                        rightWallFlag = 1;
                        rWallMaxValue = 0;
                        rWallType = 0;
                    }
                }
                else
                {
                    if(rWallFlagSum > 0)
                        rWallFlagSum --;
                }  
            }
            else if(rWallType == 2)
            {
                if(rightWallSlope < -2 && lastRightWall > currentSensor.rightAlongWall) //求得斜率>1 同时上次的沿墙值比当前的大
                {
                    rWallFlagSum ++;
                }
                else if(rightWallSlope > 1 && lastRightWall > currentSensor.rightAlongWall) //求得斜率>1 同时上次的沿墙值比当前的大
                {
                    rWallFlagSum ++;
                    if(rWallFlagSum >= 3)
                    {
                        rWallFlagSum = 0;
                        rightWallFlag = 1;
                        rWallMaxValue = 0;
                        rWallType = 0;
                    }
                }
                else
                {
                    if(rWallFlagSum > 0)
                        rWallFlagSum --;
                }  
            }
            else
            {
                rWallFlagSum = 0;
            }
                        

        //    FRIZY_LOG(LOG_DEBUG, "sen Rwall:%d,  %0.1f,  %0.1f,  %0.1f,  %0.2f .%d .%d .%d .%d\n"
        //    ,currentSensor.rightAlongWall,lastRightWall,rightWallDiff,lastRightWallDiff,rightWallSlope,rightWallFlag,rWallFlagSum,rWallMaxValue,rWallType);
            lastRightWall = currentSensor.rightAlongWall;
            lastRightWallDiff = rightWallDiff;
            rWallCheckCount = 0;
        }
        
        return rightWallFlag;

    //     static uint8_t rWallFlagSum = 0;
    //     static uint16_t rWallCheckCount = 0;
    //     double nowWallValue;
    //     rWallCheckCount ++;
    //     if(rWallCheckCount >= 40 / 20)
    //     {
    //         nowWallValue = currentSensor.wallData.rightWallDistance;
    //         if(nowWallValue == lastRightWall)
    //             return 0;
    //         rightWallDiff = lastRightWall - nowWallValue;//计算当前沿墙与上次沿墙的差值
    //         if(lastRightWallDiff != 0)
    //             rightWallSlope = rightWallDiff / lastRightWallDiff;//当前差值与上次差值 求斜率
            
    //         if(nowWallValue < 0.35f)//沿墙小于35cm
    //         {
    //             if(nowWallValue < lastRightWall)//沿墙值比上一次值大
    //             {
    //                 if(nowWallValue < rWallMinValue)//若当前沿墙值比最大记录沿墙值还大 则记录最大沿墙值
    //                 {
    //                     rWallMinValue = nowWallValue;
    //                 }
    // //                printf("senR .%d .%d\n",rWallMinValue,nowWallValue);
    //             }
    //         }
            
    //         if(rightWallSlope < -2 && lastRightWall > nowWallValue) //求得斜率>1 同时上次的沿墙值比当前的大
    //         {
    //             rWallFlagSum ++;
    //         }
    //         else if(rightWallSlope > 0.6f && lastRightWall > nowWallValue) //求得斜率>1 同时上次的沿墙值比当前的大
    //         {
    //             rWallFlagSum ++;
    //             if(rWallFlagSum >= 2)
    //             {
    //                 rWallFlagSum = 0;
    //                 rightWallFlag = 1;
    //                 rWallMinValue = 0;
    //             }
    //         }
    //         else
    //         {
    //             if(rWallFlagSum > 0)
    //                 rWallFlagSum --;
    //         }         

    //         FRIZY_LOG(LOG_DEBUG, "sen Rwall:%f,  %f,  %f,  %f,  %f .%d .%d .%d\n",\
    //         nowWallValue,lastRightWall,rightWallDiff,lastRightWallDiff,rightWallSlope,rightWallFlag,rWallFlagSum,rWallMinValue);
    //         lastRightWall = nowWallValue;
    //         lastRightWallDiff = rightWallDiff;
    //         rWallCheckCount = 0;
    //     }
        
    //     return rightWallFlag;
    }

    bool AlongWall::rotaterobot(float recordforward, int angle)
    {
            float cur_foward,aimforward;
            uint8_t recognizeFlag = 0;
            // chassis.GridPoint(&current_pos);
            // cur_foward = current_pos.forward;
            if(WallFallowPara.Dir == RIGHTAW)
            {   
                aimforward = recordforward - angle;
                if(aimforward <= 0)
                {
                    aimforward = 360 + aimforward;
                }
                recognizeFlag = recognizeBlackWall();
                // FRIZY_LOG(LOG_DEBUG, "aimforward: %f, recordforward: %f",aimforward, recordforward);
                chassis.chassisSpeed(-WALL_RATATESPEED, WALL_RATATESPEED, 1);   
                // chassis.GridPoint(&current_pos);
                // FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, fabs(360-gyo_angle_*180/_Pi));
                // if(fabs(current_pos.forward - aimforward) < error_index || fabs(current_pos.forward - aimforward) > (360 - error_index))
                
                if(recognizeFlag & 1)
                {
                    recognize = 1;
                    chassis.chassisSpeed(0, 0, 1);
                    return 1;
                }
            
                if(fabs((360-gyo_angle_*180/_Pi) - aimforward) < error_index || fabs((360-gyo_angle_*180/_Pi) - aimforward) > (360 - error_index))
                {   
                    chassis.chassisSpeed(0, 0, 1);
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                    return 1;
                }
            }
            else if(WallFallowPara.Dir == LEFTAW)
            {   
                aimforward = recordforward + angle;
                if(aimforward >= 360)
                {
                    aimforward = aimforward - 360;
                }
                
                recognizeFlag = recognizeBlackWall();
                

                // FRIZY_LOG(LOG_DEBUG, "aimforward: %f, recordforward: %f",aimforward, recordforward);
                chassis.chassisSpeed(WALL_RATATESPEED, -WALL_RATATESPEED, 1);
                // chassis.GridPoint(&current_pos);
                // FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, current_pos.forward);  
                // FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, fabs(360-gyo_angle_*180/_Pi));              
                // if(fabs(current_pos.forward - aimforward) < error_index || fabs(current_pos.forward - aimforward) > (360 - error_index))
                
                if(recognizeFlag & 1)
                {
                    recognize = 1;
                    chassis.chassisSpeed(0, 0, 1);
                    return 1;
                }

                if(fabs((360-gyo_angle_*180/_Pi) - aimforward) < error_index || fabs((360-gyo_angle_*180/_Pi) - aimforward) > (360 - error_index))
                {
                    chassis.chassisSpeed(0, 0, 1);
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");     
                    return 1;
                }
            }
            return 0;
    }

    bool AlongWall::rotateSlam(float aimForward)
    {
        if(WallFallowPara.Dir == RIGHTAW)
        {
            chassis.chassisSpeed(-WALL_RATATESPEED, WALL_RATATESPEED, 1);   
            // FRIZY_LOG(LOG_DEBUG, "aimforward = %f, current_pos.forward = %f",aimForward, current_pos.forward);       
            if(fabs(current_pos.forward - aimForward) < error_index || fabs(current_pos.forward - aimForward) > (360 - error_index))
            {
                chassis.chassisSpeed(0, 0, 1);
                FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");     
                return 1;
            }
        }
        else if(WallFallowPara.Dir == LEFTAW)
        {
            chassis.chassisSpeed(WALL_RATATESPEED, -WALL_RATATESPEED, 1);
            // FRIZY_LOG(LOG_DEBUG, "aimforward = %f, current_pos.forward = %f",aimForward, current_pos.forward);      
            if(fabs(current_pos.forward - aimForward) < error_index || fabs(current_pos.forward - aimForward) > (360 - error_index))
            {
                chassis.chassisSpeed(0, 0, 1);
                FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");     
                return 1;
            }
        }
        return 0;
    }

    void AlongWall::alongwallStart()
    {
        FRIZY_LOG(LOG_INFO, "alongwall thread start"); 
        alongwalk_run_index = false;
        alongwall_thread_ = std::make_shared<std::thread>(&AlongWall::chassisAlongWall, this);
    }
    void AlongWall::alongwallStop()
    {
        FRIZY_LOG(LOG_INFO, "alongwall thread stop"); 
        {
            alongwalk_run_index = false;
            WallFallowPara.State = WallFollowEnd;
            // log_info("exit thread id 0x%08x\n", thread_->get_id());
            alongwall_thread_->join();
            alongwall_thread_ = nullptr;
        }
    }

    bool AlongWall::isNear(float x, float y, int type)
    {
        float dis;
        // FRIZY_LOG(LOG_DEBUG, "shexian x y = %f %f", x, y);
        if(type != 0)   //充电桩禁区
        {
            for(auto p : _maze.seatPoint)
            {
                dis = sqrtf((p.first - x) * (p.first - x) + (p.second - y) * (p.second - y));
                // FRIZY_LOG(LOG_DEBUG, "seatPoint x y dis= %f %f %f", p.first, p.second, dis);
                if(dis <= 0.05)
                    return true;
            }
        }
        else            //app禁区
        {
            for(auto p : _maze.forbidenPoint)
            {
                dis = sqrtf((p.first - x) * (p.first - x) + (p.second - y) * (p.second - y));
                // FRIZY_LOG(LOG_DEBUG, "forbidenPoint x y dis= %f %f ", p.first, p.second, dis);
                if(dis <= 0.05)
                    return true;
            }
        }
        return false;
    }

    float AlongWall::getBanDis(int dir, int type)
    {
        float tmpx, tmpy, forward, tmpcan, tmp;
        if(dir == 0)        //front
            forward = current_pos.forward;
        else if(dir == 1)   //right
        {
            forward = current_pos.forward + 90;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 2)   //left
        {
            forward = current_pos.forward - 90;
            if(forward < 0) 
                forward = forward + 360;
        }
        else if(dir == 3)
        {
            forward = current_pos.forward + 45;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 4)
        {
            forward = current_pos.forward - 45;
            if(forward < 0) 
                forward = forward + 360;
        }
        tmpcan = (forward / 180) * 3.14159f;
        for(float i = 0.05f; i <= 0.5f; i += 0.05f)
        {
            tmpx = (current_pos.realx * 15 / 100) + (i * cos(tmpcan));
            tmpy = (current_pos.realy * 15 / 100) - (i * sin(tmpcan));
            // tmpx = current_pos.realx + (i * cos(tmpcan));
            // tmpy = current_pos.realy - (i * sin(tmpcan));
            if(isNear(tmpx, tmpy, type))
            {
                tmp = sqrtf((((current_pos.realx * 15 / 100) - tmpx) * ((current_pos.realx * 15 / 100) - tmpx)) + (((current_pos.realy * 15 / 100) - tmpy) * ((current_pos.realy * 15 / 100) - tmpy)));
                //FRIZY_LOG(LOG_DEBUG, "dir:%d catch forbid signal, return tmp:%f", dir, tmp);
                return tmp;
            }
        }
        return INT_MAX;
    }

    float AlongWall::getBoudaryDis(int dir) 
    {
        float out_x = 0.0f;
        float out_y = 0.0f;
        if((BOUND_X - fabsf(current_pos.x * 0.15)) > 1 && (BOUND_Y - fabsf(current_pos.y * 0.15)) > 1)
            return INT_MAX;
        float tmpx, tmpy, forward, tmpcan, tmp;
        if(dir == 0)        //front
            forward = current_pos.forward;
        else if(dir == 1)   //right
        {
            forward = current_pos.forward + 90;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 2)   //left
        {
            forward = current_pos.forward - 90;
            if(forward < 0) 
                forward = forward + 360;
        }
        else if(dir == 3)
        {
            forward = current_pos.forward + 45;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 4)
        {
            forward = current_pos.forward - 45;
            if(forward < 0) 
                forward = forward + 360;
        }
        tmpcan = (forward / 180) * 3.14159f;
        for(float i = 0.05f; i <= 0.5f; i += 0.05f)
        {
            tmpx = (current_pos.realx * 15 / 100) + (i * cos(tmpcan));
            tmpy = (current_pos.realy * 15 / 100) - (i * sin(tmpcan));
            out_x = BOUND_X - fabsf(tmpx);
            out_y = BOUND_Y - fabsf(tmpy);
            if(out_x < 0.5 && out_y < 0.5) {
                FRIZY_LOG(LOG_DEBUG, "outx outy %f %f", out_x, out_y);
                return out_x < out_y ? out_x : out_y;
            }
            else if(out_x < 0.5 || out_y < 0.5)
            {
                FRIZY_LOG(LOG_DEBUG, "outx outy %f %f", out_x, out_y);
                if(out_x < 0.5)
                    return out_x;
                if(out_y < 0.5)
                    return out_y;
            }
        }
        return INT_MAX;
    }

    float AlongWall::getwallBanDis(int dir, int type)
    {
        float tmpx, tmpy, forward, tmpcan;
        float dis,minDis = 1.0f,minX,minY,minDisMean = 0.0f;
        float outDis = 0.0f;
        if(type == 0)
        {
            for(int i = 60; i <= 100; i += 5)
            {
                if(dir == 1)   //right
                {
                    forward = current_pos.forward + i;
                    if(forward >= 360) 
                        forward = forward - 360;
                }
                else if(dir == 2)   //left
                {
                    forward = current_pos.forward - i;
                    if(forward < 0) 
                        forward = forward + 360;
                }

                tmpcan = (forward / 180) * 3.14159f;
                tmpx = (current_pos.realx * 15 / 100) + (0.17f * cos(tmpcan));
                tmpy = (current_pos.realy * 15 / 100) - (0.17f * sin(tmpcan));
                for(auto p : _maze.forbidenPoint)
                {
                    dis = HYPOTENUSE((p.first - tmpx),(p.second - tmpy));
                    if(minDis >= dis)
                    {
                        minDis = dis;
                        minX = p.first;
                        minY = p.second;
                    }
                }
                outDis = HYPOTENUSE(((current_pos.realx * 15 / 100) - minX),((current_pos.realy * 15 / 100) -minY));
            
                if(minDisMean)
                {
                    minDisMean += outDis;
                    minDisMean /= 2;
                }
                else
                {
                    minDisMean += outDis;
                }
                //FRIZY_LOG(LOG_DEBUG, "angle=%f Now X Y=%f %f Tmp X Y=%f %f Min X Y=%f %f MinDis=%f OutDis=%f MeanDis=%f i=%d",current_pos.forward,(current_pos.realx * 15 / 100),\
                //(current_pos.realy * 15 / 100),tmpx,tmpy,minX, minY,minDis,outDis,minDisMean,i);
                minDis = 1.0f;
            }
        // minDisMean /= 9;
        }
        else
        {
            for(int i = 60; i <= 100; i += 5)
            {
                if(dir == 1)   //right
                {
                    forward = current_pos.forward + i;
                    if(forward >= 360) 
                        forward = forward - 360;
                }
                else if(dir == 2)   //left
                {
                    forward = current_pos.forward - i;
                    if(forward < 0)
                        forward = forward + 360;
                }
                tmpcan = (forward / 180) * 3.14159f;
                tmpx = (current_pos.realx * 15 / 100) + (0.17f * cos(tmpcan));
                tmpy = (current_pos.realy * 15 / 100) - (0.17f * sin(tmpcan));
                dis = (BOUND_X - fabsf(tmpx)) < (BOUND_Y - fabsf(tmpy) < 0.5) ? (BOUND_X - fabsf(tmpx)) : (BOUND_Y - fabsf(tmpy) < 0.5);
                if(minDis >= dis)
                {
                    minDis = dis;
                }
                if((BOUND_X - fabsf(current_pos.realx * 0.15)) < (BOUND_Y - fabsf(current_pos.realy * 0.15)))
                    outDis = BOUND_X - fabsf(current_pos.realx * 0.15);
                else
                    outDis = BOUND_Y - fabsf(current_pos.realy * 0.15);
                if(minDisMean)
                {
                    minDisMean += outDis;
                    minDisMean /= 2;
                }
                else
                {
                    minDisMean += outDis;
                }
            }
            FRIZY_LOG(LOG_DEBUG, "mindismean:%f", minDisMean);
                
        }
        return minDisMean;
    }


}    
