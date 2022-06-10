/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-19 10:47:46
 * @Project      : UM_path_planning
 */


#include "common_function/MotionControl.h"
#include "common_function/logger.h"
#include "navigation_algorithm/AlongWall.h"
#include "navigation_algorithm/RoadPlanning.h"
#include "navigation_algorithm/UTurnPlanning.h"
//
#include <Eigen/Dense>


namespace useerobot
{
    // robotData.speed = 0.1;
    // robotData.rotateangle = 0.0;
    extern PRO process;
    extern ROAD_STATE roadState;
    extern RoadAim road_aim;
    static Grid lastaim;
    static wheel_pid wheelPid;
    wheel_mode wheel_state;
    RoadPlanning roadMotion;
    extern _UTURN UTURN;
    float referY = 1000.0;
    bool preView = true;
    MotionControl::MotionControl(/* args */)
    {
        // FRIZY_LOG(LOG_INFO, "MotionControl::MotionControl interface ");
    }
    
    MotionControl::~MotionControl()
    {
        
    }
    void MotionControl::chassisMotionInit()
    {
        FRIZY_LOG(LOG_INFO, "start to init the motion control ");
    }
    

    void MotionControl::WheelPid(wheel_speed* Speed,Grid cur,Grid aim)
    {
        
        float_t K1 = 0.5;
        float_t K2 = 0.015;
        float_t K3 = 0.2;
        
        int pre_aimagg = 0;
        int aimagg = aim.forward * 10;
        int iagg = cur.forward * 10;

        if (referY < 1000){
            FRIZY_LOG(LOG_DEBUG,"cur.real.%f,%f,aim.real.%f",cur.realx,cur.realy,referY);

            Grid tmp1 = {int(1000*cur.realx),int(1000*cur.realy)};
            Grid tmp2;

            if (cur.forward < 5 || cur.forward > 355){
                tmp2 = {int(1000*(cur.realx+6.0)),int(1000*referY)};
		        pre_aimagg = 10 * roadMotion.ConAngle(tmp1,tmp2).forward;
	        }
            else if (cur.forward > 175 && cur.forward < 185){

                tmp2 = {int(1000*(cur.realx-6.0)),int(1000*referY)};
		        pre_aimagg = 10 * roadMotion.ConAngle(tmp1,tmp2).forward;	
	        }
            else 
                pre_aimagg = aimagg;
		
            FRIZY_LOG(LOG_DEBUG,"pre_aimagg.%d   %d.%d   %d.%d"
            ,pre_aimagg,tmp1.x,tmp1.y,tmp2.x,tmp2.y);

            if (abs(pre_aimagg - aimagg) >= 6 && abs(pre_aimagg - aimagg) <= 3594){
                FRIZY_LOG(LOG_DEBUG,"fuck");
                aimagg = pre_aimagg;
            }
        }

        if (abs(aimagg - iagg) > 1800) 
        {
            if (iagg > aimagg)
            {
                iagg = -1 * (3600 - iagg);
            }
            else
            {
                iagg = iagg + 3600;
            }			
        }

        //比例
        wheelPid.acc1 = aimagg - iagg;
        
        //积分
        wheelPid.acc2 = wheelPid.acc1 + wheelPid.acc2;
        
        //微分
        wheelPid.acc3 = wheelPid.acc1 - wheelPid.lastacc1;
        wheelPid.lastacc1 = wheelPid.acc1;

        //总误差
        wheelPid.acc = K1 * wheelPid.acc1 + K2 * wheelPid.acc2 + K3 * wheelPid.acc3;
        
        printf("accerror1.%f,accerror2.%f,accerro3.%f,accerror.%f\n"
        ,K1*wheelPid.acc1, K2 * wheelPid.acc2, K3*wheelPid.acc3, wheelPid.acc);

        Speed->left = Speed->left + wheelPid.acc;
        Speed->right = Speed->right - wheelPid.acc;
    }

    void MotionControl::ClearPid()
    {
        FRIZY_LOG(LOG_DEBUG,"clearpid");
        preView = true;
        
        referY = 1000.0;
        wheelPid.acc2 = 0;
        wheelPid.acc3 = 0;
        wheelPid.lastacc1 = 0;
    }

    void MotionControl::WheelBack()
    {
        robotchassiscontrol.chassisSpeed(-150,-150,1);

    }

    void MotionControl::WheelControl(Sensor sensor,Grid cur,Grid aim)
    { 
        int force_left = 0;
        if (aim.forward == 2000 || aim.forward == 2090 || aim.forward == 2180 || aim.forward == 2270){
            FRIZY_LOG(LOG_DEBUG,"-2000");
            force_left = 1;
            aim.forward -= 2000;
        }
       
        if (roadState != roadTrue
             || (abs(cur.x-road_aim.x)+abs(cur.y-road_aim.y) == 1  && process == ROAD)){

            if (aim.x - cur.x > 0 && aim.y - cur.y == 0)
			
                aim.forward = 0;	     
            
            //90
            else if (aim.x - cur.x == 0 && aim.y - cur.y < 0)
            
                aim.forward = 90;
            
            //270
            else if (aim.x - cur.x == 0 && aim.y - cur.y > 0)
            
                aim.forward = 270;
            		
            //180					
            else if (aim.x - cur.x < 0 && aim.y - cur.y == 0)
            
                aim.forward = 180;
            
        }  

        FRIZY_LOG(LOG_DEBUG,"aim.%d,%d,%f,wheel.%d,%d",aim.x,aim.y,aim.forward,sensor.leftw,sensor.rightw);        
        
        if (lastaim.x != aim.x || lastaim.y != aim.y || lastaim.forward != aim.forward)
        {
            wheel_state = forward;
            FRIZY_LOG(LOG_DEBUG,"update.%f %f %d",lastaim.forward,aim.forward,wheel_state);
        }
        
        
        //自选期间不足判断
        if (wheel_state != left && wheel_state != right)
        {
            //停止
            if (cur.x == aim.x && cur.y == aim.y && cur.forward == aim.forward)
            {
                FRIZY_LOG(LOG_DEBUG,"stop...");
                wheel_state = stop;
            }
            //自旋
            else if (fabs(cur.forward - aim.forward) > 5 && fabs(cur.forward - aim.forward) < 355)
            {
                if ((process == PLAN
                            && (aim.forward > 359 || aim.forward < 1 || (aim.forward > 179 && aim.forward < 181)
                            || (aim.forward > 89 && aim.forward < 91) || (aim.forward > 269 && aim.forward < 271))
                        || (process == ROAD && roadState == roadTrue))  
                    && aim.forward == lastaim.forward
                    && (fabs(cur.forward - aim.forward) < 18 || fabs(cur.forward - aim.forward) > 342)
                    ){
                        _maze.forceTurn ++;
                        FRIZY_LOG(LOG_DEBUG,"unmals %f %f %d",aim.forward,cur.forward,_maze.forceTurn);
                                
                }
                else{

                    FRIZY_LOG(LOG_DEBUG,"forceTurn = 0");
                    _maze.forceTurn = 0;
                }
                
                if (force_left){
                    wheel_state = left;   
                }
                else if (cur.forward - aim.forward > 180
                         || (cur.forward - aim.forward < 0 && cur.forward - aim.forward > -180))
                {	
                    FRIZY_LOG(LOG_DEBUG,"turnright");
                    wheel_state = right;          
                }
                else{
                    FRIZY_LOG(LOG_DEBUG,"turnleft");
                    wheel_state = left;              
                }
            }
            //直行
            else
                wheel_state = forward;
        }
        lastaim = aim;

        //轮控状态机    
        if (wheel_state == stop)
        {
            ClearPid();
            StopWallFollow();
            // robotchassiscontrol.chassisSpeed(0,0,1);
        }

        if (wheel_state == right || wheel_state == left)
        {
            ClearPid();
            printf("right || left\n");
            if (fabs(cur.forward - aim.forward) <= 3 || fabs(cur.forward - aim.forward) >= 357)
            {

                if (sensor.leftw != 0 || sensor.rightw != 0)
                {
                    printf("brake\n");
                    robotchassiscontrol.chassisSpeed(0,0,1);
                    return;
                }
                else
                {
                    printf("OK1\n");
                    wheel_state = forward;
                    return;
                }
            }
            else
            {


                int speed = (process == PLAN && UTURN == ARCH)? 150:120;
                int weight = (process == PLAN && UTURN == ARCH) ? 5:4;

                int deta = fabs(cur.forward - aim.forward) >= 330 ? (360 - fabs(cur.forward - aim.forward)):fabs(cur.forward - aim.forward);
                
                if (deta <= 30)
                    speed = speed - weight*(30 - fabs(deta));
                
                if (wheel_state == left)
                     wheelSpeed.left = -1 * speed,wheelSpeed.right = speed;
                if (wheel_state == right)
                    wheelSpeed.left = speed,wheelSpeed.right = -1 * speed;

                FRIZY_LOG(LOG_DEBUG,"zixuan.%d,%d",wheelSpeed.left,wheelSpeed.right);
                robotchassiscontrol.chassisSpeed(wheelSpeed.left,wheelSpeed.right,1);
                return;
            } 
            
        }
        
        if (wheel_state == forward)
        {
            if (preView && sensor.leftw > 220 && sensor.rightw > 220 && !_maze.changeF
                &&  ((process == PLAN && UTURN != EXPLORE && (cur.forward > 355 || cur.forward < 5 || (cur.forward > 175 && cur.forward < 185)))
                    || (process == BOUND && !IsWall()
                         && (cur.forward > 355 || cur.forward < 5 || (cur.forward > 175 && cur.forward < 185)
                                 || (cur.forward > 85 && cur.
                                 forward < 95) || (cur.forward > 265 && cur.forward < 275))) 
                    )){

                preView = false;

                referY = cur.realy; 
                FRIZY_LOG(LOG_DEBUG,"referY.%f",referY);
            }

            if ((sensor.midOmnibearingSlow || sensor.radarObsState.frontSlow == 1) && process == PLAN){
                FRIZY_LOG(LOG_DEBUG,"jiansu1");
                wheelSpeed.left = 150;
                wheelSpeed.right = 150;
            }

            else if (process == PLAN && UTURN == EXPLORE){
                FRIZY_LOG(LOG_DEBUG,"jiansu2");
                wheelSpeed.left = 150;
                wheelSpeed.right = 150;                               
            }

            else if (process == PLAN && UTURN == ARCH && !_maze.changeF
                     && ((cur.forward > 85 && cur.forward < 95) || (cur.forward > 265 && cur.forward < 275))){

                FRIZY_LOG(LOG_DEBUG,"duanbian");
                wheelSpeed.left = 200;
                wheelSpeed.right = 200;                              
            }
            else if (process == ROAD && road_aim.kind == recharge
                 && abs(road_aim.x-cur.x)+abs(road_aim.y-cur.y) == 1){

                wheelSpeed.left = 150;
                wheelSpeed.right = 150;                       
            }
            else
            {
                wheelSpeed.left = 260;
                wheelSpeed.right = 260;              
            }

            WheelPid(&wheelSpeed,cur,aim);
            FRIZY_LOG(LOG_DEBUG,"zhixing.%d.%d",wheelSpeed.left,wheelSpeed.right);

            robotchassiscontrol.chassisSpeed(wheelSpeed.left,wheelSpeed.right,1);

            
        }
    }

    bool MotionControl::wheelControl(wheel_mode &mode,move_data &data)
    {
        FRIZY_LOG(LOG_INFO, "start to execute wheel motion ");
        robotcontrolMode = mode;
        robotData = data;
        // targetpose = robotData.movepose;
        memset(&robotControldata,0,sizeof(robotControldata));
        robotControldata.cmd = CHASSIS_CMD_ROAD;
        switch (robotcontrolMode)
        {
        case 0:
        {
            FRIZY_LOG(LOG_INFO, "start to execute forwad move ");
            robotchassiscontrol.MakeChassisGoStraight(robotData.speed,robotData.movepose);
            
            break;
        }
        case 1:
        {
            FRIZY_LOG(LOG_INFO, "start to execute back move ");
            robotchassiscontrol.MakeChassisGoStraight(-robotData.speed,robotData.movepose);
            
            break;
        }
            
        case 2:
        {   
            FRIZY_LOG(LOG_INFO, "start to execute turn left move ");
            robotchassiscontrol.MakeChassisTurnLeft(robotData.speed, robotData.rotateangle);
            break;
        }
            
        case 3:
        {
            FRIZY_LOG(LOG_INFO, "start to execute turn right move ");
            robotchassiscontrol.MakeChassisTurnright(robotData.speed, robotData.rotateangle);
            break;
        }
        case 4:
        {
            FRIZY_LOG(LOG_INFO, "start to go around ");
            robotchassiscontrol.MakeChassisGoTurn(robotData.leftspeed,robotData.rightspeed,robotData.times);
            break;
        }
        
        default:
            break;
        }
        return true;
    }
    bool MotionControl::chassisRecharge()
    {
        FRIZY_LOG(LOG_INFO, "start to do chassis recharge");
        if(robotchassiscontrol.MakeBaseRecharge()==0)
        {
            FRIZY_LOG(LOG_INFO, "successful done chassis recharge");
            return true;
        }
        
        else
        {
            FRIZY_LOG(LOG_INFO, "fail done chassis recharge");
            return false;
        }
    }
    bool MotionControl::chassisWallAlong(uint8_t mode,int direction)
    {
        FRIZY_LOG(LOG_INFO, "start to do chassis wallalong");
        if(robotchassiscontrol.MakeBaseAlongWall(mode,direction)==0)
        {
            FRIZY_LOG(LOG_INFO, "successful done chassis wallalong");
            return true;
        }
        
        else
        {
            FRIZY_LOG(LOG_INFO, "fail done chassis wallalong");
            return false;
        }
    }
    bool MotionControl::pathTransformtoOrder(list<Point *> &path)
    {
        FRIZY_LOG(LOG_INFO, "start to transform path to order");
        while(path.empty())
        {
            currentpose = GetCurGridPose();
            // 取path的第一个位置进行路径转控制指令
            auto &p = path.front();
            // targetpose.i = p->x; 
            // targetpose.i = p->y;
            // targetpose.forward = 0.0;
            //要移动到的目标点
            robotData.movepose.i = p->x;
            robotData.movepose.j = p->y;
            robotData.movepose.forward = 0.0;
            // robotData.speed = 0.25;
            auto angle = currentpose.forward;// 需要确定是弧度还是度数？
            if (p->x == currentpose.i && p->y == currentpose.j)
            {
                FRIZY_LOG(LOG_INFO, "don not need to move, currentpose = path.front()");
            }

            else
            {   
                // 路径点在第一象限             
                if(p->x > currentpose.i && p->y >= currentpose.j)
                {
                    auto target_orientation = atan((p->y - currentpose.j)/(p->x - currentpose.i));
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_one;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_one;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_one:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_one done well");
                        else
                            return false;
                    }

                }
                // 路径点在第四象限 
                if(p->x >= currentpose.i && p->y < currentpose.j)
                {
                    auto target_orientation = atan((p->y - currentpose.j)/(p->x - currentpose.i))+2*_Pi;
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_four;

                    }
                    if (D_value < 0)
                    {   
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_four;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_four:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_four done well");
                        else 
                            return false;
                    }
                }
                // 路径点在第二象限
                if(p->x <= currentpose.i && p->y > currentpose.j)
                {
                    auto target_orientation =_Pi + atan((p->y - currentpose.j)/(p->x - currentpose.i));
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_two;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_two;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_two:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_two done well");
                        else
                            return false;

                    }
                }
                // 路径点在第三象限
                if(p->x < currentpose.i && p->y <= currentpose.j)
                {
                    auto target_orientation = fabs(atan((p->y - currentpose.j)/(p->x - currentpose.i)))+_Pi;
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_tree;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_tree;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_tree:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_tree done well");
                        else
                            return false;
                    }
                }                        
            }
            path.pop_front(); 
        }
        return true;
    }
}

