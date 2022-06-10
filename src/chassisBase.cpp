/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-17 10:48:29
 * @LastEditTime : 2022-01-27 11:48:32
 * @Project      : UM_path_planning
 */
#include "um_chassis/chassisBase.h"
#include "navigation_algorithm/AlongWall.h"
#include "navigation_algorithm/RoadPlanning.h"

#include <random>

#ifndef __UMUSERMODE__
#define __UMUSERMODE__ 444
#endif


using namespace core;
namespace useerobot{
    
extern PRO process;
extern ROAD_STATE roadState;
extern allDirectionPointInfo_t* laserDis;
extern int suodan;
extern WallFallowPara_t WallFallowPara;
RoadPlanning road; 
static float jumpX = 0;
static float jumpY = 0;
static float jumpAngle = 0;
static int warnSign = 0;
share_path_planning_t* current_path_planning;    
current_pose_t* current_pose;
current_pose_t share_pose;
current_pose_t raw_share_pose;
int calibration_time = 0;
bool gyro_stat_resp_state = false;
static int wall = 0;
static int lastAngle = 0;
static int tempadd = 0;
static bool speedOrder = 0;
int rightWheel = 0;
int leftWheel = 0;
bool getBlockInfoIdenx = false;
bool getPointInfoIndex = false;
bool getForbbidenInfoIndex = false;
CHASSIS_CONTROL GLOBAL_CONTROL;
chassisBase *chassisBase::m_Instance = nullptr;
RoadSensorData_t sensordata;
// extern float angle_;
int round_doubel(double number)
{
return (number>0.0)?(number+0.5):(number-0.5);
}
chassisBase *chassisBase::getInstance()
{
        if (m_Instance == nullptr){
                m_Instance = new chassisBase();
        }
        return m_Instance;
}
chassisBase::chassisBase()
{
        
};
chassisBase::~chassisBase()
{
        
};

struct PointCloud
{
    vector<Point2f> clouds; 
    Grid robot_location;
    int index = 0;
    int count = 0;
    // PointCloud(int index,int count){
    //     index = index;
    //     count = count;
    // }
};


using cloudPtr = std::shared_ptr<PointCloud>;

vector<cloudPtr> clouds_list;
vector<PointCloud> dynamic_map;
const int len = 200;

inline int tranIndex(int x,int y){

    if (x >= 0 && y >= 0)
        return x + len*y; 
    else if (x < 0 && y > 0)
        return len*len + abs(x) + y * len;
    else if (x < 0 && y < 0)
        return 2*len*len + abs(x) + abs(y) * len;
    else
        return 3*len*len + x + abs(y) * len;
}

inline void InputDynMap(float x,float y){

    int tmpX = static_cast<int>(round(100*(x-dynamic_map[0].robot_location.realx)));
    int tmpY = static_cast<int>(round(100*(y-dynamic_map[0].robot_location.realy)));

    int tmpI = tranIndex(tmpX,tmpY);
    dynamic_map[tmpI].count++;

    if (abs(tmpX) < 100 && abs(tmpY) < 100 && dynamic_map[tmpI].count > 1) // 
        printf("@300=%04d,%04d,00%d\n",tmpX,tmpY,2);

}

inline int GetDynMap(float x,float y){

    int tmpX = static_cast<int>(round(100*(x-dynamic_map[0].robot_location.realx)));
    int tmpY = static_cast<int>(round(100*(y-dynamic_map[0].robot_location.realy)));  

    if (tmpX >= len || tmpY >= len)
        return 0;

    int tmpI = tranIndex(tmpX,tmpY);

    return dynamic_map[tmpI].count;
            
}

inline void DeleteDynMap(float x,float y){

    int tmpX = static_cast<int>(round(100*(x-dynamic_map[0].robot_location.realx)));
    int tmpY = static_cast<int>(round(100*(y-dynamic_map[0].robot_location.realy)));
    
    int tmpI = tranIndex(tmpX,tmpY);

    dynamic_map[tmpI].count--;

}



void test1(Grid cur){

    FRIZY_LOG(LOG_DEBUG,"test1");
    dynamic_map.clear();
    dynamic_map.resize(4*len*len);   

    cloudPtr ptr = make_shared<PointCloud>();
 
    Point2f p;
    float cur_f = cur.forward*_PI/180;
    float aim_f;
    float dis;
    
    for (int i = 1;i < laserDis->size; ++i){  

        if (!laserDis->point[i].x && !laserDis->point[i].y)
            continue;

        dis = pow(pow(laserDis->point[i].x, 2) + pow(laserDis->point[i].y, 2), 0.5);  

        if (dis > 2.0 || dis < 0.1)
            continue;
        
        aim_f = laserDis->point[i].theta + cur_f;    

        if (aim_f >= 2*_PI){
            aim_f -= 2*_PI;
        }
        
        p.x = cur.realx*0.15 + dis*cos(aim_f);
        p.y = cur.realy*0.15 - dis*sin(aim_f);

        ptr->clouds.push_back(p);

        FRIZY_LOG(LOG_DEBUG,"dis1: %f %f pxy: %f %f",dis,aim_f,p.x,p.y);

        // dynamic_map[0].robot_location.realx = 0;
        // dynamic_map[0].robot_location.realy = 0;
        // InputDynMap(p.x,p.y);
    }
    
    //return;

    ptr->robot_location.realx = cur.realx*0.15;
    ptr->robot_location.realy = cur.realy*0.15;
    ptr->robot_location.forward = cur_f;
    
    if (clouds_list.empty()){
        ptr->index = 0; 
    }else{
        ptr->index = clouds_list.back()->index + 1;
    } 

    clouds_list.push_back(ptr);

    if (!clouds_list.empty())
        FRIZY_LOG(LOG_DEBUG,"index111: %d  %d",clouds_list.back()->index,clouds_list.size());

    dynamic_map[0].robot_location.realx = ptr->robot_location.realx;
    dynamic_map[0].robot_location.realy = ptr->robot_location.realy;
    //return;
        
    if (clouds_list.back()->index >= 5){
        
        FRIZY_LOG(LOG_DEBUG,"set map : %f %f %f"
        ,ptr->robot_location.realx,ptr->robot_location.realy,ptr->robot_location.forward);

        //删掉第一帧数据,插入第六帧数据
        // int tmpX = static_cast<int>(round(100*(p.x-clouds_list[5]->robot_location.realx)));
        // int tmpY = static_cast<int>(round(100*(p.y-clouds_list[5]->robot_location.realy)));     
        // int tmpI = tranIndex(tmpX,tmpY);
        // dynamic_map[tmpI].count++;

        float dx;
        float dy;
        float de;
        Point2f p0;
        
        for (int i = 5; i >= 1;--i){

            dx = ptr->robot_location.realx - clouds_list[i]->robot_location.realx;
            dy = ptr->robot_location.realy - clouds_list[i]->robot_location.realy;

            //de = ptr->robot_location.forward - clouds_list[i]->robot_location.forward;
            // if (de < 0)
            //     de += 360;
            // de *= -1;

            for (int j = 0;j < clouds_list[i]->clouds.size();++j){

                // p0.x = clouds_list[i]->clouds[j].x*cos(de) - clouds_list[i]->clouds[j].y*sin(de) + dx;    
                // p0.y = clouds_list[i]->clouds[j].x*sin(de) + clouds_list[i]->clouds[j].y*cos(de) + dy;
                p0.x += dx;    
                p0.y += dy;                
                //FRIZY_LOG(LOG_DEBUG,"dis2: %f %f pxy: %f %f",dis,aim_f,p0.x,p0.y);
                InputDynMap(p0.x,p0.y);
            }
            
            // if (i){
            //     for (int j = 0;j < clouds_list[i]->clouds.size();++j){

            //         p0.x = clouds_list[i]->clouds[j].x*cos(de) - clouds_list[i]->clouds[j].y*sin(de) + dx;    
            //         p0.y = clouds_list[i]->clouds[j].x*sin(de) + clouds_list[i]->clouds[j].y*cos(de) + dy;
            //         //FRIZY_LOG(LOG_DEBUG,"dis2: %f %f pxy: %f %f",dis,aim_f,p0.x,p0.y);
            //         InputDynMap(p0.x,p0.y);
            //     }
            // }
            // else{
            //     for (int j = 0;j < clouds_list[i]->clouds.size();++j){

            //         p0.x = clouds_list[i]->clouds[j].x*cos(de) - clouds_list[i]->clouds[j].y*sin(de) + dx;    
            //         p0.y = clouds_list[i]->clouds[j].x*sin(de) + clouds_list[i]->clouds[j].y*cos(de) + dy;
            //         DeleteDynMap(p0.x,p0.y);
            //     }                
            // }

        }
        
        for (int i = 1;i <= 5;++i){
                
            clouds_list[i-1] = clouds_list[i];
            clouds_list[i-1]->index--;
        }


        //clouds_list[5].reset();
        clouds_list.pop_back();
        FRIZY_LOG(LOG_DEBUG,"the back: %d %d",clouds_list.back()->index,clouds_list.size());
        
    }

}


void chassisBase::updataVFH(Grid cur,int kind,vector<VFH_PARA>& valid_dir,float& frontV){

    FRIZY_LOG(LOG_DEBUG,"zhifangtu : %d",laserDis->size);

    if (!kind){
        _config.initDis = 500;
    }else{
        _config.initDis = 50000;    
    }
    
    vector<float> angle_Dis(360,_config.initDis);

    for (int i = 1;i < laserDis->size; ++i){
            
        if (!laserDis->point[i].x && !laserDis->point[i].y)
            continue;

        float dis = 100*pow(pow(laserDis->point[i].x, 2) + pow(laserDis->point[i].y, 2), 0.5);
 
        float angle = cur.forward + laserDis->point[i].theta*180/_Pi;

        float pre_dis = 100*pow(pow(laserDis->point[i-1].x, 2) + pow(laserDis->point[i-1].y, 2), 0.5);
        float pre_angle = cur.forward + laserDis->point[i-1].theta*180/_Pi;

       if (dis < 10){
            float ttt = angle;
            correctAngle(ttt);
            FRIZY_LOG(LOG_DEBUG,"less10 : %f  %f",ttt,dis);
            continue;
        }        

        if ((fabs(pre_angle-angle) < 2 || fabs(pre_angle-angle) > 358)
             && fabs(pre_dis-dis) < 10){

            //angle = (pre_angle+angle)/2;
            correctAngle(angle);
            int cell = static_cast<int>(angle);
            //FRIZY_LOG(LOG_DEBUG,"cell : %d  %f",cell,(pre_dis + dis)/2);
            if ((pre_dis + dis)/2 < angle_Dis[cell])
                angle_Dis[cell] = (pre_dis + dis)/2;
        }
    }    

    // for (int i = 0;i < angle_Dis.size();++i){
    //     FRIZY_LOG(LOG_DEBUG,"qqq: %d %f",i,angle_Dis[i]);
    // }    

    //return;


    if (!kind){
    for (int x = -10;x <= 10;++x){
        for (int y = -10;y <= 10;++y){

            if (!x && !y)
                continue;
                
            if (_maze.GetMapState(cur.x+x,cur.y+y,2) == 2
                || (_maze.GetMapState(cur.x+x,cur.y+y,1) == 4 && (abs(cur.x) > 3 || abs(cur.y) > 3))
                || cur.x+x >= static_cast<int>(_maze.homePoint.realx)+30 || cur.y+y >= static_cast<int>(_maze.homePoint.realy)+30
                || cur.x+x <= static_cast<int>(_maze.homePoint.realx)-30 || cur.y+y <= static_cast<int>(_maze.homePoint.realy)-30
                //_maze.GetMapState(cur.x+x,cur.y+y,1) == 2
                ){ //|| _maze.GetMapState(cur.x+x,cur.y+y,2) == 1

                // if (_maze.GetMapState(cur.x+x,cur.y+y,1) == 2)
                //     printf("@300=%04d,%04d,00%d\n",cur.x+x,cur.y+y,2);   
                
                Grid tmp = {cur.x+x,cur.y+y};
                float dis = 15*pow(x*x+y*y,0.5);
                float angle = road.ConAngle(cur,tmp).forward;

                int cell = static_cast<int>(angle);

                if (angle_Dis[cell] > dis){
                    FRIZY_LOG(LOG_DEBUG,"mapob: %d %f",cell,dis);
                    angle_Dis[cell] = dis;
                }       
            }
        }
    }
    }
    
    vfh_histogram.clear();
    vfh_histogram.resize(_config.nums,_config.initDis);   

    for (int i = 0;i < angle_Dis.size(); ++i){
            
        float dis = angle_Dis[i];
        float angle = static_cast<float>(i);

        correctAngle(angle);
        
        float interC = _config.robot_R/dis;
        if (interC >= 1.0){
            interC = 1.0;
        }

        float angleL = angle - asin(interC)*180/_Pi;
        float angleR = angle + asin(interC)*180/_Pi;

        //FRIZY_LOG(LOG_DEBUG,"angleLL : %f %f %f %f",angleL,angleR,angle,dis);
        
        if (angleR - angleL > 5){

            for (auto ag = angleL;ag <= angleR;ag += 2.0){

                float tmp = ag;
                correctAngle(tmp);

                int cell_sector = static_cast<int>(tmp/_config.sec);

                //float distance_cost = 100/(1 + exp(dis));                
                //float magnitude = pow(((costmap->getCost(x, window_height-y-1))/254),2);

                if (vfh_histogram[cell_sector] > dis){

                    vfh_histogram[cell_sector] = dis;
                }
            }

        }else{

            int cell_sector = static_cast<int>(angle/_config.sec);

            //float distance_cost = 100/(1 + exp(dis));
            
            //float magnitude = pow(((costmap->getCost(x, window_height-y-1))/254),2);

            if (vfh_histogram[cell_sector] > dis){
                vfh_histogram[cell_sector] = dis;
            }
        }
    }

    for (int i = 0;i < _config.nums;++i){
        FRIZY_LOG(LOG_DEBUG,"ddd: %d %f",i*_config.sec,vfh_histogram[i]);
    }    

    
    VFH_PARA trait;

    int begin = 0;       
    int end = 0;

    for (int i = 0;i < _config.nums;++i){

        if (cur.forward >= i*_config.sec && cur.forward < i*_config.sec+_config.sec){

            frontV = vfh_histogram[i];
            FRIZY_LOG(LOG_DEBUG,"frontV : %f",frontV);
        }

        if ((!i && vfh_histogram[i] > _config.threshold)
             || (vfh_histogram[i] > _config.threshold && vfh_histogram[i-1] <= _config.threshold)){
                    
            begin = i;
        }
        else if (i > 0 && vfh_histogram[i-1] > _config.threshold
                && (vfh_histogram[i] <= _config.threshold || i == _config.nums - 1)){

            end = i;
            //FRIZY_LOG(LOG_DEBUG,"nandao: %d  %d",begin,end);
            for (int j = begin;j < end;++j){

                trait.angle = j*_config.sec;
                trait.value = vfh_histogram[j];
                trait.width = j-begin+1 < end-j ? j-begin+1 : end-j;
                // if (!kind)
                //     FRIZY_LOG(LOG_DEBUG,"trait.width: %d",trait.width);
                valid_dir.push_back(trait);
            }
        }   
    }
}

double chassisBase::Conlaser(){
    
    //std::normal_distribution<double> distribution(5.0, 2.0);
    FRIZY_LOG(LOG_DEBUG,"lasersize.%d",laserDis->size);
    int sum_laser_times = 0;
    for (int i = 0;i < laserDis->size; ++i){
            
        if (!laserDis->point[0].x && !laserDis->point[0].y)
            continue;
            
        if (laserDis->point[i].theta < 4*_Pi/9 || laserDis->point[i].theta > 14*_Pi/9)
        {
            double tmp = pow(pow(laserDis->point[i].x, 2) + pow(laserDis->point[i].y, 2), 0.5);
            if (tmp > 0.1 && tmp < 0.2)
            {

                FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                sum_laser_times ++;
                if(sum_laser_times>3)
                {
                        FRIZY_LOG(LOG_DEBUG,"there is obstacle in laser data");
                        return tmp;
                }                
            }
        }
    }
    return 0;

}


    //获取当前时间  单位ms
    long long getNowTime()
    {
        struct timeval t;
        gettimeofday(&t, NULL);
        return (long long)t.tv_sec*1000 + (long long)t.tv_usec/1000;
    }

    void BubbleSort(double * dataBuf)
    {
        int dataLen = 0;
        int i,j,t; 
        dataLen = sizeof(dataBuf);

        for(j = 0; j < dataLen - 1; j ++) /*进行9次循环 实现9趟比较*/ 
        {
            for(i = 0; i < dataLen - j; i ++) /*在每一趟中进行9-j次比较*/ 
            {
                if(dataBuf[i] > dataBuf[i + 1]) /*相邻两个数比较,想降序只要改成a[i]<a[i+1]*/ 
                { 
                    t = dataBuf[i]; 
                    dataBuf[i] = dataBuf[i + 1]; 
                    dataBuf[i + 1] = t; 
                } 
            }
        }
    }

    
    static double ellipseCenterToSideDistance(double angle,double short_R,double long_R)
    {
        return sqrt( ( (pow(tan(ANGLE_TO_RADIAN(angle)),2) + 1) * pow(short_R,2) * pow(long_R,2) ) / ( pow(long_R,2) * pow(tan(ANGLE_TO_RADIAN(angle)),2) + pow(short_R,2) ) );
    }

    static radar_WallTurn_t wallCheck(){
        
        //std::normal_distribution<double> distribution(5.0, 2.0);
       // FRIZY_LOG(LOG_DEBUG,"lasersize--1.%d",laserDis->size);
        static long long lastTime = 0;

        //沿墙变量
        double wallMinData = 0;
        int left_sum_laser_times = 0,front_sum_laser_times = 0,right_sum_laser_times = 0;
        double leftTmp = 0.0f,frontTmp = 0.0f,rightTmp = 0.0f,tmp;
        static uint32_t leftSlowCount = 0, frontSlowCount = 0, rightSlowCount = 0;
        static uint32_t leftTurnCount = 0, frontTurnCount = 0, rightTurnCount = 0;
        static uint8_t leftCheckFlag = 0, frontCheckFlag = 0, rightCheckFlag = 0;
        static radar_WallTurn_t radarTmp;
        static double lastWallDistance = 0.0f;
        static uint8_t radarNoData = 0;
        double frontCheckDistance = 0.0f;
        static double leftWallShort_R = 0.0f,leftWallLong_R = 0.0f,rightWallShort_R = 0.0f,rightWallLong_R = 0.0f;
        double leftdataBuf[100],frontdataBuf[100],rightdataBuf[100];
        double leftangleBuf[100],frontangleBuf[100],rightangleBuf[100];

        if(getNowTime() - lastTime < 200/* && laserDis->size*/)
            return radarTmp;
        // FRIZY_LOG(LOG_DEBUG,"lastTime. %lld  %lld",getNowTime(),lastTime);
        lastTime = getNowTime();
        
        radarTmp.rightWallDistance = 1.0f;
        if((leftCheckFlag & 1) == 1)
        {
            if(leftSlowCount)
            {
                leftSlowCount --;
            }
            else
            {
                leftCheckFlag &= ~1;
                radarTmp.wallObsLeftSlow = 0; 
            }
        }
        if((leftCheckFlag & 2) == 2)
        {
            if(leftTurnCount)
            {
                leftTurnCount --;
            }
            else
            {
                leftCheckFlag &= ~2;
                radarTmp.wallObsLeft = 0; 
            }
        }

        if((frontCheckFlag & 1) == 1)
        {
            if(frontSlowCount)
            {
                frontSlowCount --;
            }
            else
            {
                frontCheckFlag &= ~1;
                radarTmp.wallObsFrontSlow = 0; 
            }
        }
        if((frontCheckFlag & 2) == 2)
        {
            if(frontTurnCount)
            {
                frontTurnCount --;
            }
            else
            {
                frontCheckFlag &= ~2;
                radarTmp.wallObsFront = 0; 
            }
        }
        
        if((rightCheckFlag & 1) == 1)
        {
            if(rightSlowCount)
            {
                rightSlowCount --;
            }
            else
            {
                rightCheckFlag &= ~1;
                radarTmp.wallObsRightSlow = 0; 
            }
        }
        if((rightCheckFlag & 2) == 2)
        {
            if(rightTurnCount)
            {
                rightTurnCount --;
            }
            else
            {
                rightCheckFlag &= ~2;
                radarTmp.wallObsRight = 0; 
            }
        }

        
        // memset(&radarTmp,0,sizeof(radarTmp));
        for(int i = 0; i < laserDis->size; i ++)
        {
            //左边区域
            if (laserDis->point[i].theta > ANGLE_TO_RADIAN(310) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(340))
            {
                leftTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                
                leftWallShort_R = 20.0f;
                leftWallLong_R = 30.0f;
                if (leftTmp * 100 < ellipseCenterToSideDistance( (360 - (double)RADIAN_TO_ANGLE(laserDis->point[i].theta)) , leftWallShort_R , leftWallLong_R))
                {  
                    leftCheckFlag |= 1;
                    leftangleBuf[left_sum_laser_times] = (double)RADIAN_TO_ANGLE(laserDis->point[i].theta);
                    leftdataBuf[left_sum_laser_times] = leftTmp * 100;
                    //FRIZY_LOG(LOG_DEBUG,"leftObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),leftTmp,left_sum_laser_times,leftTurnCount);
                    left_sum_laser_times ++;
                    if(left_sum_laser_times > 3)
                    {
                        leftSlowCount = 1;
                        radarTmp.wallObsLeftSlow = 1;
                        // FRIZY_LOG(LOG_DEBUG,"wallObsLeftSlow-1:%d\n",radarTmp.wallObsLeftSlow);
                        leftWallShort_R = 18.0f;
                        leftWallLong_R = 25.0f;
                        if (leftTmp * 100 < ellipseCenterToSideDistance( (360 - (double)RADIAN_TO_ANGLE(laserDis->point[i].theta)) , leftWallShort_R , leftWallLong_R))
                        {
                            leftCheckFlag |= 2;
                            leftTurnCount = 1;
                            radarTmp.wallObsLeft = 1;
                            // FRIZY_LOG(LOG_DEBUG,"wallObsLeft-2:%d\n",radarTmp.wallObsLeft);
                        }
                    }     
                             
                }
            }
            //中间区域
            if (laserDis->point[i].theta <= ANGLE_TO_RADIAN(20) || laserDis->point[i].theta > ANGLE_TO_RADIAN(340))
            {
                frontTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                if(frontTmp >= FRONT_RADAR_DIFF)
                {
                    frontTmp -= FRONT_RADAR_DIFF;
                }
                else
                {
                    frontTmp = 0.0f;
                }
                
                if (frontTmp < FRONT_OBSTACLES_SLOW)
                {  
                    frontCheckFlag |= 1;
                    frontangleBuf[front_sum_laser_times] = (double)RADIAN_TO_ANGLE(laserDis->point[i].theta);
                    frontdataBuf[front_sum_laser_times] = frontTmp * 100;
                    // FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                    //FRIZY_LOG(LOG_DEBUG,"frontObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),frontTmp,front_sum_laser_times,frontTurnCount);
                    front_sum_laser_times ++;
                    if(front_sum_laser_times > 3)
                    {
                        frontSlowCount = 1;
                        radarTmp.wallObsFrontSlow = 1;
                        if(sensordata.LeftWheel_Speed > 120 && sensordata.RightWheel_Speed > 120)
                        {
                            frontCheckDistance = 0.9f;
                        }
                        else if(sensordata.LeftWheel_Speed > 10 && sensordata.RightWheel_Speed > 10)
                        {
                            frontCheckDistance = 0.08f;
                        }
                        else
                        {
                            frontCheckDistance = 0.04f;
                        }
                        frontCheckDistance = 0.03f;
                        // FRIZY_LOG(LOG_DEBUG,"wallSpeed:%d %d",sensordata.LeftWheel_Speed,sensordata.RightWheel_Speed);
                        if (frontTmp < frontCheckDistance)
                        {
                            frontCheckFlag |= 2;
                            frontTurnCount = 1;
                            radarTmp.wallObsFront = 1;
                            //FRIZY_LOG(LOG_DEBUG,"frontTurn-2:%d\n",radarTmp.wallObsFront);
                        }
                    }             
                }
            }

            //右边区域
            if(laserDis->point[i].theta > ANGLE_TO_RADIAN(20) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(50))
            {
                rightTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                
                rightWallShort_R = 20.0f;
                rightWallLong_R = 30.0f;
                if (rightTmp * 100 < ellipseCenterToSideDistance( (double)RADIAN_TO_ANGLE(laserDis->point[i].theta) , rightWallShort_R , rightWallLong_R))
                {  
                    rightCheckFlag |= 1;
                    rightangleBuf[right_sum_laser_times] = (double)RADIAN_TO_ANGLE(laserDis->point[i].theta);
                    rightdataBuf[right_sum_laser_times] = rightTmp * 100;
                    //FRIZY_LOG(LOG_DEBUG,"rightObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),rightTmp,right_sum_laser_times,rightTurnCount);
                    right_sum_laser_times ++;
                    if(right_sum_laser_times > 3)
                    {
                        rightSlowCount = 1;
                        radarTmp.wallObsRightSlow = 1;

                        rightWallShort_R = 18.0f;
                        rightWallLong_R = 25.0f;
                        if (rightTmp * 100 < ellipseCenterToSideDistance( (double)RADIAN_TO_ANGLE(laserDis->point[i].theta) , rightWallShort_R , rightWallLong_R))
                        {
                            rightCheckFlag |= 2;
                            rightTurnCount = 1;
                            radarTmp.wallObsRight = 1;
                        }
                        //FRIZY_LOG(LOG_DEBUG,"rightTurn-2:%d\n",radarTmp.wallObsRight);
                    }              
                }
            }

            //沿墙数据
            if (laserDis->point[i].theta > ANGLE_TO_RADIAN(60) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(85))
            {
                wallMinData = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                wallMinData *= cos(fabs(laserDis->point[i].theta - (0.5f * _PI)));
                if(radarTmp.rightWallDistance >= wallMinData)
                {
                    radarTmp.rightWallDistance = wallMinData;
                    radarNoData = 0;
                }
            }
            
            if (laserDis->point[i].theta > ANGLE_TO_RADIAN(50) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(120))
            {
                // FRIZY_LOG(LOG_DEBUG,"dingq.%f,%f",(double)RADIAN_TO_ANGLE(laserDis->point[i].theta),HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y));
                // wallMinData *= cos(fabs(laserDis->point[i].theta - (0.5f * _PI)));
                // if(radarTmp.rightWallDistance >= wallMinData)
                // {
                //     radarTmp.rightWallDistance = wallMinData;
                //     radarNoData = 0;
                // }
            }
            // if(radarTmp.rightWallDistance == 1.0f)
            // {
            //     radarNoData ++;
            //     if(radarNoData >= 10)
            //     {
            //         lastWallDistance = radarTmp.rightWallDistance;
            //     }
            //     else
            //     {
            //         radarTmp.rightWallDistance = lastWallDistance;
            //     }

            // }
            // else
            // {
            //     lastWallDistance = radarTmp.rightWallDistance;
            // }
        }
        // FRIZY_LOG(LOG_DEBUG,"rightWallDistance.%f %f %d",radarTmp.rightWallDistance,lastWallDistance,radarNoData);
        return radarTmp;
    }


    static radar_SlowTurn_t ObsCheck(){
        
        static long long lastTime = 0;
        int left_sum_laser_times = 0,front_sum_laser_times = 0,right_sum_laser_times = 0;
        double leftTmp = 0.0f,frontTmp = 0.0f,rightTmp = 0.0f,tmp;
        static uint32_t leftSlowCount = 0, frontSlowCount = 0, rightSlowCount = 0;
        static uint32_t leftTurnCount = 0, frontTurnCount = 0, rightTurnCount = 0;
        static uint8_t leftCheckFlag = 0, frontCheckFlag = 0, rightCheckFlag = 0;
        static radar_SlowTurn_t radarTmp;
        static double leftWallShort_R = 0.0f,leftWallLong_R = 0.0f,rightWallShort_R = 0.0f,rightWallLong_R = 0.0f;
        
        if(getNowTime() - lastTime < 200)
            return radarTmp;
        lastTime = getNowTime();
        // memset(&radarTmp,0,sizeof(radarTmp));

        if((leftCheckFlag & 1) == 1)
        {
            if(leftSlowCount)
            {
                leftSlowCount --;
            }
            else
            {
                leftCheckFlag &= ~1;
                radarTmp.leftSlow = 0; 
            }
        }
        if((leftCheckFlag & 2) == 2)
        {
            if(leftTurnCount)
            {
                leftTurnCount --;
            }
            else
            {
                leftCheckFlag &= ~2;
                radarTmp.leftTurn = 0; 
            }
        }

        if((frontCheckFlag & 1) == 1)
        {
            if(frontSlowCount)
            {
                frontSlowCount --;
            }
            else
            {
                frontCheckFlag &= ~1;
                radarTmp.frontSlow = 0; 
            }
        }
        if((frontCheckFlag & 2) == 2)
        {
            if(frontTurnCount)
            {
                frontTurnCount --;
            }
            else
            {
                frontCheckFlag &= ~2;
                radarTmp.frontTurn = 0; 
            }
        }
        
        if((rightCheckFlag & 1) == 1)
        {
            if(rightSlowCount)
            {
                rightSlowCount --;
            }
            else
            {
                rightCheckFlag &= ~1;
                radarTmp.rightSlow = 0; 
            }
        }
        if((rightCheckFlag & 2) == 2)
        {
            if(rightTurnCount)
            {
                rightTurnCount --;
            }
            else
            {
                rightCheckFlag &= ~2;
                radarTmp.rightTurn = 0; 
            }
        }
                

        for(int i = 0; i < laserDis->size; i ++)
        {
            //左边区域
            if (laserDis->point[i].theta > ANGLE_TO_RADIAN(280) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(330))
            {
                leftTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                // if(leftTmp >= LEFT_RADAR_DIFF)
                // {
                //     leftTmp -= LEFT_RADAR_DIFF;
                // }
                // else
                // {
                //     leftTmp = 0.0f;
                // }
                
                
                leftWallShort_R = 21.0f;
                leftWallLong_R = 36.0f;
                    
                if (leftTmp * 100 < ellipseCenterToSideDistance( (360 - (double)RADIAN_TO_ANGLE(laserDis->point[i].theta)) , leftWallShort_R , leftWallLong_R))
                {  
                    leftCheckFlag |= 1;
                    // FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                    // FRIZY_LOG(LOG_DEBUG,"leftObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),leftTmp,left_sum_laser_times,leftTurnCount);
                    left_sum_laser_times ++;
                    if(left_sum_laser_times > 3)
                    {
                        leftSlowCount = 1;
                        radarTmp.leftSlow = 1;
                        // FRIZY_LOG(LOG_DEBUG,"leftSlow-1:%d\n",radarTmp.leftSlow);
                        leftWallShort_R = 17.0f;
                        leftWallLong_R = 24.0f;
                        if (leftTmp * 100 < ellipseCenterToSideDistance( (360 - (double)RADIAN_TO_ANGLE(laserDis->point[i].theta)) , leftWallShort_R , leftWallLong_R))
                        {
                            leftCheckFlag |= 2;
                            leftTurnCount = 1;
                            radarTmp.leftTurn = 1;
                            // FRIZY_LOG(LOG_DEBUG,"leftTurn-2:%d\n",radarTmp.leftTurn);
                        }
                    }           
                }
            }
            //中间区域
            if (laserDis->point[i].theta <= ANGLE_TO_RADIAN(30) || laserDis->point[i].theta > ANGLE_TO_RADIAN(330))
            {
                frontTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                if(frontTmp >= FRONT_RADAR_DIFF)
                {
                    frontTmp -= FRONT_RADAR_DIFF;
                }
                else
                {
                    frontTmp = 0.0f;
                }
                if (/*frontTmp > 0.01f && */frontTmp < FRONT_OBSTACLES_SLOW)
                {  
                    frontCheckFlag |= 1;
                    // FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                    // FRIZY_LOG(LOG_DEBUG,"frontObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),frontTmp,front_sum_laser_times,frontTurnCount);
                    front_sum_laser_times ++;
                    if(front_sum_laser_times > 3)
                    {
                        frontSlowCount = 1;
                        radarTmp.frontSlow = 1;
                        // FRIZY_LOG(LOG_DEBUG,"frontSlow-1:%d\n",radarTmp.frontSlow);
                        
                        if (frontTmp < FRONT_OBSTACLES_TURN)
                        {
                            frontCheckFlag |= 2;
                            frontTurnCount = 1;
                            radarTmp.frontTurn = 1;
                            // FRIZY_LOG(LOG_DEBUG,"frontTurn-2:%d\n",radarTmp.frontTurn);
                        }
                    }        
                }
            }
            
            //右边区域
            if(laserDis->point[i].theta > ANGLE_TO_RADIAN(30) && laserDis->point[i].theta <= ANGLE_TO_RADIAN(80))
            {
                rightTmp = HYPOTENUSE(laserDis->point[i].x,laserDis->point[i].y);
                // FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",rightTmp,laserDis->point[i].x,laserDis->point[i].y);
                // if(rightTmp >= RIGHT_RADAR_DIFF)
                // {
                //     rightTmp -= RIGHT_RADAR_DIFF;
                // }
                // else
                // {
                //     rightTmp = 0.0f;
                // }
                rightWallShort_R = 21.0f;
                rightWallLong_R = 36.0f;
                if (rightTmp * 100 < ellipseCenterToSideDistance( (double)RADIAN_TO_ANGLE(laserDis->point[i].theta) , rightWallShort_R , rightWallLong_R))
                {  
                    rightCheckFlag |= 1;
                    // FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                    // FRIZY_LOG(LOG_DEBUG,"rightObs:%f  %f %d %d\n",RADIAN_TO_ANGLE(laserDis->point[i].theta),rightTmp,right_sum_laser_times,rightTurnCount);
                    right_sum_laser_times ++;
                    if(right_sum_laser_times > 3)
                    {
                        rightSlowCount = 1;
                        radarTmp.rightSlow = 1;
                        // FRIZY_LOG(LOG_DEBUG,"rightSlow-1:%d\n",radarTmp.rightSlow);

                        rightWallShort_R = 17.0f;
                        rightWallLong_R = 24.0f;
                        if (rightTmp * 100 < ellipseCenterToSideDistance( (double)RADIAN_TO_ANGLE(laserDis->point[i].theta) , rightWallShort_R , rightWallLong_R))
                        {
                            rightCheckFlag |= 2;
                            rightTurnCount = 1;
                            radarTmp.rightTurn = 1;
                            //FRIZY_LOG(LOG_DEBUG,"rightTurn-2:%d\n",radarTmp.rightTurn);
                        }
                    }       
                }
            }

        }
        
        return radarTmp;
    }

//获取传感器数据
void chassisBase::GetSensor(Sensor* pss)
{
// FRIZY_LOG(LOG_INFO, "enter GetSensor");
    

//     for (auto tmp: vec){
//         if ()
//     }
        

    pss->laserM = 0;
//     if (!IsWall() && roadState != roadTrue)
//         pss->laserM = Conlaser();
    
    if (!IsWall()/* && roadState != roadTrue*/)
    {
        pss->radarObsState = ObsCheck();
    }
    if(IsWall())
    {
        pss->wallData = wallCheck();
    }
        
    pss->batvolume = sensordata.batVolume;

    pss -> bump = sensordata.Bump_Motor;
    
    pss -> leftw = sensordata.LeftWheel_Speed;
    pss -> rightw = sensordata.RightWheel_Speed;
    pss -> rightAlongWall = sensordata.rightAlongWallValue;
    pss -> leftAlongWall = sensordata.leftAlongWallValue;

    //全方位减速
    pss -> leftOmnibearingSlow = sensordata.rightOmnibearingSlow_index;
    pss -> rightOmnibearingSlow = sensordata.rightOmnibearingSlow_index;
    pss -> midOmnibearingSlow = sensordata.middleOmnibearingSlow_index;

    //全方位转向
    pss -> leftOmnibearingTurn = sensordata.leftOmnibearingTurn_index;   
    pss -> rightOmnibearingTurn = sensordata.rightOmnibearingTurn_index;   
    pss -> midOmnibearingTurn = sensordata.middleOmnibearingTurn_index;
    
    //全方位开关灯值
    pss -> leftOmnibearingOn = sensordata.leftOmnibearingOn_index;
    pss -> leftOmnibearingOff = sensordata.leftOmnibearingOff_index;
    pss -> midOmnibearingOn = sensordata.midOmnibearingOn_index;
    pss -> midOmnibearingOff = sensordata.midOmnibearingOff_index;
    pss -> rightOmnibearingOn = sensordata.rightOmnibearingOn_index;
    pss -> rightOmnibearingOff = sensordata.rightOmnibearingOff_index;
    // pss -> leftCliff = sensordata.leftGeologicalDetect_index;
    // pss -> rightCliff = sensordata.rightGeologicalDetect_index;
    // pss -> midCliff = sensordata.middleGeologicalDetect_index;

    pss -> leftCliff = 0;
    pss -> rightCliff = 0;
    pss -> midCliff = 0;

    pss -> leftCliffValue = abs(sensordata.leftGeologicalDetect_index_on - sensordata.leftGeologicalDetect_index_off);
    pss -> midLeftCliffValue = abs(sensordata.middleLeftGeologicalDetect_index_on - sensordata.middleLeftGeologicalDetect_index_off);
    pss -> midRightCliffValue = abs(sensordata.middleRightGeologicalDetect_index_on - sensordata.middleRightGeologicalDetect_index_off);
    pss -> rightCliffValue = abs(sensordata.rightGeologicalDetect_index_on - sensordata.rightGeologicalDetect_index_off);

    // pss -> mcuLeftCliff = sensordata.mcuLeftCliff;
    // pss -> mcuRightCliff = sensordata.mcuRightCliff;
    // pss -> mcuLeftMidCliff = sensordata.mcuLeftMidCliff;
    // pss -> mcuRightMidCliff = sensordata.mcuRightMidCliff;
    pss -> mcuLeftCliff = 0;
    pss -> mcuRightCliff = 0;
    pss -> mcuLeftMidCliff = 0;
    pss -> mcuRightMidCliff = 0;

    pss -> rechargeSign = sensordata.rechargeShrapnel_index;
    
    //脱困所需数据
    pss -> XAngle = float(sensordata.X_AngleOriginal / 10.0);
    pss -> YAngle = float(sensordata.Y_AngleOriginal / 10.0);
    pss -> ZAngle = sensordata.Z_AngleOriginal;
    pss -> XAcc = sensordata.X_AccOriginal;
    pss -> YAcc = sensordata.Y_AccOriginal;
    pss -> ZAcc = sensordata.Z_AccOriginal;
    pss -> addAngle = sensordata.AddAngle;
    pss-> Initialized = sensordata.Initialized;
    pss -> leftWheelElec = sensordata.leftWheelElectricity_index;
    pss -> rightWheelElec = sensordata.rightWheelElectricity_index;
    pss -> zGyroOriginal = sensordata.Z_GyroOriginal;
    pss -> rollBrushElec = sensordata.rollBrushElectricity;


    //41信号
    pss -> leftInfrared = sensordata.leftInfrared_index;
    pss -> rightInfrared = sensordata.rightInfrared_index;
    pss -> leftFrontInfrared = sensordata.leftFrontInfrared_index;
    pss -> rightFrontInfrared = sensordata.rightFrontInfrared_index;

    if (pss -> leftInfrared || pss -> rightInfrared || pss -> leftFrontInfrared || pss -> rightFrontInfrared)
        pss->Infrared = 1;
    else
        pss->Infrared = 0;

    
    //5253信号
    pss -> leftInfrared_5253 = sensordata.left_virtulwall_5253;
    pss -> rightInfrared_5253 = sensordata.right_virtulwall_5253;
    pss -> leftFrontInfrared_5253 = sensordata.frontLeft_virtulwall_5253;
    pss -> rightFrontInfrared_5253 = sensordata.frontRight_virtulwall_5253;
    

    if (pss -> leftInfrared_5253 || pss -> rightInfrared_5253 || pss -> leftFrontInfrared_5253 || pss -> rightFrontInfrared_5253)
        pss->Infrared_5253 = 1;
    else
        pss->Infrared_5253 = 0;

    //虚拟墙nothing
    pss -> leftVir = sensordata.leftInfrared_index;
    pss -> rightVir = sensordata.rightInfrared_index;
    pss -> leftFrontVir = sensordata.leftFrontInfrared_index;
    pss -> rightFrontVir = sensordata.rightFrontInfrared_index;


    pss -> leftBehindVir = sensordata.behindLeft_virtulwall;
    pss -> rightBehindVir = sensordata.behindRight_virtulwall;

    //边刷电流
    pss->leftSideBrushElectricity =  sensordata.leftSideBrushElectricity;
    pss->rightSideBrushElectricity = sensordata.rightSideBrushElectricity;
    if (sensordata.rightOmnibearingTurn_index
        || sensordata.leftOmnibearingTurn_index
        || sensordata.middleOmnibearingTurn_index
        || (pss->laserM > 0.1 && pss->laserM < 0.2))
        pss -> obs = 1;
    else
        pss -> obs = 0;

    // if (sensordata.leftGeologicalDetect_index
    //     || sensordata.rightGeologicalDetect_index
    //     || sensordata.middleGeologicalDetect_index 
    //     || sensordata.mcuLeftCliff
    //     || sensordata.mcuRightCliff
    //     || sensordata.mcuLeftMidCliff
    //     || sensordata.mcuRightMidCliff)
    //     pss -> cliff = 1;
    // else
        pss -> cliff = 0;
    

    //虚拟墙信号
    if(sensordata.left_virtulwall || sensordata.right_virtulwall
        || sensordata.frontLeft_virtulwall || sensordata.frontRight_virtulwall)
        pss -> magnVirWall = 1;
    else 
        pss -> magnVirWall = 0;

    //FRIZY_LOG(LOG_INFO, "pss.%d.%d.%d",pss -> bump,pss -> obs,pss -> cliff);
        
    pss -> size = 1;
     
    
}

//重新规划栅格地图
void chassisBase::GridPoint(Grid* point)
{
        {
            // 锁定共享内存，share_mem_sync在作用域内自动锁定解锁
            //FRIZY_LOG(LOG_INFO, "current_pose is %f",current_pose->theta);   
            share_mem_sync sync(current_pose);

             raw_share_pose.x = current_pose->x;
             raw_share_pose.y = current_pose->y;
             raw_share_pose.theta = current_pose->theta;
             share_pose.is_correcting = current_pose->is_correcting; //矫正参数
             point->correct_index = share_pose.is_correcting;//矫正参数
             
        }
        
        FRIZY_LOG(LOG_DEBUG,"real1.%f,%f,%f,%f %d"
        ,raw_share_pose.x,raw_share_pose.y,raw_share_pose.theta,current_pose->timeStamp,current_pose->is_correctMap);

        share_pose.x = (raw_share_pose.x * 100)/15;
        share_pose.y = (raw_share_pose.y * 100)/15;
        share_pose.theta = raw_share_pose.theta *180/_Pi;
        


        
        FRIZY_LOG(LOG_DEBUG,"real2.%f,%f,%f,%f",share_pose.x,share_pose.y,share_pose.theta,jumpAngle);

        if (fabs(jumpX - share_pose.x) > 0.3 && fabs(jumpY - share_pose.y) <= 0.3)
        {
                //jumpingCount ++;
            printf("JumpingX.%f   %d\n",fabs(jumpX - share_pose.x)*15,IsWall());
        }
        else if (fabs(jumpX - share_pose.x) <= 0.3 && fabs(jumpY - share_pose.y) > 0.3)
        {
                //jumpingCount ++
            printf("JumpingY.%f   %d\n",fabs(jumpY - share_pose.y)*15,IsWall());
        }
        else if (fabs(jumpX - share_pose.x) > 0.3 && fabs(jumpY - share_pose.y) > 0.3)
        {
                //jumpingCount ++;
            printf("JumpingXY.%f.%f   %d\n",fabs(jumpX - share_pose.x)*15,fabs(jumpY - share_pose.y)*15,IsWall());
        }
	    if (fabs(jumpAngle - share_pose.theta) > 10 && fabs(jumpAngle - share_pose.theta) < 90)
        {
                //jumpingCount ++;
            printf("JumpingAngle.%f   %d\n",fabs(jumpAngle - share_pose.theta),IsWall());
        }


        static int warnTime = 0;

        if (GLOBAL_CONTROL == WHEEL_RUN){

            if (fabs(share_pose.x - jumpX) < 0.0001 && fabs(share_pose.y - jumpY) < 0.0001
                && fabs(share_pose.theta - jumpAngle) < 1){
            //if (1){
                FRIZY_LOG(LOG_DEBUG,"warntime %d",warnTime);
                ++warnTime;

            }else{
                warnTime = 0;
            }
            if (warnTime > 5000){
                FRIZY_LOG(LOG_DEBUG,"baojing2");
                chassisSpeed(0,0,1);
                exit(0);
            }
            if (fabs(share_pose.x) > 200 || fabs(share_pose.y) > 200){
                FRIZY_LOG(LOG_DEBUG,"baojing3");
                chassisSpeed(0,0,1);
                exit(0);
            }
        }


        jumpX = share_pose.x;
        jumpY = share_pose.y;
        jumpAngle = share_pose.theta;

        point->realx = share_pose.x;
        point->realy = share_pose.y;

        if (fabs(share_pose.x) < 0.1 && fabs(share_pose.y) < 0.1
            && fabs(point->x - share_pose.x)+fabs(point->y - share_pose.y) > 2.0){
            FRIZY_LOG(LOG_DEBUG,"fuwei");
            point->x = 0;
            point->y = 0;
        }
        else{
            //重塑XY整数坐标与子坐标
            if (share_pose.x - point->x >= 1) point->x ++;
                    
            if (share_pose.x - point->x <= -1) point->x --;	

            if (suodan < 1000){
                printf("suozhu.%d",suodan);
                point->y = suodan;
            }
            else{
                if (share_pose.y - point->y >= 1) point->y ++;
                if (share_pose.y - point->y <= -1) point->y --;	
            }
        }
                                        
        //
        point->dx = 1000*(share_pose.x - point->x);
        point->dy = 1000*(share_pose.y - point->y);


        //计算当前朝向
        //current_pose.theta = int(current_pose.theta);
        if (share_pose.theta <= 0 && share_pose.theta >= -180) 
                point->forward = fabs(share_pose.theta); 
        else
                point->forward = 360 - share_pose.theta;

        // point->forward = share_pose.theta;

        

        //计算累积角度
        int diff = 0;
        int iagg = point->forward;
        if (abs(iagg - lastAngle) > 300)
        {
            if (iagg < lastAngle)
                diff = 360 - lastAngle + iagg;
            else
                diff = iagg - 360 -  lastAngle;	
        }
        else
            diff = iagg - lastAngle;

        
        lastAngle = iagg;

        tempadd = tempadd + diff;
        
        point->addAngle = tempadd;

        return;

}
void chassisBase::getPlanningInfo(Planning_info* planning_info)
{ 

        share_mem_sync sync(current_path_planning);

        //
        planning_info->charger_front_position.x = current_path_planning->frontOfChargingPile.x;
        planning_info->charger_front_position.y = current_path_planning->frontOfChargingPile.y;
        planning_info->charger_seat_position.x = current_path_planning->chargingPilePos.x;
        planning_info->charger_seat_position.y = current_path_planning->chargingPilePos.y;

        planning_info->charger_seat_position.isNew = current_path_planning->chargingPilePos.isNew;

        
        FRIZY_LOG(LOG_DEBUG,"seatPoint  %f %f"
        ,planning_info->charger_seat_position.x,planning_info->charger_seat_position.y);

        if (fabs(planning_info->charger_seat_position.x) > 100
             || fabs(planning_info->charger_seat_position.y) > 100){

            FRIZY_LOG(LOG_DEBUG,"guiling!");     
            planning_info->charger_seat_position.x = 0;
            planning_info->charger_seat_position.y = 0;     

        }

        FRIZY_LOG(LOG_DEBUG,"charger_front.%f,%f isNew = %d"
        ,current_path_planning->frontOfChargingPile.x,current_path_planning->frontOfChargingPile.y
        ,current_path_planning->selectedPointPos.isNew);


        // need to add
        planning_info->cleanBlock->isNew = current_path_planning->selectedBlock->isNew;
        planning_info->cleanPointPos.isNew = current_path_planning->selectedPointPos.isNew;
        
        //
        if(getBlockInfoIdenx == true)
        {
            _maze.current_planning_info.cleanBlock->bottomLeftCorner = current_path_planning->selectedBlock->bottomLeftCorner; 
            _maze.current_planning_info.cleanBlock->bottomRightCorner = current_path_planning->selectedBlock->bottomRightCorner;
            _maze.current_planning_info.cleanBlock->topLeftCorner = current_path_planning->selectedBlock->topLeftCorner;
            _maze.current_planning_info.cleanBlock->topRightCorner = current_path_planning->selectedBlock->topRightCorner;

            
            // planning_info->cleanBlock->bottomLeftCorner = current_path_planning->selectedBlock->bottomLeftCorner; 
            // planning_info->cleanBlock->bottomRightCorner = current_path_planning->selectedBlock->bottomRightCorner;
            // planning_info->cleanBlock->topLeftCorner = current_path_planning->selectedBlock->topLeftCorner;
            // planning_info->cleanBlock->topRightCorner = current_path_planning->selectedBlock->topRightCorner;
            
            _maze.blockCenterPoint.x = (current_path_planning->selectedBlock->bottomLeftCorner.x*100/15 + current_path_planning->selectedBlock->topRightCorner.x*100/15)/2;
            _maze.blockCenterPoint.y = (current_path_planning->selectedBlock->bottomLeftCorner.y*100/15 + current_path_planning->selectedBlock->topRightCorner.y*100/15)/2;
            
            FRIZY_LOG(LOG_DEBUG, "the huaqu is %d %d",_maze.blockCenterPoint.x,_maze.blockCenterPoint.y); 
            //current_path_planning->selectedBlock->isNew = 0;
            getBlockInfoIdenx = false;
        }
        // FRIZY_LOG(LOG_DEBUG, "getPointInfoIndex = %d",getPointInfoIndex);
        // FRIZY_LOG(LOG_DEBUG, "planning_info->cleanPointPos.isNew = %d",planning_info->cleanPointPos.isNew);
        if(getPointInfoIndex == true)
        {           
            _maze.pointCleanPosition.x = round_doubel(current_path_planning->selectedPointPos.x*100/15);
            _maze.pointCleanPosition.y = round_doubel(current_path_planning->selectedPointPos.y*100/15);
            //planning_info->cleanPointPos.isNew = IS_OLD;
            getPointInfoIndex = false;
            FRIZY_LOG(LOG_DEBUG, "the zhina is %f %f",_maze.pointCleanPosition.x,_maze.pointCleanPosition.y);
            //FRIZY_LOG(LOG_DEBUG, "cleanPointPos == %f ,%f",current_path_planning->selectedPointPos.x,current_path_planning->selectedPointPos.y);
        }
        
        if(getForbbidenInfoIndex == true)
        {
            for(int i =0 ; i < MAX_BLOCK_NBR;i++)
            {
                //if(current_path_planning->prohibitedBlock.Block[i].isNew == 1)
                
                _maze.current_planning_info.forbidenArea.Block[i].bottomLeftCorner = current_path_planning->prohibitedBlock.Block[i].bottomLeftCorner;
                _maze.current_planning_info.forbidenArea.Block[i].bottomRightCorner = current_path_planning->prohibitedBlock.Block[i].bottomRightCorner;
                _maze.current_planning_info.forbidenArea.Block[i].topLeftCorner = current_path_planning->prohibitedBlock.Block[i].topLeftCorner;
                _maze.current_planning_info.forbidenArea.Block[i].topRightCorner = current_path_planning->prohibitedBlock.Block[i].topRightCorner;
                    //planning_info->forbidenArea.Block[i].isNew = 1;
                       
                FRIZY_LOG(LOG_DEBUG, "forbin1 is %f %f  %f %f"
                ,_maze.current_planning_info.forbidenArea.Block[i].bottomLeftCorner.x
                ,_maze.current_planning_info.forbidenArea.Block[i].bottomRightCorner.x
                ,_maze.current_planning_info.forbidenArea.Block[i].topLeftCorner.x
                ,_maze.current_planning_info.forbidenArea.Block[i].topRightCorner.x);  
                                    
                    
            }
            getForbbidenInfoIndex = false;
            //FRIZY_LOG(LOG_DEBUG, " update the  ForbbidenInfo ");
        }
}

WHEELSTATE chassisBase::getWheelState()
{
        int leftSpeed = 0;
        int rightSpeed = 0;
        // GetSensor(&cur_sensor);
        leftSpeed = getSendSpeed().first;
        rightSpeed = getSendSpeed().second;
        if(IsWall() != EXIT_WALL && leftSpeed > 0 && rightSpeed >0)
            return WHEELFIXSPEED;
        else if((leftSpeed > 0 && rightSpeed > 0) && abs(leftSpeed - rightSpeed) <= 30)
            return WHEELFRONT;
        else if((leftSpeed < 0 && rightSpeed < 0) && abs(leftSpeed - rightSpeed) <= 30)
            return WHEELBEHIND;
        else if((leftSpeed < 0 && leftSpeed < rightSpeed && abs(rightSpeed + leftSpeed) >= 50) || 
            (rightSpeed > 0 && rightSpeed > leftSpeed && abs(rightSpeed + leftSpeed) >= 50))
            return WHEELNUILROTLEFT;
        else if((rightSpeed < 0 && rightSpeed < leftSpeed && abs(rightSpeed + leftSpeed) >= 50) ||
            (leftSpeed > 0 && rightSpeed < leftSpeed && abs(rightSpeed + leftSpeed) >= 50))
            return WHEELNUILROTRIGHT;
        else if(leftSpeed < rightSpeed && abs(leftSpeed + rightSpeed) <= 20)
            return WHEELLEFTSPIN;
        else if(leftSpeed > rightSpeed && abs(leftSpeed + rightSpeed) <= 20)
            return WHEELRIGHTSPIN;
        else if(leftSpeed == 0 && rightSpeed == 0)
            return WHEELSTOP; 
}

void chassisBase::wheelCtrlStop()
{
    FRIZY_LOG(LOG_DEBUG, "wheelCtrlStop");
    // while(getWheelState() != WHEELSTOP)
    // {
    //     GetSensor(&cur_sensor);
    //     if(abs(cur_sensor.midOmnibearingOn - cur_sensor.midOmnibearingOff) >= 3500)
    //     {
    //         chassisSpeed(0, 0, 1);
    //         return;
    //     }
    //     else 
    //         chassisSpeed(cur_sensor.leftw * 0.5, cur_sensor.rightw * 0.5, 1);
    //     usleep(20 * 1000);
    // }
    GetSensor(&cur_sensor);
    chassisSpeed(cur_sensor.leftw * 0.5, cur_sensor.rightw * 0.5, 1);
    for(int i = 0; i < 20; i++)
    {
        usleep(20 * 1000);
    }
    chassisSpeed(0, 0, 1);
}

int chassisBase::MakeChassisGoStraight(int16_t speed,GridPose targetPose)
{
        FRIZY_LOG(LOG_INFO, "start to chassis go straight ");
        //printf("Speed = %d; mode:goSTRAIGHT")
        robotPose = GetCurGridPose();
        while(fabs(sqrt((robotPose.i - targetPose.i)*(robotPose.i - targetPose.i)+(robotPose.j - targetPose.j)*(robotPose.j - targetPose.j)) )> variance)
        {
           chassisSpeed(speed,goSTRAIGHT);
           robotPose = GetCurGridPose();     
        }
}       

int chassisBase::MakeChassisTurnLeft(int16_t speed, int angle)
{
        robotPose = GetCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        //how to rotato angle??? to do
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {chassisSpeed(speed,turnLEFT);
        robotPose = GetCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d left done ",angle);
}
int chassisBase::MakeChassisTurnright(int16_t speed, int angle)
{
        //how to rotato angle??? to do
        robotPose = GetCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {
        chassisSpeed(speed,turnRIGHT);
        robotPose = GetCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d right done ",angle);
}
int chassisBase::MakeChassisGoTurn(int16_t leftspeed,int16_t rightspeed,int16_t times)
{
        FRIZY_LOG(LOG_DEBUG, "start to excute go turn ");
        targetRobotPose = GetCurGridPose();
        for(int i =0 ;i <times;i++)
        {
              chassisSpeed(leftspeed,rightspeed,1);
              robotPose = GetCurGridPose();
              while(sqrt((robotPose.i - targetRobotPose.i)*(robotPose.i - targetRobotPose.i)+(robotPose.j - targetRobotPose.j)*(robotPose.j - targetRobotPose.j))>0.1)//判断条件太生硬
              {
                   chassisSpeed(leftspeed,rightspeed,1);   
              }
        }
}

int chassisBase::MakeBaseRecharge()
{
        FRIZY_LOG(LOG_INFO, "start to excute base Recharge ");
        chassisRecharge();
}
int chassisBase::MakeBaseEscapeJail()
{
        FRIZY_LOG(LOG_INFO, "start to excute base escapejail ");
        escapeJail();
}
int chassisBase::MakeBaseAlongWall(uint8_t mode,int direction)
{
        if(direction == right_allwalldirection){
                alongWall(mode,RightAlongwall);
        }
        if(direction == left_allwalldirection){
                alongWall(mode,LeftAlongwall);
        }
}
int chassisBase::MakeSmartRecharge()
{

}
int chassisBase::chassisSpeed(int16_t speed,int way)
{       
        
        int16_t leftWhell = 0;
        int16_t rightWhell = 0;

        if(way == goSTRAIGHT){
                leftWhell = speed;
                rightWhell = speed;  
        }
        //how to rotato angle??? to do
        if (way == turnLEFT){
                leftWhell = -speed;
                rightWhell = speed;
        }
        //how to rotato angle??? to do
        if(way == turnRIGHT){
                leftWhell = speed;
                rightWhell = -speed;
        }   


        if(GLOBAL_CONTROL==WHEEL_RUN)
        {
            FRIZY_LOG(LOG_INFO, "start to excute the chassisspeed ");
            UMAPI_MainWheelSpeed(1,leftWhell,rightWhell);}
        if(GLOBAL_CONTROL == WHEEL_PAUSE || GLOBAL_CONTROL == WHEEL_STOP)
        {
                FRIZY_LOG(LOG_INFO, "start to stop the chassisspeed ");
                UMAPI_MainWheelSpeed(1,0,0);
        }               
        return 0;
}

// int chassisBase::chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode)
// {
//         memset(&controlData,0,sizeof(controlData));
//         controlData.cmd = CHASSIS_CMD_ROAD;
//         controlData.len = 10;

//         controlData.data[0] =0xFF;
//         controlData.data[1] =0xFE;
//         controlData.data[2] =0x42;
//         controlData.data[3] =0x05;
//         if(leftspeed < 0 ){
//                 leftspeed = leftspeed + 0xFFFF;
//         }
//         if(rightspeed < 0 ){
//                 rightspeed = rightspeed + 0xFFFF;
//         }
//         controlData.data[4] = (leftspeed & 0x00FF);
//         controlData.data[5] = (leftspeed & 0xFF00) >> 8;
//         controlData.data[6] = (rightspeed & 0x00FF);
//         controlData.data[7] = (rightspeed & 0xFF00) >> 8;
//         controlData.data[8] = mode; //1 :normal mode 2:accelerate mode 
//         //calc 校验
//         for(int i = 0; i < controlData.len -1; i++){
//                 controlData.data[9] ^= controlData.data[i];
//         }
//         //共享内存写入 umweritemem ;
//         // UM_WriteControlMem(controlData);
//         return 0;        

// }

bool chassisBase::getSpeedOrder()
{
    return speedOrder;
}

std::pair<int, int> chassisBase::getSendSpeed()
{
    std::pair<int, int> tmp;
    tmp.first = leftWheel;
    tmp.second = rightWheel;
    return tmp;
}

int lastLeft = 0;
int lastRight = 0;
int lastLeft2 = 0;
int lastRight2 = 0;
static int stopTime = 0;
int chassisBase::chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode)
{
        rightWheel = rightspeed;
        leftWheel = leftspeed;
        if(leftspeed == 0 && rightspeed == 0)
            speedOrder = 0;
        else 
            speedOrder = 1;
        if(GLOBAL_CONTROL==WHEEL_RUN)
        {

                if (IsWall() == 0)
                {
                        if (leftspeed > 0 && rightspeed > 0 && lastLeft + lastRight == 0 && lastLeft2 + lastRight2 == 0  
                                && mode != 100)
                        {
                                stopTime = 3;
                        }

                        lastLeft2 = lastLeft;
                        lastRight2 = lastRight;
                        lastLeft = leftspeed;
                        lastRight = rightspeed;
                        FRIZY_LOG(LOG_DEBUG,"send.%d.%d",lastLeft,lastRight);
                        if (stopTime > 0)
                        {
                                FRIZY_LOG(LOG_DEBUG,"'stop!!");
                                MotionControl chassisMotion;
                                chassisMotion.ClearPid();
                                leftspeed = 0,rightspeed = 0;
                                // if(process == PLAN && calibration_time >2400)
                                // {
                                //         GetSensor(&curSensor);
                                //         if(curSensor.rightw ==0 && curSensor.leftw == 0)
                                //         {                                                
                                //                 Angle_calibration();
                                //                 usleep(50*1000);                                              
                                //         }
                                // }
                                stopTime --; 

                        }
                }
                // FRIZY_LOG(LOG_INFO, "start to excute the chassisspeed ");
                if(leftspeed == 0 && rightspeed == 0)
                {
                        FRIZY_LOG(LOG_INFO,"robot stop spin");
                }
                if (mode == 100) mode = 1;
                UMAPI_MainWheelSpeed(mode,leftspeed,rightspeed);
        }
        if (mode == 100) mode = 1;
        
        if(GLOBAL_CONTROL == WHEEL_PAUSE || GLOBAL_CONTROL == WHEEL_STOP)
        {
            FRIZY_LOG(LOG_INFO, " start to stop chassisspeed %d ",GLOBAL_CONTROL);
            UMAPI_MainWheelSpeed(1,0,0);
        }
        return 0;        

}

int chassisBase::chassisRoadFIX()
{
        
}
int chassisBase::escapeJail()
{


        UMAPI_CtrlWalkState(7);
        return 0;
}
int chassisBase::chassisRecharge()
{
        //标准回冲


        FRIZY_LOG(LOG_INFO, "start to recharge ");
        UMAPI_RobotMode(5);
        return 0;
}


int chassisBase::alongWall(uint8_t mode,uint8_t direction)
{
        if (mode == 1) wall = 1;
        if (mode == 0) wall = 0;
        if (mode == 1 && direction == 2) wall = 2;
        
        
        //unwerite
        // UM_WriteControlMem(controlData);
        FRIZY_LOG(LOG_INFO, "start to alongwalk ");
        UMAPI_RobotMode(3);
        return 0;
}
void chassisBase::leaveChargerParaInit()
{
        leaveCharger_backTime = 225;
        robot_recharge_rotate_times = 16;
        leaveCharger_backsign = 1;
        leaveCharger_rotateSign = 1;
        leaveCharger_aimForward = 0.0;
        leaveCharger_recordForward = 0.0;
        
        
}
static bool gyo_correct =false;
int chassisBase::rechargeRetreat()
{
        
        FRIZY_LOG(LOG_INFO,"ENTER TO rechargeRetreat Mode");
        GetSensor(&curSensor);
        //是否在回充座上
        FRIZY_LOG(LOG_INFO,"curSensor.rechargeSign:%d",curSensor.rechargeSign);
        if(!curSensor.rechargeSign)
        {
               
                FRIZY_LOG(LOG_ERROR,"robot no charge");
                // return -1;
        }
        FRIZY_LOG(LOG_INFO,"xialai!");
        leaveChargerParaInit();
        // FRIZY_LOG(LOG_INFO,"THE GLOBAL_CONTROL = %u",GLOBAL_CONTROL);

        while(GLOBAL_CONTROL == WHEEL_RUN)
        {       
                if(GLOBAL_CONTROL != WHEEL_RUN)
                {
                    FRIZY_LOG(LOG_ERROR,"robot not in wheel_run mode");
                    chassisSpeed(0, 0, 1);
                    usleep(100*1000);
                    return -1;
                }
                //后退50cm
                GetSensor(&curSensor);
                usleep(20 * 1000);
                // FRIZY_LOG(LOG_INFO,"curSensor.rechargeSign:%d , leaveCharger_backTime: %d",curSensor.rechargeSign,leaveCharger_backTime);
                if(leaveCharger_backTime && (GLOBAL_CONTROL == WHEEL_RUN))
                {
                        chassisSpeed(-100, -100, 1);
                        leaveCharger_backTime --;
                        if(leaveCharger_backTime)
                        {
                                continue;
                                        // FRIZY_LOG(LOG_ERROR,"continue test");
                        }
                        else
                        {
                                if(leaveCharger_backsign)
                                {
                                        chassisSpeed(0, 0, 1);
                                        leaveCharger_backsign = 0;
                                        FRIZY_LOG(LOG_INFO,"backtime:%d,后退完毕,发送停止轮速",leaveCharger_backTime);
                                        
                                }
                        }
                }
                
                // if(curSensor.Initialized == 1)
                // // if(curSensor.XAcc == 0.0 &&curSensor.XAngle == 0.0&&curSensor.YAcc ==0 && curSensor.YAngle ==0 &&curSensor.ZAcc ==0&& curSensor.ZAngle ==0)
                // {
                //       FRIZY_LOG(LOG_INFO,"陀螺仪初始化成功,等待旋转");
                //       gyo_correct = true;
                                           
                // }
                // else
                // {
                //      usleep(100*1000);
                //      gyo_correct = false;
                //      continue;
                     
                // }

                // if(gyo_correct == true)
                {                        
                        // if(leaveCharger_rotateSign)
                        // {
                        //         usleep(3000*1000);
                        //         FRIZY_LOG(LOG_INFO,"确定目标角度");
                        //         // GridPoint(&curGrid);
                        //         // recordForward = curGrid.forward;
                        //         leaveCharger_aimForward = gyo_angle_*180/_Pi + 180;
                        //         if(leaveCharger_aimForward >= 360)
                        //                 leaveCharger_aimForward = leaveCharger_aimForward - 360;
                        //         FRIZY_LOG(LOG_DEBUG,"recordForward: %f, aimforward: %f",leaveCharger_recordForward, leaveCharger_aimForward);
                        //         leaveCharger_rotateSign = 0;
                        // }
                        // if(gyo_angle_*180/_Pi != 0)
                        {
                        usleep(200*1000);
                        
                        FRIZY_LOG(LOG_INFO, " START TO ROTATE IN RECHARGE MODE");
                        if(robot_recharge_rotate_times)
                        {
                                chassisSpeed(100,-100,1);
                                robot_recharge_rotate_times--;
                                FRIZY_LOG(LOG_DEBUG, " robot_recharge_rotate_times : %d",robot_recharge_rotate_times);
                        }
                        else
                        {
                                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                                chassisSpeed(0,0,1);
                                usleep(20 * 1000);
                                return 0;
                        }

                        // GetSensor(&curSensor);
                        // FRIZY_LOG(LOG_DEBUG,"rotate cursensor.leftw: %d,cursensor.rightw: %d",curSensor.leftw,curSensor.rightw);
                        // FRIZY_LOG(LOG_DEBUG,"当前角度: %f,目标角度:%f",gyo_angle_*180/_Pi,leaveCharger_aimForward);
                        // if(fabs(leaveCharger_aimForward - gyo_angle_*180/_Pi) > 0 && fabs(leaveCharger_aimForward - gyo_angle_*180/_Pi) < 3)
                        // {
                        //         FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");     
                        //         chassisSpeed(0,0,1);
                        //         usleep(20 * 1000);
                        //         gyo_correct =false;
                        //         return 0;
                        // }
                        }
                }                 
        }
}
static int calibration_state;
static int tmp_state;
static void gyro_stat_resp(int calibration_state)
{
        FRIZY_LOG(LOG_DEBUG,"gyro_stat_resp function");
        tmp_state = calibration_state;
}
bool chassisBase::_gyo_calibration()
{
      FRIZY_LOG(LOG_INFO, "GYO CALIBRATION");
      UMAPI_CtrlGyro(1,gyro_stat_resp,&tmp_state);
}
void chassisBase::judy_gyo_calibration_ok()
{
        // GetSensor(&curSensor);
        if (tmp_state ==1)
        {             
              gyro_stat_resp_state = true;
        }
}
int chassisBase::gyo_calibration()
{
        
        if(GLOBAL_CONTROL != WHEEL_RUN)
        {
                gyo_calibration_rotate_times = 3; 
                gyo_calibration_forward_times = 3;                
        }
        FRIZY_LOG(LOG_INFO, "GYO CALIBRATION");
        FRIZY_LOG(LOG_DEBUG,"start to rotate for gyo_calibration");
        if(gyo_calibration_rotate_times&&GLOBAL_CONTROL == WHEEL_RUN)
        {
                chassisSpeed(100,-100,1);
                gyo_calibration_rotate_times-- ;
                return 1 ;  
        }
        FRIZY_LOG(LOG_DEBUG,"start to forward for gyo_calibration");
        if(gyo_calibration_forward_times&&GLOBAL_CONTROL == WHEEL_RUN)
        {
                chassisSpeed(100,100,0);
                gyo_calibration_forward_times--;
                return 1;
        }

        FRIZY_LOG(LOG_DEBUG,"start to send the calibration sign to mcu");
        UMAPI_CtrlGyro(1,gyro_stat_resp,&tmp_state);
        GetSensor(&curSensor);
        FRIZY_LOG(LOG_DEBUG,"curSensor.Initialized = %d",curSensor.Initialized);
        
        if(curSensor.Initialized != 1 && GLOBAL_CONTROL == WHEEL_RUN)
        {
                
                if(curSensor.Initialized != 1)
                {
                        usleep(20*1000);
                        GetSensor(&curSensor);
                        gyo_record_times ++;
                        if(gyo_record_times > 200)
                        {
                                gyo_calibration_index =false;
                                gyo_record_times =0;
                                return 1 ;
                        }
                        return 1 ;
                        
                        
                }
                else if (curSensor.Initialized ==1 )
                {
                        // if(gyo_calibration_rotate_times == 0 && gyo_calibration_forward_times == 0)
                        // {
                        //         gyo_calibration_rotate_times = 100; 
                        //         gyo_calibration_forward_times = 100;
                        // }
                        GetSensor(&curSensor);
                        if(gyo_calibration_for_X_Y_angle_times)
                        {
                                gyo_calibration_for_X_Y_angle_times-- ;
                                chassisSpeed(100,-100,1);
                                min(minXAngle ,curSensor.XAngle);
                                max(maxXAngle,curSensor.XAngle);
                                min(minYAngle ,curSensor.YAngle);
                                max(maxYAngle,curSensor.YAngle);                      
                                return 1 ;
                                
                        }
                        if(fabs(minXAngle -maxXAngle)<3 && fabs(minYAngle -maxYAngle)<3)
                        {
                                gyo_calibration_index =true;
                                gyo_calibration_rotate_times = 3; 
                                gyo_calibration_forward_times = 3; 
                                return 1;
                        }
                        else
                        {
                                gyo_calibration_index =false;
                                gyo_calibration_rotate_times = 3; 
                                gyo_calibration_forward_times = 3; 
                                return 0;                                
                        }
                        
                        
                }
        }

        

}
void chassisBase::relocation()
{
        
        

}

static int gyro_stat_resp_time = 60;
void chassisBase::Angle_calibration()
{
        FRIZY_LOG(LOG_DEBUG,"start to Angle_calibration in uturn mode"); 
        // chassisSpeed(0,0,1);
        UMAPI_CtrlGyro(2,gyro_stat_resp,&tmp_state);
        calibration_time = 0;
        // usleep(2500*1000);
        while(gyro_stat_resp_time&&calibration_state!=1)
        {
                usleep(100*1000);
                FRIZY_LOG(LOG_DEBUG,"wait for calibration for mcu ");
                gyro_stat_resp_time--;

        }
        FRIZY_LOG(LOG_DEBUG,"resume the mcu mode to auto "); 
        gyro_stat_resp_time = 60;
        UMAPI_RobotMode(1);
}

}