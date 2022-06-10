/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-27 15:59:33
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef ROADPLANNING_H
#define ROADPLANNING_H

#include "common_function/logger.h"
#include "navigation_algorithm/GlobalPlanning.h"
#include "common_function/MotionControl.h"
#include "navigation_algorithm/LocalPlanning.h"
#include "common_function/ExceptionHanding.h"
#include "navigation_algorithm/AlongWall.h"

using namespace std;

namespace useerobot
{
    extern int  recharge_first_index ;
    enum ROAD_KIND
    {
        idle = 0,
        searchUnclean = 1,
        searchInside,
        searchBound,
        searchLast,
        archwallSign,
        backBound,
        recharge,
        searchWall,
        lastUnclean,
        column,
        zoning,
        appoint,
        none,
        home,
        breakClean,
    };
    enum ROAD_STATE
    {
        roadIdle,
        roadTrue,
        roadFalse,
    };


    struct RoadAim
    {
        int x;
        int y;
        enum ROAD_KIND kind;
    };


    using Points = std::vector<std::pair<int, int>>;
    class RoadPlanning
    {
    private:
        /* data */
        
        AlongWall _wall;
        chassisBase chassisRoad;
        MotionControl motionRoad;

        Trouble _trouble;
        int sum_recharge_times;
        
        int pathType;
        //vector <pair<float, float>> astarArr;
        Points astarArr;
        void DwaRunning(Sensor sensor,Grid cur);
        void NullRunning(Sensor sensor,Grid cur);
        int ArriveRoad(Sensor sensor,Grid cur);
        
    public:
        RoadPlanning(/* args */);
        ~RoadPlanning();
        Grid ConAngle(Grid cur,Grid aim);
        int8_t BroadArea(int16_t x,int16_t y);
        void reCall(Grid curr,Grid center,int kind);
        Grid _aim;
        Grid ewall;
        int nearpoint;
        int astarAgain;
        int againCount;
        bool fast_near_charge = false;
        int ewallTime;
        int timeV;
        int bumpCount;
        bool dewallSign;
        int failCount;
        int forceWall;
        int start_add;
        int goWall;
        // Maze _maze;
        dwa roadDwa;
        aStar roadAstar;
        vector<Grid> CloseA;
        vector<Grid> workOb;
        vector<Grid> otherSide;
            
            
        Grid curr;
        Grid cur_work;
        int BUMP_MAX = 8;
        bool call_recharge_index = false;
        void init();
        void SetroadAim(RoadAim aim);
        void StartRoad(Sensor sensor,Grid cur,Trouble trouble);

    };
       
}
#endif

