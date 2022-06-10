/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-08 17:32:52
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef INTERNALSPIRAL_PLANNING_H
#define INTERNALSPIRAL_PLANNING_H

#include "common_function/MotionControl.h"
#include "navigation_algorithm/GlobalPlanning.h"

using namespace std;

namespace useerobot
{
    extern CHASSIS_CONTROL GLOBAL_CONTROL;
    extern RoadSensorData_t sensordata;
    class InternalSpiralPlanning
    {
    private:
        /* data */

        Point * astarResult = NULL;
        list<Point*> astarPath;
        int cleaning_interval = int(robot_radius/resolution);
        Point tempPoint; 
        MotionControl _motioncontrol;
        wheel_mode _wheel_mode;
        move_data  _move_data;
        Sensor current_sensor;
        chassisBase chassis_;
        Sensor fix_sensor_;
        bool running_;
    public:
        InternalSpiralPlanning(/* args */);
        ~InternalSpiralPlanning();
        /**
         * @description: 
         * @event: 
         * @param {Point} &startPoint
         * @param {Point} &endPoint
         * @param {RobotType} &robotShape
         * @return {*}
         */        
        bool goPoint(Point &startPoint,Point &endPoint,RobotType &robotShape);
        /**
         * @description: 
         * @event: 
         * @param {Point} &cleanPoint
         * @param {RobotType} &robotShape
         * @param {int} &times
         * @return {*}
         */     
        bool internalSpiralCleanPoint(Point &cleanPoint,RobotType &robotShape,int &layers);
        /**
         * @description: 
         * @event: 
         * @param {Point} &cleanPoint
         * @param {RobotType} &robotShape
         * @param {int} &times
         * @return {*}
         */        
        bool pointClean(RobotType &robotShape);

        void fixPointDeal();

        void fixPointInit();

        void randomClean();

        void backDis(int speed, int dis);

        int spin(int lSpeed, int rSpeed, int angle);
                
        void stopPointClean();

        bool getRunning();
        list<Point*> InteralSpiralPath;
        bool randomClean_start_index = false;
        int pointCleanTime = 1200;
    };
    

    
}


#endif
