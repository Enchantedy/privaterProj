/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-26 14:49:55
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/UTurnPlanning.h"
#include <math.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <boost/heap/d_ary_heap.hpp>


int cur_x = 0;
int cur_y = 0;
extern useerobot::Maze _maze;
namespace useerobot
{
    extern bool left_charger_index;                               
    extern int areaClean;
    extern PRO process;
    extern current_pose_t* current_pose;
    RoadAim record_charge_aim;
    extern vector <Grid> boundPoint;
    extern vector <boundary> boundArr;
    extern int lasttake;
    bool cleanTaskOverIndex = false;
    ROAD_KIND searchKind = idle;
    vector <GIVEUP> giveUp;
    //static int archwallSign = 0;
    static ARCHWALL arch_wall;

    static Grid gaSengAim;
    vector <GIVEUP> obarr;
    int suodan = 1001;
    
    _UTURN UTURN;
    

    UTurnPlanning::UTurnPlanning(/* args */)
    {

    }

    UTurnPlanning::~UTurnPlanning()
    {

    }

    static int sign1 = 5;

    void UTurnPlanning::init()
    {
        sign1 = 5;
        suodan = 1001,recordY = 1001;
        spin720 = 0;

        
        // if (_maze.reclean) 
        //     UTURN = SEARCH_WALL; 
        // else
        UTURN = EXPLORE;
        
        //UTURN = ARCH;
        searchKind = idle;   
        ARCH_STATE = LEFT_0;
        keepwallTime = 0;
        aim.x = 1000;
        aim.y = 1000;
        aim.forward = 1000;
          
        obarr.clear();    
        giveUp.clear();
        bumpwallTime = 0;

        forTime = 0;
        spinTime = 0;
        exploreTime = 0;
        ex_state = EX_SPIN;
    }
    
    //判断有没有可能是假点
    int8_t UTurnPlanning::Virob(int x,int y)
    {
	
        int8_t temps = 0;
        
        //shang
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x-1,y,2) == 1 
            && _maze.GetMapState(x+1,y,2) == 0 && _maze.GetMapState(x+1,y,1) != 2)
            temps = 1;
        
        //xia
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x+1,y,2) == 1 
            && _maze.GetMapState(x-1,y,2) == 0 && _maze.GetMapState(x-1,y,1) != 2)
            temps = 2;
        //zuo
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x,y-1,2) == 1 
            && _maze.GetMapState(x,y+1,2) == 0 && _maze.GetMapState(x,y+1,1) != 2)
            temps = 3;						
        //	you
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x,y+1,2) == 1 
            && _maze.GetMapState(x,y-1,2) == 0 && _maze.GetMapState(x,y-1,1) != 2)
            temps = 4;	

        if (temps != 0)
        {
            for (int i = 0;i < obarr.size();i++)
            {
                if (obarr[i].x == x && obarr[i].y == y)
                {
                    FRIZY_LOG(LOG_DEBUG,"gangan.%d.%d",x,y);
                    return 0;
                }
            }   
            //FRIZY_LOG(LOG_DEBUG,"temps.%d",temps);     
            return temps;
        }
        else
            return 0;
    }
    
    void UTurnPlanning::Kmeans(int x,int y,int kinds)
    {
        int8_t count = 0;
        FRIZY_LOG(LOG_DEBUG,"Kmeans.%d,%d.%d",x,y,kinds);

        switch (kinds)
        {
            case 1 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x,y-i) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida1\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x,y-i);
                        obarr.push_back({x,y-i,kinds,0});
                    }
                    else if (count > 0)
                        break;
                }	
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;
            }
            case 2 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x,y+i) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida2\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x,y+i);
                        obarr.push_back({x,y+i,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;                

            }
            case 3 :
            {

                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x-i,y) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida3\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x-i,y);
                        obarr.push_back({x-i,y,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;    
            }
            case 4 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x+i,y) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida3\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x+i,y);
                        obarr.push_back({x+i,y,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;  
            }  
       

       
        }

    }
    
    int UTurnPlanning::ClearOB(Grid cur)
    {
            
        //obarr.clear();
        //FRIZY_LOG(LOG_DEBUG,"zuihouzuihou.%d.%d.%d.%d",archBound.up,archBound.down,archBound.left,archBound.right);

        for (int y = archBound.right+1; y < archBound.left; y++)
        {
            for (int x = archBound.down+1; x < archBound.up; x++)
            {
                int kinds = Virob(x,y);
            
                if (kinds != 0)
                {
                    Kmeans(x,y,kinds);
                }
            }
        }

        for (int i = 0;i < obarr.size();i++)
        {

            FRIZY_LOG(LOG_DEBUG,"gangan2.%d.%d.%d.%d",obarr[i].x,obarr[i].y,obarr[i].count,obarr[i].close);
            
        }   
        FRIZY_LOG(LOG_DEBUG,"clear1.%d",obarr.size());
        //obarr.clear();
        //obarr这一数组提取出目标点,1000下一个索引
        //选取ASTAR距离最近的点
        //FRIZY_LOG(LOG_DEBUG,"clear2.%d",obarr.size());
        
        int dis = 1000;
        int num = obarr.size() - 2;
        RobotType sss;

        for (int i = 0;i < num;i++)
        {
            Grid temp = {obarr[i+2].x,obarr[i+2].y};

            if ((obarr[i].x == 1000 || i == 0)
                && temp.x != 1000 && JudgeGiveup(cur,temp,searchKind) == 0 
                && obarr[i+2].close == 0
                && obarr[i+1].x != 1000 && obarr[i+2].x != 1000)
            {
                vector<pair<int, int>> astarL;
                if (searchKind == searchBound || searchKind == searchUnclean)
                    astarL = astarPlan.astarLength(cur.x,cur.y,obarr[i+2].x,obarr[i+2].y,sss,0,1);
                else
                    astarL = astarPlan.astarLength(cur.x,cur.y,obarr[i+2].x,obarr[i+2].y,sss,0,0);   

                int tempdis = astarL.size();
                if (tempdis == 0)
                    tempdis =  500 + abs(cur.x - obarr[i+2].x) + abs(cur.y - obarr[i+2].y);                 
                FRIZY_LOG(LOG_DEBUG,"obaa.%d,%d,%d",obarr[i+2].x,obarr[i+2].y,tempdis);
                
                if (dis > tempdis)
                {
                    _aim.x = obarr[i+2].x;
                    _aim.y = obarr[i+2].y;
                    dis = tempdis;
                    lasttake = obarr[i+2].count;
                }
            }
        }

        //不存在值得尝试的点
        if (dis == 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"no vir");
            //sssss5 = 0;
            return 0;
        }
        else
        {
            
            FRIZY_LOG(LOG_DEBUG,"333.a == %d,b == %d,dis == %d",_aim.x,_aim.y,dis);

            return 1;			
        }
    }

    void UTurnPlanning::PlanSearch(RoadAim aim,int flag)
    {
        if (flag == 2)
        {
            searchKind = archwallSign;
            gaSengAim.x  = aim.x;
            gaSengAim.y  = aim.y;
            return;
        }

        searchKind = aim.kind;

        if (flag == -1)
            return;

        //flag >= 0
        for (int i = 0; i < giveUp.size();i++)
        {
            if (giveUp[i].x == aim.x && giveUp[i].y == aim.y)
            {
                if (flag == 1)
                {
                    FRIZY_LOG(LOG_DEBUG,"_jia5 : %d %d",aim.x,aim.y);
                    giveUp[i].count = 5; 
                }
                   
                else{
                    FRIZY_LOG(LOG_DEBUG,"_jia1 : %d %d",aim.x,aim.y);
                    giveUp[i].count ++;
                }  
                
                return;
            }    
        }

        GIVEUP temp;
        temp.x = aim.x;
        temp.y = aim.y;
        temp.count = 0;

        if (flag == 1)
            temp.count = 5; 
        else  
            temp.count = 1;

        FRIZY_LOG(LOG_DEBUG,"giveup.%d.%d.%d",temp.x,temp.y,temp.count);
        
        giveUp.push_back(temp);  
    }
    
    int UTurnPlanning::CheckFor(Grid cur,DIVB forward)
    {
        int temps = 0;
        if (forward == UP)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x+i,cur.y,2) != 0)
                {
                    temps = 1;
                    break;
                }        
            }
        }
        if (forward == DOWN)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x-i,cur.y,2) != 0)
                {
                    temps = 1;
                    break;
                }   
            }
        }        
        if (forward == LEFT)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x,cur.y+i,2) != 0)
                {
                    temps = 1;
                    break;
                }   
            }
        }  
        if (forward == RIGHT)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x,cur.y-i,2) != 0)
                {
                    temps = 1;
                    break;

                }   
            }
        } 
        if (temps == 0)
            return 0;
        else
            return 1;
    }    

    int UTurnPlanning::JudgeGiveup(Grid curr,Grid cur,ROAD_KIND kind)
    {


        if (kind == searchInside){
            if (cur.x == archBound.up || cur.x == archBound.down
                || cur.y == archBound.left || cur.y == archBound.right)
                return 1;
        }

        if (kind == searchUnclean || kind == searchBound || kind == searchInside){

            if (kind == searchUnclean || kind == searchInside)
                if (_maze.GetMapState(cur.x+1,cur.y,2) == 2 || _maze.GetMapState(cur.x-1,cur.y,2) == 2
                    || _maze.GetMapState(cur.x,cur.y+1,2) == 2 || _maze.GetMapState(cur.x,cur.y-1,2) == 2
                    ){ //||  _maze.GetMapState(cur.x,cur.y,1) == 2
    
                        FRIZY_LOG(LOG_DEBUG,"ONO1.%d.%d   %d",cur.x,cur.y,_maze.GetMapState(cur.x,cur.y,1));
                        return 1;
                }
                
            if (kind == searchBound)
                if ((_maze.GetMapState(cur.x+1,cur.y,2) == 2 && _maze.GetMapState(cur.x-1,cur.y,2) == 2)
                    || (_maze.GetMapState(cur.x,cur.y+1,2) == 2 && _maze.GetMapState(cur.x,cur.y-1,2) == 2)
                    ){ //||  _maze.GetMapState(cur.x,cur.y,1) == 2
    
                        FRIZY_LOG(LOG_DEBUG,"ONO2.%d.%d   %d",cur.x,cur.y,_maze.GetMapState(cur.x,cur.y,1));
                        return 1;
                }

            //小范围区域丢弃
            CloseArr.clear();

            int tmpS = 0;
            if (kind == searchUnclean || kind == searchInside){

                if (cur.x + 1 == archBound.up || cur.x - 1 == archBound.down
                    || cur.y + 1 == archBound.left || cur.y - 1 == archBound.right)
                    tmpS = 10;
                else
                    tmpS = Dsearch(cur.x,cur.y);
            }  
            else{

                if (_maze.GetMapState(cur.x,cur.y,1) == 2){

                    FRIZY_LOG(LOG_DEBUG,"shiqiang.%d.%d",cur.x,cur.y);
                    return 1;
                }
                int rSign = 0;
                for (int x=cur.x-1; x<=cur.x+1; ++x)
                {
                    if (rSign)
                        break;
                    for (int y=cur.y-1; y<=cur.y+1; ++y)
                        if (_maze.GetMapState(x,y,2) == 0 && abs(x-cur.x) + abs(y-cur.y) == 1){
                            tmpS = Dsearch(x,y);
                            rSign = 1;
                            FRIZY_LOG(LOG_DEBUG,"tmps.%d.%d.%d",tmpS,x,y);
                            break;
                        }
                }
            }
        
            int threshold = kind == searchBound ? 8 : 4;

            if (tmpS < threshold){
                FRIZY_LOG(LOG_DEBUG,"diuqi.%d.%d.%d",cur.x,cur.y,tmpS);
                return 1;   
            }
        }

        // if (kind == searchBound)
        // {
        //     for (int x = cur.x - 1;x <= cur.x + 1;x++)
        //       for (int y = cur.y - 1;y <= cur.y + 1;y++)  
        //         if (_maze.GetMapState(x,y,1) == 2 || _maze.GetMapState(x,y,2) == 2)
        //         {
        //             if (_maze.GetMapState(x,y,1) == 2)
        //                 FRIZY_LOG(LOG_DEBUG,"shiqiang.%d.%d",x,y);
        //             return 1;
        //         }    
        // }

        if ((cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x-1,cur.y,2) == 0)
        || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x+1,cur.y,2) == 0)
        || (cur.y + 1 == archBound.left && _maze.GetMapState(cur.x,cur.y+1,2) == 1 && _maze.GetMapState(cur.x,cur.y-1,2) == 0)           
        || (cur.y - 1 == archBound.right && _maze.GetMapState(cur.x,cur.y-1,2) == 1 && _maze.GetMapState(cur.x,cur.y+1,2) == 0))
        {
            if (kind == searchInside)
            {
                if ((cur.x + 1 == archBound.up && CheckFor(cur,DOWN) == 0)
                    || (cur.x - 1 == archBound.down && CheckFor(cur,UP) == 0)
                    || (cur.y + 1 == archBound.left && CheckFor(cur,RIGHT) == 0)
                    || (cur.y - 1 == archBound.right && CheckFor(cur,LEFT) == 0))
                {
                    FRIZY_LOG(LOG_DEBUG,"guolvlv.%d.%d",cur.x,cur.y);
                    return 1;
                }
            }
            if (kind == searchUnclean){
                // if ((cur.x + 1 == archBound.up && _maze.GetMapState(cur.x,cur.y+1,2) != 1 && _maze.GetMapState(cur.x,cur.y-1,2) != 1)
                //     || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x,cur.y+1,2) != 1 && _maze.GetMapState(cur.x,cur.y-1,2) != 1)
                //     || (cur.y + 1 == archBound.left && _maze.GetMapState(cur.x+1,cur.y,2) != 1 && _maze.GetMapState(cur.x-1,cur.y,2) != 1)
                //     || (cur.y - 1 == archBound.right && _maze.GetMapState(cur.x+1,cur.y,2) != 1 && _maze.GetMapState(cur.x-1,cur.y,2) != 1)){
                if ((cur.x + 1 == archBound.up && !passPoint(cur.x,cur.y+1) && !passPoint(cur.x,cur.y-1))
                    || (cur.x - 1 == archBound.down && !passPoint(cur.x,cur.y+1) && !passPoint(cur.x,cur.y-1))
                    || (cur.y + 1 == archBound.left && !passPoint(cur.x+1,cur.y) && !passPoint(cur.x-1,cur.y))
                    || (cur.y - 1 == archBound.right && !passPoint(cur.x+1,cur.y) && !passPoint(cur.x-1,cur.y))){ 

                        FRIZY_LOG(LOG_DEBUG,"bianjiefangqi");
                        return 1;
                }
                
            }
                
        }

        const int cK = 4;
        
        Grid tmpC;
        RobotType sss;
        for (auto temp : giveUp)
        { 
            if (kind == searchBound)
            {
                for (int i = -1;i <= 1;i++)
                {
                    // if (temp.y + i > archBound.left || temp.y + i < archBound.right)
                    //     continue;
                         
                    if (temp.x == cur.x && temp.y + i == cur.y && temp.count >= cK
                        //&& (temp.x == archBound.up || temp.x == archBound.down) 
                        && _maze.GetMapState(cur.x,cur.y,2) == 1)
                    {
                        FRIZY_LOG(LOG_DEBUG,"boundfangqi.%d.%d",cur.x,cur.y);

                        if (!_maze.trouble && find(cccV.begin(), cccV.end(),cur) == cccV.end()){
                            
                            auto astarTmp = astarPlan.astarLength(curr.x,curr.y,cur.x,cur.y,sss,0,1);
                            //cccV.push_back(cur);
                            FRIZY_LOG(LOG_DEBUG,"relive1.%d.%d %d",cur.x,cur.y,astarTmp.size()); 

                            for (int j = -AREA_LENGTH;j < AREA_LENGTH;++j){
                                
                                tmpC = {cur.x+j,cur.y};
                                if (SameLine(cur,tmpC,1) == 1){
                                    cccV.push_back(tmpC);
                                }
                            }
                            for (int j = -AREA_LENGTH;j < AREA_LENGTH;++j){
                                
                                tmpC = {cur.x,cur.y+j};
                                if (SameLine(cur,tmpC,1) == 1){
                                    cccV.push_back(tmpC);
                                }
                            }
                            if (astarTmp.size()){
                                FRIZY_LOG(LOG_DEBUG,"relive2.%d.%d",cur.x,cur.y);   
                                return 0;
                            }    
                        }

                        return 1;

                    }																
                }
                for (int i = -1;i <= 1;i++)
                {
                    // if (temp.x + i > archBound.up || temp.x + i < archBound.down)
                    //     continue;
                         
                    if (temp.y == cur.y && temp.x + i == cur.x && temp.count >= cK
                        //&& (temp.y == archBound.left || temp.y == archBound.right) 
                        && _maze.GetMapState(cur.x,cur.y,2) == 1)
                    {
                        FRIZY_LOG(LOG_DEBUG,"boundfangqi.%d.%d",cur.x,cur.y);


                        if (!_maze.trouble && find(cccV.begin(), cccV.end(),cur) == cccV.end()){
                            
                            auto astarTmp = astarPlan.astarLength(curr.x,curr.y,cur.x,cur.y,sss,0,1);
                            //cccV.push_back(cur);
                            FRIZY_LOG(LOG_DEBUG,"relive1.%d.%d %d",cur.x,cur.y,astarTmp.size()); 
                            for (int j = -AREA_LENGTH;j < AREA_LENGTH;++j){
                                
                                tmpC = {cur.x+j,cur.y};
                                if (SameLine(cur,tmpC,1) == 1){
                                    cccV.push_back(tmpC);
                                }
                            }
                            for (int j = -AREA_LENGTH;j < AREA_LENGTH;++j){
                                
                                tmpC = {cur.x,cur.y+j};
                                if (SameLine(cur,tmpC,1) == 1){
                                    cccV.push_back(tmpC);
                                }
                            }
                            if (astarTmp.size()){
                                FRIZY_LOG(LOG_DEBUG,"relive2.%d.%d",cur.x,cur.y);   
                                return 0;
                            }    
                        }


                        return 1;
                    }																
                }

            }
            if (kind == searchUnclean || kind == searchInside)
            {
                Grid aim;
                if (temp.count >= cK)
                {
                    aim.x = temp.x,aim.y = temp.y;

                    if (SameLine(cur,aim,0) == 1){
                        FRIZY_LOG(LOG_DEBUG,"fangqile.%d.%d,%d.%d",cur.x,cur.y,aim.x,aim.y);

                        return 1;
                    } 
                }
            }
        }
        return 0;
    }

    inline int UTurnPlanning::passPoint(int x,int y){

        Grid tmp = {x,y};

        if (_maze.GetMapState(x,y,2) == 1){
            if (x < archBound.up && x > archBound.down && y < archBound.left && y > archBound.right)
                return 1;
            else if (find(_maze.linePoint.begin(), _maze.linePoint.end(),tmp) != _maze.linePoint.end())
                return 1;
            else
                return 0;
        }else
            return 0;
            
    }

    int UTurnPlanning::SameLine(Grid cur,Grid aim,int state)
    {
        //printf("infor.%d.%d.%d.%d.%d\n",cur.x,cur.y,aim.x,aim.y,state);
        if (cur.y == aim.y)
        {
            if (aim.x > cur.x)
            {
                for (int i = 0;cur.x + i <= aim.x;i++)
                {
                    if (_maze.GetMapState(cur.x+i,cur.y,2) != state)
                    {

                        //printf("return 01:%d.%d.%d\n",cur.x+i,cur.y,_maze.GetMapState(cur.x+i,cur.y,2));
                        return 0;
                    }
                }
            }
            else
            {
                for (int i = 0;aim.x + i <= cur.x;i++)
                {
                    if (_maze.GetMapState(aim.x+i,aim.y,2) != state)
                    {
                        //printf("return 02\n");
                        return 0;
                    }
                    
                }						
            }
            //printf("return1\n");
            return 1;	

        }
        if (cur.x == aim.x)
        {
            if (aim.y > cur.y)
            {
                for (int i = 0;cur.y + i <= aim.y;i++)
                {
                    if (_maze.GetMapState(cur.x,cur.y+i,2) != state)
                    {
                        return 0;
                    }
                }
            }
            else
            {
                for (int i = 0;aim.y + i <= cur.y;i++)
                {
                    if (_maze.GetMapState(aim.x,aim.y+i,2) != state)
                    {
                        return 0;
                    }
                }						
            }	
            //printf("return1\n");
            return 1;
        }
        return 0;

    }

    bool UTurnPlanning::CleanRecharge(){

        chassisPlan.getPlanningInfo(&charger_planning_info);

        FRIZY_LOG(LOG_DEBUG,"saowanhuichong : %f %f"
        ,charger_planning_info.charger_front_position.x,charger_planning_info.charger_front_position.y);

        if(charger_planning_info.charger_front_position.x > 50 || charger_planning_info.charger_front_position.y > 50)
        {
            if(record_charge_aim.x != 0 || record_charge_aim.y != 0)
            {
                FRIZY_LOG(LOG_DEBUG,"slamdian2 %d,%d",record_charge_aim.x,record_charge_aim.y);
                _aim.x = record_charge_aim.x;
                _aim.y = record_charge_aim.y;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"slamdian3");
                _aim.x = 0;
                _aim.y = 0;
            }
        }
        else
        {
            _aim.x = round_doubel((charger_planning_info.charger_front_position.x * 100)/15);
            _aim.y = round_doubel((charger_planning_info.charger_front_position.y * 100)/15);  
            FRIZY_LOG(LOG_DEBUG, "slamdian1 %d ,%d",_aim.x,_aim.y); 

            for (int i = -1;i <= 1;++i){
                for (int j = -1;j <= 1;++j){
                    if (_maze.GetMapState(_aim.x+i,_aim.y+j,2) == 2 && _maze.GetMapState(_aim.x+i,_aim.y+j,1) != 4)
                        _maze.InputRecord(_aim.x+i,_aim.y+j,1);
                }
            }

            _maze.Seat.addAngle = 1;  

            if (!areaClean)
                _maze.reclean = 1;

            areaClean = 0;
            for (int i = -10;i <= 10;++i){
                for (int j = -10;j <= 10;++j){
                    if (_maze.GetMapState(_aim.x+i,_aim.y+j,2) != 0){
                        _maze.reclean = 0;
                    }

                }
            }
        }          

        _aim.kind = recharge;
        FRIZY_LOG(LOG_DEBUG,"recharge.%d.%d\n",_aim.x,_aim.y);
        roadPlan.SetroadAim(_aim);
        process = ROAD;
        UTURN = ARCH,ARCH_STATE = NO_ARCH;
        searchKind = idle;
        cleanTaskOverIndex = true;
        return true;   
    }

    
    int UTurnPlanning::Dsearch(int x,int y){

        Grid tmp = {x,y};
        if(CloseArr.size() > 10 || _maze.GetMapState(x,y,2) != 0
             || find(CloseArr.begin(), CloseArr.end(),tmp) != CloseArr.end())
            return 0;
        
        CloseArr.push_back(tmp);
        int num = 1;
        num += Dsearch(x+1,y);
        num += Dsearch(x,y+1);
        num += Dsearch(x-1,y);
        num += Dsearch(x,y-1);
        return num;        
    }

    int UTurnPlanning::LastUnclean(Grid cur){

        RobotType sss;
        int n = _maze.wallPoint.size();
        FRIZY_LOG(LOG_DEBUG,"zuihouzuihou %d  %d %d %d %d",n, archBound.up,archBound.down,archBound.left,archBound.right);
        Point2i p;
        int tmpI = 0;
        for (int i = 0;i < n;++i){
            p = {_maze.wallPoint[i].x,_maze.wallPoint[i].y};
            FRIZY_LOG(LOG_DEBUG,"ppp: %d %d",p.x,p.y);
            if (p.x > archBound.up-3 || p.x < archBound.down+3 || p.y > archBound.left-3 || p.y < archBound.right+3)
                continue;

            if (roadPlan.BroadArea(p.x+1,p.y) == 1 || roadPlan.BroadArea(p.x-1,p.y) == 1
                || roadPlan.BroadArea(p.x,p.y+1) == 1 || roadPlan.BroadArea(p.x,p.y-1) == 1){
                FRIZY_LOG(LOG_DEBUG,"tmpI %d",tmpI);     
                ++tmpI;
            }else{
                FRIZY_LOG(LOG_DEBUG,"tmpI = 0"); 
                tmpI = 0;
            }
            if (tmpI > 2){
                FRIZY_LOG(LOG_DEBUG,"zhaodaole %d %d i:%d",p.x,p.y,i);
                for (int j = i-2;j > 0;j -= 2){
                    Point2i p2;
                    if (_maze.GetMapState(_maze.wallPoint[j].x,_maze.wallPoint[j].y,2) != 1)
                        continue;
                    auto astarL = astarPlan.astarLength(cur.x,cur.y,_maze.wallPoint[j].x,_maze.wallPoint[j].y,sss,0,1);
                    if (astarL.size()){
                        FRIZY_LOG(LOG_DEBUG,"dudu %d %d",_maze.wallPoint[j].x,_maze.wallPoint[j].y);
                        _maze.wallAim = {_maze.wallPoint[j].x,_maze.wallPoint[j].y};
                        return 1;
                    }
                }
                break;
            }
        }

        return 0;
    }

    //搜索漏扫区 获取目标点
    void UTurnPlanning::SearchUnlean(Sensor sensor,Grid cur)
    {
        if (IsWall() != 0)
        {
            do{
                FRIZY_LOG(LOG_DEBUG,"road stop1");
                StopWallFollow();
                chassisPlan.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);
        }
        else{
            do{
                FRIZY_LOG(LOG_DEBUG,"road stop2");
                controlw.WheelControl(sensor,cur,cur);
                chassisPlan.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);
        }

        //searchKind = searchUnclean;

        if (archBound.left == 1000 || areaClean)
        {
            archBound.left = 200;
            archBound.right = -200;
        }
        if (archBound.up == 1000 || areaClean)
        {
            archBound.up = 200;
            archBound.down = -200;
        }
        recordY = 1001,suodan = 1001;

        cccV.clear();

        if (searchKind == recharge){
            FRIZY_LOG(LOG_DEBUG,"tiqianhuichong2");
            CleanRecharge();
        }

        if (searchKind == backBound)
        {
            FRIZY_LOG(LOG_DEBUG,"BackBound");

            vector <Grid> array;

            Grid temp;
            //开始搜集需要返回的点
            int states = 1;

            while(1){

                for (int x = archBound.down;x <= archBound.up;x ++)
                {
                    if ((_maze.GetMapState(x,archBound.left,2) == 1 || _maze.GetMapState(x,archBound.left,2) == states)
                        && _maze.GetMapState(x+1,archBound.left,2) != 2
                        && _maze.GetMapState(x-1,archBound.left,2) != 2)
                    {
                        temp.x = x;
                        temp.y = archBound.left;
                        array.push_back(temp);
                    }
                    if ((_maze.GetMapState(x,archBound.right,2) == 1 || _maze.GetMapState(x,archBound.right,2) == states)
                        && _maze.GetMapState(x+1,archBound.right,2) != 2
                        && _maze.GetMapState(x-1,archBound.right,2) != 2)
                    {
                        temp.x = x;
                        temp.y = archBound.right;

                        array.push_back(temp);					
                    }
                }
                
                for (int y = archBound.right;y <= archBound.left;y ++)
                {
                    if ((_maze.GetMapState(archBound.up,y,2) == 1 || _maze.GetMapState(archBound.up,y,2) == states)
                        && _maze.GetMapState(archBound.up,y+1,2) != 2
                        && _maze.GetMapState(archBound.up,y-1,2) != 2)
                    {
                        temp.x = archBound.up;
                        temp.y = y;
                        array.push_back(temp);
                    }
                    if ((_maze.GetMapState(archBound.down,y,2) == 1 || _maze.GetMapState(archBound.down,y,2) == states)
                        && _maze.GetMapState(archBound.down,y+1,2) != 2
                        && _maze.GetMapState(archBound.down,y-1,2) != 2)
                    {
                        temp.x = archBound.down;
                        temp.y = y;
                        array.push_back(temp);					
                    }
                }	
                
                if (!array.size() && !states)
                {
                    FRIZY_LOG(LOG_DEBUG,"fk3");
                    searchKind = searchBound;
                    return;
                }
                
                int dis = 1000;
                for (int i = 0;i < array.size();i++)
                {
                    if (dis > abs(cur.x - array[i].x) + abs(cur.y - array[i].y))
                    {
                        dis = abs(cur.x - array[i].x) + abs(cur.y - array[i].y);
                        _aim.x = array[i].x;
                        _aim.y = array[i].y;
                    }
                }

                if (states == 1 && dis > 5){
                    FRIZY_LOG(LOG_DEBUG,"fk1");
                    array.clear();
                    states = 0;
                }else{
                    FRIZY_LOG(LOG_DEBUG,"fk2");
                    break;
                }
            }

            _aim.kind = backBound;

            FRIZY_LOG(LOG_DEBUG,"backBound.%d.%d.%d",_aim.x,_aim.y,_maze.GetMapState(_aim.x,_aim.y,2));
            
            roadPlan.SetroadAim(_aim);

            process = ROAD;        
            
            return;
        }

        if (searchKind == searchUnclean || searchKind == searchInside)
        {
            FRIZY_LOG(LOG_DEBUG,"start to to");
            
            if (searchKind == searchInside)
               FRIZY_LOG(LOG_DEBUG,"searchInside"); 

            //test
                // int x ,y;
                // for(int i = -50 ;i<50;i++)
                // {
                //     for(int j = -50 ;j<50;j++)
                    
                //     {
                //         int tem_n = _maze.GetMapState(i,j,1);
                //         printf("@300=%04d,%04d,00%d\n",i,j,tem_n);
                    
                //     }
                // }

            vector <Grid> uncleanPoint;

            for (int y = archBound.right;y <= archBound.left;y++)
            {
                for (int x = archBound.down;x <= archBound.up;x++)
                {
                    if ((_maze.GetMapState(x,y,2) == 0 && _maze.VirBound(x,y) == 0)
                        && ((_maze.GetMapState(x,y+1,2) == 1 && (_maze.GetMapState(x+1,y+1,2) == 1 || _maze.GetMapState(x-1,y+1,2) == 1))
                            || (_maze.GetMapState(x,y-1,2) == 1 && (_maze.GetMapState(x+1,y-1,2) == 1 || _maze.GetMapState(x-1,y-1,2) == 1))
                            || ((_maze.GetMapState(x+1,y,2) == 1 || _maze.GetMapState(x-1,y,2) == 1) 
                                && _maze.GetMapState(x,y+1,2) != 2 && _maze.GetMapState(x,y-1,2) != 2)))
                    {
                        Grid temps;
                        temps.x = x,temps.y = y,temps.forward = 0; 

                        if (JudgeGiveup(cur,temps,searchKind) == 1)
                            continue;
                        
                        FRIZY_LOG(LOG_DEBUG,"temp.%d.%d",temps.x,temps.y);
             
                        uncleanPoint.push_back(temps);
                    }
                }
            }

            FRIZY_LOG(LOG_DEBUG,"the length == %d",uncleanPoint.size());

            if (uncleanPoint.size() == 0
                || (uncleanPoint.size() < 3 && searchKind == searchInside))
            {

                FRIZY_LOG(LOG_DEBUG,"length == 0");

                if (areaClean){
                    //areaClean = 0;
                    FRIZY_LOG(LOG_DEBUG,"huaquhuichong");
                    CleanRecharge();
                    return;
                }

                // if (LastUnclean(cur) == 0)
                //     searchKind = searchBound;
                // else
                // {
                //     //searchKind = searchBound;
                //     _aim.kind = lastUnclean;
                //     _aim.x = _maze.wallAim.x,_aim.y = _maze.wallAim.y;
                //     roadPlan.SetroadAim(_aim);
                //     process = ROAD;
                //     UTURN = ARCH,ARCH_STATE = NO_ARCH;
                //     searchKind = idle;
                             
                // }

                searchKind = searchBound;
                return;

            }

            int dis = 1000;
            
            if ((searchKind == searchUnclean || searchKind == searchInside)
                && uncleanPoint.size() > 10 && archBound.left != 200 && archBound.up != 200
                && cur.x <= archBound.up && cur.x >= archBound.down && cur.y <= archBound.left && cur.y >= archBound.right){

                all_path.clear();
                if (searchKind == searchUnclean)
                    astarPlan.Dijkstra(cur.x,cur.y,all_path,uncleanPoint,1);
                else
                    astarPlan.Dijkstra(cur.x,cur.y,all_path,uncleanPoint,0);
                
                if (all_path.empty()){

                    for(int i = 0;i < uncleanPoint.size();i++){
                        if (dis > abs(cur.x - uncleanPoint[i].x) + abs(cur.y - uncleanPoint[i].y)){
                            dis = abs(cur.x - uncleanPoint[i].x) + abs(cur.y - uncleanPoint[i].y);
                            aim.x = uncleanPoint[i].x;
                            aim.y = uncleanPoint[i].y;
                        }
                    }
                }
                else{

                    for (auto& tmp : all_path){
                        if (dis > tmp.size()){
                            dis = tmp.size();
                            aim.x = tmp.back().x;
                            aim.y = tmp.back().y;
                        }
                    }
                    if (searchKind == searchUnclean){
                        Grid other;
                        if (_maze.changeF){
                            other = {aim.x,aim.y+LeftorRight(cur,aim,1).redis};
                        }
                        else{
                            other = {aim.x+UporDown(cur,aim,1).redis,aim.y};
                        }
                        

                        if (caculateLine(cur,other,other.forward)){
                            FRIZY_LOG(LOG_DEBUG,"keyi.%d",other.x);
                            aim.x = other.x;   
                        }
                    }   
                }
                FRIZY_LOG(LOG_DEBUG,"dijie.%d.%d",aim.x,aim.y);
            }
            else{

            for(int i = 0;i < uncleanPoint.size();i++)
            {       
                // auto it = find(PassPoint.begin(),PassPoint.end(),uncleanPoint[i]);

                // if (it != PassPoint.end())
                //     continue;
            
                if (uncleanPoint[i].forward == 1000)
                    continue;
              
                
                RobotType sss;

                Grid tempP;

                if (_maze.changeF){
                    tempP = {uncleanPoint[i].x
                            ,uncleanPoint[i].y+LeftorRight(cur,uncleanPoint[i],1).redis};
                }
                else{
                    tempP = {uncleanPoint[i].x+UporDown(cur,uncleanPoint[i],1).redis
                            ,uncleanPoint[i].y};
                }

                // tempP.x = uncleanPoint[i].x + UporDown(cur,uncleanPoint[i],1).redis;   
                // tempP.y = uncleanPoint[i].y;
                
                vector<pair<int, int>> astarL;
                int tempdis;

                if (cur.x <= archBound.up && cur.x >= archBound.down && cur.y <= archBound.left && cur.y >= archBound.right
                    || uncleanPoint.size() <= 10){

                    FRIZY_LOG(LOG_DEBUG,"time cost!");
                    if (searchKind == searchUnclean)
                        astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,1);
                    else
                        astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,0);  

                    tempdis = astarL.size();                  
                }else{
                    tempdis = 0;
                }

                // if (searchKind == searchUnclean)
                //     astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,1);
                // else
                //     astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,0);   

                if (tempdis == 0)
                    tempdis =  500 + abs(cur.x - tempP.x) + abs(cur.y - tempP.y);

                FRIZY_LOG(LOG_DEBUG,"111.%d,%d,%d",uncleanPoint[i].x,uncleanPoint[i].y,tempdis);
                FRIZY_LOG(LOG_DEBUG,"_111.%d,%d,%d",tempP.x,tempP.y,tempdis);
                
                if (dis >= tempdis)
                {
                    aim.x = tempP.x;
                    aim.y = tempP.y;
                    dis = tempdis;
                }
                
                
                //SameLine(array[i].x,array[i].y,array[j].x,array[j].y,1)
                for(int j = 0;j < uncleanPoint.size();j++)
                {
                    // if ((uncleanPoint[i].x == uncleanPoint[j].x)
                    //     || (uncleanPoint[i].y == uncleanPoint[j].y))  
                    //printf(".%d.%d\n",uncleanPoint[j].x,uncleanPoint[j].y);

                    if (SameLine(uncleanPoint[i],uncleanPoint[j],0) == 1)
                    {
                        uncleanPoint[j].forward = 1000;
                        //PassPoint.push_bacsearchboundk(uncleanPoint[j]);
                    }
                }
            }

            }


            FRIZY_LOG(LOG_DEBUG,"_222.%d.%d.%d\n",aim.x,aim.y,dis);

            _aim.kind = searchKind;
            _aim.x = aim.x,_aim.y = aim.y;

           // RoadPlanning temp;
            roadPlan.SetroadAim(_aim);

            process = ROAD;

			cur_x = cur.x;
			cur_y = cur.y;

            printf("cur_x,cur_y:%d.%d\n",cur_x ,cur_y);
            UTURN = ARCH,ARCH_STATE = NO_ARCH;
            searchKind = idle;
            return;
        }

        if (searchKind == searchBound)
        {

            FRIZY_LOG(LOG_DEBUG,"searchbound");
            _maze.changeF = 0;
            for (auto temp : archBoundArr)
            {
                FRIZY_LOG(LOG_DEBUG,"bound!.%d.%d.%d.%d,%d",temp.up,temp.down,temp.left,temp.right,archBoundArr.size());
            }
            
            
            _maze._map = {-1000,1000,-1000,1000};
            vector <Grid> uncleanPoint;
            Grid temps;
            for (auto temp : archBoundArr)
            {
                if (temp.left == 1000)
                    temp.left = 200,temp.right = -200;
                if (temp.up == 1000)
                    temp.up = 200,temp.down = -200;    
                                    
                for (int x = temp.down+1;x < temp.up;x++)
                {
                    if (!_maze.GetMapState(x,temp.left,2) && !_maze.GetMapState(x,temp.left,2)
                        && _maze.GetMapState(x,temp.left-1,2) == 1){
                        _maze.InputRecord(x,temp.left,1);
                        FRIZY_LOG(LOG_DEBUG,"buchang1: %d",x,temp.left);    
                    }
                    if (!_maze.GetMapState(x,temp.right,2) && !_maze.GetMapState(x,temp.right,2)
                        && _maze.GetMapState(x,temp.right+1,2) == 1){
                        _maze.InputRecord(x,temp.right,1);
                        FRIZY_LOG(LOG_DEBUG,"buchang2: %d",x,temp.right);    
                    }   

                    if (_maze.GetMapState(x,temp.left,2) == 1
                        && _maze.VirBound(x,temp.left) == 0
                        && _maze.GetMapState(x,temp.left+1,2) == 0
                        && _maze.VirBound(x,temp.left+1) == 0
                        //&& _maze.GetMapState(x+1,temp.left,2) == 1 && _maze.GetMapState(x-1,temp.left,2) == 1
                        //&& _maze.GetMapState(x,temp.left-1,2) == 1
                        && (_maze.GetMapState(x+1,temp.left+1,2) != 2 || _maze.GetMapState(x-1,temp.left+1,2) != 2))
                    {
                        temps.x = x;
                        temps.y = temp.left;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(cur,temps,searchKind) == 1)
                            continue;
                        uncleanPoint.push_back(temps);
                    }
                    if (_maze.GetMapState(x,temp.right,2) == 1
                        && _maze.VirBound(x,temp.right) == 0
                        && _maze.GetMapState(x,temp.right-1,2) == 0
                        && _maze.VirBound(x,temp.right-1) == 0
                        //&& _maze.GetMapState(x+1,temp.right,2) == 1 && _maze.GetMapState(x-1,temp.right,2) == 1
                        //&& _maze.GetMapState(x,temp.right+1,2) == 1
                        && (_maze.GetMapState(x+1,temp.right-1,2) != 2 || _maze.GetMapState(x-1,temp.right-1,2) != 2))
                    {
                        temps.x = x;
                        temps.y = temp.right;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(cur,temps,searchKind) == 1)
                            continue;
                        uncleanPoint.push_back(temps);
                    }
                }
                for (int y = temp.right+1;y < temp.left;y++)
                {
                    if (_maze.GetMapState(temp.down,y,2) == 1
                        && _maze.VirBound(temp.down,y) == 0
                        && _maze.GetMapState(temp.down-1,y,2) == 0
                        && _maze.VirBound(temp.down-1,y) == 0
                        //&& _maze.GetMapState(temp.down,y+1,2) == 1 && _maze.GetMapState(temp.down,y-1,2) == 1
                        //&& _maze.GetMapState(temp.down+1,y,2) == 1
                        && (_maze.GetMapState(temp.down-1,y+1,2) != 2 || _maze.GetMapState(temp.down-1,y-1,2) != 2))
                    {
                        temps.x = temp.down;
                        temps.y = y;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(cur,temps,searchKind) == 1)
                            continue;                       
                        uncleanPoint.push_back(temps);
                    }
                    if (_maze.GetMapState(temp.up,y,2) == 1
                        && _maze.VirBound(temp.up,y) == 0
                        && _maze.GetMapState(temp.up+1,y,2) == 0
                        && _maze.VirBound(temp.up+1,y) == 0
                        //&& _maze.GetMapState(temp.up,y+1,2) == 1 && _maze.GetMapState(temp.up,y-1,2) == 1
                        //&& _maze.GetMapState(temp.up-1,y,2) == 1
                        && (_maze.GetMapState(temp.up+1,y+1,2) != 2 || _maze.GetMapState(temp.up+1,y-1,2) != 2))
                    {
                        temps.x = temp.up;
                        temps.y = y;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(cur,temps,searchKind) == 1)
                            continue;                        
                        uncleanPoint.push_back(temps);
                    }                    
                }
            }

            if (uncleanPoint.size() == 0)
            {
                
                for (int x = -95;x <= 95;x++)
                {
                    for (int y = -95;y <= 95;y++)
                    {
                        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) == 0
                            && _maze.GetMapState(x+1,y,2) == 1 && _maze.GetMapState(x-1,y,2) == 1
                            && _maze.GetMapState(x,y+1,2) == 1 && _maze.GetMapState(x,y-1,2) == 1
                            && _maze.GetMapState(x+1,y+1,2) == 1 && _maze.GetMapState(x-1,y-1,2) == 1
                            && _maze.GetMapState(x+1,y-1,2) == 1 && _maze.GetMapState(x-1,y+1,2) == 1){

                            FRIZY_LOG(LOG_DEBUG,"gandiao.%d %d",x,y);
                            _maze.InputRecord(x,y,1);    
                                 
                        }
                        if (_maze.GetMapState(x,y,1) == 2)
                        {                            
                            printf("@300=%04d,%04d,00%d\n",x,y,2);
                        }
                    }	
                }  

                sleep(1);
                
                CleanRecharge();
                    
                return;                 
            }
            else
            {
   
                int dis = 1000;

                if (uncleanPoint.size() > 10 && archBound.left != 200 && archBound.up != 200
                    && cur.x <= archBound.up && cur.x >= archBound.down && cur.y <= archBound.left && cur.y >= archBound.right){
                    
                    all_path.clear();
                    astarPlan.Dijkstra(cur.x,cur.y,all_path,uncleanPoint,2);

                    if (all_path.empty()){

                        for(int i = 0;i < uncleanPoint.size();i++){
                            if (dis > abs(cur.x - uncleanPoint[i].x) + abs(cur.y - uncleanPoint[i].y)){
                                dis = abs(cur.x - uncleanPoint[i].x) + abs(cur.y - uncleanPoint[i].y);
                                aim.x = uncleanPoint[i].x;
                                aim.y = uncleanPoint[i].y;
                            }
                        }
                    }
                    else{

                        for (auto& tmp : all_path){
                            if (dis > tmp.size()){
                                dis = tmp.size();
                                aim.x = tmp.back().x;
                                aim.y = tmp.back().y;
                            }
                        }
                    }
                    
                    _aim.x = aim.x;
                    _aim.y = aim.y;

                    FRIZY_LOG(LOG_DEBUG,"dijie2.%d.%d",aim.x,aim.y);
                }
                else{
                    for (int i = 0;i < uncleanPoint.size();i++){
                        if (dis > abs(cur.x - uncleanPoint[i].x) +  abs(cur.y - uncleanPoint[i].y))
                        {
                            dis = abs(cur.x - uncleanPoint[i].x) +  abs(cur.y - uncleanPoint[i].y);
                            _aim.x = uncleanPoint[i].x;
                            _aim.y = uncleanPoint[i].y;
                        }
                    }
                }

                _aim.kind = searchBound;
                printf("tempX == %d,tempY == %d\n",_aim.x,_aim.y);
                     
            }


            //RoadPlanning roadclass;
            roadPlan.SetroadAim(_aim);
            process = ROAD;
            UTURN = ARCH,ARCH_STATE = NO_ARCH;
            searchKind = idle;
            return;

        }
    }


    void UTurnPlanning::ArchWall(Sensor sensor,Grid cur)
    {
        searchKind = idle;
        switch (arch_wall)
        {


        case LEFTWALL_90:{

            if (cur.x >= aim.x - sensor.size && cur.x < aim.x
                
                && cur.y - 1 > archBound.right
                && (_maze.GetMapState(cur.x+1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x+1,cur.y-2,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.x == aim.x)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = UP_90;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  
        }            



        case RIGHTWALL_270:{

            if (cur.x >= aim.x - sensor.size && cur.x < aim.x
                
                && cur.y + 1 < archBound.left
                && (_maze.GetMapState(cur.x+1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x+1,cur.y+2,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.x == aim.x)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = UP_270;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  
        }

        case RIGHTWALL_90:{

            if (cur.x <= aim.x + sensor.size && cur.x > aim.x
                
                && cur.y - 1 > archBound.right
                && (_maze.GetMapState(cur.x-1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x-1,cur.y-2,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.x == aim.x)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = DOWN_90;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  
        }

        case LEFTWALL_270:{

            if (cur.x <= aim.x + sensor.size && cur.x > aim.x
                
                && cur.y + 1 < archBound.left
                && (_maze.GetMapState(cur.x-1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x-1,cur.y+2,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.x == aim.x)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = DOWN_270;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  
        }

        case RIGHTWALL_0:{

            if (cur.y <= aim.y + sensor.size && cur.y > aim.y
                
                && cur.x + 1 < archBound.up
                && (_maze.GetMapState(cur.x+1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x+2,cur.y-1,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = RIGHT_0;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  
        }
        case RIGHTWALL_180:{
            if (cur.y <= aim.y + sensor.size && cur.y > aim.y
                && cur.x - 1 > archBound.down
                && (_maze.GetMapState(cur.x-1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x-2,cur.y-1,2) == 0))
            {
                printf("walling2\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out2\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);    

                ARCH_STATE = RIGHT_180;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out6\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }   
        }
        case LEFTWALL_0:{
            if (cur.y >= aim.y - sensor.size && cur.y < aim.y
                && cur.x + 1 < archBound.up
                && (_maze.GetMapState(cur.x+1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x+2,cur.y+1,2) == 0))
            {
                printf("walling3\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out3\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = LEFT_0;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out7\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }   
        }
        case LEFTWALL_180:{
            if (cur.y >= aim.y - sensor.size && cur.y < aim.y
                && cur.x - 1 > archBound.down
                && (_maze.GetMapState(cur.x-1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x-2,cur.y+1,2) == 0))
            {
                printf("walling4\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out4\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = LEFT_180;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out8\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;					
            }
        }                          
        
        
        default:
            break;
        }        

    }

    
    Rearch UTurnPlanning::UporDown(Grid cur,Grid aim,int othersign)
    {
		int8_t tempup = 0,tempdown = 0;
        Rearch temp;
        
		for (int i = 1;i < 50;i++)
		{
            if (_maze.GetMapState(aim.x+i,aim.y,2) == 0 && _maze.VirBound(aim.x+i,aim.y) == 0 
                && (_maze.GetMapState(aim.x+i,aim.y+1,2) == 1 || _maze.GetMapState(aim.x+i,aim.y-1,2) == 1))

                    tempup ++;
            else
                break;
        }
        for (int i = 1;i < 50;i++) 
        {   
		   if (_maze.GetMapState(aim.x-i,aim.y,2) == 0 && _maze.VirBound(aim.x-i,aim.y) == 0 
                && (_maze.GetMapState(aim.x-i,aim.y+1,2) == 1 || _maze.GetMapState(aim.x-i,aim.y-1,2) == 1))
                    tempdown ++;
            else
                break;
		}

        if (othersign == 1)
        {
            int upX = aim.x + tempup;
            int downX = aim.x - tempdown;
            
            if (abs(cur.x - upX) > abs(cur.x - downX))
            {   
                FRIZY_LOG(LOG_DEBUG,"xiafang");
                tempdown = -1 * tempdown;
                //temp.redis = tempdown + 1;
                temp.redis = tempdown;
            }
            else if (abs(cur.x - upX) == abs(cur.x - downX))
            {
                FRIZY_LOG(LOG_DEBUG,"zhongfang");
                temp.redis = 0;
            }            
            else
            {
                FRIZY_LOG(LOG_DEBUG,"shangfang");
                //temp.redis = tempup - 1;
                temp.redis = tempup;
            }
            return temp;
        }

		FRIZY_LOG(LOG_DEBUG,"tempup.%d,tempdown.%d",tempup,tempdown);
		
        if (tempup == 0 && tempdown == 0)
        {
            if (_maze.GetMapState(aim.x-1,aim.y,2) == 0 && _maze.GetMapState(aim.x+1,aim.y,2) != 0)
            {
                if (_maze.GetMapState(aim.x,aim.y+1,2) == 0 && _maze.GetMapState(aim.x,aim.y-1,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr11");
                    temp.refor = LEFT_180;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr1");
                    temp.refor = RIGHT_180;
                }
            }
            else
            {
                if (_maze.GetMapState(aim.x,aim.y+1,2) == 0 && _maze.GetMapState(aim.x,aim.y-1,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr22");
                    temp.refor = LEFT_0;
                }
                else
                {                
                    FRIZY_LOG(LOG_DEBUG,"rrr2");
                    temp.refor = RIGHT_0;
                }
            }   
            temp.redis = 0;
            return temp;	        
        }
		else if (tempup < tempdown)
		{
            if (_maze.GetMapState(aim.x,aim.y+1,2) == 1 && _maze.GetMapState(aim.x,aim.y-1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra1");
                temp.refor = RIGHT_180;
            }
            else if (_maze.GetMapState(aim.x,aim.y-1,2) == 1 && _maze.GetMapState(aim.x,aim.y+1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra2");
                temp.refor =  LEFT_180;
            }
            else if (cur.y <= archBound.right+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn2");
                temp.refor = LEFT_180;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn1");
                temp.refor = RIGHT_180;
            } 

            temp.redis = tempup - 1;
            FRIZY_LOG(LOG_DEBUG,"1o.%d",temp.redis);
            

            return temp;	
		}
		else
		{
            if (_maze.GetMapState(aim.x,aim.y+1,2) == 1 && _maze.GetMapState(aim.x,aim.y-1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra3");
                temp.refor = RIGHT_0;
            }
            else if (_maze.GetMapState(aim.x,aim.y-1,2) == 1 && _maze.GetMapState(aim.x,aim.y+1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra4");
                temp.refor = LEFT_0;    
            }
            else if (cur.y <= archBound.right+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn4");
                temp.refor = LEFT_0;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn3");
                temp.refor = RIGHT_0;
            }
            
            
            tempdown = -1 * tempdown;
            temp.redis = tempdown + 1;
            printf("2o.%d\n",temp.redis);

   
            return temp;            
		}
    
    
    }

    Rearch UTurnPlanning::LeftorRight(Grid cur,Grid aim,int othersign){

		int8_t templ = 0,tempr = 0;
        Rearch temp;
        
		for (int i = 1;i < 50;i++)
		{
            if (_maze.GetMapState(aim.x,aim.y+i,2) == 0 && _maze.VirBound(aim.x,aim.y+i) == 0 
                && (_maze.GetMapState(aim.x+1,aim.y+i,2) == 1 || _maze.GetMapState(aim.x-1,aim.y+i,2) == 1))

                    templ ++;
            else
                break;
        }
        for (int i = 1;i < 50;i++) 
        {   
            if (_maze.GetMapState(aim.x,aim.y-i,2) == 0 && _maze.VirBound(aim.x,aim.y-i) == 0 
                && (_maze.GetMapState(aim.x+1,aim.y+i,2) == 1 || _maze.GetMapState(aim.x-1,aim.y+i,2) == 1))
                    tempr ++;
            else
                break;
		}

        if (othersign == 1)
        {
            int leftY = aim.y + templ;
            int rightY = aim.y - tempr;
            
            if (abs(cur.y - leftY) > abs(cur.y - rightY))
            {   
                FRIZY_LOG(LOG_DEBUG,"youfang");
                tempr = -1 * tempr;
                //temp.redis = tempdown + 1;
                temp.redis = tempr;
            }
            else if (abs(cur.y - leftY) == abs(cur.y - rightY))
            {
                FRIZY_LOG(LOG_DEBUG,"zhongfang");
                temp.redis = 0;
            }            
            else
            {
                FRIZY_LOG(LOG_DEBUG,"zuofang");
                //temp.redis = tempup - 1;
                temp.redis = templ;
            }
            return temp;
        }

		FRIZY_LOG(LOG_DEBUG,"templ.%d,tempr.%d",templ,tempr);
		
        if (templ == 0 && tempr == 0)
        {
            if (_maze.GetMapState(aim.x,aim.y-1,2) == 0 && _maze.GetMapState(aim.x,aim.y+1,2) != 0)
            {
                if (_maze.GetMapState(aim.x+1,aim.y,2) == 0 && _maze.GetMapState(aim.x-1,aim.y,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr11");
                    temp.refor = UP_90;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr1");
                    temp.refor = DOWN_90;
                }
            }
            else
            {
                if (_maze.GetMapState(aim.x+1,aim.y,2) == 0 && _maze.GetMapState(aim.x-1,aim.y,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr22");
                    temp.refor = UP_270;
                }
                else
                {                
                    FRIZY_LOG(LOG_DEBUG,"rrr2");
                    temp.refor = DOWN_270;
                }
            }   
            temp.redis = 0;
            return temp;	        
        }
		else if (templ < tempr)
		{
            if (_maze.GetMapState(aim.x+1,aim.y,2) == 1 && _maze.GetMapState(aim.x-1,aim.y,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra1");
                temp.refor = DOWN_90;
            }
            else if (_maze.GetMapState(aim.x-1,aim.y,2) == 1 && _maze.GetMapState(aim.x+1,aim.y,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra2");
                temp.refor =  UP_90;
            }
            else if (cur.x <= archBound.down+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn2");
                temp.refor = UP_90;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn1");
                temp.refor = DOWN_90;
            } 

            temp.redis = templ - 1;
            FRIZY_LOG(LOG_DEBUG,"1o.%d",temp.redis);
            

            return temp;	
		}
		else
		{
            if (_maze.GetMapState(aim.x+1,aim.y,2) == 1 && _maze.GetMapState(aim.x-1,aim.y,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra3");
                temp.refor = DOWN_270;
            }
            else if (_maze.GetMapState(aim.x-1,aim.y,2) == 1 && _maze.GetMapState(aim.x+1,aim.y,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra4");
                temp.refor = UP_90;    
            }
            else if (cur.x <= archBound.down+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn4");
                temp.refor = UP_90;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn3");
                temp.refor = DOWN_270;
            }
            
            
            tempr = -1 * tempr;
            temp.redis = tempr + 1;
            printf("2o.%d\n",temp.redis);

   
            return temp;            
		}
    

    }
    
    bool UTurnPlanning::StopArch(Sensor sensor,Grid cur)
    {
        FRIZY_LOG(LOG_DEBUG,"start arch.%d.%d",_maze.GetMapState(cur.x+1,cur.y,2),_maze.GetMapState(cur.x-1,cur.y,2));
        
        //获取地图的最大最小值
        if (cur.x > _maze._map.xmax) _maze._map.xmax = cur.x;
        if (cur.x < _maze._map.xmin) _maze._map.xmin = cur.x;
        if (cur.y > _maze._map.ymax) _maze._map.ymax = cur.y;
        if (cur.y < _maze._map.ymin) _maze._map.ymin = cur.y;

        FRIZY_LOG(LOG_DEBUG,"MAX.%d.%d.%d.%d"
            ,_maze._map.xmax,_maze._map.xmin,_maze._map.ymax,_maze._map.ymin);
        
        if (archBound.up == 1000)
        {
            if (cur.x - _maze._map.xmin >= AREA_LENGTH && !_maze.changeF
                && (ARCH_STATE == LEFT_0 || ARCH_STATE == RIGHT_0 || UTURN == BUMP_WALL))
            {
                archBound.up = cur.x;
                archBound.down = _maze._map.xmin;
                FRIZY_LOG(LOG_DEBUG,"gong1.%d.%d",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);                
            }
            if (_maze._map.xmax - cur.x >= AREA_LENGTH && !_maze.changeF
                && (ARCH_STATE == LEFT_180 || ARCH_STATE == RIGHT_180 || UTURN == BUMP_WALL))
            {
                archBound.up = _maze._map.xmax;
                archBound.down = cur.x;							
                FRIZY_LOG(LOG_DEBUG,"gong2.%d.%d",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);			
            }

            if (cur.x - _maze._map.xmin >= AREA_LENGTH - 1 && _maze.changeF 
                && (ARCH_STATE == UP_270 || ARCH_STATE == UP_90 || UTURN == BUMP_WALL))
            {
                archBound.up = cur.x + 1;
                archBound.down = _maze._map.xmin;

                FRIZY_LOG(LOG_DEBUG,"gong5.%d.%d\n",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);
            }

            if (_maze._map.xmax - cur.x >= AREA_LENGTH - 1 && _maze.changeF 
                && (ARCH_STATE == DOWN_270 || ARCH_STATE == DOWN_90 || UTURN == BUMP_WALL))
            {
                archBound.up = _maze._map.xmax;
                archBound.down = cur.x - 1;

                FRIZY_LOG(LOG_DEBUG,"gong6.%d.%d\n",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);
            }


        }

        if (archBound.left == 1000)
        {
            if (cur.y - _maze._map.ymin >= AREA_LENGTH - 1 && !_maze.changeF 
                && (ARCH_STATE == LEFT_0 || ARCH_STATE == LEFT_180 || UTURN == BUMP_WALL))
            {
                archBound.left = cur.y + 1;
                archBound.right = _maze._map.ymin;

                FRIZY_LOG(LOG_DEBUG,"gong3.%d.%d\n",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);
            }

            if (_maze._map.ymax - cur.y >= AREA_LENGTH - 1 && !_maze.changeF 
                && (ARCH_STATE == RIGHT_0 || ARCH_STATE == RIGHT_180 || UTURN == BUMP_WALL))
            {
                archBound.left = _maze._map.ymax;
                archBound.right = cur.y - 1;						
                FRIZY_LOG(LOG_DEBUG,"gong4.%d.%d\n",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);				
            }	

            if (cur.y - _maze._map.ymin >= AREA_LENGTH && _maze.changeF
                && (ARCH_STATE == UP_270 || ARCH_STATE == DOWN_270 || UTURN == BUMP_WALL))
            {
                archBound.left = cur.y;
                archBound.right = _maze._map.ymin;
                FRIZY_LOG(LOG_DEBUG,"gong7.%d.%d",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);                
            }
            if (_maze._map.ymax - cur.y >= AREA_LENGTH && _maze.changeF
                && (ARCH_STATE == UP_90 || ARCH_STATE == DOWN_90 || UTURN == BUMP_WALL))
            {
                archBound.left = _maze._map.ymax;
                archBound.right = cur.y;
                FRIZY_LOG(LOG_DEBUG,"gong8.%d.%d",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);                
            }
        }

        if (UTURN == BUMP_WALL)
            return 0;

        int temps = 0;
        for(int x = cur.x - 1;x <= cur.x + 1; x++)
        {
            for(int y = cur.y - 1; y <= cur.y + 1; y++)
            {
                temps = _maze.GetMapState(x,y,2) == 0 ? 0:temps+1;
            }
        } 

        if (temps == 9
            //|| 
            || (_maze.GetMapState(cur.x,cur.y,1) == 4
                && (cur.x > _maze.Seat.x+2 || cur.x < _maze.Seat.x-2 || cur.y > _maze.Seat.y+2 || cur.y < _maze.Seat.y-2))

            || (cur.y >= archBound.left && !_maze.changeF
				&& (sensor.bump || sensor.obs || sensor.cliff || sensor.rightFrontVir || sensor.leftFrontVir
                    || (_maze.GetMapState(cur.x+1,cur.y,2) != 0 && ARCH_STATE == LEFT_0) 
				    || (_maze.GetMapState(cur.x-1,cur.y,2) != 0 && ARCH_STATE == LEFT_180)))
            
		    || (cur.y <= archBound.right && !_maze.changeF 
				&& (sensor.bump || sensor.obs || sensor.cliff || sensor.rightFrontVir || sensor.leftFrontVir
                    || (_maze.GetMapState(cur.x+1,cur.y,2) != 0 && ARCH_STATE == RIGHT_0) 
				    || (_maze.GetMapState(cur.x-1,cur.y,2) != 0 && ARCH_STATE == RIGHT_180)))

            || (cur.x >= archBound.up && !_maze.changeF
                && (cur.y >= archBound.left || cur.y <= archBound.right) 
				&& (ARCH_STATE == LEFT_0 || ARCH_STATE == RIGHT_0))

		    || (cur.x <= archBound.down && !_maze.changeF
                && (cur.y >= archBound.left || cur.y <= archBound.right) 
				&& (ARCH_STATE == LEFT_180 || ARCH_STATE == RIGHT_180))

            || (cur.x >= archBound.up && _maze.changeF
				&& (sensor.bump || sensor.obs || sensor.cliff || sensor.rightFrontVir || sensor.leftFrontVir
                    || (_maze.GetMapState(cur.x,cur.y+1,2) != 0 && ARCH_STATE == UP_270) 
				    || (_maze.GetMapState(cur.x,cur.y-1,2) != 0 && ARCH_STATE == UP_90))) 

            || (cur.x <= archBound.down && _maze.changeF
				&& (sensor.bump || sensor.obs || sensor.cliff || sensor.rightFrontVir || sensor.leftFrontVir
                    || (_maze.GetMapState(cur.x,cur.y+1,2) != 0 && ARCH_STATE == DOWN_270) 
				    || (_maze.GetMapState(cur.x,cur.y-1,2) != 0 && ARCH_STATE == DOWN_90)))    

            || (cur.y >= archBound.left && _maze.changeF
                && (cur.x >= archBound.up || cur.x <= archBound.down) 
				&& (ARCH_STATE == UP_270 || ARCH_STATE == DOWN_270))

            || (cur.y <= archBound.right && _maze.changeF
                && (cur.x >= archBound.up || cur.x <= archBound.down) 
				&& (ARCH_STATE == UP_90 || ARCH_STATE == DOWN_90))

        ){
            searchKind = searchUnclean;
            FRIZY_LOG(LOG_DEBUG,"beibaowei");
            return 1;
        }
            
    }
 
    void UTurnPlanning::BumpWall(Sensor sensor,Grid cur)
    {
        StopArch(sensor,cur);

        if ((cur.addAngle - bumpwall.addAngle > 180 && abs(cur.x-bumpwall.x)+abs(cur.y-bumpwall.y)<= 1) 
            || cur.addAngle - bumpwall.addAngle > 330
            || ((cur.x <= archBound.down || cur.x >= archBound.up
                    || cur.y <= archBound.right || cur.y >= archBound.left)
                && abs(cur.x - bumpwall.x) + abs(cur.y - bumpwall.y) > 0) 
            || abs(cur.x - bumpwall.x) + abs(cur.y - bumpwall.y) > 10
            //|| (boundPoint.size() && find(boundPoint.begin(), boundPoint.end(),cur) != boundPoint.end()
            || bumpwallTime > 100 * 6
            )
            //find(v.begin(), v.end(), key)-v.begin() 获得索引
        {   
            
            FRIZY_LOG(LOG_DEBUG,"chongxinsou.%d,%d,%d",bumpwall.x,bumpwall.y,bumpwall.addAngle);

            bumpwallTime = 0;
            do{
                StopWallFollow();
                chassisPlan.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);    

            if (cur.x <= archBound.up && cur.x >= archBound.down && cur.y <= archBound.left && cur.y >= archBound.right
                && ((_maze.GetMapState(cur.x+1,cur.y,2) == 0 && cur.x+1 <= archBound.up)
                    || (_maze.GetMapState(cur.x-1,cur.y,2) == 0 && cur.x-1 >= archBound.down)
                    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && cur.y+1 <= archBound.left)
                    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && cur.y-1 >= archBound.right))
                    ){
                    
                    FRIZY_LOG(LOG_DEBUG,"rearchle");
                    do{
                        StopWallFollow();
                        chassisPlan.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);  

                    UTURN = ARCH,ARCH_STATE = NO_ARCH;
                    searchKind = idle;
                    return;	                   
            }   
            searchKind = searchUnclean;
            UTURN = SEARCH_UNCLEAN;
            return;			
        }  
        else{

            FRIZY_LOG(LOG_DEBUG,"bumpwalling %d %d",boundPoint.size(),bumpwallTime);
            ++ bumpwallTime;
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            return;            

        }   
    }

    int RebumpWAll(Grid cur){
        if (fabs(_maze.GetXY(cur.x,cur.y)->realx - cur.realx) < 1.0
            || fabs(_maze.GetXY(cur.x,cur.y)->realy - cur.realy) < 1.0
            || fabs(_maze.GetXY(cur.x-1,cur.y)->realx - cur.realx) < 1.0
            || fabs(_maze.GetXY(cur.x+1,cur.y)->realx - cur.realx) < 1.0
            || fabs(_maze.GetXY(cur.x,cur.y-1)->realy - cur.realy) < 1.0
            || fabs(_maze.GetXY(cur.x,cur.y+1)->realy - cur.realy) < 1.0){
            FRIZY_LOG(LOG_DEBUG,"RebumpWAll");       
            return 1;
        }
            
        return 0;
    }

    //弓字型
    void UTurnPlanning::Arch(Sensor sensor,Grid cur)
    {

        if (StopArch(sensor,cur))
        {
            UTURN = SEARCH_UNCLEAN;
            return;
        }

        if (ARCH_STATE == NO_ARCH)
        {
            
            //_maze.changeF = 0;    
            if (_maze.changeF){
                FRIZY_LOG(LOG_DEBUG,"huanle");
                ARCH_STATE = LeftorRight(cur,cur,0).refor;
            }else{
                ARCH_STATE = UporDown(cur,cur,0).refor;
            }
            
            FRIZY_LOG(LOG_DEBUG,"rearch.%d",ARCH_STATE);
        }
        
        switch (ARCH_STATE)
        {

            case UP_270:{

            printf("UP_270.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x,cur.y+1,2) == 1 && _maze.GetMapState(cur.x,cur.y+2,2) == 1)
                || ((fabs(_maze.GetXY(cur.x,cur.y+1)->realy - cur.realy) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realy - cur.realy) < 1.0) && _maze.GetMapState(cur.x,cur.y+1,2))

                || cur.y == archBound.left 
                || (cur.y + 1 == archBound.left && _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x,cur.y+1) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;

			    // if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                //     && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                //             && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0)
				// 	    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                //             && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0))
                //     && _maze.VirBound(cur.x,cur.y+1) == 0)
                // {
                //     FRIZY_LOG(LOG_DEBUG,"arch zhuan5 : %f",cur.realy);
                //     aim.x = cur.x;
                //     aim.y = cur.y + sensor.size;
                //     aim.forward = 270;
                //     ARCH_STATE = LEFT_PRE_180;
                //     controlw.WheelControl(sensor,cur,aim);                    
                // }
                // else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                //         && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0
                //         && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                //         && cur.x < archBound.up - 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                //         && !areaClean
                //         && !RebumpWAll(cur))
                // {
                //     FRIZY_LOG(LOG_DEBUG,"_bumpwall1.%d.%d.%d",cur.x,cur.y,cur.addAngle);
                //     bumpwall = cur;
                //     UTURN = BUMP_WALL;
                //     return;
                // }
                // else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan1 : %f",cur.realy);
                    aim.x = cur.x + sensor.size;
                    aim.y = cur.y;
                    aim.forward = 0;
                    ARCH_STATE = UP_PRE_90;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                printf("right0\n");
                aim.x = cur.x;
                aim.y = cur.y + 1;
                aim.forward = 270;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }
                       
            case UP_PRE_90:{

            // if (recordY < 1000){
            //     // if (fabs(cur.realy - recordY) < 1.0){
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
            //     //     suodan = cur.y;    
            //     // }
            //     // else{
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //     //     suodan = aim.y;
            //     // }
            //     if (fabs(cur.realy - recordY) >= 1.0){
            //         FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //         suodan = aim.y;
            //     }
            // }

            if (cur.x == aim.x || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get1\n"); 

                // if (cur.addAngle < -810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(120,-120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }

                ARCH_STATE = UP_90;
                // aim.x = cur.x - 1;
                // aim.y = cur.y;
                // aim.forward = 180;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            
            else if ((_maze.GetMapState(cur.x+1,cur.y,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+1,cur.y-2,2) != 0)
				|| _maze.VirBound(cur.x+1,cur.y) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou1\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}
            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall1\n");

                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_90;
                return;                   
            }
            else
            {
                printf("right90\n");
                // if (!sensor.leftw && !sensor.rightw && cur.forward > 85 && cur.forward < 95){
                //     FRIZY_LOG(LOG_DEBUG,"recordle1 %f",cur.realy);
                //     recordY = cur.realy;
                // }
                    
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;
            }

            case UP_90:{

            printf("UP_90.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x,cur.y-1,2) == 1 && _maze.GetMapState(cur.x,cur.y-2,2) == 1)

                || ((fabs(_maze.GetXY(cur.x,cur.y-1)->realy - cur.realy) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realy - cur.realy) < 1.0) && _maze.GetMapState(cur.x,cur.y-1,2))

                || cur.y == archBound.right 
                || (cur.y - 1 == archBound.right && _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x,cur.y-1) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    // if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                //      && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y+1,2) == 0 
                //             && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0)
				// 	    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                //             && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0))
                //     && _maze.VirBound(cur.x,cur.y+1) == 0)
                // {
                //     FRIZY_LOG(LOG_DEBUG,"arch zhuan6 : %f",cur.realy);
                //     aim.x = cur.x;
                //     aim.y = cur.y + sensor.size;
                //     aim.forward = 270;
                //     ARCH_STATE = LEFT_PRE_0;
                //     controlw.WheelControl(sensor,cur,aim);
                // }

                // else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                //         && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0
                //         && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                //         && cur.x > archBound.down + 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                //         && !areaClean
                //         && !RebumpWAll(cur))
                // {
                //     FRIZY_LOG(LOG_DEBUG,"_bumpwall2.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                //     bumpwall = cur;
                //     UTURN = BUMP_WALL;
                //     return;
                // }                
                // else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan2 : %f",cur.realy);
                    aim.x = cur.x + sensor.size;
                    aim.y = cur.y;
                    aim.forward = 0;
                    ARCH_STATE = UP_PRE_270;
                    controlw.WheelControl(sensor,cur,aim);                    
                }

            }
            else
            {
                printf("right180\n");
                aim.x = cur.x;
                aim.y = cur.y - 1;
                aim.forward = 90;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case UP_PRE_270:{

            // if (recordY < 1000){
            //     // if (fabs(cur.realy - recordY) < 1.0){
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
            //     //     suodan = cur.y;    
            //     // }
            //     // else{
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //     //     suodan = aim.y;
            //     // }
            //     if (fabs(cur.realy - recordY) >= 1.0){
            //         FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //         suodan = aim.y;
            //     }
            // }


            if (cur.x == aim.x || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get2\n"); 

                // if (cur.addAngle > 810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(-120,120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }   

                ARCH_STATE = UP_270;
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x+1,cur.y,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+1,cur.y+2,2) != 0)
				|| _maze.VirBound(cur.x+1,cur.y) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou2\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall2\n");
                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_270;
                
                return;                   
            }                
            else
            {
            //    if (!sensor.leftw && !sensor.rightw && cur.forward > 85 && cur.forward < 95){
            //         FRIZY_LOG(LOG_DEBUG,"recordle2 %f",cur.realy);
            //         recordY = cur.realy;
            //     }              
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;  
            }

            case DOWN_270:{

            printf("DOWN_270.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x,cur.y+1,2) == 1 && _maze.GetMapState(cur.x,cur.y+2,2) == 1)
                || ((fabs(_maze.GetXY(cur.x,cur.y+1)->realy - cur.realy) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realy - cur.realy) < 1.0) && _maze.GetMapState(cur.x,cur.y+1,2))

                || cur.y == archBound.left 
                || (cur.y + 1 == archBound.left && _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x,cur.y+1) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    // if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                //     && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 0 
                //             && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0)
				// 	    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                //             && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0))
                //     && _maze.VirBound(cur.x,cur.y-1) == 0)
                // {
                //     FRIZY_LOG(LOG_DEBUG,"arch zhuan7 : %f",cur.realy);
                //     aim.x = cur.x;
                //     aim.y = cur.y - sensor.size;
                //     aim.forward = 90;
                //     ARCH_STATE = RIGHT_PRE_180;
                //     controlw.WheelControl(sensor,cur,aim);

                // }
                // else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                //         && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0
                //         && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0
                //         && cur.x < archBound.up - 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                //         && !areaClean
                //         && !RebumpWAll(cur))
                // {
                //     FRIZY_LOG(LOG_DEBUG,"_bumpwall3.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                //     bumpwall = cur;
                //     UTURN = BUMP_WALL;
                //     return;
                // }                   
                // else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan3 : %f",cur.realy);
                    aim.x = cur.x - sensor.size;
                    aim.y = cur.y;
                    aim.forward = 180;
                    ARCH_STATE = DOWN_PRE_90;
                    controlw.WheelControl(sensor,cur,aim);
                }
            }
            else
            {
                aim.x = cur.x;
                aim.y = cur.y + 1;
                aim.forward = 270;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case DOWN_PRE_90:{

            // if (recordY < 1000){
            //     // if (fabs(cur.realy - recordY) < 1.0){
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
            //     //     suodan = cur.y;    
            //     // }
            //     // else{
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //     //     suodan = aim.y;
            //     // }
            //     if (fabs(cur.realy - recordY) >= 1.0){
            //         FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //         suodan = aim.y;
            //     }
            // }

            if (cur.x == aim.x || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get3\n"); 

                // if (cur.addAngle > 810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(-120,120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }
                ARCH_STATE = DOWN_90;

                // aim.x = cur.x - 1;
                // aim.y = cur.y;
                // aim.forward = 180;
                // controlw.WheelControl(sensor,cur,aim);
            }

            else if ((_maze.GetMapState(cur.x-1,cur.y,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-1,cur.y-2,2) != 0)
				|| _maze.VirBound(cur.x-1,cur.y) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou3\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall3\n");
                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_90;
                      
                return;                   
            }                
            else
            {
            //    if (!sensor.leftw && !sensor.rightw && cur.forward > 265 && cur.forward < 275){
            //         FRIZY_LOG(LOG_DEBUG,"recordle3 %f",cur.realy);
            //         recordY = cur.realy;
            //     }                
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;
            }

            case DOWN_90:{

            printf("DOWN_90.%d %d\n",sensor.bump,_maze.forceTurn);


            if ((_maze.GetMapState(cur.x,cur.y-1,2) == 1 && _maze.GetMapState(cur.x,cur.y-2,2) == 1)

                || ((fabs(_maze.GetXY(cur.x,cur.y-1)->realy - cur.realy) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realy - cur.realy) < 1.0) && _maze.GetMapState(cur.x,cur.y-1,2))

                || cur.y == archBound.right 
                || (cur.y - 1 == archBound.right && _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x,cur.y-1) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    // if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                //     && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0 
                //             && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0)
				// 	    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                //             && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0))
                //     && _maze.VirBound(cur.x,cur.y-1) == 0)
                // {
                //     FRIZY_LOG(LOG_DEBUG,"arch zhuan8 : %f",cur.realy);
                //     aim.x = cur.x;
                //     aim.y = cur.y - sensor.size;
                //     aim.forward = 90;
                //     ARCH_STATE = RIGHT_PRE_0;
                //     controlw.WheelControl(sensor,cur,aim);

                // }  
                // else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                //         && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0
                //         && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                //         && cur.x > archBound.down + 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                //         && !areaClean
                //         && !RebumpWAll(cur))
                // {
                //     FRIZY_LOG(LOG_DEBUG,"_bumpwall4.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                //     bumpwall = cur;
                //     UTURN = BUMP_WALL;
                //     return;
                // }                     
                // else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan4 : %f",cur.realy);
                    aim.x = cur.x - sensor.size;
                    aim.y = cur.y;
                    aim.forward = 180;
                    ARCH_STATE = DOWN_PRE_270;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                aim.x = cur.x;
                aim.y = cur.y - 1;
                aim.forward = 90;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case DOWN_PRE_270:{
                
            // if (recordY < 1000){
            //     // if (fabs(cur.realy - recordY) < 1.0){
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
            //     //     suodan = cur.y;    
            //     // }
            //     // else{
            //     //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //     //     suodan = aim.y;
            //     // }
            //     if (fabs(cur.realy - recordY) >= 1.0){
            //         FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
            //         suodan = aim.y;
            //     }
            // }

            if (cur.x == aim.x || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get4\n"); 

                // if (cur.addAngle < -810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(120,-120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }
                ARCH_STATE = DOWN_270;
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x-1,cur.y,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-1,cur.y+2,2) != 0)
				|| _maze.VirBound(cur.x-1,cur.y) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou4\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                    
                return;							
			}             

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall4\n");
                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_270;
                //
                return;                   
            }                  
            else
            {
            //    if (!sensor.leftw && !sensor.rightw && cur.forward > 265 && cur.forward < 275){
            //         FRIZY_LOG(LOG_DEBUG,"recordle4 %f",cur.realy);
            //         recordY = cur.realy;
            //     }                   
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;                 
            }
   



            case RIGHT_0:{

            printf("RIGHT_0.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x+2,cur.y,2) == 1)
                || ((fabs(_maze.GetXY(cur.x+1,cur.y)->realx - cur.realx) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realx - cur.realx) < 1.0) && _maze.GetMapState(cur.x+1,cur.y,2))

                || cur.x == archBound.up 
                || (cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x+1,cur.y) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;

			    if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                            && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                            && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y+1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan5 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
                else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                        && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                        && cur.x < archBound.up - 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                        && !areaClean
                        && !RebumpWAll(cur))
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall1.%d.%d.%d",cur.x,cur.y,cur.addAngle);
                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan1 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                printf("right0\n");
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }
                       
            case RIGHT_PRE_180:{

            if (recordY < 1000){
                // if (fabs(cur.realy - recordY) < 1.0){
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
                //     suodan = cur.y;    
                // }
                // else{
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                //     suodan = aim.y;
                // }
                if (fabs(cur.realy - recordY) >= 1.0){
                    FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                    suodan = aim.y;
                }
            }

            if (cur.y == aim.y || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get1\n"); 

                // if (cur.addAngle < -810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(120,-120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }

                ARCH_STATE = RIGHT_180;
                // aim.x = cur.x - 1;
                // aim.y = cur.y;
                // aim.forward = 180;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            
            else if ((_maze.GetMapState(cur.x,cur.y-1,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y-1) == 1)
			{
                
                recordY = 1001,suodan = 1001;
				printf("zhijiesou1\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}
            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall1\n");

                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_180;
                return;                   
            }
            else
            {
                printf("right90\n");
                if (!sensor.leftw && !sensor.rightw && cur.forward > 85 && cur.forward < 95){
                    FRIZY_LOG(LOG_DEBUG,"recordle1 %f",cur.realy);
                    recordY = cur.realy;
                }
                    
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;
            }

            case RIGHT_180:{

            printf("RIGHT_180.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x-2,cur.y,2) == 1)

                || ((fabs(_maze.GetXY(cur.x-1,cur.y)->realx - cur.realx) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realx - cur.realx) < 1.0) && _maze.GetMapState(cur.x-1,cur.y,2))

                || cur.x == archBound.down 
                || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x-1,cur.y) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                     && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y+1,2) == 0 
                            && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                            && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y+1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan6 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);
                }

                else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                        && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                        && cur.x > archBound.down + 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                        && !areaClean
                        && !RebumpWAll(cur))
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall2.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan2 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);                    
                }

            }
            else
            {
                printf("right180\n");
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case RIGHT_PRE_0:{

            if (recordY < 1000){
                // if (fabs(cur.realy - recordY) < 1.0){
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
                //     suodan = cur.y;    
                // }
                // else{
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                //     suodan = aim.y;
                // }
                if (fabs(cur.realy - recordY) >= 1.0){
                    FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                    suodan = aim.y;
                }
            }


            if (cur.y == aim.y || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get2\n"); 

                // if (cur.addAngle > 810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(-120,120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }   

                ARCH_STATE = RIGHT_0;
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y-1,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y-1) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou2\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall2\n");
                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_0;
                
                return;                   
            }                
            else
            {
               if (!sensor.leftw && !sensor.rightw && cur.forward > 85 && cur.forward < 95){
                    FRIZY_LOG(LOG_DEBUG,"recordle2 %f",cur.realy);
                    recordY = cur.realy;
                }              
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;  
            }

            case LEFT_0:{

                printf("LEFT_0.%d %d\n",sensor.bump,_maze.forceTurn);

            if ((_maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x+2,cur.y,2) == 1)
                || ((fabs(_maze.GetXY(cur.x+1,cur.y)->realx - cur.realx) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realx - cur.realx) < 1.0) && _maze.GetMapState(cur.x+1,cur.y,2))
                || cur.x == archBound.up
                || (cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x+1,cur.y) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 0 
                            && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                            && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y-1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan7 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);

                }
                else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                        && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0
                        && cur.x < archBound.up - 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                        && !areaClean
                        && !RebumpWAll(cur))
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall3.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                   
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan3 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);
                }
            }
            else
            {
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case LEFT_PRE_180:{

            if (recordY < 1000){
                // if (fabs(cur.realy - recordY) < 1.0){
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
                //     suodan = cur.y;    
                // }
                // else{
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                //     suodan = aim.y;
                // }
                if (fabs(cur.realy - recordY) >= 1.0){
                    FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                    suodan = aim.y;
                }
            }

            if (cur.y == aim.y || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get3\n"); 

                // if (cur.addAngle > 810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(-120,120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }
                ARCH_STATE = LEFT_180;

                // aim.x = cur.x - 1;
                // aim.y = cur.y;
                // aim.forward = 180;
                // controlw.WheelControl(sensor,cur,aim);
            }

            else if ((_maze.GetMapState(cur.x,cur.y+1,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y+1) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou3\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall3\n");
                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_180;
                      
                return;                   
            }                
            else
            {
               if (!sensor.leftw && !sensor.rightw && cur.forward > 265 && cur.forward < 275){
                    FRIZY_LOG(LOG_DEBUG,"recordle3 %f",cur.realy);
                    recordY = cur.realy;
                }                
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;
            }

            case LEFT_180:{

            printf("LEFT_180.%d %d\n",sensor.bump,_maze.forceTurn);


            if ((_maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x-2,cur.y,2) == 1)

                || ((fabs(_maze.GetXY(cur.x-1,cur.y)->realx - cur.realx) < 1.0
                    || fabs(_maze.GetXY(cur.x,cur.y)->realx - cur.realx) < 1.0) && _maze.GetMapState(cur.x-1,cur.y,2))

                || cur.x == archBound.down
                || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x-1,cur.y) == 1
                || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
			    if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0 
                            && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                            && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y-1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan8 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);

                }  
                else if ((sensor.bump != 0 || (sensor.obs && !sensor.leftFrontVir && !sensor.rightFrontVir))
                        && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                        && cur.x > archBound.down + 3 && cur.y < archBound.left-3 && cur.y > archBound.right+3
                        && !areaClean
                        && !RebumpWAll(cur))
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall4.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                     
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan4 : %f",cur.realy);
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;
            }

            case LEFT_PRE_0:{
                
            if (recordY < 1000){
                // if (fabs(cur.realy - recordY) < 1.0){
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",cur.y);
                //     suodan = cur.y;    
                // }
                // else{
                //     FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                //     suodan = aim.y;
                // }
                if (fabs(cur.realy - recordY) >= 1.0){
                    FRIZY_LOG(LOG_DEBUG,"suodan %d",aim.y);
                    suodan = aim.y;
                }
            }

            if (cur.y == aim.y || spin720)
            {
                recordY = 1001,suodan = 1001;
                printf("get4\n"); 

                // if (cur.addAngle < -810 && !spin720){
                //     FRIZY_LOG(LOG_DEBUG,"zhunbei720");
                //     spin720 = cur.addAngle;
                // }
                // if (spin720 && abs(spin720 - cur.addAngle) < 720){
                //     FRIZY_LOG(LOG_DEBUG,"720ing");
                //     chassisPlan.chassisSpeed(120,-120,1);
                //     return;
                // }else{
                //     spin720 = 0;
                // }
                ARCH_STATE = LEFT_0;
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                
                // controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y+1,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y+1) == 1)
			{
                recordY = 1001,suodan = 1001;
				printf("zhijiesou4\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                    
                return;							
			}             

            else if (sensor.bump != 0 || sensor.obs != 0 || _maze.forceTurn > 1)
            {
                _maze.forceTurn = 0;
                recordY = 1001,suodan = 1001;
                printf("archwall4\n");
                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_0;
                //
                return;                   
            }                  
            else
            {
               if (!sensor.leftw && !sensor.rightw && cur.forward > 265 && cur.forward < 275){
                    FRIZY_LOG(LOG_DEBUG,"recordle4 %f",cur.realy);
                    recordY = cur.realy;
                }                   
                controlw.WheelControl(sensor,cur,aim);
                return;
            }
            break;                 
            }

            default:
                break;
        }
        
    }

   int NearSeat(Grid cur,Grid aim,int kind){

        if (!_maze.seatPoint.empty() || kind == 2){

           FRIZY_LOG(LOG_DEBUG,"NearSeat0 %f",aim.forward);

            if (aim.forward > 360){
                return 0;
            }

           if ((aim.forward > 310 || aim.forward < 50)
                && (fabs(aim.forward-cur.forward) < 5 || fabs(aim.forward-cur.forward) > 355)){

               if (_maze.GetMapState(cur.x+1,cur.y,1) == 4 || _maze.GetMapState(cur.x+2,cur.y,1) == 4){
                FRIZY_LOG(LOG_DEBUG,"near1");
                return 1;
               }
           } 
            
           if ((aim.forward > 40 && aim.forward < 140)
                && (fabs(aim.forward-cur.forward) < 5 || fabs(aim.forward-cur.forward) > 355)){
               if (_maze.GetMapState(cur.x,cur.y-1,1) == 4 || _maze.GetMapState(cur.x,cur.y-2,1) == 4){
                FRIZY_LOG(LOG_DEBUG,"near2");
                return 1;
               }
           } 

           if ((aim.forward > 130 && aim.forward < 230)
                && (fabs(aim.forward-cur.forward) < 5 || fabs(aim.forward-cur.forward) > 355)){

               if (_maze.GetMapState(cur.x-1,cur.y,1) == 4 || _maze.GetMapState(cur.x-2,cur.y,1) == 4){
                FRIZY_LOG(LOG_DEBUG,"near3");
                return 1;
               }
           } 
            
           if ((aim.forward > 220 && aim.forward < 320)
                && (fabs(aim.forward-cur.forward) < 5 || fabs(aim.forward-cur.forward) > 355)){
               if (_maze.GetMapState(cur.x,cur.y+1,1) == 4 || _maze.GetMapState(cur.x,cur.y+2,1) == 4){

                FRIZY_LOG(LOG_DEBUG,"near4");
                return 1;
               }
           } 

            return 0;

       }else{

           return 0;
       }
   }

   //zhaoqiang
    void UTurnPlanning::SearchWall(Sensor sensor,Grid cur)
    {
        
        printf("zhao!  %d\n",left_charger_index);
        
        // StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
        // process = BOUND;
        // return;

        if (_trouble.type == windcolumn)
        {
            printf("RI\n");

            if (!_maze.reclean)
                current_pose->exploreOver = 1;
            _maze.reclean = 0;

            _maze._map = {-1000,1000,-1000,1000};
            
            if (IsWall() != 0)
            {
                do{
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10*1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);
            }
            
            _trouble.type = nothing;
            archBound.down = cur.x;
            archBound.up = cur.x + AREA_LENGTH;
            archBound.left = 1000;
            archBound.right = -1000;
            blockPlan.SetBound(archBound);
            process = BOUND;
            return;
        }
        

        int nearVir = 0;
        for(auto p : _maze.seatPoint)
            if (sqrtf((p.first - cur.realx*0.15) * (p.first - cur.realx*0.15)
                    + (p.second - cur.realy*0.15) * (p.second - cur.realy*0.15)) < 0.25){
                FRIZY_LOG(LOG_DEBUG,"nearVir");        
                nearVir = 1;
                break;
            }
                
            
        if ((abs(cur.x - aim.x) + abs(cur.y - aim.y) < sensor.size * 3 && (sensor.bump || sensor.obs))
            || abs(cur.x - aim.x) + abs(cur.y - aim.y) == 0
            || keepwallTime > 400
            // || sensor.leftFrontVir || sensor.rightFrontVir
            // || sensor.leftVir || sensor.rightVir
            || (NearSeat(cur,aim,1) && nearVir)
            )
        {
            FRIZY_LOG(LOG_DEBUG,"get wall");
            _maze._map = {-1000,1000,-1000,1000};


            if (!_maze.reclean)
                current_pose->exploreOver = 1;
                
            _maze.reclean = 0;

            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            process = BOUND;
            return;
        }
        
        if (aim.x == 1000 || (keepwallTime % 100 == 0 && keepwallTime > 0))
        {
            aim.forward = 1000;
            int dis = 1000;
            //右
            for (int i = -100; i <= cur.y; i++)
            {
                if (_maze.GetMapState(cur.x,i,1) == 2
                    && (_maze.GetMapState(cur.x+1,i,1) == 2 && _maze.GetMapState(cur.x+2,i,1) == 2
                            && _maze.GetMapState(cur.x-1,i,1) == 2 && _maze.GetMapState(cur.x-2,i,1) == 2))
                {
                    if (abs(cur.y - i) < dis)
                    {
                        aim.x = cur.x;
                        aim.y = i;
                        dis = abs(cur.y - i);
                    }
                    break;
                }
            }



            //左
            for (int i = 100; i >= cur.y; i--)
            {
                if (_maze.GetMapState(cur.x,i,1) == 2
                    && (_maze.GetMapState(cur.x+1,i,1) == 2 && _maze.GetMapState(cur.x+2,i,1) == 2
                            && _maze.GetMapState(cur.x-1,i,1) == 2 && _maze.GetMapState(cur.x-2,i,1) == 2))
                {
                    if (abs(cur.y - i) < dis)
                    {
                        aim.x = cur.x;
                        aim.y = i;
                        dis = abs(cur.y - i);
                    }
                    break;
                }
            }

            //上
            for (int i = 100; i >= cur.x; i--)
            {
                if (_maze.GetMapState(i,cur.y,1) == 2
                    && (_maze.GetMapState(i,cur.y+1,1) == 2 && _maze.GetMapState(i,cur.y+2,1) == 2
                            && _maze.GetMapState(i,cur.y-1,1) == 2 && _maze.GetMapState(i,cur.y-2,1) == 2))
                {
                    if (abs(cur.x - i) < dis)
                    {
                        aim.x = i;
                        aim.y = cur.y;
                        dis = abs(cur.x - i);
                    }
                    break;
                }
            }

            //下
            for (int i = -100; i <= cur.x; i++)
            {
                if (_maze.GetMapState(i,cur.y,1) == 2
                    && (_maze.GetMapState(i,cur.y+1,1) == 2 && _maze.GetMapState(i,cur.y+2,1) == 2
                            && _maze.GetMapState(i,cur.y-1,1) == 2 && _maze.GetMapState(i,cur.y-2,1) == 2))
                {
                    if (abs(cur.x - i) < dis)
                    {
                        aim.x = i;
                        aim.y = cur.y;
                        dis = abs(cur.x - i);
                    }
                    break;
                }
            }
                        
            FRIZY_LOG(LOG_DEBUG,"searchwall a.%d,b.%d,dis.%d",aim.x,aim.y,dis);
        }

        if (aim.x == 1000){

            int dis = 1000;
            for (int x = -95;x <= 95;x++)
            {
                for (int y = -95;y <= 95;y++)
                {
                    if (_maze.GetMapState(x,y,1) == 2 && abs(cur.x-x) + abs(cur.y-y) < dis)
                    {
                        dis = abs(cur.x-x) + abs(cur.y-y);
                        aim.x = x;
                        aim.y = y;
                    }
                }	
            }   

            FRIZY_LOG(LOG_DEBUG,"jiaqiang.%d,%d",aim.x,aim.y);        
        }
		
        if ((sensor.bump || sensor.obs)
            && IsWall() == 0 && aim.x != 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"start wall wall");
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
        }

        else if (IsWall() != 0)
        {
            FRIZY_LOG(LOG_DEBUG,"wall wall");
            keepwallTime ++;
            
        }

        else if (aim.x != 1000)
        {

            if (left_charger_index){
                
                _maze.setForbindenInfo(left_charger_index);
            }
                
            
            if (!_maze.seatPoint.empty()
               && abs(cur.x-_maze.Seat.x) + abs(cur.y-_maze.Seat.y) < 8
               ){
                
                aim.x = _maze.Seat.x;
                aim.y = _maze.Seat.y;
                if (aim.forward == 1000)
                    aim = roadPlan.ConAngle(cur,aim);
                FRIZY_LOG(LOG_DEBUG,"suo seat  %d  %d  %f",_maze.Seat.x,_maze.Seat.y,aim.forward);
                controlw.WheelControl(sensor,cur,aim); 
                return;
            }


            if (aim.forward == 1000)
                aim = roadPlan.ConAngle(cur,aim);
            
            if (IsWall() != 0)
            {
                do{
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);
            }
            
            // static int sign1 = 5;

            // if (sign1 > 0)
            // {
            //     sign1 --;
            //     for (int x = -20;x <= 20;x++)
            //         for (int y = -20;y <= 20;y++)
            //             if (_maze.GetMapState(x,y,1) == 2)
            //                 printf("@300=%04d,%04d,00%d\n",x,y,2); 
            // }     


            FRIZY_LOG(LOG_DEBUG,"go wall.%d.%d.%f",aim.x,aim.y,aim.forward);
            
            controlw.WheelControl(sensor,cur,aim); 
        }
        else
        {
            FRIZY_LOG(LOG_DEBUG,"wait...");

            controlw.ClearPid();
            chassisPlan.chassisSpeed(100,-100,1);
            
        }   
        
        return;
        
    }

    void UTurnPlanning::Explore(Sensor sensor,Grid cur){

        FRIZY_LOG(LOG_DEBUG,"tansuo : %d  %f %f"
        ,current_pose->relocation,current_pose->correctX,current_pose->correctY);
        
        vector<VFH_PARA> valid_dir;    
        exploreTime ++;
        float totalV = -1000;
        float igg = 0;
        float frontDis = 0;

        const int crospinTime = 196; 
        const int crooverTime = 100*18; 
        
        if (!spinTime){
            FRIZY_LOG(LOG_DEBUG,"homepoint: %d %d",int(cur.realx),int(cur.realy));
            _maze.homePoint.realx = cur.realx;
            _maze.homePoint.realy = cur.realy;
        }

        if (_maze.rechargeEX && sensor.Infrared){
            FRIZY_LOG(LOG_DEBUG,"rechargeEX = 2");
            _maze.rechargeEX = 2;
            return;
        }    

        if (current_pose->relocation == 1
            || (current_pose->relocation == 2 && (spinTime >= 2.5*crospinTime))
            || exploreTime >= crooverTime
            // || spinTime >= 2.5*crospinTime || valid_dir.empty()
            // || abs(cur.x-_maze.homePoint.realx) >= AREA_LENGTH 
            // || abs(cur.y-_maze.homePoint.realy) >= AREA_LENGTH
            ){

            exploreTime = 0;
            
            if (_maze.rechargeEX == 1){
                current_pose->exploreOver = 1;
                if (current_pose->relocation == 1){

                    FRIZY_LOG(LOG_DEBUG,"xiaosiye1");
                    _maze.rechargeEX = 0;
                    CleanRecharge();
                    return;

                }else{

                    FRIZY_LOG(LOG_DEBUG,"xiaosiye2");
                    _maze.rechargeEX = 0;
                    CleanRecharge();
                    return;
                }
            }

                
            if (current_pose->relocation == 1){

                FRIZY_LOG(LOG_DEBUG,"LLL: %f %f"
                ,_maze.current_planning_info.charger_front_position.x
                ,_maze.current_planning_info.charger_front_position.y);

                //chassisPlan.getPlanningInfo(&charger_planning_info);

                if (_maze.current_planning_info.charger_front_position.x < 50
                    && _maze.current_planning_info.charger_front_position.y < 50){

                            
                    FRIZY_LOG(LOG_DEBUG,"hard life");
                    left_charger_index = true;
                    _maze.setForbindenInfo(left_charger_index);
                }
            }

            RoadAim _aim;
            _aim.kind = home;
            if (current_pose->relocation == 1){
                _aim.x = round(current_pose->correctX/0.15);
                _aim.y = round(current_pose->correctY/0.15);
            }
            else{
                _aim.x = round(_maze.homePoint.realx);
                _aim.y = round(_maze.homePoint.realy);
            }

            roadPlan.SetroadAim(_aim);
            
            process = ROAD;
            chassisPlan.chassisSpeed(0,0,1);
            FRIZY_LOG(LOG_DEBUG,"huijia %d %d %d %d",_aim.x,_aim.y,spinTime,valid_dir.size());

            return;
        }

        switch (ex_state)
        {

        case EX_FOR:

            chassisPlan.updataVFH(cur,0,valid_dir,frontDis);

            // if (current_pose->relocation == 1
            //     //|| current_pose->relocation == 2
            //     || spinTime >= 2.5*crospinTime || valid_dir.empty()
            //     || abs(cur.x-_maze.homePoint.realx) >= AREA_LENGTH 
            //     || abs(cur.y-_maze.homePoint.realy) >= AREA_LENGTH){
                
            //     RoadAim _aim;

            //     _aim.kind = home;

            //     _aim.x = round(_maze.homePoint.realx);
            //     _aim.y = round(_maze.homePoint.realy);
                
            //     roadPlan.SetroadAim(_aim);
                
            //     process = ROAD;
            //     chassisPlan.chassisSpeed(0,0,1);
            //     FRIZY_LOG(LOG_DEBUG,"huijia %d %d %d %d",_aim.x,_aim.y,spinTime,valid_dir.size());
            //     return;
            // }
                     
            for (auto v : valid_dir){
                
                float det = fabs(cur.forward-v.angle) > 180 ? 360-fabs(cur.forward-v.angle):fabs(cur.forward-v.angle);

                float ks;

                if (det > 160)
                    ks = 3.0;
                else if (det > 90 && det <= 160)
                    ks = 2.0;
                else if (det > 45 && det <= 90)
                    ks = 1.5;
                else
                    ks = 1.0;      

                float tmpV = 1.5*v.value + v.width - ks*det;

                FRIZY_LOG(LOG_DEBUG,"totalV: %f value: %f width: %d det: %f angle: %f"
                ,tmpV,v.value,10*v.width,det,v.angle);

                if (tmpV > totalV){
                    igg = v.angle;
                    totalV = tmpV;
                }
            }

            if (spinTime == crospinTime+1){
                spinTime++;
                aimAngle = cur.forward;
                FRIZY_LOG(LOG_DEBUG,"chushi: %f",aimAngle); 
            }

            FRIZY_LOG(LOG_DEBUG,"aimAngle: %f igg:%f %f",aimAngle,igg,frontDis);    


            if (sensor.bump || sensor.obs || forTime > 80 || NearSeat(cur,cur,1)
                || frontDis < 30){

                //
                if (valid_dir.empty()){
                    
                    aimAngle += 90;
                    aimAngle = aimAngle >= 360 ? aimAngle-360:aimAngle;
                    FRIZY_LOG(LOG_DEBUG,"kongde: %f",aimAngle);
                }else
                    aimAngle = igg;

                if (fabs(aimAngle-cur.forward) < 5 || fabs(aimAngle-cur.forward) > 355){
                    FRIZY_LOG(LOG_DEBUG,"bubian");
                    aimAngle = cur.forward;
                }

                FRIZY_LOG(LOG_DEBUG,"stepp1 forTime:%d %f",forTime,aimAngle);
                forTime = 0;
                ex_state = EX_SPIN;
                return;

            }
            else{

                FRIZY_LOG(LOG_DEBUG,"stepp2 %d %f",forTime,aimAngle);
                ++forTime;
                Grid tmp = {cur.x+1,cur.y+1,aimAngle};
                controlw.WheelControl(sensor,cur,tmp);
                //chassisPlan.chassisSpeed(150,150,1);
                return;
            }
            break;

        case EX_SPIN:

            if (spinTime == crospinTime
                || ((fabs(cur.forward - aimAngle) < 4 || fabs(cur.forward - aimAngle) > 356) && spinTime >= crospinTime)){

                ++spinTime;
                FRIZY_LOG(LOG_DEBUG,"stepp3.%d",spinTime);

                ex_state = EX_FOR;  
                
                return;

            }else{

                FRIZY_LOG(LOG_DEBUG,"stepp4.%d",spinTime);

                ++spinTime;
                if (spinTime > crospinTime
                    && ((cur.forward - aimAngle < 180 && cur.forward - aimAngle > 0)
                         || cur.forward - aimAngle < -180)){

                    FRIZY_LOG(LOG_DEBUG,"zhuoyou1 %f",aimAngle);
                    chassisPlan.chassisSpeed(-70,70,1);
                 }
                 else{
                    FRIZY_LOG(LOG_DEBUG,"zhuoyou2 %f",aimAngle);
                    chassisPlan.chassisSpeed(70,-70,1);
                 }
                
                return;
            }
            break;

        default:
            break;
        }



        //
        //const int crospinTime = 200; 
        
        // switch (ex_state)
        // {
   
        // case EX_FOR:
            
        //     if (sensor.bump || sensor.obs || forTime > 80 || NearSeat(cur,cur)){

        //         FRIZY_LOG(LOG_DEBUG,"stepp1 %d",forTime);
        //         forTime = 0;
        //         ex_state = EX_SPIN;
        //         return;
        //     }
        //     else{
        //         FRIZY_LOG(LOG_DEBUG,"stepp2 %d",forTime);
        //         ++forTime;
        //         chassisPlan.chassisSpeed(150,150,1);
                
        //         return;
        //     }
        //     break;

        // case EX_SPIN:

        //     if (spinTime % (crospinTime/4) == 0 && spinTime >= crospinTime){

        //         ++spinTime;
        //         FRIZY_LOG(LOG_DEBUG,"stepp3.%d",spinTime);

        //         ex_state = EX_FOR;  

        //         if (spinTime >= crospinTime + crospinTime){
                    
        //             RoadAim _aim;
        //             _aim.kind = home;
        //             _aim.x = round(_maze.homePoint.realx);
        //             _aim.y = round(_maze.homePoint.realy);
                    
        //             roadPlan.SetroadAim(_aim);
                    
        //             process = ROAD;
        //             chassisPlan.chassisSpeed(0,0,1);
        //             FRIZY_LOG(LOG_DEBUG,"huijia %d %d %d",_aim.x,_aim.y,spinTime);
        //             return;
        //         }
                
        //         return;
        //     }else{

        //         FRIZY_LOG(LOG_DEBUG,"stepp4.%d",spinTime);
        //         ++spinTime;
        //         chassisPlan.chassisSpeed(70,-70,1);
        //         return;
        //     }
        //     break;

        // default:
        //     break;
        // }
    }

    //区域内部行走逻辑
    void UTurnPlanning::InsidePlanning(Sensor sensor,Grid cur,Trouble trouble)
    {
        
        blockPlan.GetBound(&archBoundArr,&archBound); 
        _trouble = trouble;
        
        if (areaClean)
            archBound = _maze.boundMap;

        FRIZY_LOG(LOG_DEBUG,"kaishigong.%d.%d,bound:%d,%d,%d,%d,searchKind.%d,%d"
        ,UTURN,sensor.bump,archBound.up,archBound.down,archBound.left,archBound.right,searchKind,archBoundArr.size());     


        if (searchKind != idle && searchKind != archwallSign)
            UTURN = SEARCH_UNCLEAN;
        
        if (searchKind == archwallSign)
        {
            FRIZY_LOG(LOG_DEBUG,"gaseng %d",_maze.changeF);
            UTURN = ARCH_WALL;
            searchKind = idle;
            Rearch gaSeng;
 
            if (_maze.changeF){

                gaSeng = LeftorRight(cur,gaSengAim,0);

                if (UP_270 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng5");
                    aim.x = cur.x + 1;
                    arch_wall = RIGHTWALL_270;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }
                if (UP_90 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng6");
                    aim.x = cur.x + 1;
                    arch_wall = LEFTWALL_90;
                    StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                }
                if (DOWN_270 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng7");
                    aim.x = cur.x - 1;
                    arch_wall = LEFTWALL_270;
                    StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                }
                if (DOWN_90 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng8");
                    aim.x = cur.x - 1;
                    arch_wall = RIGHTWALL_90;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }
                                
            }
            else{
                gaSeng = UporDown(cur,gaSengAim,0);

                if (RIGHT_0 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng1");
                    aim.y = cur.y - 1;
                    arch_wall = RIGHTWALL_0;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }
                if (RIGHT_180 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng2");
                    aim.y = cur.y - 1;
                    arch_wall = RIGHTWALL_180;
                    StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                }
                if (LEFT_0 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng3");
                    aim.y = cur.y + 1;
                    arch_wall = LEFTWALL_0;
                    StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                }
                if (LEFT_180 == gaSeng.refor)
                {
                    FRIZY_LOG(LOG_DEBUG,"gaseng4");
                    aim.y = cur.y + 1;
                    arch_wall = LEFTWALL_180;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }   
                
            }


        }

        switch (UTURN)
        {
            case EXPLORE:
                Explore(sensor,cur);
                break;

            case SEARCH_WALL:
                SearchWall(sensor,cur);
                break;     

            case BUMP_WALL:
                BumpWall(sensor,cur);
                break;       
             
            case ARCH_WALL:
                ArchWall(sensor,cur);
                break;

            case SEARCH_UNCLEAN:
                SearchUnlean(sensor,cur);
                break;  

            case ARCH:
                Arch(sensor,cur);
                break;

            default:
                break;
        }

    }

}