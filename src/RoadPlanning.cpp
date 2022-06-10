/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-27 15:55:40
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/RoadPlanning.h"
#include "navigation_algorithm/UTurnPlanning.h"
#include <stack>
using namespace useerobot;
extern Maze _maze;
namespace useerobot
{   extern bool cleanTaskOverIndex;
    extern PRO process;
    extern vector <Grid> rollOb;   
    extern int justwind;
    extern vector <boundary> boundArr;
    extern boundary limit; 
    extern vector <Grid> boundPoint;
    extern int tmpAdd;
    extern int deta;
    extern RoadAim record_charge_aim;
    extern vector <GIVEUP> giveUp;
    int lasttake;
    static RoadAim Null_aim;
    RoadAim last_road_aim;  
    RoadAim road_aim;
    vector <Grid> deleteOb;
    UTurnPlanning planRoad;
    ROAD_STATE roadState;
    extern _UTURN UTURN;
    RoadPlanning::RoadPlanning(/* args */)
    {
    }
   
    RoadPlanning::~RoadPlanning()
    {

    }
    
    void RoadPlanning::init()
    {
        deleteOb.clear();
        
        BUMP_MAX = 8;
        roadState = roadIdle;
        nearpoint = 0;
        astarAgain = 1;
        ewall.x = 1000;
        ewallTime = 0;
        bumpCount = 0;
        dewallSign = 0;    
        call_recharge_index = false;
        fast_near_charge = false;  
        failCount = 0;
        forceWall = 0;
        start_add = 0;
        goWall = 0;

    }    
    void RoadPlanning::SetroadAim(RoadAim aim)
    {
        road_aim.x = aim.x;
        road_aim.y = aim.y;
        road_aim.kind = aim.kind;
        recharge_first_index = 0;
        
    }   

    Grid RoadPlanning::ConAngle(Grid cur,Grid aim)
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
            float_t tempcan = float_t(abs(aim.y - cur.y))/abs(aim.x - cur.x);
            
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
        FRIZY_LOG(LOG_DEBUG,"confor : %f",aim.forward);
        return aim;
    }
   
    int8_t RoadPlanning::BroadArea(int16_t x,int16_t y){
        
		int8_t temps = 0;
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x+i,y+j,2) != 0)
                {
                    temps++;
                    break;
                }
            }
		}
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x+i,y-j,2) != 0)
                {
                    temps++;
                    break;
                }
            }
		}		
		
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x-i,y-j,2) != 0)
                {
                    temps++;
                    break;
                }
            }
		}		
		
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x-i,y+j,2) != 0)
                {
                    temps++;
                    break;
                }
            }
		}		
	
		if (temps < 4 && x < limit.up &&  x > limit.down && y < limit.left && y > limit.right)
			return 1;	
		else
			return 0;
    }   

    int ggggg(int x,int y){

        if (!giveUp.size()){
            FRIZY_LOG(LOG_DEBUG,"gcount: %d aim:%d %d",0,road_aim.x,road_aim.y);
            return 0; 
        }
            
        for (auto g:giveUp){
            Grid g1 = {x,y};
            Grid g2 = {g.x,g.y};
            if (planRoad.SameLine(g1,g2,0) == 1){
                FRIZY_LOG(LOG_DEBUG,"gcount: %d aim:%d %d",g.count,road_aim.x,road_aim.y);
                return g.count;
            }
                
            // if (g.x == x && g.y == y){
                
            // }

        }
        FRIZY_LOG(LOG_DEBUG,"gcount: %d aim:%d %d",0,road_aim.x,road_aim.y);
        return 0;
    }
    
    void RoadPlanning::reCall(Grid curr,Grid center,int kind){


        stack <Grid> GridStack;

        Grid cur = curr;

        int maxT = 0;

        GridStack.push(cur);
        otherSide.assign(_maze.wallPoint.begin(), _maze.wallPoint.end());

        while (!GridStack.empty()){

            cur = GridStack.top();

            GridStack.pop();
            ++maxT;
            if (CloseA.size() > 390 || workOb.size() > 390){
                FRIZY_LOG(LOG_DEBUG,"zuida: %d",maxT);
                break;
            }


            for (int i = -1;i <= 1;++i){
              for (int j = -1;j <= 1;++j){

                Grid tt = {cur.x+i,cur.y+j};

                //int tmpI = find(otherSide.begin(),otherSide.end(),tt) - otherSide.begin();

                
                for (auto it = otherSide.begin(); it != otherSide.end();){

                    if((*it) == tt)
                        it = otherSide.erase(it); //不能写成arr.erase(it);
                    
                    else
                        ++it;
                }  

              }
            }




            //Grid cur_work;
            if (_maze.GetMapState(cur.x+1,cur.y,2) == 2 && _maze.GetMapState(cur.x+1,cur.y,1) != 2
                
                // && _maze.GetMapState(cur.x+1+2,cur.y,1) != -1 && _maze.GetMapState(cur.x+1-2,cur.y,1) != -1
                // && _maze.GetMapState(cur.x+1,cur.y+2,1) != -1 && _maze.GetMapState(cur.x+1,cur.y-2,1) != -1

                && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right

                ){
                cur_work = {cur.x+1,cur.y};
                if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                    FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                    workOb.push_back(cur_work);
                }      
            }
                
            if (_maze.GetMapState(cur.x-1,cur.y,2) == 2 && _maze.GetMapState(cur.x-1,cur.y,1) != 2

                // && _maze.GetMapState(cur.x-1+2,cur.y,1) != -1 && _maze.GetMapState(cur.x-1-2,cur.y,1) != -1
                // && _maze.GetMapState(cur.x-1,cur.y+2,1) != -1 && _maze.GetMapState(cur.x-1,cur.y-2,1) != -1

                && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right

                ){

                cur_work = {cur.x-1,cur.y};
                if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                    FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                    workOb.push_back(cur_work);
                }     
            }

            if (_maze.GetMapState(cur.x,cur.y+1,2) == 2 && _maze.GetMapState(cur.x,cur.y+1,1) != 2

                // && _maze.GetMapState(cur.x+2,cur.y+1,1) != -1 && _maze.GetMapState(cur.x-2,cur.y+1,1) != -1
                // && _maze.GetMapState(cur.x,cur.y+1+2,1) != -1 && _maze.GetMapState(cur.x,cur.y+1-2,1) != -1

                && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right

                ){
                cur_work = {cur.x,cur.y+1};
                if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                    FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                    workOb.push_back(cur_work);
                }    
            }

            if (_maze.GetMapState(cur.x,cur.y-1,2) == 2 && _maze.GetMapState(cur.x,cur.y-1,1) != 2

                // && _maze.GetMapState(cur.x+2,cur.y-1,1) != -1 && _maze.GetMapState(cur.x-2,cur.y-1,1) != -1
                // && _maze.GetMapState(cur.x,cur.y-1+2,1) != -1 && _maze.GetMapState(cur.x,cur.y-1-2,1) != -1

                && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right

                ){
                cur_work = {cur.x,cur.y-1};
                if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                    FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                    workOb.push_back(cur_work);
                }    
            }

            if (((cur.x >= limit.up || cur.x <= limit.down || cur.y >= limit.left || cur.y <= limit.right)
                    && kind == 0)
                || cur.x >= center.x+40 || cur.x <= center.x-40 || cur.y >= center.y+40 || cur.y <= center.y-40
                || _maze.GetMapState(cur.x,cur.y,1) == -1 || _maze.GetMapState(cur.x,cur.y,1) == 2
                || _maze.GetMapState(cur.x,cur.y,2) == 2 || _maze.GetMapState(cur.x,cur.y,2) == 0
                || find(CloseA.begin(), CloseA.end(),cur) != CloseA.end()
                    ){
                
                        
                FRIZY_LOG(LOG_DEBUG,"tuichu %d %d %d %d  maxT %d",cur.x,cur.y,CloseA.size(),workOb.size(),maxT);


                continue;
            }

            
            CloseA.push_back(cur);
            GridStack.push({cur.x+1,cur.y});
            GridStack.push({cur.x-1,cur.y});
            GridStack.push({cur.x,cur.y+1});
            GridStack.push({cur.x,cur.y-1});
            
        }
        
        return;

        //Grid cur_work;
        if (_maze.GetMapState(cur.x+1,cur.y,2) == 2 && _maze.GetMapState(cur.x+1,cur.y,1) != 2
            && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right
            ){
            cur_work = {cur.x+1,cur.y};
            if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                workOb.push_back(cur_work);
            }      
        }
            
        if (_maze.GetMapState(cur.x-1,cur.y,2) == 2 && _maze.GetMapState(cur.x-1,cur.y,1) != 2
            && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right
            ){

            cur_work = {cur.x-1,cur.y};
            if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                workOb.push_back(cur_work);
            }     
        }

        if (_maze.GetMapState(cur.x,cur.y+1,2) == 2 && _maze.GetMapState(cur.x,cur.y+1,1) != 2
            && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right
            ){
            cur_work = {cur.x,cur.y+1};
            if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                workOb.push_back(cur_work);
            }    
        }

        if (_maze.GetMapState(cur.x,cur.y-1,2) == 2 && _maze.GetMapState(cur.x,cur.y-1,1) != 2
            && cur.x < limit.up && cur.x > limit.down && cur.y < limit.left && cur.y > limit.right
            ){
            cur_work = {cur.x,cur.y-1};
            if (find(workOb.begin(), workOb.end(),cur_work) == workOb.end()){
                FRIZY_LOG(LOG_DEBUG,"work : %d %d",cur_work.x,cur_work.y);
                workOb.push_back(cur_work);
            }    
        }

        // FRIZY_LOG(LOG_DEBUG,"qianzhi %d %d %d %d center: %d %d"
        // ,limit.up,limit.down,limit.left,limit.right,center.x,center.y);

        //curr = {cur.x,cur.y};

        if (((cur.x >= limit.up || cur.x <= limit.down || cur.y >= limit.left || cur.y <= limit.right)
                && kind == 0)
            || cur.x >= center.x+40 || cur.x <= center.x-40 || cur.y >= center.y+40 || cur.y <= center.y-40
            || _maze.GetMapState(cur.x,cur.y,1) == -1 || _maze.GetMapState(cur.x,cur.y,1) == 2
            || _maze.GetMapState(cur.x,cur.y,2) == 2 || _maze.GetMapState(cur.x,cur.y,2) == 0
            || find(CloseA.begin(), CloseA.end(),cur) != CloseA.end()
            || CloseA.size() > 200 || workOb.size() > 200){
            
                    
            // FRIZY_LOG(LOG_DEBUG,"tuichu %d %d %d %d  map %d %d",cur.x,cur.y,CloseA.size(),workOb.size(),
            //     _maze.GetMapState(cur.x,cur.y,1),_maze.GetMapState(cur.x,cur.y,2));

            return;
        }


        //FRIZY_LOG(LOG_DEBUG,"cur : %d %d %d %d",cur.x,cur.y,CloseA.size(),workOb.size());

        CloseA.push_back(cur);

        //cur = {cur.x+1,cur.y};
        reCall({cur.x+1,cur.y},center,kind); 

        //cur = {cur.x-1,cur.y};
        reCall({cur.x-1,cur.y},center,kind); 

        //cur = {cur.x,cur.y+1};
        reCall({cur.x,cur.y+1},center,kind); 

        //cur = {cur.x,cur.y-1};
        reCall({cur.x,cur.y-1},center,kind); 

    }

    void RoadPlanning::NullRunning(Sensor sensor,Grid cur)
    {

        if (road_aim.kind == home){

            FRIZY_LOG(LOG_DEBUG,"home null %d",bumpCount);
            _maze.InitRecordMap();
            roadDwa.init();

            roadState = roadIdle;
            UTURN = SEARCH_WALL;  
            process = PLAN;
            return; 
        }
        if (road_aim.kind == breakClean){
            FRIZY_LOG(LOG_DEBUG,"baojing4");
            _maze.appWarn = 1;
            return;
        }

        if (deleteOb.size() > 1 && deleteOb[0].x != 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"shanchu");

            for (int i = 0; i < deleteOb.size();i++)
            {
                FRIZY_LOG(LOG_DEBUG,"shan.%d.%d",deleteOb[i].x,deleteOb[i].y);
                _maze.InputRecord(deleteOb[i].x,deleteOb[i].y,1);                
            }

            deleteOb[0].x = 1000;
            roadState = roadIdle;
            return;
        }   

        if (road_aim.kind == searchBound || road_aim.kind == searchUnclean){
            
            
            if (!ggggg(road_aim.x,road_aim.y) && dewallSign && _maze.trouble < 3
                 && (_aim.x != road_aim.x || _aim.y != road_aim.y)){

                _maze.trouble ++;  

                FRIZY_LOG(LOG_DEBUG,"zhunbeishanchu %d %d %d %d trouble: %d"
                    ,road_aim.x,road_aim.y,_aim.x,_aim.y, _maze.trouble);
                
                
                Point2i p;

            
                CloseA.resize(401);
                workOb.resize(401);

                CloseA.clear();
                workOb.clear();


                reCall(cur,cur,0);    

                FRIZY_LOG(LOG_DEBUG,"workOb: %d %d %d %d"
                ,workOb.size(),CloseA.size(),_maze.wallPoint.size(),otherSide.size());

                int count = 0;
                for (int i = 0;i < _maze.wallPoint.size();++i){

                    p = {_maze.wallPoint[i].x,_maze.wallPoint[i].y};

                    if (_maze.GetMapState(p.x,p.y,2) == 2){

                        FRIZY_LOG(LOG_DEBUG,"gg : %d %d ",p.x,p.y);
                        for (int x = -2;x <= 2;++x){
                            for (int y = -2;y <= 2;++y){

                                if (_maze.GetMapState(p.x+x,p.y+y,2) != 2

                                    || _maze.GetMapState(p.x+x,p.y+y,1) == -1

                                    || _maze.GetMapState(p.x+x+1,p.y+y,1) == -1 || _maze.GetMapState(p.x+x-1,p.y+y,1) == -1
                                    || _maze.GetMapState(p.x+x,p.y+y+1,1) == -1 || _maze.GetMapState(p.x+x,p.y+y-1,1) == -1   

                                    || _maze.GetMapState(p.x+x,p.y+y,1) == 2
                                    || _maze.GetMapState(p.x+x+1,p.y+y,1) == 2 || _maze.GetMapState(p.x+x-1,p.y+y,1) == 2
                                    || _maze.GetMapState(p.x+x,p.y+y+1,1) == 2 || _maze.GetMapState(p.x+x,p.y+y-1,1) == 2
                                    
                                    || _maze.VirBound(p.x+x,p.y+y) ||  _maze.VirBound(p.x+x+2,p.y+y) || _maze.VirBound(p.x+x-2,p.y+y)
                                    || _maze.VirBound(p.x+x,p.y+y+2) || _maze.VirBound(p.x+x,p.y+y-2)
                                    || p.y+y >= limit.left || p.y+y <= limit.right)
                                    continue;

                                Grid tmp = {p.x+x,p.y+y};
                                Grid othertmp;    

                                int otherS = 0;
                                for(int m =-2;m <= 2;++m){
                                    for(int n =-2;n <= 2;++n){
                                        othertmp = {tmp.x+m,tmp.y+n};
                                        if (find(otherSide.begin(),otherSide.end(),othertmp) != otherSide.end()){
                                            FRIZY_LOG(LOG_DEBUG,"otherside: %d %d",othertmp.x,othertmp.y);
                                            ++otherS;
                                        }
                                    }
                                }

                                if (!otherS)
                                    continue;

                                if (find(_maze.virPoint.begin(),_maze.virPoint.end(),tmp) != _maze.virPoint.end()){
                                    FRIZY_LOG(LOG_DEBUG,"virPoint: %d %d",tmp.x,tmp.y);
                                    continue;
                                }

                                if (find(_maze.specialWall.begin(),_maze.specialWall.end(),tmp)
                                     == _maze.specialWall.end())
                                    _maze.specialWall.push_back(tmp);

                                if (workOb.size() > 1 && find(workOb.begin(),workOb.end(),tmp) != workOb.end()){
                                  
                                    count++;
                                    _maze.InputRecord(tmp.x,tmp.y,1); 
                                }
                            }
                        } 
                    }
                }

                FRIZY_LOG(LOG_DEBUG,"count : %d ",count);
                
                for (int i = 0; i < _maze.specialWall.size();++i){
                    FRIZY_LOG(LOG_DEBUG,"specialWall : %d %d",_maze.specialWall[i].x,_maze.specialWall[i].y);
                }

                if (count > 1){
                    FRIZY_LOG(LOG_DEBUG,"jinitaimei : %d %d %d",count,road_aim.x,road_aim.y);
                    roadState = roadIdle;
                    _aim.x = road_aim.x,_aim.y = road_aim.y;
                    planRoad.PlanSearch(road_aim,0);
                    return;                    
                }        

            }
        }

        _aim.x = road_aim.x,_aim.y = road_aim.y;
        
        if (_aim.forward == 1000)
        {
            if (Null_aim.x == _aim.x && Null_aim.y == _aim.y && Null_aim.kind == none)
            {
                FRIZY_LOG(LOG_DEBUG,"dewallSign = 1");
                dewallSign = 1;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"dewallSign = 0");
                Null_aim.x = road_aim.x,Null_aim.y = road_aim.y,Null_aim.kind = none;
                //dewallSign = 0;
            }
        }

        if (goWall){
            FRIZY_LOG(LOG_DEBUG,"goWallle");
            dewallSign = 0;
        }
        goWall = 0;

        if (IsWall() == 0 && dewallSign == 0)
        {
            if (_aim.forward == 1000 || cur.x == _aim.x || cur.y == _aim.y)
                _aim = ConAngle(cur,_aim);
            
            motionRoad.WheelControl(sensor,cur,_aim);

            if (sensor.bump != 0)
            {
                printf("impossible!\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
        }
        else
        {
            FRIZY_LOG(LOG_DEBUG,"ewalling.%d.%d.%f.%d %d"
            ,ewall.x,ewall.y,ewall.forward,ewallTime,_maze.overTime);

            if (abs(cur.x - ewall.x) + abs(cur.y - ewall.y) > 2 
                && BroadArea(cur.x,cur.y) == 1
                && road_aim.kind == searchUnclean)
			{
                printf("zhijietuichu\n");

                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0); 

                _trouble.type = nothing;
                ewallTime = 0;
                
                road_aim.kind = idle;
                planRoad.PlanSearch(road_aim,-1);
                process = PLAN;
                return;  						
			}
           
            timeV = 100 * 3;

            if (road_aim.kind == recharge){

                if (!start_add){
                    FRIZY_LOG(LOG_DEBUG,"startadd1: %d",cur.addAngle);
                    start_add = cur.addAngle;
                }
                if (cur.addAngle - start_add < -1440 || failCount >= 45){
                    FRIZY_LOG(LOG_DEBUG,"can not find recharge");
                
                    _maze.appWarn = 2;
                    return;
                }
            }

            if (road_aim.kind == searchUnclean || road_aim.kind == searchBound){

                if (!start_add){
                    FRIZY_LOG(LOG_DEBUG,"startadd2: %d",cur.addAngle);
                    start_add = cur.addAngle;
                }
                if ((cur.addAngle - start_add < -1440 && _maze.overTime > 100 * 12 * 3)
                    || _maze.overTime > 100 * 12 * 15){
                        
                    FRIZY_LOG(LOG_DEBUG,"tiqianhuichong1 : %d %d",cur.addAngle - start_add,_maze.overTime);
                    //_maze.overTime = -100;
                    road_aim.kind = recharge;
                    planRoad.PlanSearch(road_aim,-1);
                    process = PLAN;
                    return;      
                }
            }
            
            if ((abs(cur.x - ewall.x) + abs(cur.y - ewall.y) > 15 * sensor.size && ewall.x != 1000)
                || ewallTime > 2*timeV 
                || (ewallTime > timeV && abs(cur.x - ewall.x) + abs(cur.y - ewall.y) > 3)
                || cur.addAngle - ewall.addAngle < -360
                || _trouble.type == windcolumn
                || road_aim.kind == searchLast)            
            {

                if (_trouble.type == windcolumn){
                    FRIZY_LOG(LOG_DEBUG,"goWall = 1");
                    goWall = 1;
                    if (road_aim.kind == searchUnclean || road_aim.kind == searchBound)
                        start_add = 0;
                   
                }
                
                if (abs(cur.x - road_aim.x) + abs(cur.y - road_aim.y) > 60
                    && (abs(cur.x) > 180 || abs(cur.y) > 180)){

                    if (road_aim.kind == searchUnclean){

                        FRIZY_LOG(LOG_DEBUG,"yimixi1");
                        road_aim.kind = searchBound;
                        planRoad.PlanSearch(road_aim,-1);
                        process = PLAN;
                        return;                              
                    }
                    if (road_aim.kind == searchBound){

                        FRIZY_LOG(LOG_DEBUG,"yimixi2");
                        road_aim.kind = recharge;
                        planRoad.PlanSearch(road_aim,-1);
                        process = PLAN;
                        return;                           
                    }
                }

                _trouble.type = nothing;
                FRIZY_LOG(LOG_DEBUG,"out le.%d.%d",cur.x,cur.y);
                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);  

                ewallTime = 0;
                
                if (road_aim.kind == recharge)
                {  
                    failCount++;
                    astarAgain = 1;
                    FRIZY_LOG(LOG_DEBUG,"simida1 : %d",failCount);
                    if (failCount > 10
                        || (failCount > 1 && abs(road_aim.x-cur.x) + abs(road_aim.y-cur.y) < 10))
                        fast_near_charge = true;

                    roadState = roadIdle;
                    roadDwa.init();
                    return;  
                }
                
                else if (road_aim.kind == column)
                {
                    deta = cur.addAngle - tmpAdd;

                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta; 

                    FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                    justwind = 1;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                    process = BOUND;
                    return;                    
                }
                else if (road_aim.kind == searchLast){
                    FRIZY_LOG(LOG_DEBUG,"simida3");
                    planRoad.PlanSearch(road_aim,1);
                    process = PLAN;
                    return;                         
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"the kind.%d",road_aim.kind);   

                    planRoad.PlanSearch(road_aim,0);
                    process = PLAN;
                    return;                       
                }  

            }



            //two mode
            int specialSign = 0;
            for (auto sp : _maze.specialWall){
                if (abs(cur.x - sp.x) + abs(cur.y - sp.y) <= 1){
                    specialSign = 1;
                    break;
                }
            }

            _maze.overTime ++;

            if (//(ggggg(road_aim.x,road_aim.y) == 1 || !ggggg(road_aim.x,road_aim.y))
                !_maze.specialWall.empty()
                && specialSign
                ){
                FRIZY_LOG(LOG_DEBUG,"teshuyanqiang %d %d",road_aim.x,road_aim.y);
                ewallTime ++; 
                StartWallFollow(RandomWallFollow, RIGHTAW, FAST_QUICK);  
            }
            else{
                ewallTime ++;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);  
            }
        }
        return;
   
    }

    void RoadPlanning::DwaRunning(Sensor sensor,Grid cur)
    {
        vector <Sensor> sss;
        int count1,count2;

        while (count2)
        {
            count2 --;
            chassisRoad.chassisSpeed(sss[count1].leftw,sss[count1].rightw,1);
            if (count2 == 0)
                count1 ++,count2 = 10;
            return;
        }
    }

    int RoadPlanning::ArriveRoad(Sensor sensor,Grid cur)
    {
        if (last_road_aim.x != road_aim.x || last_road_aim.y != road_aim.y)
        {
            FRIZY_LOG(LOG_DEBUG,"aim change");
            if (road_aim.kind == recharge && last_road_aim.kind != road_aim.kind){
                FRIZY_LOG(LOG_DEBUG,"churu");
                againCount = 0;
                start_add = 0;

            }
            if (road_aim.kind == recharge)    
                deleteOb.clear();

            roadDwa.init();
            _maze.forceTurn = 0;

            bumpCount = 0;
            astarAgain = 1;
            
            roadState = roadIdle;
            Null_aim.kind = idle;
            
        }

        last_road_aim = road_aim;
   
        FRIZY_LOG(LOG_DEBUG,"road_aim.%d.%d.%d.%d.%d",road_aim.x,road_aim.y,road_aim.kind,dewallSign,roadState);

        if (road_aim.kind == searchLast && IsWall() == 2){
            FRIZY_LOG(LOG_DEBUG,"raoing");
        }

        if (forceWall){
            FRIZY_LOG(LOG_DEBUG,"forceWall");
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            ewallTime++;

            if (abs(cur.x - _maze.wallAim.x) + abs(cur.y - _maze.wallAim.y) > 2 
                && BroadArea(cur.x,cur.y) == 1){

                printf("zhijietuichu2\n");
                forceWall = 0;
                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0); 
                _trouble.type = nothing;
                ewallTime = 0;
                
                road_aim.kind = idle;
                planRoad.PlanSearch(road_aim,-1);
                process = PLAN;
                return 1;     
            
            }
            if (abs(cur.x-_maze.wallAim.x)+abs(cur.y-_maze.wallAim.y) > 10
                || ewallTime > 100 * 12
                || cur.x >= limit.up-1 || cur.x <= limit.down+1 || cur.y >= limit.left-1 || cur.y <= limit.right+1){

                FRIZY_LOG(LOG_DEBUG,"forceshibai");
                forceWall = 0;
                _trouble.type = nothing;
              
                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);  

                ewallTime = 0;        
                road_aim.kind = searchBound;
                FRIZY_LOG(LOG_DEBUG,"the kind.%d",road_aim.kind);      
                planRoad.PlanSearch(road_aim,-1);
                process = PLAN;
                return 1;           
            }
            return 1;
        }

        if (cur.x == road_aim.x && cur.y == road_aim.y)
        {
            againCount = 0;
            _maze.trouble = 0;
            _maze.specialWall.clear();
            _maze.overTime = 0;
            switch (road_aim.kind)
            {

            case recharge:

                FRIZY_LOG(LOG_DEBUG,"huichong0");
                
                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);  

                cleanTaskOverIndex = false;
                call_recharge_index = true;                
                recharge_first_index++;
                FRIZY_LOG(LOG_DEBUG,"recharge_first_index = %d",recharge_first_index);
                chassisRoad.chassisSpeed(0,0,6);
                return 1;

            case breakClean:{
                FRIZY_LOG(LOG_DEBUG,"daodale10");
                
                if (_maze.breakClean.pro == BOUND){  

                    process = BOUND;
                    deta = cur.addAngle - _maze.breakClean.breakPonit.addAngle;
                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta;   
                        
                    if (_maze.breakClean.iswall){

                        FRIZY_LOG(LOG_DEBUG,"cool1");           
                        justwind = 1;
                        StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                        return 1;
                    }else{

                        FRIZY_LOG(LOG_DEBUG,"cool2");           
                                   
                        return 1;
                    }

                }else{
                    FRIZY_LOG(LOG_DEBUG,"cool3");
                    process = PLAN;

                    break;
                }
            }
            
            case home:{
                FRIZY_LOG(LOG_DEBUG,"daodale9");
                
                _maze.InitRecordMap();
                roadDwa.init();
                roadState = roadIdle;
                UTURN = SEARCH_WALL;  
                process = PLAN;
                return 1;                         
            }

            case lastUnclean:{
                FRIZY_LOG(LOG_DEBUG,"daodale8");
                ewallTime = 0;
                forceWall = 1;
                return 1;
            }
            case searchBound:
                FRIZY_LOG(LOG_DEBUG,"daodale2");
                process = BOUND;
                break;

            case searchUnclean:
                FRIZY_LOG(LOG_DEBUG,"daodale1");
                process = PLAN;
                break;

            case searchInside:
                FRIZY_LOG(LOG_DEBUG,"daodale0");
                process = PLAN;
                break;
            
            
            case backBound:
                
                deta = cur.addAngle - tmpAdd;
                FRIZY_LOG(LOG_DEBUG,"daodale3.%d",deta);
                for (Grid tmp : boundPoint)
                    tmp.addAngle = tmp.addAngle + deta; 
                process = BOUND;
                
                break;

            case column:
            {
                FRIZY_LOG(LOG_DEBUG,"daodale4");

                if ((rollOb.back().x == cur.x && rollOb.back().y == cur.y)
                    || _maze.GetMapState(cur.x+1,cur.y,1) == 4 || _maze.GetMapState(cur.x-1,cur.y,1) == 4
                    || _maze.GetMapState(cur.x,cur.y+1,1) == 4 || _maze.GetMapState(cur.x,cur.y-1,1) == 4)
                {
                    deta = cur.addAngle - tmpAdd;
                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta; 

                    FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                    // boundArr.push_back(limit);
                    // boundPoint.clear();
                    // process = PLAN;
                    // road_aim.kind = searchInside;
                    // planRoad.PlanSearch(road_aim,-1);  
                    justwind = 1;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                    process = BOUND;
                    return 1;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"bbqle2");
                    for (int i = 0;i < rollOb.size();i++)
                    {
                        if (rollOb[i].x == cur.x && rollOb[i].y == cur.y)
                        {
                            
                            if (i + 3 < rollOb.size())
                                road_aim.x = rollOb[i+3].x,road_aim.y = rollOb[i+3].y;
                            else
                                road_aim.x = rollOb.back().x,road_aim.y = rollOb.back().y;

                            FRIZY_LOG(LOG_DEBUG,"chulai.%d.%d",road_aim.x,road_aim.y);

                            break;
                        }
                    }
                }
                break;
            }

            case zoning:{
                FRIZY_LOG(LOG_DEBUG,"daodale5");
                process = PLAN;
                break;
            }
            case appoint:
            {
                FRIZY_LOG(LOG_DEBUG,"daodale7");
                process = POINT;
                break;
            }
            case searchLast:{
                FRIZY_LOG(LOG_DEBUG,"daodale6");
                process = PLAN;
                break;        

                //到达假障碍点
                // if (lasttake != IDLE && iswall == 0)
                // {
                //     if (a == x && b == y)
                //     {
                //         printf("5s.\n");
                //         if (bumpstate == 1
                //             || (GetMapState(x,y+1) != 0 && GetMapState(x,y-1) == 0 && GetMapState(x+1,y) == 0 && GetMapState(x-1,y) == 0))
                //         {
                //             s5.x = x,s5.y = y;
                //             printf("s52.%d.%d\n",s5.x,s5.y);						
                //             iswall = 1;	
                //         }
                //         if (GetMapState(x,y+1) == 0)
                //         {
                //             printf("w1\n");
                //             pointTemp.x = x;
                //             pointTemp.y = y + 1;
                //             return pointTemp;					
                //         }
                //         if (GetMapState(x,y-1) == 0)
                //         {
                //             printf("w2\n");
                //             pointTemp.x = x;
                //             pointTemp.y = y-1;
                //             return pointTemp;					
                //         }				
                //         if (GetMapState(x+1,y) == 0)
                //         {
                //             printf("w3\n");
                //             pointTemp.x = x + 1;
                //             pointTemp.y = y;
                //             return pointTemp;					
                //         }		
                //         if (GetMapState(x-1,y) == 0)
                //         {
                //             printf("w4\n");
                //             pointTemp.x = x - 1;
                //             pointTemp.y = y;
                //             return pointTemp;					
                //         }								
                //     }
                //     else
                //     {
                //         printf("sssss5 = 0\n");
                //         sssss5 = 0;
                //     }
                // }
            }           

            default:
                break;
            }

            if (IsWall() != 0)
            {
                do{
                    FRIZY_LOG(LOG_DEBUG,"outroad1");
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(20 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                   
            }
            roadDwa.init();
            roadState = roadIdle;
            return 1;
        }     

        if (sensor.bump || sensor.obs || (_maze.forceTurn > 1 && roadState == roadTrue)
            || (!IsWall() && roadState == roadFalse && NearSeat(cur,cur,2) && sensor.leftw > 100 && sensor.rightw > 100)) 
        {  
            _maze.forceTurn = 0;    

            if (roadState == roadTrue){
                roadDwa.init();
                bumpCount ++;
            }
            
            switch (road_aim.kind){

            case recharge:{
                if ((abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2)){
                            
                    FRIZY_LOG(LOG_DEBUG,"huichong1");
                    chassisRoad.chassisSpeed(0,0,6);
                    cleanTaskOverIndex = false;
                    call_recharge_index = true;                
                    recharge_first_index++;
                    break;
                }
                
                FRIZY_LOG(LOG_DEBUG,"remove9.%d",againCount);
                chassisRoad.chassisSpeed(0,0,1);
                break;
            }  

            case breakClean:{
                FRIZY_LOG(LOG_DEBUG,"remove11");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2){


                    if (_maze.breakClean.pro == BOUND){  

                        process = BOUND;
                        deta = cur.addAngle - _maze.breakClean.breakPonit.addAngle;
                        for (Grid tmp : boundPoint)
                            tmp.addAngle = tmp.addAngle + deta;   
                            
                        if (_maze.breakClean.iswall){

                            FRIZY_LOG(LOG_DEBUG,"cool1");           
                            justwind = 1;
                            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                            return 1;
                        }else{

                            FRIZY_LOG(LOG_DEBUG,"cool2");           
                                    
                            return 1;
                        }

                    }else{
                        FRIZY_LOG(LOG_DEBUG,"cool3");
                        process = PLAN;
                        
                        return 1;
                    }
                }
                              
                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"baojing5");
                    _maze.appWarn = 1;
                    return 1;                     
                }
                chassisRoad.chassisSpeed(0,0,1);
                break; 

            }
            case home:{
                FRIZY_LOG(LOG_DEBUG,"remove10");

                if (bumpCount > BUMP_MAX 
                    || ((abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                        && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2))
                    )
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax7 %d",bumpCount);
                    _maze.InitRecordMap();
                    roadDwa.init();
                    roadState = roadIdle;
                    UTURN = SEARCH_WALL;  
                    process = PLAN;
                    return 1;                     
                }
                
                chassisRoad.chassisSpeed(0,0,1);        
                break;     
            }
            case lastUnclean:{
                
                FRIZY_LOG(LOG_DEBUG,"remove8");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 1)
                {
                   
                    ewallTime = 0;
                    forceWall = 1;
            
                    return 1;
                }

                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax6");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }
                break;                
            }
            case searchLast:{

                FRIZY_LOG(LOG_DEBUG,"remove6");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 1)
                {

                    FRIZY_LOG(LOG_DEBUG,"5s wall");
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);  

                    roadDwa.init();
                    roadState = roadIdle;

                    return 1;
                }

                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax6");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }
                break;
            }   

            case searchUnclean:{
                FRIZY_LOG(LOG_DEBUG,"remove1");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2)
                {

                    printf("into wall\n");
                    againCount = 0;
                    if (IsWall() != 0)
                    {
                        do{
                            FRIZY_LOG(LOG_DEBUG,"outroad2");
                            StopWallFollow();
                            chassisRoad.GetSensor(&sensor);
                            usleep(10 * 1000);
                        }while (sensor.leftw != 0 || sensor.rightw != 0);                   
                    }  

                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,2);  
                    process = PLAN;
                    return 1;
                }

                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax1");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }
               
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                }          
                break;
            }
            case searchBound: {
                FRIZY_LOG(LOG_DEBUG,"remove2");
                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax2");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }  
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                }                
                break; 
            }
            case searchInside:{
                FRIZY_LOG(LOG_DEBUG,"remove3");
                if (bumpCount > 2*BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax3");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }                 
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                } 
            }
            case backBound:{
                FRIZY_LOG(LOG_DEBUG,"remove4");

                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"manle5");
                    roadDwa.init();
                    roadState = roadIdle;
                    boundArr.push_back(limit);
                    boundPoint.clear();
                    road_aim.kind = searchInside;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;                     
                }   

                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                } 
                break;
            }
            case column:{

                FRIZY_LOG(LOG_DEBUG,"remove12");

                if (bumpCount > 2 || roadState == roadFalse
                     || (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                        && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2)){

                    deta = cur.addAngle - tmpAdd;
                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta; 

                    FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                    justwind = 1;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                    process = BOUND;
                    return 1;
                }

                chassisRoad.chassisSpeed(0,0,1);        
                break;    
            }

            case zoning:{


                FRIZY_LOG(LOG_DEBUG,"zoning remove");

                if ((abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 2)){
                            
                    FRIZY_LOG(LOG_DEBUG,"zoning1");
                
                    process = PLAN;
                    return 1;
                }

                chassisRoad.chassisSpeed(0,0,1);
                //process = PLAN;
                break;
            }
            
            case appoint:{
                FRIZY_LOG(LOG_DEBUG,"remove7");
                chassisRoad.chassisSpeed(0,0,1);
                // process = POINT;
                break;
            }
            default:
                break;
            }
            
            switch (roadState){

            case roadTrue:{

                FRIZY_LOG(LOG_DEBUG,"roadbump1");
                roadDwa.init();
                roadState = roadIdle;
                return 1;
            }
            case roadFalse:{

                if (IsWall() == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"roadbump2");
                    dewallSign = 1;
                    ewall = cur,ewallTime = 0; 
                } 
                break;
            }
            default:
                break;
            }
        }
        return 0;
    } 

    void RoadPlanning::StartRoad(Sensor sensor,Grid cur,Trouble trouble)
    {
        _trouble = trouble;

        RobotType sss;

        if (ArriveRoad(sensor,cur))
            return;

        switch (roadState)
        {
        case roadIdle:
            //平滑曲线
            do{
                FRIZY_LOG(LOG_DEBUG,"road stop3");
                motionRoad.WheelControl(sensor,cur,cur);
                chassisRoad.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);

            FRIZY_LOG(LOG_DEBUG,"kaishijisuan");
            _aim.forward = 1000;
            ewallTime = 0,ewall = cur;

            if (road_aim.kind == searchBound || road_aim.kind == searchUnclean || road_aim.kind == recharge){
                pathType = 1;
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,1);
            }
            else if(road_aim.kind == appoint || road_aim.kind == zoning
                 || road_aim.kind == home || road_aim.kind == breakClean
                 || road_aim.kind == column){

                pathType = 2;
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,2);
            }    
            else{
                pathType = 0;
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,0); 
            }
                 

            // astarArr.push_back({cur.x*0.15,cur.x*0.15});
            // astarArr.push_back({road_aim.x*0.15,road_aim.y*0.15});
            FRIZY_LOG(LOG_DEBUG,"astar step2");
            
            if (astarArr.empty())
            {
                if (astarAgain
                    && (road_aim.kind == searchBound || road_aim.kind == searchUnclean || road_aim.kind == recharge)){
                    
                    int tmpB;
                    if (BUMP_MAX == 8) tmpB = BUMP_MAX - 4;
                    else  tmpB = BUMP_MAX;

                    if (road_aim.kind == recharge){
                        tmpB = 0;
                    }    

                    if (againCount >= tmpB){
                        FRIZY_LOG(LOG_DEBUG,"zhekezazheng.%d",againCount);
                        pathType = 2;
                        astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,2);
                    }
                    else{
                        pathType = 0;
                        astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,0);
                    }
                        

                    if (astarArr.empty()){

                        FRIZY_LOG(LOG_DEBUG,"yiranshibai");

                        astarAgain = 0;

                        if (road_aim.kind == recharge){

                            if (!nearpoint){
                                nearpoint = 1;
                                
                                int dis = 1000;
                                Point2i p;
                                p.x = 100;
                                for (int x = -5;x < 5;++x){
                                    for (int y = -5;y < 5;++y){
                                        if (_maze.GetMapState(road_aim.x+x,road_aim.y+y,1) == 4
                                            || _maze.GetMapState(road_aim.x+x,road_aim.y+y,1) == -1)
                                            continue;
                                            
                                        if (dis > abs(x)+abs(y)
                                             && _maze.GetMapState(road_aim.x+x,road_aim.y+y,2) == 1
                                             && _maze.GetMapState(road_aim.x+x+1,road_aim.y+y,2) == 1
                                             && _maze.GetMapState(road_aim.x+x-1,road_aim.y+y,2) == 1
                                             && _maze.GetMapState(road_aim.x+x,road_aim.y+y+1,2) == 1
                                             && _maze.GetMapState(road_aim.x+x,road_aim.y+y-1,2) == 1
                                             && _maze.GetMapState(road_aim.x+x+1,road_aim.y+y+1,2) == 1
                                             && _maze.GetMapState(road_aim.x+x+1,road_aim.y+y-1,2) == 1
                                             && _maze.GetMapState(road_aim.x+x-1,road_aim.y+y+1,2) == 1
                                             && _maze.GetMapState(road_aim.x+x-1,road_aim.y+y-1,2) == 1
                                             ){

                                            dis = abs(x)+abs(y);
                                            p.x = road_aim.x+x,p.y = road_aim.y+y;
                                        }
                                    }
                                }
                                if (p.x != 100){
                                    road_aim.x = p.x;
                                    road_aim.y = p.y;
                                }

                                FRIZY_LOG(LOG_DEBUG,"slamdian4 %d %d",road_aim.x,road_aim.y);
                                return;

                            }else if (nearpoint == 1 && !_maze.Seat.addAngle
                                 && _maze.GetMapState(record_charge_aim.x,record_charge_aim.y,2) != 2){

                                FRIZY_LOG(LOG_DEBUG,"slamdian5");
                                nearpoint = 2;
                                road_aim.x = record_charge_aim.x;
                                road_aim.y = record_charge_aim.y;
                                
                                
                                return;
                            }
                        }
  
                        roadState = roadFalse;                     
                    }
                    else{
                        FRIZY_LOG(LOG_DEBUG,"chenggongle");
                        againCount++;
                        astarAgain = 1;
                        roadState = roadTrue;                           
                    } 
                }
                else{
                    FRIZY_LOG(LOG_DEBUG,"astar fail");
                    roadState = roadFalse;
                }
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"astar success");
                // roadDwa._maze = _maze;
                roadState = roadTrue;
            }
            break;

        case roadTrue:

            FRIZY_LOG(LOG_DEBUG,"start run1");

            if (!nearpoint && road_aim.kind == recharge){
                FRIZY_LOG(LOG_DEBUG,"never change");
                nearpoint = 2;
            }

            dewallSign = 0;
            start_add = 0;
            
            goWall = 0;
            if (!roadDwa.start_path_planner(sensor,cur,astarArr,pathType))
            {
                FRIZY_LOG(LOG_DEBUG,"turanshibai");
                roadState = roadFalse;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"dwa ok");
                Null_aim.kind = idle;
            }
            
            FRIZY_LOG(LOG_DEBUG,"dwa over");
            
            break;

        case roadFalse:

            FRIZY_LOG(LOG_DEBUG,"start run2");
            if (road_aim.kind == appoint || road_aim.kind == zoning){
                FRIZY_LOG(LOG_DEBUG,"baojing1");
                _maze.appWarn = 1;
                return;
            }
            NullRunning(sensor,cur);
            
            break;
        default:
            break;
        }
    }

}    