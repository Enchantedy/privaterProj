/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-18 16:30:07
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/LocalPlanning.h"
#include "navigation_algorithm/RoadPlanning.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <sys/time.h>

using namespace std;

// extern double x_ ;			 //Odometer x coordinates
// extern double y_ ;			 //Odometer y coordinates
// extern float angle_;		 //Odometer angle
extern useerobot::Maze _maze;

namespace useerobot
{
Point2f lockP;
extern RoadAim road_aim;
float radius;
float last_wspeed = 0.0;
Grid dstart;
Grid dend;
int onceSpin;
int pathType;

	Grid CaculateAngle(Grid cur,Grid aim)
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

	// double round(double x)
	// {
  //   return (x > 0.0) ? floor(x + 0.5) : ceil(x - 0.5);
	// }

	inline bool Isfree(int x,int y){
		return _maze.GetMapState(x,y,2) != 2
			&& (pathType != 2 || _maze.GetMapState(x,y,1) != 2)
			&& (_maze.GetMapState(x,y,1) != 4 || _maze.GetMapState(dstart.x,dstart.y,1) == 4
					);
	}
	
	inline bool nearOb(Grid& cur,float forwords){
		
		// if (((cur.forward >= 0 && cur.forward <= 22) || cur.forward >= 338) && _maze.GetMapState(cur.x+1,cur.y,2) == 2)
		// 	return true;
		// if (cur.forward > 157 && cur.forward < 202 && _maze.GetMapState(cur.x-1,cur.y,2) == 2)
		// 	return true;
		// if (cur.forward > 77 && cur.forward < 112 && _maze.GetMapState(cur.x,cur.y-1,2) == 2)
		// 	return true;
		// if (cur.forward > 245 && cur.forward < 292 && _maze.GetMapState(cur.x,cur.y+1,2) == 2)
		// 	return true;
		// if (cur.forward > 22 && cur.forward <= 77 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 2)
		// 	return true;
		// if (cur.forward >= 112 && cur.forward <= 157 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 2)
		// 	return true;
		// if (cur.forward >= 202 && cur.forward <= 245 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 2)
		// 	return true;
		// if (cur.forward >= 292 && cur.forward < 338 && _maze.GetMapState(cur.x+1,cur.y+1,2) == 2)
		// 	return true;

		//FRIZY_LOG(LOG_DEBUG,"tmpa  %f   %d %d",forwords,cur.x,cur.y);

		if (((forwords >= 0 && forwords <= 27) || forwords >= 333) && _maze.GetMapState(cur.x+1,cur.y,2) == 2)
			return true;
		
		if (forwords > 152 && forwords < 207 && _maze.GetMapState(cur.x-1,cur.y,2) == 2)
			return true;

		if (forwords > 72 && forwords < 117 && _maze.GetMapState(cur.x,cur.y-1,2) == 2)
			return true;

		if (forwords > 240 && forwords < 297 && _maze.GetMapState(cur.x,cur.y+1,2) == 2)
			return true;

		if (forwords > 17 && forwords <= 82 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 2)
			return true;

		if (forwords >= 107 && forwords <= 162 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 2)
			return true;

		if (forwords >= 197 && forwords <= 250 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 2)
			return true;

		if (forwords >= 287 && forwords < 343 && _maze.GetMapState(cur.x+1,cur.y+1,2) == 2)
			return true;		

		return false;
	}
	 

	bool caculateLine(Grid& cur,Grid& aim,float& forward){

		Grid c1;

		if ((onceSpin || abs(cur.x-dstart.x)+abs(cur.y-dstart.y) == 0)
				&& abs(aim.x-dstart.x)+abs(aim.y-dstart.y)==1)
			{
				FRIZY_LOG(LOG_DEBUG,"qisi");
				c1 = {int(1000*cur.x),int(1000*cur.y)};
			}
			
		else
			c1 = {int(1000*cur.realx),int(1000*cur.realy)};

		Grid c2 = {int(1000*aim.x),int(1000*aim.y)};


		float tmpfor = CaculateAngle(c1,c2).forward;
		
		if (nearOb(cur,tmpfor)){
			//FRIZY_LOG(LOG_DEBUG,"nearOb  %f",tmpfor);
			return false;
		}
			
		if (tmpfor >= 315 || tmpfor <= 45){
			float k = tan(tmpfor*_PI/180);

			for (int x = 1;cur.x+x <= aim.x;++x){
				int y = cur.y - round(x*k);
				if (abs(y - road_aim.y) + abs(cur.x+x - road_aim.x) < 2)
					continue;

				if (!Isfree(cur.x+x,y) || !Isfree(cur.x+x,y+1) || !Isfree(cur.x+x,y-1)){
					//FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x+x,y,tmpfor);
					radius = pow(x*x + round(x*k)*round(x*k),0.5)+3;
					return false;
				}	
			}
		}
		
		else if (tmpfor >= 135 && tmpfor <= 225){
			float k = tan(tmpfor*_PI/180);
			for (int x = 1;cur.x-x >= aim.x;++x){
				int y = cur.y + round(x*k);
				if (abs(y - road_aim.y) + abs(cur.x-x - road_aim.x) < 2)
					continue;

				if (!Isfree(cur.x-x,y) || !Isfree(cur.x-x,y+1) || !Isfree(cur.x-x,y-1)){
					//FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x-x,y,tmpfor);
					radius = pow(x*x + round(x*k)*round(x*k),0.5)+3;
					return false;
				}	
			}			
		}
		else if (tmpfor > 45 && tmpfor < 135){
			float k = tan(_PI/2 - tmpfor*_PI/180);
			for (int y = 1;cur.y-y >= aim.y;++y){

				int x = cur.x + round(y*k);

				if (abs(cur.y-y - road_aim.y) + abs(x - road_aim.x) < 2)
					continue;	

				if (!Isfree(x,cur.y-y) || !Isfree(x+1,cur.y-y) || !Isfree(x-1,cur.y-y)){
					//FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x,cur.y-y,tmpfor);
					radius = pow(round(y*k)*round(y*k) + y*y,0.5)+3;
					return false;
				}	
			}			
		}
		else{
			float k = tan(_PI/2 - tmpfor*_PI/180);
			for (int y = 1;cur.y+y <= aim.y;++y){
				int x = cur.x - round(y*k);
				if (abs(cur.y+y - road_aim.y) + abs(x - road_aim.x) < 2)
					continue;	

				if (!Isfree(x,cur.y+y) || !Isfree(x+1,cur.y+y) || !Isfree(x-1,cur.y+y)){
					//FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x,cur.y+y,tmpfor);
					radius = pow(round(y*k)*round(y*k) + y*y,0.5)+3;
					return false;
				}	
			}			
		}	
		
		forward = tmpfor;
		return true;	
	}

	void dwa::init(){
		onceSpin = 1;
		initDwa = 1;
		lastspeedW = 0;
		maxIndex1 = 0;
		maxIndex2 = 0;
		locked = 0;
		forceS = 0;
	}
	void dwa::updataGoalPos(Grid& cur,Points& astarPath,Grid& aim)
	{
		
		int size = astarPath.size();
		float dis = 100.0;
		int index = -1;

		Grid tmp = {100,100};
		for (int i = 0;i < size-1;++i){

			Point2i p = {astarPath[i].first,astarPath[i].second};

			if (i < maxIndex1)
				continue;


			if (road_aim.kind == appoint || road_aim.kind == zoning){
					
				if (_maze.GetMapState(p.x,p.y,1) == 4 && abs(p.x) > 3 && abs(p.y) > 3){
						FRIZY_LOG(LOG_DEBUG,"baojing6 %d %d",p.x,p.y);
						_maze.appWarn = 1;
						return;                        
				}	
			}


			if (fabs(p.x-cur.realx) + fabs(p.y-cur.realy) <= dis){

				if (cur.x == dstart.x && cur.y == dstart.y
					 && abs(p.x-cur.x)+abs(p.y-cur.y) == 2
					 && abs(tmp.x-cur.x)+abs(tmp.y-cur.y) == 2){

					FRIZY_LOG(LOG_DEBUG,"fule %d %d",p.x,p.y);
					continue;

				}else{
					tmp.x = p.x,tmp.y = p.y;
					dis = fabs(p.x-cur.realx) + fabs(p.y-cur.realy);
					index = i;
				}
			}
		}

		
		if (onceSpin && abs(cur.x-dstart.x)+abs(cur.y-dstart.y) == 0){
			FRIZY_LOG(LOG_DEBUG,"suozhule");
			index = 0;
		}

		//是否需要更新 maxIndex1
		FRIZY_LOG(LOG_DEBUG,"index111 %d %f ",index,dis);

		if (index > maxIndex1){

				tmp = {astarPath[index+1].first,astarPath[index+1].second};

				Point2i p = {astarPath[index-1].first,astarPath[index-1].second};
				float tmpdis = fabs(p.x-cur.realx) + fabs(p.y-cur.realy);

				if (caculateLine(cur,tmp,tmp.forward)
					 || dis < 1.5
					 || abs(cur.x-astarPath[index].first)+abs(cur.y-astarPath[index].second) == 0
					 || tmpdis > 10*dis){
					
					FRIZY_LOG(LOG_DEBUG,"keda %d %f %f",index,dis,tmpdis);
					maxIndex1 = max(maxIndex1,index);
				}
		}

		
		index = -1;
		aim.forward = 1001;
		
		for (int i = size-1; i > maxIndex1 && i >= maxIndex2 && i > 0;--i){
			
			stage_aim = {astarPath[i].first,astarPath[i].second};

			if (!caculateLine(cur,stage_aim,aim.forward)){
				continue;
			}
			else{
				index = i;
				maxIndex2 = max(maxIndex2,index);
				aim.x = stage_aim.x,aim.y = stage_aim.y;
				FRIZY_LOG(LOG_DEBUG,"sss1.%d %d,%d  max %d %d",index,aim.x,aim.y,maxIndex1,maxIndex2);
				FRIZY_LOG(LOG_DEBUG,"tmpfor.%f  %f  %d,%d",aim.forward,cur.forward,stage_aim.x,stage_aim.y);
				break;
			}
		}

		if (index == -1){
			index = maxIndex1 >= maxIndex2 ? maxIndex1+1 : maxIndex2;
			index = min(index,size-1);
			ready_aim = {astarPath[index].first,astarPath[index].second};
			aim.forward = CaculateAngle(cur,ready_aim).forward;
			aim.x = ready_aim.x,aim.y = ready_aim.y;
			FRIZY_LOG(LOG_DEBUG,"sss2 %d,%d  %f  max %d %d %d",ready_aim.x,ready_aim.y,aim.forward,maxIndex1,maxIndex2,index);
			FRIZY_LOG(LOG_DEBUG,"tmpfor.%f  %f  %d,%d",aim.forward,cur.forward,ready_aim.x,ready_aim.y);
		}

		if (locked == 1 && aim == firstPoint
				&& (firstPoint.x != dend.x || firstPoint.y != dend.y)){ // && !caculateLine(cur,firstPoint,tmp.forward)

			  vector<VFH_PARA> valid_dir;    

        float frontDis = 0;	
				chassisDwa.updataVFH(cur,1,valid_dir,frontDis);
				 
				radius = 15*pow((cur.realx-firstPoint.x)*(cur.realx-firstPoint.x)
											+(cur.realy-firstPoint.y)*(cur.realy-firstPoint.y),0.5);

				FRIZY_LOG(LOG_DEBUG,"radius: %f frontDis: %f",radius,frontDis);

				if (frontDis < radius){

					FRIZY_LOG(LOG_DEBUG,"dongtai: %f",firstPoint.forward);
					float _ang;
					float _dis;
					
					for (float thea = 5;thea <= 30;thea+=5){

						_ang = firstPoint.forward + thea;
						
						if (_ang >= 360) _ang -= 360;

						_dis = chassisDwa.vfh_histogram[static_cast<int>(_ang/5)];

						if (_dis > frontDis+20 || _dis > radius){

							float maxD = min(frontDis+20,radius)/15;	
							aim.x = round(cur.realx + maxD*cos(_ang*_PI/180));
							aim.y = round(cur.realy - maxD*sin(_ang*_PI/180));
							aim.forward = _ang;
							lockP.x = cur.realx + maxD*cos(_ang*_PI/180);
							lockP.y = cur.realy - maxD*sin(_ang*_PI/180);
							locked = 2;

							FRIZY_LOG(LOG_DEBUG,"keyile1 %d %d %f maxD: %f",aim.x,aim.y,aim.forward,maxD);

							int tar3 = abs(cur.x-firstPoint.x) + abs(cur.y - firstPoint.y);
							int tar4 = abs(cur.x-aim.x) + abs(cur.y - aim.y);
							int tar1 = abs(aim.x-firstPoint.x) + abs(aim.y - firstPoint.y);
							int tar2 = abs(astarPath[index+1].first-firstPoint.x) + abs(astarPath[index+1].second-firstPoint.y);

							FRIZY_LOG(LOG_DEBUG,"tar12: %d %d tar34: %d %d",tar1,tar2,tar3,tar4);

							if ((tar1 > 0 && tar1 <= 2 && tar2 > tar1 && tar3 <= tar4)
									|| tar4 < tar3){
								
								if (tar3 >= tar4 + 5 && index > 1){
									FRIZY_LOG(LOG_DEBUG,"yyy1 : %d",index);
									pair<int,int> tp = {aim.x,aim.y};	
									astarPath.insert(astarPath.begin()+index,tp);

								}else if (abs(tar3-tar4) <= 2){
									FRIZY_LOG(LOG_DEBUG,"yyy2");	
									for (auto& gri: astarPath){
										
										if (gri.first == firstPoint.x && gri.second == firstPoint.y){
											gri.first = aim.x;
											gri.second = aim.y;
											break;
										}
									}
								}
											
								for (auto& gri: astarPath)
									FRIZY_LOG(LOG_DEBUG,"gir: %d %d",gri.first,gri.second);
							}

							return;
						}
						
						_ang = firstPoint.forward - thea;
						
						if (_ang < 0) _ang += 360;

						_dis = chassisDwa.vfh_histogram[static_cast<int>(_ang/5)];

						if (_dis > frontDis+20 || _dis > radius){
									
							float maxD = min(frontDis+20,radius)/15;	
							aim.x = round(cur.realx + maxD*cos(_ang*_PI/180));
							aim.y = round(cur.realy - maxD*sin(_ang*_PI/180));
							aim.forward = _ang;
							lockP.x = cur.realx + maxD*cos(_ang*_PI/180);
							lockP.y = cur.realy - maxD*sin(_ang*_PI/180);
							locked = 2;
							FRIZY_LOG(LOG_DEBUG,"keyile2 %d %d %f maxD: %f",aim.x,aim.y,aim.forward,maxD);

							int tar3 = abs(cur.x-firstPoint.x) + abs(cur.y - firstPoint.y);
							int tar4 = abs(cur.x-aim.x) + abs(cur.y - aim.y);

							int tar1 = abs(aim.x-firstPoint.x) + abs(aim.y - firstPoint.y);
							int tar2 = abs(astarPath[index+1].first-firstPoint.x) + abs(astarPath[index+1].second-firstPoint.y);
							FRIZY_LOG(LOG_DEBUG,"tar12: %d %d tar34: %d %d",tar1,tar2,tar3,tar4);
							
							if ((tar1 > 0 && tar1 <= 2 && tar2 > tar1 && tar3 <= tar4)
									|| tar4 < tar3){
								
								if (tar3 >= tar4 + 5 && index > 1){

									FRIZY_LOG(LOG_DEBUG,"yyy1 : %d",index);

									pair<int,int> tp = {aim.x,aim.y};	
									astarPath.insert(astarPath.begin()+index,tp);

								}else if (abs(tar3-tar4) <= 2){

									FRIZY_LOG(LOG_DEBUG,"yyy2");	
									for (auto& gri: astarPath){
										
										if (gri.first == firstPoint.x && gri.second == firstPoint.y){
											gri.first = aim.x;
											gri.second = aim.y;
											break;
										}
									}
								}

								for (auto& gri: astarPath)
									FRIZY_LOG(LOG_DEBUG,"gir: %d %d",gri.first,gri.second); 								
							}

							return;
						}
					}
					FRIZY_LOG(LOG_DEBUG,"wandan %d %d %f",tmp.x,tmp.y,tmp.forward);
				}
				locked = 0;
			}

	}

	bool dwa::start_path_planner(Sensor sensor,Grid cur,Points& astarArr,int type){

			pathType = type;
			if((cur.x == road_aim.x && cur.y == road_aim.y)
				|| sensor.bump){

				FRIZY_LOG(LOG_DEBUG,"dwa run finished");
				init();				
				return true;
			}

			WheelTrans(sensor,cur,&RobotPos);
			//printf("speed1.%f,%f\n",RobotPos.v,RobotPos.w);
			dynamicWindow(&speedWindow);
			//printf("speed2.%f,%f  %f,%f\n",speedWindow.v_max,speedWindow.v_min,speedWindow.w_max,speedWindow.w_min);

			FRIZY_LOG(LOG_DEBUG,"initDwa.%d.locked.%d onceSpin.%d pathType.%d",initDwa,locked,onceSpin,pathType);

			if (locked != 2)
				updataGoalPos(cur,astarArr,_aim);

			FRIZY_LOG(LOG_DEBUG,"aim:: %d %d %f  first:: %d %d %f"
				,_aim.x,_aim.y,_aim.forward,firstPoint.x,firstPoint.y,firstPoint.forward);

			switch (initDwa){

			case 1:{
				FRIZY_LOG(LOG_DEBUG,"kaishila");
				dstart = {cur.x,cur.y};
				dend = {astarArr.back().first,astarArr.back().second};
				firstPoint = _aim;
				initDwa = 2;
				return true;
			}

			case 0:{
				//达到临时目标角度	
				if (fabs(_aim.forward - cur.forward) < 5 || fabs(_aim.forward - cur.forward) > 355){

					//锁定ing
					if (locked == 2){

						if (fabs(_aim.forward - cur.forward) < 2 || fabs(_aim.forward - cur.forward) > 358){
							FRIZY_LOG(LOG_DEBUG,"hehe1");
							locked = 0;
						}
						FRIZY_LOG(LOG_DEBUG,"locking");
						motionDwa.WheelControl(sensor,cur,_aim);
						return true;
					}

					FRIZY_LOG(LOG_DEBUG,"nima1");
					motionDwa.ClearPid();
					firstPoint = _aim;
					initDwa = 2;
					onceSpin = 0;


				//达到临时目标坐标
				}else if (fabs(_aim.x-cur.realx) + fabs(_aim.y-cur.realy) < 0.5){

					if (locked == 2){
						FRIZY_LOG(LOG_DEBUG,"hehe2");
						locked = 0;
						return true;
					}
					//直接直行干就完了
					FRIZY_LOG(LOG_DEBUG,"nima3");
					motionDwa.ClearPid();
					firstPoint = _aim;
					firstPoint.forward = cur.forward;
					initDwa = 2;										
				}
				else{
					break;
				}
			}
		
			case 2:{
				//更新了目标点
				if (_aim.x != firstPoint.x || _aim.y != firstPoint.y){

					// if (locked == 1){
					// 	FRIZY_LOG(LOG_DEBUG,"locked = 2");
					// 	locked = 2;
					// }	

					FRIZY_LOG(LOG_DEBUG,"nima2");
					initDwa = 0;
					forceS = 0;
					break;
				}
				else if (forceS){
					if (fabs(cur.forward-_aim.forward)>5 && fabs(cur.forward-_aim.forward)<355
						&& (cur.x != dend.x || cur.y != dend.y)){

						FRIZY_LOG(LOG_DEBUG,"zhengzhazhong %f  %f",cur.forward,_aim.forward);
						motionDwa.WheelControl(sensor,cur,_aim);
						return true;
					}
					else{
						FRIZY_LOG(LOG_DEBUG,"daoweile");
						forceS = 0;
					}
				}
				else{
					if (abs(cur.x-dend.x) + abs(cur.y-dend.y) <= 1){
						FRIZY_LOG(LOG_DEBUG,"lastlast.%d.%d",dend.x,dend.y);
						motionDwa.WheelControl(sensor,cur,dend);
					}
					//锁定角度保持直行
					else{
						
						FRIZY_LOG(LOG_DEBUG,"firstPointing %d.%d  %f",firstPoint.x,firstPoint.y,firstPoint.forward);

						if (onceSpin && sensor.leftw > 150 && sensor.rightw > 150
							 && (fabs(firstPoint.forward - cur.forward) < 5 || fabs(firstPoint.forward - cur.forward) > 355)){
								 FRIZY_LOG(LOG_DEBUG,"onceSpin = 0");
								 onceSpin = 0;
							 }


						if (onceSpin
							 || (fabs(firstPoint.forward - cur.forward) < 5 || fabs(firstPoint.forward - cur.forward) > 355)
							 ){

								// //gaidong1
								if (!onceSpin && 
									 	(abs(firstPoint.x-cur.x) > 1 || abs(firstPoint.y-cur.y) > 1)){

									FRIZY_LOG(LOG_DEBUG,"prepeng %d",firstPoint == _aim);
									
									locked = 1;
								}

								// if (firstPoint.x == dend.x && firstPoint.y == dend.y
								// 	 && (cur.x == dend.x || cur.y == dend.y)
								// 	 && abs(cur.x-dend.x) + abs(cur.y-dend.y) > 1
								// 	 && caculateLine(cur,dend,dend.forward)
								// 	){
								// 		FRIZY_LOG(LOG_DEBUG,"kezhida  cur: %d %d end: %d %d",cur.x,cur.y,dend.x,dend.y);
								// }

								motionDwa.WheelControl(sensor,cur,firstPoint);

						}	
						else{
							FRIZY_LOG(LOG_DEBUG,"qiang1");
							break;
						}	
					}
					return true;
				}
			}


			default:
				break;
			}
			
			//FRIZY_LOG(LOG_DEBUG,"aimfor %d %d %f  %d %d",_aim.x,_aim.y,_aim.forward,maxIndex1,maxIndex2);

			//在这添加VFH算法,修正aimfor
			//gaidong3

			Trajectory best_traj;

			priority_queue<Node> q;

			float k1,k2;
			float det = fabs(cur.forward-_aim.forward) > 180 ? 360-fabs(cur.forward-_aim.forward):fabs(cur.forward-_aim.forward);
			
			k1 = (det/45);	
			FRIZY_LOG(LOG_DEBUG,"k1 : %f",k1);
			k2 = 1.0;

			for (float v = speedWindow.v_min; v <= speedWindow.v_max; v += cfg.v_reso){
					for (float w = speedWindow.w_min; w <= speedWindow.w_max; w += cfg.yawrate_reso){
					
					Trajectory traj;

					s_left = s_right = 0;
					s_left = static_cast<int>(1000*(v + w * cfg.wheel_L / 2.0));
					s_right = static_cast<int>(1000*(v - w * cfg.wheel_L / 2.0));

					if (!PredictTrajectory(v,w,traj)){
						FRIZY_LOG(LOG_DEBUG,"bbb.%d.%d",s_left,s_right);
						continue;
					}
					
					if ((s_left < 120 && s_right < 120 && s_left >= 0 && s_right >= 0)
							|| (s_left + s_right == 0 && abs(s_left) > 130)
							|| s_left > 250 || s_right > 250 || abs(s_left - s_right) > 240
							|| s_left * s_right <= 0)
						continue;

					
					float v_forward = 0.0;

					int v_index = 60;

					if (_aim.forward < 1000){

						for (int i = 0;i < traj.size();++i){
							
							Grid c1 = {static_cast<int>(1000*traj[i].x/0.15),static_cast<int>(1000*traj[i].y/0.15)};
							
							Grid c2 = {1000*_aim.x,1000*_aim.y};

							if (locked == 2)
								c2 = {static_cast<int>(1000*lockP.x),static_cast<int>(1000*lockP.y)};

							float tmpfor = CaculateAngle(c1,c2).forward;

							float tmp = fabs(fabs(traj[i].theta*180/_PI - tmpfor) - 180);

							if (tmp > v_forward){
								v_forward = tmp;
								v_index = i;
								if (fabs(v_forward - 180) < 1)
									break;
							}
						}
					}

					node = {s_left,s_right,v_forward,v_index};
					q.push(node);

					// float value = 5*v_forward - v_index
					// + k1*abs(s_left-s_right) + k2*(abs(s_left)+abs(s_right));					
					
					// FRIZY_LOG(LOG_DEBUG,"v: %f  %d  %f  %f s: %d  %d  %f  value: %f"
					// ,v_forward,v_index,traj[v_index].v,traj[v_index].w
					// ,s_left
					// ,s_right
					// ,traj[v_index].theta*180/_PI
					// ,value);
					
					// //float value = 10 - fabs(w) - fabs(w-lastspeedW) - 20 * fabs(0.15-v) + v_forward - v_index;

					// if (value > max_value){
					// 		max_value = value;
					// 		best_traj = traj;
					// }  

				}
			}

			vector<Node> vec;

			float threshold = 178;
			// if (q.empty() || q.top().v_for < threshold){
			// 		FRIZY_LOG(LOG_DEBUG,"faille.%d",best_traj.size());
			// 		init();
			// 		return false;
			// }
			// else{
			// 	while(!q.empty() && q.top().v_for > threshold){
			// 			vec.push_back(q.top());
			// 			q.pop();
			// 		}
			// }initDwa


			if (q.empty() || q.top().v_for < threshold-5){


					if (!forceS && fabs(cur.forward-_aim.forward)>5 && fabs(cur.forward-_aim.forward)<355){

						if (!q.empty())
							FRIZY_LOG(LOG_DEBUG,"zhengzha.%f",q.top().v_for);

						motionDwa.WheelControl(sensor,cur,_aim);
						initDwa = 2;
						firstPoint = _aim;
						forceS = 1;
						return true;
					}

					if (!q.empty())
						FRIZY_LOG(LOG_DEBUG,"faille.%f",q.top().v_for);

					
					init();
					return false;
			}
			else{
				if (q.top().v_for < threshold)
					threshold -= 5;

				while(!q.empty() && q.top().v_for > threshold){
						vec.push_back(q.top());
						q.pop();
					}
			}
			
			
			FRIZY_LOG(LOG_DEBUG,"vec1: %d %d",vec.size(),threshold);
			
			// if (vec.size() > 20){
			// 	vec.resize(vec.size()/2);
			// }
			// FRIZY_LOG(LOG_DEBUG,"vec2: %d",vec.size());

			float max_value = std::numeric_limits<float>::min(); // > 0
			max_value = -1000;
			for (int i = 0;i < vec.size();++i){

				float value = -vec[i].v_i
				+ k1*abs(vec[i].sl-vec[i].sr) + k2*(vec[i].sl+vec[i].sr);					
				
				// FRIZY_LOG(LOG_DEBUG,"v: %f  %d  s: %d  %d  value: %f"
				// 	,vec[i].v_for,vec[i].v_i,vec[i].sl,vec[i].sr,
				// 	value);
				
				//float value = 10 - fabs(w) - fabs(w-lastspeedW) - 20 * fabs(0.15-v) + v_forward - v_index;

				if (value > max_value){
					max_value = value;
					s_left = vec[i].sl;
					s_right = vec[i].sr;
				}  
			}
			 
			FRIZY_LOG(LOG_DEBUG,"last : %d,%d",s_left,s_right);

			
			if (!s_left && s_right){
				s_left = 1;
			}
			if (s_left && !s_right){
				s_right = 1;
			}
			// lastspeedW = best_traj.back().w;
			// float lastv = best_traj.back().v;
			// float lastw = best_traj.back().w;
			// s_left = static_cast<int>(1000*(lastv + lastw * cfg.wheel_L / 2.0));
			// s_right = static_cast<int>(1000*(lastv - lastw * cfg.wheel_L / 2.0));
			// FRIZY_LOG(LOG_DEBUG,"last: %f  %f  %d,%d ",lastv,lastw,s_left,s_right);

			chassisDwa.chassisSpeed(s_left,s_right,100);
			return true;

	}

	bool dwa::PredictTrajectory(float v,float w,Trajectory& traj)
	{
			float time = 0.0f;

			// Trajectory traj = {x_init};

			// RobotData x = x_init;

			RobotData _new;
			_new = {RobotPos.x,RobotPos.y,RobotPos.theta};

			while (time <= cfg.predict_time)
			{
				_new.theta += w * cfg.dt;
				if (_new.theta >= 2*_PI) _new.theta -= 2*_PI;
				if (_new.theta < 0) _new.theta += 2*_PI;
				_new.x += v * cfg.dt * cos(_new.theta);
				_new.y -= v * cfg.dt * sin(_new.theta);
				_new.v = v;
				_new.w = w;

				//Grid tmp;

				//tmp.realx = x_new.x,tmp.realy = x_new.y;
				int x1 = _new.x/0.15;
				int y1 = _new.y/0.15;

				int x2 = _new.x/0.15 >= 0 ? x1+1:x1-1;
				int y2 = _new.y/0.15 >= 0 ? y1+1:y1-1;

				// for (int x = -1;x <= 1;++x)
				// {
				// 		for (int y = -1;y <= 1;++y){
				// 				tmp.x += x;
				// 				tmp.y += y;
				// 				if (_maze.GetMapState(tmp.x,tmp.y,2) == 2){
				// 						printf("okk.%f,%f  %f,%f\n",tmp.realx,tmp.realy,x_new.v,x_new.w);
				// 						return false;
				// 				}
				// 		}
				// }
				traj.push_back(_new);
				time += cfg.dt;
			}

			return true;
	}



}