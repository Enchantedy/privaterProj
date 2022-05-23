#include "include/alongWall.h"

static bool escape_charge = false;
static bool escape_forbid = false;
static int forbid_charge = 0;
static int small_place_start_Angle = 0;
static int small_place_flag = 0;
static int enter_small_place_flag = 0;
static long long small_place_cnt = 0;
static float recharge_x, recharge_y, esc_charge_angle;
void AlongWall::startAlongWall()
{
    along_wall_running_ = true;
}

void AlongWall::stopAlongWall()
{
    along_wall_running_ = false;
}

int AlongWall::init()
{
    memset(&wall_para_, 0, sizeof(along_wall_para));
}

void AlongWall::initPid()
{
    memset(&wall_para_.pid, 0, sizeof(PID));
}

bool AlongWall::isNearForbidArea(float x, float y, int area_type)
{
    if(area_type != 0)   //回充座禁区
        {
            for(auto p : _maze.seatPoint)
            {
                dis = sqrtf((p.first - x) * (p.first - x) + (p.second - y) * (p.second - y));
                // FRIZY_LOG(LOG_DEBUG, "seatPoint x y dis= %f %f %f", p.first, p.second, dis);
                if(dis <= 0.05)
                    return true;
            }
        }
        else            //普通禁区
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

float AlongWall::getForbidAreaDis(int dir, int area_type)
{
    float x, y, forward, rad, tmp;
    if(dir == 0)        //front
            forward = current_pos_.forward;
        else if(dir == 1)   //right
        {
            forward = current_pos_.forward + 90;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 2)   //left
        {
            forward = current_pos_.forward - 90;
            if(forward < 0) 
                forward = forward + 360;
        }
        else if(dir == 3)
        {
            forward = current_pos_.forward + 45;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 4)
        {
            forward = current_pos_.forward - 45;
            if(forward < 0) 
                forward = forward + 360;
        }
        rad = (forward / 180) * 3.14159f;
        for(float i = 0.05f; i <= 0.51f; i += 0.05f)
        {
            y = (current_pos.realx * 15 / 100) + (i * cos(rad));
            x = (current_pos.realy * 15 / 100) - (i * sin(rad));
            // tmpx = current_pos.realx + (i * cos(tmpcan));
            // tmpy = current_pos.realy - (i * sin(tmpcan));
            if(isNear(x, y, area_type))
            {
                tmp = sqrtf((((current_pos.realx * 15 / 100) - x) * ((current_pos.realx * 15 / 100) - x)) + (((current_pos.realy * 15 / 100) - y) * ((current_pos.realy * 15 / 100) - y)));
                //FRIZY_LOG(LOG_DEBUG, "dir:%d catch forbid signal, return tmp:%f", dir, tmp);
                return tmp;
            }
        }
        return 300;
}

bool AlongWall::isInChargeArea()
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

bool AlongWall::isInForbidArea()
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

bool AlongWall::isAwayChargeArea()
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

bool AlongWall::isAwayForbidArea()
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


void AlongWall::bumpDetect()
{
    static int pid_count = 0;
    float front_charge_dis = getForbidAreaDis(0, 1);
    float front_forbid_dis = getForbidAreaDis(0, 0);
    bool in_charge_area = isInChargeArea();
    bool in_forbid_area = isInForbidArea();
    if(!in_charge_area)
    {
        if(escape_charge) {
            if(isAwayChargeArea()) {
                FRIZY_LOG(LOG_DEBUG, "away vir area");
                escape_charge = false;
            }
        }
    }
    if(!front_forbid_dis)
    {
        if(escape_forbid) {
            if(isAwayForbidArea()) {
                FRIZY_LOG(LOG_DEBUG, "away for area");
                escape_forbid = false;
            }
        }
    }
    if(in_forbid_area)
        escape_forbid = true;
    if(in_charge_area && !escape_charge) {
        wall_para_.bump_record = InCharge;
        escape_charge = true;
    }
    else if(!escape_charge && wall_para_.mode == PID_ALONG_WALL && wall_para_.dir = RIGHT_ALONG_WALL && 
            (front_charge_dis < 0.25 || getForbidAreaDis(4, 1) < 0.15 || getForbidAreaDis(3, 1) < 0.15)) {
        wall_para_.bump_record = ForbidChargeFront;
        forbid_charge = 1;
    }
    else if(!escape_charge && wall_para_.mode == PID_ALONG_WALL && wall_para_.dir = LEFT_ALONG_WALL && 
            (front_charge_dis < 0.25 || getForbidAreaDis(4, 1) < 0.15 || getForbidAreaDis(3, 1) < 0.15)) {
        wall_para_.bump_record = ForbidChargeFront;
        forbid_charge = 1;
    }
    else if(!escape_charge && wall_para_.mode == CHARGE_ALONG_WALL && wall_para_.dir = RIGHT_ALONG_WALL && 
            (front_charge_dis < 0.2 || getForbidAreaDis(3, 1) < 0.2)) {
        wall_para_.bump_record = ForbidChargeRight;
        forbid_charge = 1;
    }
    else if(!escape_charge && wall_para_.mode == CHARGE_ALONG_WALL && wall_para_.dir = LEFT_ALONG_WALL && 
            (front_charge_dis < 0.2 || getForbidAreaDis(4, 1) < 0.2)) {
        wall_para_.bump_record = ForbidChargeLeft;
        forbid_charge = 1;
    }
    else if(!escape_forbid && !fanFlag && wall_para_.dir = RIGHT_ALONG_WALL &&
            (front_forbid_dis < 0.25 || getForbidAreaDis(4, 0) < 0.3)) {
        wall_para_.bump_record = ForbidFront;
        forbid_charge = 0;
    }
    else if(!escape_forbid && !fanFlag && wall_para_.dir = LEFT_ALONG_WALL &&
            (front_forbid_dis < 0.25 || getForbidAreaDis(3, 0) < 0.3)) {
        wall_para_.bump_record = ForbidFront;
        forbid_charge = 0;
    }
    else if(!escape_forbid && !fanFlag && wall_para_.dir = RIGHT_ALONG_WALL &&
            (front_forbid_dis < 0.25 || getForbidAreaDis(3, 0) < 0.25)) {
        wall_para_.bump_record = ForbidRight;
        forbid_charge = 0;
    }
    else if(!escape_forbid && !fanFlag && wall_para_.dir = RIGHT_ALONG_WALL &&
            (front_forbid_dis < 0.25 || getForbidAreaDis(4, 0) < 0.25)) {
        wall_para_.bump_record = ForbidLeft;
        forbid_charge = 0;
    }
    else if(current_sensor_.leftCliff || current_sensor_.mcuLeftCliff) {
        wall_para_.bump_record = CliffLeft;
    }
    else if(current_sensor_.rightCliff || current_sensor_.mcuRightCliff) {
        wall_para_.bump_record = CliffRight;
    }
    else if(current_sensor_.midCliff || current_sensor_.mcuLeftMidCliff || current_sensor_.mcuRightMidCliff) {
        wall_para_.bump_record = CliffInter;
    }
    else if(current_sensor_.bump) {
        wall_para_.bump_record = Bump;
    }
    else if(!wall_para_.vir_turn_cnt && (_maze.seatPoint.empty() || (!_maze.seatPoint.empty() && 
        sqrtf((((current_pos.realx * 15 / 100) - rechargeX) * ((current_pos.realx * 15 / 100) - rechargeX)) + 
            (((current_pos.realy * 15 / 100) - rechargeY) * ((current_pos.realy * 15 / 100) - rechargeY))) > 2))) {
        if(current_sensor_.leftInfrared) {
            if(wall_para_.dir != RIGHT_ALONG_WALL) {
                wall_para_.bump_record = InsideVirtual;
            }
            else
                wall_para_.bump_record = BumpNo;
        }
        else if(current_sensor_.rightInfrared) {
            if(wall_para_.dir != LEFT_ALONG_WALL) {
                wall_para_.bump_record = InsideVirtual;
            }
            else
                wall_para_.bump_record = BumpNo;
        }
        else if(current_sensor_.leftFrontInfrared || current_sensor_.rightFrontInfrared) {
            wall_para_.bump_record = FrontVirtual;
        }
        else 
        {
            wall_para_.bump_record = BumpNo;
        }
    }
    else 
    {
        wall_para_.bump_record = BumpNo;
    }
    if(small_place_flag == 2 && wall_para_.bump_record == Bump)
        wall_para_.fast_wall_count ++;
    if(wall_para_.fast_wall_count && small_place_flag == 2) {
        if(++wall_para_.fast_wall_count > 1000 / 20)
            wall_para_.fast_wall_count = 1000 / 20;
        else {
            if(wall_para_.bump_record == Bump)
                wall_para_.bump_record = BumpNo;
        }
    }
    if(small_place_flag == 2)
        small_place_flag = 0;
    static int last_wall_bump_state = -1;
    if(last_wall_bump_state != wall_para_.bump_record) {
        FRIZY_LOG(LOG_DEBUG, "Wallbump:%d,  %d", last_wall_bump_state, wall_para_.bump_record);
        last_wall_bump_state = wall_para_.bump_record;
    }
    if(wall_para_.bump_record == in_charge_area)
    {
        chassis.chassisSpeed(0, 0, 1);
        wall_para_.mode = CHARGE_ALONG_WALL;
        wall_para_.bump_flag = 1;
        wall_para_.restricted_zone = 1;
        wall_para_.bump_deal_state = Turn;
        Grid recharge_seat = {(int)round(recharge_x*100/15), (int)round(recharge_y*100/15)};
        Grid cur_point = {current_pos.x, current_pos.y};
        Grid tmp = caculateAngle(cur_point, recharge_seat);
        esc_charge_angle = tmp.forward - 110;
        if(esc_charge_angle <= 0)
        {
            esc_charge_angle = 360 + esc_charge_angle;
        }
        pid_count = 0;
    }
    else if(wall_para_.bump_record == ForbidChargeFront || wall_para_.bump_record == ForbidChargeLeft || wall_para_.bump_record == ForbidChargeRight) {
        chassis.chassisSpeed(0, 0, 1);
        if(wall_para_.bump_record == ForbidChargeFront)
            wall_para_.mode = CHARGE_ALONG_WALL;
        wall_para_.bump_deal_state = Turn;
        wall_para_.bump_flag = 1;
        pid_count = 0;
        if(wall_para_.restricted_zone == 1)
            wall_para_.restricted_zone = 0;
    }
    else if(wall_para_.bump_record == OBSFront || wall_para_.bump_record == OBSLeft || wall_para_.bump_record == OBSRight ||
            wall_para_.bump_record == RadarFront || wall_para_.bump_record == RadarLeft || wall_para_.bump_record == RadarRight) {
        // if(current_sensor_.Infrared || current_sensor_.cliff)   
        chassis.chassisSpeed(0, 0, 1);
        fanFlag = 0;
        wall_para_.bump_deal_state = Turn;
        wall_para_.bump_flag = 1;
        if(wall_para_.mode ==  CHARGE_ALONG_WALL)
            wall_para_.vir_turn_cnt = 0;
        if(wall_para_.bump_record == RadarFront && wall_para_.bump_record == RIGHT_ALONG_WALL/* && (currentSensor.wallData.rightWallDistance < 0.22f)*/) 
            wall_para_.bump_record = BumpSignalBack;
        pid_count = 0;
        if(wall_para_.restricted_zone == 1)
            wall_para_.restricted_zone = 0;
    }
    else if(current_sensor_.bump || current_sensor_.cliff) {
        chassis.chassisSpeed(0, 0, 1);
        wall_para_.bump_deal_state = BumpSignalBack;
        wall_para_.bump_flag = 1;
        fanFlag = 0;
        pid_count = 0;
    }

}
void AlongWall::bumpDeal()
{

}

void AlongWall::wallFollowDeal()
{

}

int AlongWall::wallPid(int now_value, int aim_value)
{
    int out;
    aim_value = 35;
    wall_para_.pid.p = (now_value - aim_value) * 90 / 9;
    wall_para_.pid.d = (wall_para_.pid.p - wall_para_.pid.last_p) * 11;
    wall_para_.pid.last_p = wall_para_.pid.p;
    out = (wall_para_.pid.p + wall_para_.pid.d);
    switch((now_value)
    {
        case 30:    out-=80;    break;
        case 31:	out-=35;    break;
        case 32:	out-=25;    break;
        case 33:	out-=10;    break;
        case 34:	out-=18;    break;
        case 35:	out-=17;    break;
        case 36:	out-=16;    break;
        case 37:	out-=15;    break;
        case 38:	out-=14;    break;
        case 39:	out-=12;    break;
        case 40:	out-=10;    break;
        case 41:	out-=8;     break;
        case 42:	out-=6;     break;
        case 43:	out-=4;     break;
        case 44:	out-=2;     break;
    }
    if(out > 300)
        out = 300;
    else if(out < -300)
        out -= 300;
    printf("p d last_p now_value[%d %d %d %d]\b", wall_para_.pid.p, wall_para_.pid.d, wall_para_.pid.last_p, now_value);

}

bool AlongWall::threadLoop()
{
    if(!along_wall_running_) {
        usleep(100 * 1000);
        return true;
    }
    chassis_.GetSensor(&current_sensor);          
    chassis_.GridPoint(&current_pos);
    if(!wall_para_.bump_flag)
        bumpDetect();
    if(wall_para_.bump_flag)
        bumpDeal();
    else
        wallFollowDeal();
    return true;
    usleep(20 * 1000);
}
