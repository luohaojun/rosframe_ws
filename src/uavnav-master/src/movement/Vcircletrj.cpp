#include "Vcircletrj.h"
#include "trj_config.h"
#include "stdio.h"

using namespace std;

VcircleTrj::VcircleTrj()
{
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}
VcircleTrj::VcircleTrj(double time,
                     double start_x,  double start_y, double start_z,
                     double center_x, double center_y, double center_z,
                     double turn_rad, double duration, int facing)
{
    this->start_time = time;
    this->startx = start_x;
    this->starty = start_y;
    this->startz = start_z;
    this->centerx = center_x;
    this->centery = center_y;
    this->centerz = center_z;
    this->startyaw = atan2(starty-centery,startz-centerz);
    double dx=startx-centerx;
    double dy=starty-centery;
    double dz=startz-centerz;
    this->r = sqrt(dx*dx+dy*dy+dz*dz);
    this->est_t = duration;
    this->length_rad = turn_rad;
//    cout << "radious "  << r << endl;
//    cout << "startyaw " << startyaw << endl;
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
    facingsetup = facing;
}

void VcircleTrj::getPose(double time,
                        geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    if(est_flag == 0)
    {
        if(est_t>0.0 && est_t<1000.0)
        {
            av = length_rad / est_t;
            //cout << "angular velocity " << av << endl;
            est_flag = 1;
        }
    }
    if(est_flag == 1)
    {
        double dt=curr_time-start_time;
        double x=centerx;
        double y=(sin(startyaw+(dt*av)))*r+centery;
        double z=(cos(startyaw+(dt*av)))*r+centerz;
        Quaterniond q;
        if(facingsetup==CIRCLE_TRJ_FACING_CENTER){
        //   double yaw = atan2(centery-y,centerz-z);
          double yaw = 0;
          q = rpy2Q(Vec3(0,0,yaw));
        }
        if(facingsetup==CIRCLE_TRJ_FACING_FIXED){
          double yaw = 0;
          q = rpy2Q(Vec3(0,0,yaw));
        }
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
}

void VcircleTrj::getEnding(double &x, double &y, double &z, double &yaw)
{
    x=centerx;
    y=(sin(startyaw+length_rad))*r+centery;
    z=(cos(startyaw+length_rad))*r+centerz;
    yaw = atan2(centery-y,centerz-z);
}

int VcircleTrj::finished()
{
    if((curr_time-start_time)>=est_t)
    {return 1;}
    else
    {return 0;}
}

void VcircleTrj::setVerticalVelocityLimit(double v)
{
    vv_limit = v;
}

void VcircleTrj::setHorizonVelocityLimit(double v)
{
    hv_limit = v;
}

void VcircleTrj::setAngularSpeedRadLIMIT(double w)
{
    av_limit = w;
}
