//SYSTEM HEAD FILE
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <fstream>

//ROS HEAD FILE
#include <ros/ros.h>
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

//USER HEAD FILE
#include </home/luo/rosframe_ws/src/pedol/include/pedol/common.h>
#include </home/luo/rosframe_ws/src/pedol/src/visualization/include/rviz_path.h>

using namespace std;


namespace pedol_ns{
    class PathDispNodeleteClass : public nodelet::Nodelet
    {
        private:
        SE3 T_w_c;
        ros::Subscriber pose_sub;
        RVIZPath* path_pub;
        SE3 pose;
        size_t AgentId_;

        public:
        PathDispNodeleteClass(){ ; }
        ~PathDispNodeleteClass(){ ; }
        virtual void onInit()
        {
            ROS_INFO("--------A new node has been created---------");
            ros::NodeHandle nh = getPrivateNodeHandle();

            //transform to world frame
            Mat3x3 R_w_c;
            R_w_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            Vec3 t_w_c = Vec3(0, 0, 0);
            T_w_c = SE3(R_w_c, t_w_c);

            int AgentId;
            nh.getParam("AgentId", AgentId);
            AgentId_ = static_cast<size_t>(AgentId);
            ROS_WARN("Agent %lu init ", AgentId_);
            ROS_INFO("AgentId: %lu", AgentId_);

            path_pub = new RVIZPath(nh,"/gt","livox_frame",1,10000);
            pose_sub = nh.subscribe("/mavros/local_position/odom",10000,&PathDispNodeleteClass::cb_gt, this);

            
        }
        void cb_gt(const nav_msgs::Odometry::ConstPtr &msg_gt)
        {

            Quaterniond q_w_gt;
            Vec3 pos_w_gt;
            q_w_gt = Quaterniond(msg_gt->pose.pose.orientation.w, msg_gt->pose.pose.orientation.x,
                                 msg_gt->pose.pose.orientation.y, msg_gt->pose.pose.orientation.z);
            pos_w_gt = Vec3(msg_gt->pose.pose.position.x, msg_gt->pose.pose.position.y, msg_gt->pose.pose.position.z);
            pose = SE3(q_w_gt, pos_w_gt);
            path_pub->pubPathT_w_c(T_w_c * pose, msg_gt->header.stamp);
        }
    };

}

PLUGINLIB_EXPORT_CLASS(pedol_ns::PathDispNodeleteClass, nodelet::Nodelet)