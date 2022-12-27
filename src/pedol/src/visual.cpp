#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

#include <unordered_set>

#include <algorithm>

#include <string>
#include <vector>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/ia_fpcs.h> // 4PCS算法
#include <pcl/surface/mls.h>//mls

#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#include "visualization/include/rviz_frame.h"
#include "visualization/include/rviz_mesh.h"

using namespace std;


// string groundtruth_file = "/home/luo/rosframe_ws/src/pedol/example/groundtruth.txt";



void V_Segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PassThrough<pcl::PointXYZ> pt;
    // Filter in z direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-0.5f, 3.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in x direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(-1.0f, 7.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // Filter in y direction
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-2.0f, 2.0f);
    pt.setFilterLimitsNegative(false);
    pt.filter(*cloud_in);
    // cout<<"after segement size: "<<cloud_in->points.size()<<endl;
}

void V_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float vg_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(vg_size,vg_size, vg_size);
    vg.filter(*cloud_out);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr V_cloud_load(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr V_cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr V_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr V_cloud_rf(new pcl::PointCloud<pcl::PointXYZ>);



namespace pedol_ns {
    class VisualNodeletClass : public nodelet::Nodelet
    {
    private:
        ros::Subscriber sub_livox;
        ros::Subscriber sub_gt_lidar;
        ros::Subscriber sub_gt_uav;

        ros::Publisher pub_livox;
        ros::Publisher pub_rf;

        Sophus::SE3 pose;

        size_t AgentId_;
        string AgentFrameId;

        RVIZMesh*    drone_pub;
        RVIZFrame* frame_pub_agent;

        pcl::console::TicToc time;

        std::vector<int> indices;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity ();

        std::vector<Vector3d> point_for_draw;
        Eigen::Quaterniond q;
        Eigen::Matrix3d R_w_l;
        Vector3d t_w_l;
        Vector3d offset_w_l = Vector3d(-0.08, 0.15, 0.18);
        

    public:
        VisualNodeletClass() { ; }
        ~VisualNodeletClass() { ; }
        virtual void onInit()
        {
            ROS_INFO("--------A new node has been created---------");
            //6D pose of livox frame in world coordinate
            q = Eigen::Quaterniond(0.7078744910776736, -0.34103999507254684, -0.6155129277130567, -0.061231221289006724);
            R_w_l = q.normalized().toRotationMatrix();
            t_w_l = Vector3d(-3.2542568088986417, -0.05063836406141371, 0.7125622094255287);

            //----------load file-----------------
            ros::NodeHandle nh = getMTPrivateNodeHandle();
            if (pcl::io::loadPLYFile("/home/luo/Group_UAV/Lidar_code/rosbag/0908/0908_asc.ply", *V_cloud_rf) < 0)
            {
                PCL_ERROR("Cloudn't read cloud_in file!");
            }
            std::cout << "\nLoaded file "
                      << "0908_asc.ply"
                      << " (" << V_cloud_rf->size() << " points) "  << std::endl;
            pcl::removeNaNFromPointCloud(*V_cloud_rf, *V_cloud_rf, indices);
            // cout<<"cloud_tr size after removeNaN: "<<(*cloud_tr).size()<<endl;
            V_downsample(V_cloud_rf, V_cloud_rf, 35.0);
            std::cout<<"cloud_rf size: "<<V_cloud_rf->points.size()<<endl;

            Eigen::Matrix3d R_l_rf;
            R_l_rf.block(0, 0, 3, 1) = Vector3d(0.514747,-0.744363,0.425394);
            R_l_rf.block(0, 1, 3, 1) = Vector3d(-0.71721,-0.645717,-0.262031);
            R_l_rf.block(0, 2, 3, 1) = Vector3d(0.46973,-0.170217,-0.866245);
            
            Vector3d t_l_rf = Vector3d(0,0,0.5);
            // SE3 T_l_w = SE3(R_l_w,t_l_w);
            Vector3d p;

            //---------scale---------
            for (int i = 0; i < V_cloud_rf->size(); i++)
            {
                p[0] = V_cloud_rf->points[i].x / 1000.0;
                p[1] = V_cloud_rf->points[i].y / 1000.0;
                p[2] = V_cloud_rf->points[i].z / 1000.0;
                p = R_l_rf * p + t_l_rf;
                // p = R_w_l.inverse() * p - t_w_l;
                V_cloud_rf->points[i].x = p[0];
                V_cloud_rf->points[i].y = p[1];
                V_cloud_rf->points[i].z = p[2];
            }
            


            int AgentId;
            nh.getParam("AgentId", AgentId);
            AgentId_ = static_cast<size_t>(AgentId);
            ROS_WARN("Agent %lu init ", AgentId_);
            nh.getParam("AgentFrameId", this->AgentFrameId);
            ROS_INFO("AgentFrameId: %s", this->AgentFrameId.c_str());
            ROS_INFO("AgentId: %lu", AgentId_);

            drone_pub  = new RVIZMesh(nh, "/drone", AgentId_, AgentFrameId); // visualize by mesh
            frame_pub_agent = new RVIZFrame(nh,"/vo_camera_pose","/vo_curr_frame", AgentId_, AgentFrameId);

            pub_livox = nh.advertise<sensor_msgs::PointCloud2>("out", 24000);
            pub_rf = nh.advertise<sensor_msgs::PointCloud2>("rf_cloud",24000);

            sub_gt_uav = nh.subscribe("/mavros/local_position/odom",10000,&VisualNodeletClass::cb_gt_uav, this);
            sub_livox = nh.subscribe<sensor_msgs::PointCloud2>("in", 24000, &VisualNodeletClass::callback, this);
            
            
        }

        void cb_gt_uav(const nav_msgs::Odometry::ConstPtr& msg_gt)
        {
            //publish gt path
            Quaterniond q_w_gt;
            Vec3 pos_w_gt;
            q_w_gt = Quaterniond(msg_gt->pose.pose.orientation.w, msg_gt->pose.pose.orientation.x,
                                msg_gt->pose.pose.orientation.y, msg_gt->pose.pose.orientation.z);
            pos_w_gt = Vec3(msg_gt->pose.pose.position.x, msg_gt->pose.pose.position.y, msg_gt->pose.pose.position.z);
            pose = SE3(q_w_gt, pos_w_gt);
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_livox)
        {
            //publish reference pointcloud
            sensor_msgs::PointCloud2 msg_rf;
            pcl::toROSMsg(*V_cloud_rf, msg_rf);
            msg_rf.header.frame_id = "livox_frame";
            pub_rf.publish(msg_rf);
            
            //publish segmented point cloud
            pcl::fromROSMsg(*msg_livox, *V_cloud_load);
            ros::Time tstamp = msg_livox->header.stamp;

            

            pcl::removeNaNFromPointCloud(*V_cloud_load, *V_cloud_ds, indices);
            V_Segmentation(V_cloud_ds);
            //---------transform to world frame---------
            for (int i = 0; i < V_cloud_ds->size(); i++)
            {
                Vector3d temp_p;
                temp_p[0] = V_cloud_ds->points[i].x;
                temp_p[1] = V_cloud_ds->points[i].y;
                temp_p[2] = V_cloud_ds->points[i].z;
                temp_p =  temp_p + t_w_l + offset_w_l;
                V_cloud_ds->points[i].x = temp_p[0];
                V_cloud_ds->points[i].y = temp_p[1];
                V_cloud_ds->points[i].z = temp_p[2];
                // cout<<"R_w_l: \n"<<R_w_l.transpose()<<endl;
                // cout<<"t_w_l: \n"<<t_w_l<<endl;
            }
            sensor_msgs::PointCloud2 msg_out;
            pcl::toROSMsg(*V_cloud_ds, msg_out);
            pub_livox.publish(msg_out);

            drone_pub->PubT_w_i(pose, tstamp, AgentId_);
            

        }

        

        
    };
}

PLUGINLIB_EXPORT_CLASS(pedol_ns::VisualNodeletClass, nodelet::Nodelet)