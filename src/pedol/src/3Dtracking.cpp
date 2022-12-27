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
#include "visualization/include/rviz_path.h"

using namespace std;



// string groundtruth_file = "/home/luo/rosframe_ws/src/pedol/example/groundtruth.txt";


void T_Segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr T_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cl)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    T_Segmentation(cloud_cl);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_cl);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

    // Creating the KdTree object for the search method of the extraction  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08); // 8cm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

        int j = 0;
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = cloud_filtered->header;
        cloud_cluster->sensor_origin_ = cloud_filtered->sensor_origin_;
        cloud_cluster->sensor_orientation_ = cloud_filtered->sensor_orientation_;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        cloud_out = cloud_cluster;
        j++;
    }
    return cloud_out;
}

void T_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float vg_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(vg_size,vg_size, vg_size);
    vg.filter(*cloud_out);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr T_cloud_load(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr T_cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr T_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr T_cloud_rf(new pcl::PointCloud<pcl::PointXYZ>);




namespace pedol_ns {
    class TDTrackingNodeletClass : public nodelet::Nodelet
    {
    private:
        ros::Subscriber sub_livox;
        ros::Subscriber sub_gt;
        ros::Publisher pub_livox;
        ros::Publisher pub_rf;

        Sophus::SE3 pose_est;
        Sophus::SE3 pose_gt;
        Sophus::SE3 T_w_c;

        size_t AgentId_;
        string AgentFrameId;

        RVIZMesh*    drone_pub;
        RVIZFrame* frame_pub_agent;
        RVIZPath* path_pub_gt;
        RVIZPath* path_pub_est;

        pcl::console::TicToc time;

        std::vector<int> indices;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity ();

        Sophus::SE3 T_l_w = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0));

        std::vector<Vector3d> point_for_draw;

        Vector4f centroid_seg;
        Vector4f centroid_rf;
        Vector3d meas_pose_3d;
        Vector3d pred_pose_3d;
        Vector3d pose_3d;


        /*Set up kalman filter*/
        int stateSize = 6;
        int measSize = 3;
        int contrSize = 0;
        unsigned int type = CV_32F;
        cv::KalmanFilter kf; // define kalmanfiter class
        cv::Mat state;                         // [x,y,z,v_x,v_y,v_z]
        cv::Mat meas;                           // [z_x,z_y,z_z]
        //variable for kalman filter
        bool initialized = false;//kf filter initialized
        double ticks = 0;
        double dT;
        /*Set up kalman filter*/
        double time_start, time_end;
        

    public:
        TDTrackingNodeletClass() { ; }
        ~TDTrackingNodeletClass() { ; }
        void onInit()
        {
            ROS_INFO("--------A new node has been created---------");

            //transform to world frame
            Mat3x3 R_w_c;
            R_w_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            Vec3 t_w_c = Vec3(0, 0, 0);
            T_w_c = SE3(R_w_c, t_w_c);

            /*Set up kalman filter*/
            kf.init(stateSize, measSize, contrSize, type); // define kalmanfiter class
            state = cv::Mat(stateSize, 1, type);                         // [x,y,z,v_x,v_y,v_z]
            meas = cv::Mat(measSize, 1, type);                           // [z_x,z_y,z_z]
            cv::setIdentity(kf.transitionMatrix);

            kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
            kf.measurementMatrix.at<float>(0) = 1.0f;
            kf.measurementMatrix.at<float>(7) = 1.0f;
            kf.measurementMatrix.at<float>(14) = 1.0f;

            //Q matrix
            kf.processNoiseCov.at<float>(0) = 1e-2;
            kf.processNoiseCov.at<float>(7) = 1e-2;
            kf.processNoiseCov.at<float>(14) = 1e-2;
            kf.processNoiseCov.at<float>(21) = 5.0f;
            kf.processNoiseCov.at<float>(28) = 5.0f;
            kf.processNoiseCov.at<float>(35) = 5.0f;

            cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(2.0));//R matrix
            /*Set up kalman filter*/



            //----------load file-----------------
            ros::NodeHandle nh = getPrivateNodeHandle();
            if (pcl::io::loadPLYFile("/home/luo/Group_UAV/Lidar_code/rosbag/0908/0908_asc.ply", *T_cloud_rf) < 0)
            {
                PCL_ERROR("Cloudn't read cloud_in file!");
            }
            std::cout << "\nLoaded file "
                      << "0908_asc.ply"
                      << " (" << T_cloud_rf->size() << " points) "  << std::endl;
            pcl::removeNaNFromPointCloud(*T_cloud_rf, *T_cloud_rf, indices);
            // cout<<"cloud_tr size after removeNaN: "<<(*cloud_tr).size()<<endl;
            T_downsample(T_cloud_rf, T_cloud_rf, 35.0);
            std::cout<<"cloud_rf size: "<<T_cloud_rf->points.size()<<endl;

            Eigen::Matrix3d R_l_w;
            R_l_w.block(0, 0, 3, 1) = Vector3d(0.514747,-0.744363,0.425394);
            R_l_w.block(0, 1, 3, 1) = Vector3d(-0.71721,-0.645717,-0.262031);
            R_l_w.block(0, 2, 3, 1) = Vector3d(0.46973,-0.170217,-0.866245);
            
            Vector3d t_l_w = Vector3d(0,0,0.5);
            // SE3 T_l_w = SE3(R_l_w,t_l_w);
            Vector3d p;

            //---------scale---------
            for (int i = 0; i < T_cloud_rf->size(); i++)
            {
                p[0] = T_cloud_rf->points[i].x / 1000.0;
                p[1] = T_cloud_rf->points[i].y / 1000.0;
                p[2] = T_cloud_rf->points[i].z / 1000.0;
                p = R_l_w * p + t_l_w;
                T_cloud_rf->points[i].x = p[0];
                T_cloud_rf->points[i].y = p[1];
                T_cloud_rf->points[i].z = p[2];
            }
            
            pcl::compute3DCentroid(*T_cloud_rf, centroid_rf);

            



            int AgentId;
            nh.getParam("AgentId", AgentId);
            AgentId_ = static_cast<size_t>(AgentId);
            ROS_WARN("Agent %lu init ", AgentId_);
            nh.getParam("AgentFrameId", this->AgentFrameId);
            ROS_INFO("AgentFrameId: %s", this->AgentFrameId.c_str());
            ROS_INFO("AgentId: %lu", AgentId_);

            drone_pub  = new RVIZMesh(nh, "/drone", AgentId_, AgentFrameId); // visualize by mesh
            frame_pub_agent = new RVIZFrame(nh,"/vo_camera_pose","/vo_curr_frame", AgentId_, AgentFrameId);
            path_pub_gt = new RVIZPath(nh,"/gt","livox_frame",1,10000);
            path_pub_est = new RVIZPath(nh,"/est","livox_frame",1,10000);


            pub_livox = nh.advertise<sensor_msgs::PointCloud2>("out", 24000);
            sub_gt = nh.subscribe("/mavros/local_position/odom",10000,&TDTrackingNodeletClass::cb_gt, this);
            pub_rf = nh.advertise<sensor_msgs::PointCloud2>("rf_cloud",24000);
            sub_livox = nh.subscribe<sensor_msgs::PointCloud2>("in", 24000, &TDTrackingNodeletClass::callback, this);
            
            
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg_livox)
        {
            //publish reference pointcloud
            sensor_msgs::PointCloud2 msg_rf;
            pcl::toROSMsg(*T_cloud_rf, msg_rf);
            msg_rf.header.frame_id = "livox_frame";
            pub_rf.publish(msg_rf);


            
            pcl::fromROSMsg(*msg_livox, *T_cloud_load);
            ros::Time tstamp = msg_livox->header.stamp;
            
            pcl::removeNaNFromPointCloud(*T_cloud_load, *T_cloud_ds, indices);

            time_start = ros::Time::now().toSec();
            T_cloud_seg = T_cluster(T_cloud_ds);
            time_end = ros::Time::now().toSec();
            cout << "ms: " << time_end - time_start << endl;

            // E_Segmentation(E_cloud_ds);

            point_for_draw.clear();
            for(int i = 0; i<T_cloud_seg->points.size(); i++)
            {
                Vector3d p;
                p[0] = T_cloud_ds->points[i].x;
                p[1] = T_cloud_ds->points[i].y;
                p[2] = T_cloud_ds->points[i].z;
                point_for_draw.push_back(p);
            }

            if (T_cloud_seg->points.size()>=1)
            {
                
                sensor_msgs::PointCloud2 msg_out;
                pcl::toROSMsg(*T_cloud_seg, msg_out);
                pub_livox.publish(msg_out);

                pcl::compute3DCentroid(*T_cloud_seg,centroid_seg);
                meas_pose_3d = Vector3d((centroid_seg-centroid_rf)[0],(centroid_seg-centroid_rf)[1],(centroid_seg-centroid_rf)[2]);

                /*********************kalman filter************************/
                double precTick = ticks;
                ticks = (double)cv::getTickCount();

                dT = (ticks - precTick) / cv::getTickFrequency(); // seconds
                if (initialized)
                {
                    // >>>> Matrix A
                    kf.transitionMatrix.at<float>(3) = dT;
                    kf.transitionMatrix.at<float>(10) = dT;
                    kf.transitionMatrix.at<float>(17) = dT;

                    state = kf.predict();
                    pred_pose_3d[0] = state.at<float>(0);
                    pred_pose_3d[1] = state.at<float>(1);
                    pred_pose_3d[2] = state.at<float>(2);
                }

                meas.at<float>(0) = meas_pose_3d[0];
                meas.at<float>(1) = meas_pose_3d[1];
                meas.at<float>(2) = meas_pose_3d[2];

                if (!initialized) // First detection!
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(7) = 1; // px
                    kf.errorCovPre.at<float>(14) = 1;
                    kf.errorCovPre.at<float>(21) = 1;
                    kf.errorCovPre.at<float>(28) = 1; // px
                    kf.errorCovPre.at<float>(35) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);
                    state.at<float>(1) = meas.at<float>(1);
                    state.at<float>(2) = meas.at<float>(2);
                    state.at<float>(3) = 0;
                    state.at<float>(4) = 0;
                    state.at<float>(5) = 0;
                    // <<<< Initialization

                    kf.statePost = state;
                    pose_3d[0] = state.at<float>(0);
                    pose_3d[1] = state.at<float>(1);
                    pose_3d[2] = state.at<float>(2);
                    initialized = true;
                }
                else
                {
                    kf.correct(meas); // Kalman Correction                    

                    pose_3d[0] = state.at<float>(0);
                    pose_3d[1] = state.at<float>(1);
                    pose_3d[2] = state.at<float>(2);
                }
                 /*********************kalman filter************************/

                pose_est = SE3(Matrix3d::Identity(),pose_3d);
                drone_pub->PubT_w_i(pose_est, tstamp, AgentId_);
                frame_pub_agent->pubFramePtsPoseT_c_w(point_for_draw,
                                              T_l_w,
                                              tstamp);
                path_pub_est->pubPathT_w_c(T_w_c * pose_est, msg_livox->header.stamp);
            }
            
        }

        

        void cb_gt(const nav_msgs::Odometry::ConstPtr& msg_gt)
        {
            Quaterniond q_w_gt;
            Vec3 pos_w_gt;
            q_w_gt = Quaterniond(msg_gt->pose.pose.orientation.w, msg_gt->pose.pose.orientation.x,
                                msg_gt->pose.pose.orientation.y, msg_gt->pose.pose.orientation.z);
            pos_w_gt = Vec3(msg_gt->pose.pose.position.x, msg_gt->pose.pose.position.y, msg_gt->pose.pose.position.z);
            pose_gt = SE3(q_w_gt, pos_w_gt);
            path_pub_gt->pubPathT_w_c(T_w_c * pose_gt, msg_gt->header.stamp);
        }
    };
}

PLUGINLIB_EXPORT_CLASS(pedol_ns::TDTrackingNodeletClass, nodelet::Nodelet)