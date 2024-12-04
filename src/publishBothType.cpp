#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver2/CustomMsg.h>

class CustomToPointCloudConverter
{
public:
    CustomToPointCloudConverter()
    {
        // ROSノードハンドルの作成
        nh_ = ros::NodeHandle();

        // パブリッシャーの初期化
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/lidar/pc2", 10);

        // サブスクライバーの初期化
        sub_ = nh_.subscribe("/livox/lidar", 10, &CustomToPointCloudConverter::customMsgCallback, this);
    }

    void customMsgCallback(const livox_ros_driver2::CustomMsg::ConstPtr &input)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl_cloud.header.frame_id = input->header.frame_id;
        pcl_cloud.header.stamp = input->header.stamp.toNSec() / 1000;

        for (size_t i = 0; i < input->points.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = input->points[i].x;
            point.y = input->points[i].y;
            point.z = input->points[i].z;
            point.intensity = input->points[i].reflectivity;
            pcl_cloud.points.push_back(point);
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pcl_cloud, output);
        output.header = input->header; // ヘッダーの設定

        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;                // ノードハンドル
    ros::Publisher pub_;                // パブリッシャー
    ros::Subscriber sub_;               // サブスクライバー
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_to_pointcloud_converter");

    CustomToPointCloudConverter converter;
    ros::spin();

    return 0;
}