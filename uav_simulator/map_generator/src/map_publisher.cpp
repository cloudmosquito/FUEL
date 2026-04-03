#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

std::string file_name;

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_pub");
  ros::NodeHandle node;

  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud);
  if (status == -1) {
    std::cout << "can't read file." << std::endl;
    return -1;
  }

  // Find range of map
  Eigen::Vector2d mmin(0, 0), mmax(0, 0);
  for (auto pt : cloud) {
    mmin[0] = std::min(mmin[0], double(pt.x));
    mmin[1] = std::min(mmin[1], double(pt.y));
    mmax[0] = std::max(mmax[0], double(pt.x));
    mmax[1] = std::max(mmax[1], double(pt.y));
  }

  // Add ground
  for (double x = mmin[0]; x <= mmax[0]; x += 0.1)
    for (double y = mmin[1]; y <= mmax[1]; y += 0.1) {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";

  while (ros::ok()) {
    ros::Duration(0.3).sleep();
    cloud_pub.publish(msg);
  }

  return 0;
}