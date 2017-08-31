#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>

#include <iostream>
#include <vector>

void newCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  // Convert from ros point cloud to pcl point cloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloudMsg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  // Extract clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05);
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  static tf2_ros::TransformBroadcaster br;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header = cloudMsg->header;

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // Iterate through all of the clusters and find the centroid of each one
  int n = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

    pcl::PointXYZ centroid;
	pcl::CentroidPoint<pcl::PointXYZ> centroid_calc;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	  centroid_calc.add(cloud->points[*pit]);
    centroid_calc.get(centroid);
  
	std::stringstream ss;
	ss << "cluster_" << n;
	transformStamped.child_frame_id = ss.str();
	transformStamped.transform.translation.x = centroid.x;
	transformStamped.transform.translation.y = centroid.y;
	transformStamped.transform.translation.z = centroid.z;

	ROS_DEBUG("Centroid of cluster %d: %f %f %f", n, centroid.x, centroid.y, centroid.z);

    br.sendTransform(transformStamped);
  	n++;
  }	
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_segmentation");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("cloud", 1, newCloud);

    ros::spin();
}

