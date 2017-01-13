/**
 * @file object_localizer_client_example.cpp
 * @brief Example demonstrating the usage of the LocalizeObjects service.
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2016
 */

#include <eigen_conversions/eigen_msg.h>
#include <object_recognition_node/object_localizer_service.h>
#include <perception_utils/pcl_typedefs.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

using std::vector;
using std::string;
using namespace sbpl_perception;

#define MAX_X 0.17
#define MAX_Y 0.17
#define MAX_Z 0.85

#define MIN_X -0.2
#define MIN_Y -0.2
#define MIN_Z 0.55

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_localizer_client_node");
  // The camera pose and preprocessed point cloud, both in world frame.
  Eigen::Isometry3d camera_pose;
  /*camera_pose.matrix() <<
                       0.00974155,   0.997398, -0.0714239,  -0.031793,
                                     -0.749216,  -0.040025,  -0.661116,   0.743224,
                                     -0.662254,  0.0599522,   0.746877,   0.878005,
                                     0,          0,          0,          1;*/
  camera_pose.setIdentity();

  const string demo_pcd_file = ros::package::getPath("sbpl_perception") +
                               "/demo/crop.pcd";
  // Objects for storing the point clouds.
  pcl::PointCloud<PointT>::Ptr cloud_in(new PointCloud);

  // Read the input PCD file from disk.
  if (pcl::io::loadPCDFile<PointT>(demo_pcd_file.c_str(),
                                   *cloud_in) != 0) {
    std::cerr << "Could not find demo PCD file!" << endl;
    return -1;
  }

  cloud_in->header.frame_id = "world";
  //std::cout << "PCD height: " << cloud_in->height << std::endl;
  //std::cout << "PCD width: " << cloud_in->width << std::endl;
  //float minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN, minZ = INT_MAX, maxZ = INT_MIN;

  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<object_recognition_node::LocalizeObjects>("object_localizer_service");
  object_recognition_node::LocalizeObjects srv;
  auto &req = srv.request;
  req.x_min = MIN_X;
  req.x_max = MAX_X;
  req.y_min = MIN_Y;
  req.y_max = MAX_Y;
  req.support_surface_height = 0.0;
  //req.object_ids = vector<string>({"tilex_spray", "tide", "glass_7"});
  //req.object_ids = vector<string>({"tilex_spray", "tide", "glass_7", "vf_paper_bowl"});
  req.object_ids = vector<string>({"bowl_1"});
  tf::matrixEigenToMsg(camera_pose.matrix(), req.camera_pose);
  pcl::toROSMsg(*cloud_in, req.input_organized_cloud);

  if (client.call(srv)) {
    ROS_INFO("Episode Statistics\n");

    for (size_t ii = 0; ii < srv.response.stats_field_names.size(); ++ii) {
      ROS_INFO("%s: %f", srv.response.stats_field_names[ii].c_str(), srv.response.stats[ii]);
    }

    ROS_INFO("Model to scene object transforms:");

    for (size_t ii = 0; ii < req.object_ids.size(); ++ii) {

      Eigen::Matrix4d pose(srv.response.object_transforms[ii].data.data());
      Eigen::Affine3d object_transform;
      // Transpose to convert column-major raw data initialization to row-major.
      object_transform.matrix() = pose.transpose();

      ROS_INFO_STREAM("Object: " << req.object_ids[ii] << std::endl << object_transform.matrix() << std::endl << std::endl);
    }
  } else {
    ROS_ERROR("Failed to call the object localizer service");
    return 1;
  }

  return 0;
}

