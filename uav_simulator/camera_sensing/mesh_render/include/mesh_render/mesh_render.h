#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <open3d/Open3D.h>
#include <open3d/visualization/rendering/Model.h>
#include <open3d/visualization/rendering/filament/FilamentEngine.h>
#include <open3d/visualization/rendering/filament/FilamentRenderer.h>

using namespace open3d;
using namespace open3d::visualization::gui;
using namespace open3d::visualization::rendering;
using namespace std;
class MeshRender {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MeshRender> Ptr;
  typedef std::shared_ptr<const MeshRender> ConstPtr;

  MeshRender();
  ~MeshRender();

  int initialize(ros::NodeHandle &nh);

  void odometryCallback(const nav_msgs::OdometryConstPtr &msg,int i);
  void renderCallback(const ros::TimerEvent &event);

private:
  template <typename Scalar>
  void xmlRpcToEigen(XmlRpc::XmlRpcValue &xml_rpc, Eigen::Matrix<Scalar, 4, 4> *eigen);
  template <typename Scalar>
  bool getTransformationMatrixfromConfig(ros::NodeHandle &nh, Eigen::Matrix<Scalar, 4, 4> *T,
                                         const std::string &param_name);

  bool saveImage(const std::string &filename, const std::shared_ptr<geometry::Image> &image);

  vector<ros::Publisher> color_image_pub_, depth_image_pub_, sensor_pose_pub_;
  vector<ros::Subscriber> pose_sub_;

  ros::Timer render_timer;

  bool enable_color_;
  bool enable_depth_;

  std::string map_file_;
  double scale_;

  // Camera intrinsics
  double fx_, fy_, cx_, cy_;
  int width_, height_;
  double min_depth_, max_depth_;
  
  double render_rate_;

  //&&&&&&&&&&&&&&&&&&&&
  int drone_num_;

  // Coordinate system: camera, body, world, map
  // Input mesh file (map) may have different coordinate system with uav simulator (world)
  // e.g. complex_office.stl: x-right, y-up, z-backward, SolidWorks coordinate system
  Eigen::Matrix4d T_b_c_,  T_w_c_;
  std::vector<Eigen::Matrix4d> T_w_b_;
  Eigen::Matrix4d T_m_w_, T_m_b_, T_m_c_;
  std::vector<ros::Time> latest_odometry_timestamp_;

  // Open3D elements
  open3d::visualization::rendering::TriangleMeshModel model;
  FilamentRenderer *renderer;
  Open3DScene *scene;

  std::vector<bool> init_odom_;

  // Debug output
  bool verbose_;
};