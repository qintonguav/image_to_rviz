#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cv.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_to_rviz/paramsConfig.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


using namespace Eigen;

double lu_x, lu_y;
double yaw;
double sizeSq;
double z;
Matrix3d qic;
Vector3d tic;
cv::Mat src;
ros::Publisher marker_pub;

void callback(image_to_rviz::paramsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf", 
            config.size_param, config.dx_param, 
            config.dy_param, 
            config.yaw,
            config.z);

  lu_x = config.dx_param;
  lu_y = config.dy_param;
  yaw = config.yaw;
  sizeSq = config.size_param;
  z = config.z;
};

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
      // build keyframe
      Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                            pose_msg->pose.pose.position.y,
                            pose_msg->pose.pose.position.z);
      Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                               pose_msg->pose.pose.orientation.x,
                               pose_msg->pose.pose.orientation.y,
                               pose_msg->pose.pose.orientation.z).toRotationMatrix();
      T = T + R * tic;
      R = R * qic;

      visualization_msgs::Marker image;
      image.header = pose_msg->header;
      image.ns = "image";
      image.id = 0;
      image.action = visualization_msgs::Marker::ADD;
      image.type = visualization_msgs::Marker::TRIANGLE_LIST;
      image.scale.x = 1;
      image.scale.y = 1;
      image.scale.z = 1;

      lu_x = 0;
      lu_y = 0;
      yaw = -3.14;
      sizeSq = 1;
      z = -10;

      double pix;
      geometry_msgs::Point p;
      std_msgs::ColorRGBA crgb;

      double center_x = src.rows / 2.0;
      double center_y = src.cols / 2.0;

      pix = sizeSq / src.rows;
      image.points.clear();
      image.colors.clear();
      double scale = 0.05;

      for(int r = 0; r < src.rows; ++r) {
          for(int c = 0; c < src.cols; ++c) {
            cv::Vec3b intensity = src.at<cv::Vec3b>(r, c);
            crgb.r = intensity.val[2] / 255.0;
            crgb.g = intensity.val[1] / 255.0;
            crgb.b = intensity.val[0] / 255.0;
            crgb.a = 1.0;

            Vector3d p_cam, p_w;
            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0);
            p.y = p_w(1);
            p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);
          }
      }
      marker_pub.publish(image);
}

int main( int argc, char** argv )
{

      ros::init(argc, argv, "image_to_rviz");
      ros::NodeHandle n("~");
      ros::Rate r(10);
      qic << 0, -1, 0,
               1,  0, 0,
               0,  0, 1;
      tic << -0.02, -0.06, 0.01;




      dynamic_reconfigure::Server<image_to_rviz::paramsConfig> server;
      dynamic_reconfigure::Server<image_to_rviz::paramsConfig>::CallbackType f;

      f = boost::bind(&callback, _1, _2);
      server.setCallback(f);

      std::string packagePath = ros::package::getPath("image_to_rviz");
      src = cv::imread(packagePath + "/img/sea_front.png", 1 );
      cv::resize(src, src, cv::Size(80, 60));

      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
      
      ros::spin();

}