#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cv.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_to_rviz/paramsConfig.h>
#include <fstream>
#include <vector>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

double lu_x = 0, lu_y = 0;
double yaw = -3.14;
double sizeSq = 1;
double z = -10;


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

int main( int argc, char** argv )
{

      ros::init(argc, argv, "image_to_rviz");
      ros::NodeHandle n("~");
      ros::Rate r(2);
      ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      ros::Publisher pub_path1 = n.advertise<nav_msgs::Path>("path1", 1000);
      ros::Publisher pub_path2 = n.advertise<nav_msgs::Path>("path2", 1000);
      ros::Publisher pub_path3 = n.advertise<nav_msgs::Path>("path3", 1000);
      nav_msgs::Path path1, path2, path3;

      dynamic_reconfigure::Server<image_to_rviz::paramsConfig> server;
      dynamic_reconfigure::Server<image_to_rviz::paramsConfig>::CallbackType f;

      f = boost::bind(&callback, _1, _2);
      server.setCallback(f);

      cv::Mat src;
      std::string packagePath = ros::package::getPath("image_to_rviz");
      src = cv::imread(packagePath + "/img/map.png", 1 );
      //cv::resize(src, src, cv::Size(src.cols / 4, src.rows / 4));

      visualization_msgs::Marker image;
      image.header.frame_id = "/world";
      image.header.stamp = ros::Time::now();
      image.ns = "image";
      image.id = 0;
      image.action = visualization_msgs::Marker::ADD;
      image.type = visualization_msgs::Marker::TRIANGLE_LIST;
      image.scale.x = 1;
      image.scale.y = 1;
      image.scale.z = 1;

      double pix;
          geometry_msgs::Point p;
      std_msgs::ColorRGBA crgb;

      double center_x = src.rows / 2.0;
      double center_y = src.cols / 2.0;


      // pub trajectory
      {
            std::string dataPath = packagePath + "/data/vio_loop.csv";
                  // load image list
            FILE* file;
            file = std::fopen(dataPath.c_str() , "r");
            if(file == NULL){
                printf("cannot find file \n");
                ROS_BREAK();
                return 0;          
            }
            std::vector<double> poseX;
            std::vector<double> poseY;
            std::vector<double> poseZ;
            double tmp;
            double x, y, z;
            while (fscanf(file, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf,", 
                  &tmp, &x, &y, &z, &tmp, &tmp, &tmp, &tmp) != EOF)
            {
                  printf("x: %lf, y:%lf, z:%lf \n", x, y, z);
                  poseX.push_back(x);
                  poseY.push_back(y);
                  poseZ.push_back(z);
            }
            std::fclose(file);


            for(int i = 0; i < 2850; i++)
            {

                  geometry_msgs::PoseStamped pose_stamped;
                  pose_stamped.header.stamp = ros::Time::now();
                  pose_stamped.header.frame_id = "world";
                  pose_stamped.pose.position.x = poseX[i];
                  pose_stamped.pose.position.y = poseY[i];
                  pose_stamped.pose.position.z = poseZ[i];
                  path1.header.stamp = ros::Time::now();
                  path1.header.frame_id = "world";
                  path1.poses.push_back(pose_stamped);
                  //pub_path.publish(path);
            }

            for(int i = 2851; i < 5400; i++)
            {

                  geometry_msgs::PoseStamped pose_stamped;
                  pose_stamped.header.stamp = ros::Time::now();
                  pose_stamped.header.frame_id = "world";
                  pose_stamped.pose.position.x = poseX[i];
                  pose_stamped.pose.position.y = poseY[i];
                  pose_stamped.pose.position.z = poseZ[i];
                  path2.header.stamp = ros::Time::now();
                  path2.header.frame_id = "world";
                  path2.poses.push_back(pose_stamped);
                  //pub_path.publish(path);
            }

            for(int i = 5401; i < poseX.size(); i++)
            {

                  geometry_msgs::PoseStamped pose_stamped;
                  pose_stamped.header.stamp = ros::Time::now();
                  pose_stamped.header.frame_id = "world";
                  pose_stamped.pose.position.x = poseX[i];
                  pose_stamped.pose.position.y = poseY[i];
                  pose_stamped.pose.position.z = poseZ[i];
                  path3.header.stamp = ros::Time::now();
                  path3.header.frame_id = "world";
                  path3.poses.push_back(pose_stamped);
                  //pub_path.publish(path);
            }
      }


      while (ros::ok())
      {
            pix = sizeSq / src.rows;
            image.points.clear();
            image.colors.clear();
                  for(int r = 0; r < src.rows; ++r) {
                      for(int c = 0; c < src.cols; ++c) {
                        cv::Vec3b intensity = src.at<cv::Vec3b>(r, c);
                        crgb.r = intensity.val[2] / 255.0;
                        crgb.g = intensity.val[1] / 255.0;
                        crgb.b = intensity.val[0] / 255.0;
                        crgb.a = 1.0;

                        p.z = z;
                        double x, y;
                        x = (r - center_x);
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);
                        
                        x = (r - center_x) + 1;
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x);
                        y = (c - center_y) + 1;
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x) + 1;
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x) + 1;
                        y = (c - center_y) + 1;
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x);
                        y = ((c - center_y) + 1);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);
                        
                      }
                  }

            marker_pub.publish(image);
            pub_path1.publish(path1);
            pub_path2.publish(path2);
            pub_path3.publish(path3);
            ros::spinOnce();
            r.sleep();
            
      }
}