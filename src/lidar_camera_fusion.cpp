#include "lidar_camera_fusion.h"

#include <stdio.h>

#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include "livox_ros_driver/CustomMsg.h"
#include "io_utils.h"

// #define IS_DEBUG

LidarCameraFusion::LidarCameraFusion() {
  ROS_INFO("------------ intialize ----------\n");
  raw_lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void LidarCameraFusion::LoadInitParametersPy(InitialParametersStruct* init_params) {
  std::string lidar_path(init_params->lidar_address);
  bool lidar_type;
  if (std::string(init_params->lidar_type) == std::string("true")) {
    lidar_type = true;
  } else {
    lidar_type = false;
  }
  std::string img_path(init_params->img_address);
  std::cout << "lidar_path = " << lidar_path << ", "
            << "lidar_type = " << lidar_type << ", "
            << "img_path = " << img_path << std::endl;
  if (!IsFileExist(lidar_path.c_str()) || !IsFileExist(img_path.c_str())) {
    std::cout << "Not Found" << std::endl;
  } else {
    RegisterImage(img_path);
    RegisterLidar(lidar_path, lidar_type);
  }
}

ImageStruct* LidarCameraFusion::LoadCalibParametersPy(CalibrationParametersStruct* calib_params) {
  fx_ = calib_params->fx;
  fy_ = calib_params->fy;
  cx_ = calib_params->cx;
  cy_ = calib_params->cy;
  k1_ = calib_params->k1;
  k2_ = calib_params->k2;
  k3_ = 0.0f;
  p1_ = calib_params->p1;
  p2_ = calib_params->p2;
  tx_ = calib_params->tx;
  ty_ = calib_params->ty;
  tz_ = calib_params->tz;
  roll_ = calib_params->roll;
  pitch_ = calib_params->pitch;
  yaw_ = calib_params->yaw;
  #ifdef IS_DEBUG
  std::cout << "fx = " << fx_ << ", " << "fy = " << fy_ << ", "
            << "cx = " << cx_ << ", " << "cy = " << cy_ << ", "
            << "k1 = " << k1_ << ", " << "k2 = " << k2_ << ", " << "k3 = " << k3_  << ", "
            << "p1 = " << p1_ << ", " << "p2 = " << p2_ << ", "
            << "tx = " << tx_ << ", " << "ty = " << ty_ << ", " << "tz = " << tz_ << ", "
            << "roll = " << roll_ << ", " << "pitch = " << pitch_ << ", " << "yaw = " << yaw_ << std::endl; 
  std::cout << "row = " << image_.rows << ", " << "col = " << image_.cols << std::endl;
  #endif
  Vector6d params;
  params[0] = yaw_;
  params[1] = pitch_;
  params[2] = roll_;
  params[3] = tx_;
  params[4] = ty_;
  params[5] = tz_;
  cv::Mat img = GetProjectionImg(params);
  int row = img.rows;
  int col = img.cols;
  int channel = img.channels();
  ImageStruct* img_struct = new ImageStruct;
  img_struct->row = row;
  img_struct->col = col;
  img_struct->channel = channel;
  for(int i = 0; i < row; ++i) {
    for(int j = 0; j < col; ++j) {
      for(int k = 0; k < channel; ++k) {
        img_struct->img[i * (col * channel) + j * channel + k] = img.data[i * (col * channel) + j * channel + k];
      }
    }
  }
  return img_struct;
}

void LidarCameraFusion::RegisterImage(const std::string& img_dir) {
  image_ = cv::imread(img_dir, 1);
  width_ = image_.cols;
  height_ = image_.rows;
  std::cout << "in RegisterImage: ROW, COL = " << height_ << ", " << width_ << std::endl;
}

void LidarCameraFusion::RegisterLidar(const std::string& lidar_dir, bool CUSTOM_MSG) {
  // reference: https://github.dev/hku-mars/livox_camera_calib
  // CustomMsg: intensity整数部分是强度，小数部分e->int(1 / e - 1 + 0.5)是线数
  // PointCloud2: intensity是强度
  ROS_INFO("------------ START GET LIDAR %s! ------------", lidar_dir.c_str());
  rosbag::Bag in_bag;
  in_bag.open(lidar_dir, rosbag::bagmode::Read); 
  std::vector<std::string> topics; 
  topics.push_back(std::string("/livox/lidar"));   // pointcloud2
  topics.push_back(std::string("/livox/lidar/")); // livox_ros_driver:customMsg
  rosbag::View view_in_bag(in_bag, rosbag::TopicQuery(topics));
  int frame_num = -1;
  for(rosbag::MessageInstance const msg: view_in_bag) 
  {
      frame_num++;
      if (CUSTOM_MSG){
        livox_ros_driver::CustomMsgPtr livox_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg!= NULL)
        {
          for (int i = 0; i < livox_msg->point_num; ++i) {
            if (livox_msg->points[i].tag != 16) {
                continue;
            }
            pcl::PointXYZI pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            float scan = livox_msg->points[i].line;
            pt.intensity = livox_msg->points[i].reflectivity + 1.0 / (scan + 1.0);
            if(pt.x == 0 && pt.y == 0 && pt.z == 0) {
              continue;
            }
            raw_lidar_cloud_->push_back(pt);
          }
        }
      }
      else{
        sensor_msgs::PointCloud2ConstPtr livox_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
        pcl::fromROSMsg(*livox_msg, *raw_pcl_ptr);
        int size = raw_pcl_ptr->points.size();
        if (livox_msg!= NULL)
        {
            for (int i = 0; i < size; ++i) {
                pcl::PointXYZI pt;
                pt.x = raw_pcl_ptr->points[i].x;
                pt.y = raw_pcl_ptr->points[i].y;
                pt.z = raw_pcl_ptr->points[i].z;
                pt.intensity = raw_pcl_ptr->points[i].intensity;
                if(pt.x == 0 && pt.y == 0 && pt.z == 0){continue;}
                raw_lidar_cloud_->push_back(pt);
            }
        }
      }
  }
  ROS_INFO("frame num = %d", frame_num);
  ROS_INFO("Before Uniform Sampling, lidar point cloud num = %d", raw_lidar_cloud_->points.size());
  pcl::UniformSampling<pcl::PointXYZI> sor;
  sor.setInputCloud(raw_lidar_cloud_);
  sor.setRadiusSearch(0.006f);
  sor.filter(*raw_lidar_cloud_);
  ROS_INFO("After Uniform Sampling, lidar point cloud num = %d", raw_lidar_cloud_->points.size());
  in_bag.close();
}

cv::Mat LidarCameraFusion::GetProjectionImg(const Vector6d& extrinsic_params) {
  cv::Mat depth_projection_img = cv::Mat::zeros(height_, width_, CV_8UC1);
  ProjectLidar2Img(extrinsic_params, raw_lidar_cloud_, INTENSITY, false,
             depth_projection_img);
  cv::Mat map_img = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (int x = 0; x < map_img.cols; x++) {
    for (int y = 0; y < map_img.rows; y++) {
      uint8_t r, g, b;
      float norm = depth_projection_img.at<uchar>(y, x) / 256.0;
      MapJet(norm, 0, 1, &r, &g, &b);
      map_img.at<cv::Vec3b>(y, x)[0] = b;
      map_img.at<cv::Vec3b>(y, x)[1] = g;
      map_img.at<cv::Vec3b>(y, x)[2] = r;
    }
  }
  cv::Mat merge_img = cv::Mat::zeros(height_, width_, CV_8UC3);
  merge_img = 0.5 * map_img + 0.8 * image_;
  return merge_img;
}

void LidarCameraFusion::ProjectLidar2Img(
    const Vector6d& extrinsic_params,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud,
    const ProjectionType projection_type,
    bool is_fill_img,
    cv::Mat& projection_img) {
  std::vector<cv::Point3f> pts_3d;
  std::vector<float> intensity_list;
  Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  for (size_t i = 0; i < lidar_cloud->size(); i++) {
    pcl::PointXYZI point_3d = lidar_cloud->points[i];
    float depth =
        sqrt(pow(point_3d.x, 2) + pow(point_3d.y, 2) + pow(point_3d.z, 2));
    if (depth > min_depth_ && depth < max_depth_) {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      intensity_list.emplace_back(lidar_cloud->points[i].intensity);
    }
  }
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
              rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
              rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  cv::Mat image_project = cv::Mat::zeros(height_, width_, CV_16UC1);
  cv::Mat rgb_image_project = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    cv::Point2f point_2d = pts_2d[i];
    if (point_2d.x <= 0 || point_2d.x >= width_ || point_2d.y <= 0 ||
        point_2d.y >= height_) {
      continue;
    } else {
      // test depth and intensity both
      if (projection_type == DEPTH) {
        float depth = sqrt(pow(pts_3d[i].x, 2) + pow(pts_3d[i].y, 2) +
                           pow(pts_3d[i].z, 2));
        float intensity = intensity_list[i];
        float depth_weight = 1;
        float grey = depth_weight * depth / max_depth_ * 65535 +
                     (1 - depth_weight) * intensity / 150 * 65535;
        if (image_project.at<ushort>(point_2d.y, point_2d.x) == 0) {
          image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
          rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
              depth / max_depth_ * 255;
          rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
              intensity / 150 * 255;
        } else if (depth < image_project.at<ushort>(point_2d.y, point_2d.x)) {
          image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
          rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
              depth / max_depth_ * 255;
          rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
              intensity / 150 * 255;
        }
      } else {
        float intensity = intensity_list[i];
        if (intensity > 100) {
          intensity = 65535;
        } else {
          intensity = (intensity / 150.0) * 65535;
        }
        image_project.at<ushort>(point_2d.y, point_2d.x) = intensity;
      }
    }
  }
  image_project.convertTo(projection_img, CV_8UC1, 1 / 256.0);
  if (is_fill_img) {
    for (int i = 0; i < 5; i++) {
      projection_img = FillImg(projection_img, UP, LEFT);
    }
  }
}

cv::Mat LidarCameraFusion::FillImg(const cv::Mat &input_img,
                              const Direction first_direct,
                              const Direction second_direct) {
  cv::Mat fill_img = input_img.clone();
  for (int y = 2; y < input_img.rows - 2; y++) {
    for (int x = 2; x < input_img.cols - 2; x++) {
      if (input_img.at<uchar>(y, x) == 0) {
        if (input_img.at<uchar>(y - 1, x) != 0) {
          fill_img.at<uchar>(y, x) = input_img.at<uchar>(y - 1, x);
        } else {
          if ((input_img.at<uchar>(y, x - 1)) != 0) {
            fill_img.at<uchar>(y, x) = input_img.at<uchar>(y, x - 1);
          }
        }
      } else {
        int left_depth = input_img.at<uchar>(y, x - 1);
        int right_depth = input_img.at<uchar>(y, x + 1);
        int up_depth = input_img.at<uchar>(y + 1, x);
        int down_depth = input_img.at<uchar>(y - 1, x);
        int current_depth = input_img.at<uchar>(y, x);
        if ((current_depth - left_depth) > 5 &&
            (current_depth - right_depth) > 5 && left_depth != 0 &&
            right_depth != 0) {
          fill_img.at<uchar>(y, x) = (left_depth + right_depth) / 2;
        } else if ((current_depth - up_depth) > 5 &&
                   (current_depth - down_depth) > 5 && up_depth != 0 &&
                   down_depth != 0) {
          fill_img.at<uchar>(y, x) = (up_depth + down_depth) / 2;
        }
      }
    }
  }
  return fill_img;
}

void LidarCameraFusion::MapJet(double v, double vmin, double vmax, 
                          uint8_t* r, uint8_t* g, uint8_t* b) {
  *r = 255;
  *g = 255;
  *b = 255;
  if (v < vmin) {
    v = vmin;
  }
  if (v > vmax) {
    v = vmax;
  }
  double dr, dg, db;
  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }
  *r = static_cast<uint8_t>(255 * dr);
  *g = static_cast<uint8_t>(255 * dg);
  *b = static_cast<uint8_t>(255 * db);
}

extern "C" {

LidarCameraFusion edge_detector;

void SetInitialParameters(InitialParametersStruct* init_par_struct) {
  edge_detector.LoadInitParametersPy(init_par_struct);
}

ImageStruct* SetCalibrationParmaters(CalibrationParametersStruct* calib_par_struct) {
  return edge_detector.LoadCalibParametersPy(calib_par_struct);
}

}
