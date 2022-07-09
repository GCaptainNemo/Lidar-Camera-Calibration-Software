#pragma once

#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef struct InitialParametersStruct {
  char* lidar_address;
  char* lidar_type;
  char* img_address;
} InitialParametersStruct;

typedef struct CalibrationParametersStruct {
  float fx; 
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float p1;
  float p2;
  float tx;
  float ty;
  float tz;
  float roll;
  float pitch;
  float yaw;
} CalibrationParametersStruct;

typedef struct ImageStruct {
    uchar img[6220800];
    int row;
    int col;
    int channel;
} ImageStruct;

class LidarCameraFusion {
 public:
  enum ProjectionType { DEPTH, INTENSITY, BOTH };
  enum Direction { UP, DOWN, LEFT, RIGHT };
 
 public:
  LidarCameraFusion();
  void LoadInitParametersPy(InitialParametersStruct* init_params);
  ImageStruct* LoadCalibParametersPy(CalibrationParametersStruct* calib_params);
  void RegisterImage(const std::string& img_dir);
  void RegisterLidar(const std::string& lidar_dir, bool CUSTOM_MSG);
  cv::Mat GetProjectionImg(const Vector6d& extrinsic_params);
  void ProjectLidar2Img(const Vector6d& extrinsic_params,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud,
                        const ProjectionType projection_type,
                        bool is_fill_img,
                        cv::Mat& projection_img);
  cv::Mat FillImg(const cv::Mat& input_img,
                  const Direction first_direct,
                  const Direction second_direct);
  void MapJet(double v, double vmin, double vmax,
              uint8_t* r, uint8_t* g, uint8_t* b);
 
 private:
  float fx_ = 0.0f;
  float fy_ = 0.0f;
  float cx_ = 0.0f;
  float cy_ = 0.0f;
  float k1_ = 0.0f;
  float k2_ = 0.0f;
  float p1_ = 0.0f;
  float p2_ = 0.0f;
  float k3_ = 0.0f;
  float tx_ = 0.0f;
  float ty_ = 0.0f;
  float tz_ = 0.0f;
  float roll_ = 0.0f;
  float pitch_ =  0.0f;
  float yaw_ = 0.0f;
  int width_ = 0.0f;
  int height_ = 0.0f;
  int min_depth_ = 2.5;
  int max_depth_ = 50;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_cloud_ = nullptr;
  cv::Mat image_;
  pcl::PCDWriter writer;
};

