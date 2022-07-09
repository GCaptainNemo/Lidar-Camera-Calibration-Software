#pragma once

#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

typedef struct InitialParametersStruct {
  char* calib_file_address;
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
  void RegisterImage(const std::string& img_dir, cv::Mat* img);
  void RegisterLidar(const std::string& lidar_dir,
                     PointCloudXYZI::Ptr& pc_ptr_xyzi_,
                     bool CUSTOM_MSG);
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
  float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_;
  float tx_, ty_, tz_, roll_, pitch_, yaw_;
  bool lidar_type_;
  int width_;
  int height_;
  cv::Mat image_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_cloud_;  // 存储从pcd/bag处获取的原始点云
  int min_depth_ = 2.5;
  int max_depth_ = 50;
  pcl::PCDWriter writer;
  std::string lidar_path_;
  std::string img_path_;
  std::string calib_setting_path_;
};

