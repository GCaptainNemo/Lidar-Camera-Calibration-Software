#pragma once

#include <opencv2/opencv.hpp>

int IsFolderExist(const char* path);
int IsFileExist(const char* path);
void matwrite(const std::string& filename, const cv::Mat& mat);
void matread(const std::string& filename, cv::Mat& read_mat);

