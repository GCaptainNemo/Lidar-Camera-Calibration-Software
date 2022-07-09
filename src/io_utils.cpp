#include "io_utils.h"

#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>


int IsFolderExist(const char* path)
{
    DIR *dp;
    if ((dp = opendir(path)) == NULL)
    {
        return 0;
    }
 
    closedir(dp);
    return -1;
}

int IsFileExist(const char *path)
{
    return !access(path, F_OK);
}

void matwrite(const std::string &filename, const cv::Mat &mat)
{
    std::ofstream file;
    file.open (filename, std::fstream::binary);
    if (!file.is_open())
        return ;
    file.write(reinterpret_cast<const char *>(&mat.rows), sizeof(int));
    file.write(reinterpret_cast<const char *>(&mat.cols), sizeof(int));
    const int depth = mat.depth();
    const int type  = mat.type();
    const int channels = mat.channels();
    file.write(reinterpret_cast<const char *>(&depth), sizeof(depth));
    file.write(reinterpret_cast<const char *>(&type), sizeof(type));
    file.write(reinterpret_cast<const char *>(&channels), sizeof(channels));
    int sizeInBytes = mat.step[0] * mat.rows;
    file.write(reinterpret_cast<const char *>(&sizeInBytes), sizeof(int));
    file.write(reinterpret_cast<const char *>(mat.data), sizeInBytes);
    file.close();
}

void matread(const std::string &filename, cv::Mat &read_mat)
{
    std::cout << "start reading!!!" << std::endl;
    int rows, cols, data, depth, type, channels;
    std::ifstream file (filename, std::fstream::binary);
    if (!file.is_open())
        return;
    try {
        std::cout << "size of int = " << sizeof(int) << std::endl;
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        file.read(reinterpret_cast<char *>(&depth), sizeof(depth));
        file.read(reinterpret_cast<char *>(&type), sizeof(type));
        file.read(reinterpret_cast<char *>(&channels), sizeof(channels));
        file.read(reinterpret_cast<char *>(&data), sizeof(data));
        std::cout << "rows = " << rows << "cols = " << cols << "type = " << type << "channels = " << channels << std::endl; 
        read_mat = cv::Mat(rows, cols, type);
        file.read(reinterpret_cast<char *>(read_mat.data), data);
    } catch (...) {
        file.close();
        return;
    }

    file.close();
}
