#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>

#include "iss_keypoints.h"

int create_pcd_names(std::vector<std::string>& names, const int file_num)
{
    const std::string file_base_name = "000000_withoutground.pcd";
    const char digits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

    names.clear();
    names.resize(file_num);
    for (size_t i = 0; i < file_num; i++)
    {
        std::string filename = file_base_name;
        int index(i), pos(5);

        while (index > 0)
        {
            filename[pos] = digits[index % 10];
            index /= 10;
            pos--;
        }

        names[i] = filename;
    }
}


// Terminal: $./program_name folder num
int main(int argc, char* argv[])
{
    std::string folder_in = "/home/jadon/Desktop/keypoints_extraction/data/";
    std::string folder_out = "/home/jadon/Desktop/keypoints_extraction/data/";
    folder_in += argv[1];
    folder_out = folder_out + "iss_" + argv[1];
    int file_num = atoi(argv[2]);

    // load .pcd filenames
    std::vector<std::string> filenames;
    create_pcd_names(filenames, file_num);

    std::vector<std::string> bad_cloud_names;
    std::vector<int> bad_cloud_sizes;

    for (size_t i = 0; i < filenames.size(); i++)
    {
        std::cout << "--------------------------------------------" << std::endl;
        std::string filename = folder_in + '/' + filenames[i];
        std::cout << "Filename " << i << " : " << filename << std::endl;

        // load cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) != 0)
		    return -1;
        std::cout << "Input size: " << cloud->size() << std::endl;

        // extract keypoints
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
        pcke::IssKeyponts issKeypoints(cloud, keypoints);
        std::cout << "Keypoints size: " << keypoints->size() << std::endl;

        // save cloud
        std::string filename_out = folder_out + "/" + filenames[i];
        // std::cout << "Saved keypoints in " << filename_out << std::endl;
        pcl::io::savePCDFileASCII(filename_out, *keypoints);
        std::cout << "Saved keypoints in " << filename_out << std::endl;

        if (keypoints->size() < 1000)
        {
            bad_cloud_names.push_back(filenames[i]);
            bad_cloud_sizes.push_back(keypoints->size());
        }
    }

    std::cout << "===================Bad Clouds==================" << std::endl;
    std::cout << "num: " << bad_cloud_names.size() << std::endl;
    for (size_t i = 0; i < bad_cloud_names.size(); i++)
        std::cout << bad_cloud_names[i] << " : " << bad_cloud_sizes[i] << std::endl;


    return 0;
}