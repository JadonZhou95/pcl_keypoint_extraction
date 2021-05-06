#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>

namespace pcke
{
    class IssKeyponts
    {
        public:
            IssKeyponts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

    };

};