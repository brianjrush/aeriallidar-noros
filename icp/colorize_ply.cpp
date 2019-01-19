#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
//#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv) {
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPLYFile(argv[1], *cloud);

  //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  //sor.setInputCloud(orig_cloud);
  //sor.setMeanK(50);
  //sor.setStddevMulThresh(1.0);
  //sor.filter(*cloud);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr next (new pcl::PointCloud<pcl::PointXYZ>);
  //std::vector<int> indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");

  //std::cout << "Found " << indices.size() << " ply files" << std::endl;
  
  pcl::PointXYZRGBA min;
  pcl::PointXYZRGBA max;

  pcl::getMinMax3D(*cloud, min, max);

  std::cout << "All points between " << min.z << " and " << max.z <<std::endl;

  double lut_scale = 255.0 / (max.z - min.z);
  int value = 0;

  for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin(); it != cloud->end(); it++) {
    it->x = it->x;
    it->y = it->y;
    it->z = it->z;
    value = (it->z-min.z)*lut_scale;
    it->b = 255-value;
    it->g = 0;
    it->r = (value/255.f)*(value/255.f)*255;
    it->a = 255;
    //std::cout << 255.f*n << " " << 255.f*(1-n) << " " << n << std::endl;
  }

  pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output-colored.ply", *cloud);
  return 0;
}
