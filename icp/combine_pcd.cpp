#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr next (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr next_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  std::string output = "output";
  int size = -1;
  int idx = pcl::console::parse_argument(argc, argv, "-o", output);
  pcl::console::parse_argument(argc, argv, "-s", size);
  std::vector<int> pcdIndices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
  std::vector<int> plyIndices = pcl::console::parse_file_extension_argument(argc, argv, "ply");

  std::cout << "Found " << pcdIndices.size() << " pcd files" << std::endl;
  std::cout << "Found " << plyIndices.size() << " ply files" << std::endl;
  int start = 0;
  int count = 1;
  for (int i = 0; i < pcdIndices.size(); i++) {
    pcl::io::loadPCDFile(argv[pcdIndices[i]], *next);
//    pcl::PointXYZ min;
//    pcl::PointXYZ max;
//    pcl::getMinMax3D(*next, min, max);
//    std::cout << min << std::endl << max << std::endl;
//    pcl::VoxelGrid<pcl::PointXYZ> vox;
//    vox.setInputCloud(next);
//    vox.setLeafSize(0.1f, 0.1f, 0.1f);
//    vox.filter(*next_filtered);

    *combinedCloud += *next;
    std::cout << i << " - " << combinedCloud->size() << std::endl;
    count++;
  }
  
  std::cout << idx << std::endl;
  for (int i = 0; i < plyIndices.size(); i++) {
    std::cout << plyIndices[i] << std::endl;
    if (plyIndices[i] == idx) {
      std::cout << "Skipping output ply" << std::endl;
      continue;
    }
    pcl::io::loadPLYFile(argv[plyIndices[i]], *next);
    *combinedCloud += *next;
    std::cout << i << " - " << combinedCloud->size() << std::endl;
    if (size != -1 && count % size == 0) {
      pcl::io::savePLYFileBinary(output+"-"+std::to_string(start)+"-"+std::to_string(start+count)+".ply", *combinedCloud);
      start = start + count;
      combinedCloud->clear();
      count = 0;
    }
    count++;
  }
  

  //pcl::io::loadPLYFile("/home/e4e/data/2018-08-01/04000.ply", *combinedCloud);
  //pcl::io::loadPLYFile("/home/e4e/data/2018-08-01/08000.ply", *nextCloud);
  //std::cout << in1cloud->size() << std::endl;
  //std::cout << in2cloud->size() << std::endl;

  //pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  //icp.setInputSource(in1cloud);
  //icp.setInputTarget(in2cloud);
  //pcl::PointCloud<pcl::PointXYZRGB> Final;
  //icp.align(Final);
  //std::cout <<"has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;

  //pcl::io::savePLYFileBinary(output, *combinedCloud);
  pcl::io::savePLYFileBinary(output+"-"+std::to_string(start)+"-"+std::to_string(start+count)+".ply", *combinedCloud);
  return 0;
}
