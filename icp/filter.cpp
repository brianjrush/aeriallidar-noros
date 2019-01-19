#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPLYFile(argv[1], *cloud);

  std::cout << "Starting statistical outlier removal" << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*filtered);
  pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output-sor.ply", *filtered);
  
  //std::cout << "Starting voxel grid" << std::endl;

  //pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  //vox.setInputCloud(filtered);
  //vox.setLeafSize(0.1f, 0.1f, 0.1f);
  //vox.filter(*voxcloud);
  //pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output-sor-voxel.ply", *voxcloud);

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

  //pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output-voxel-sor.ply", *filtered);
  return 0;
}
