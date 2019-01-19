#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
  pcl::io::loadPLYFile(argv[indices[0]], *combinedCloud);
  
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setMaximumIterations(10);
  icp.setRANSACOutlierRejectionThreshold(0.05);
  icp.setMaxCorrespondenceDistance(5);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1);
  
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setLeafSize(0.1f, 0.1f, 0.1f);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  
  sor.setInputCloud(combinedCloud);
  sor.filter(*filtered);
  vox.setInputCloud(filtered);
  vox.filter(*combinedCloud);

  for(int i = 1; i < indices.size(); i++) {
    std::cout << i << "/" << indices.size() << std::endl;
    if (i % 300 == 0) {
      pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output-temp.ply", *combinedCloud);
    }
    pcl::io::loadPLYFile(argv[indices[i]], *nextCloud);
    if (nextCloud->size() == 0) continue;

    sor.setInputCloud(nextCloud);
    sor.filter(*filtered);
    vox.setInputCloud(filtered);
    vox.filter(*nextCloud);

    icp.setInputTarget(combinedCloud);
    icp.setInputSource(nextCloud);
    icp.align(*Final);
    std::cout <<"has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    if (icp.hasConverged()) {
      *combinedCloud += *Final;
    }
  }

  pcl::io::savePLYFileBinary("/home/e4e/aeriallidar-noros/output.ply", *combinedCloud);
  return 0;
}
