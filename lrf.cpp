/******************************************
 * Author: Bo Sun
 * Afflication: TAMS, University of Hamburg
 * E-Mail: bosun@informatik.uni-hamburg.de
 * Date: Jan 5, 2015
 *******************************************/

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <time.h>
#include <fstream>

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::ReferenceFrame FeatureT;
typedef pcl::SHOTLocalReferenceFrameEstimation<PointT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

int
main (int argc, char **argv)
{
// Initiate Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_downsample(new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  
  clock_t t_whole;
  
// Get input object and scene
  if (argc != 2)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd\n", argv[0]);
    return (1);
  }
  
  pcl::console::print_highlight ("Loading point clouds...\n");
  
  if (pcl::io::loadPCDFile<PointT> (argv[1], *object) < 0)
  {
    pcl::console::print_error ("Error loading object file!\n");
    return (1);
  }

// Remove the nan points
  std::vector<int> indices_object_nan;
  pcl::removeNaNFromPointCloud(*object, *object, indices_object_nan);
  
  t_whole=clock();
  
// Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.2, 0.2, 0.2);
  grid.setInputCloud (object);
  grid.filter (*object_downsample);

// Estimate features
  pcl::console::print_highlight ("Estimating LRF...\n");
  FeatureEstimationT fest;
  pcl::search::KdTree<PointT>::Ptr tree_fest(new pcl::search::KdTree<PointT> ());
  fest.setSearchMethod(tree_fest);
  fest.setRadiusSearch (1.0);
  fest.setInputCloud (object_downsample);
  fest.setSearchSurface(object);
  fest.compute (*object_features);
  pcl::io::savePCDFileASCII("test_object_lrf.pcd",*object_features);

  pcl::console::print_highlight("The Local Reference Frame on points [%f, %f, %f] is\n",
                                (*object_downsample).points[100].x,
                                (*object_downsample).points[100].y,
                                (*object_downsample).points[100].z);
  pcl::console::print_highlight("x_axis    [%f, %f, %f] \n",
                                (*object_features)[100].x_axis[0],
                                (*object_features)[100].x_axis[1],
                                (*object_features)[100].x_axis[2]
                                );
  pcl::console::print_highlight("y_axis    [%f, %f, %f] \n",
                                (*object_features)[100].y_axis[0],
                                (*object_features)[100].y_axis[1],
                                (*object_features)[100].y_axis[2]
                                );
  pcl::console::print_highlight("z_axis    [%f, %f, %f] \n",
                                (*object_features)[100].z_axis[0],
                                (*object_features)[100].z_axis[1],
                                (*object_features)[100].z_axis[2]
                                );
  
  return (0);
}
