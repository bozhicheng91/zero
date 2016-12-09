#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/distances.h>

#include <pcl/console/time.h>   // TicToc

#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PT;
typedef pcl::PointXYZRGB PTRGB;
typedef pcl::PointCloud<PT> PCT;
typedef pcl::PointCloud<PTRGB> PCTRGB;
typedef pcl::PointCloud<pcl::Normal> PCTN;
typedef pcl::PointCloud<pcl::PointNormal> PCTPN;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PCTRGBN;
typedef pcl::visualization::PCLVisualizer PCLViewer;