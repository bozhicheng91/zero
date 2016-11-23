#include "stdafx.h"
#ifndef ZERO_SEARCH_H_
#define ZERO_SEARCH_H_

#include "src/modules/Zero_exports.h"

namespace zero
{
	class ZERO_EXPORTS kdtree
	{
	public:
		kdtree() {}
		~kdtree() {}
		template <typename PointT>
		void zeronearestsearch(pcl::KdTreeFLANN<PointT>& tree,
			pcl::PointCloud<PointT>& cloud,
			PointT &searchPoint,
			std::vector<int> &pointIdxNKNSearch,
			std::vector<float> &pointNKNSquaredDistance,
			int K,
			double r)
		{
			tree.setInputCloud(cloud.makeShared());
			if (K > 1 && r < 1e-10)
			{
				if (tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) < 0);
				{
					WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "can not find k nearest point set of searchPoint from cloud.\n");
					exit(-1);
				}
			}

			if (r > 0 && K < 2)
			{
				if (tree.radiusSearch(searchPoint, r, pointIdxNKNSearch, pointNKNSquaredDistance) < 0);
				{
					WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "can not find r nearest point set of searchPoint from cloud.\n");
					exit(-1);
				}
			}
		}

	private:
	};

	class ZERO_EXPORTS octree
	{
	public:
		octree() {}
		~octree() {}
		template <typename PointT>
		void zeronearestsearch(pcl::octree::OctreePointCloudSearch<PointT> &tree,
			PointT &searchPoint,
			std::vector<int> &pointIdxNKNSearch,
			std::vector<float> &pointNKNSquaredDistance,
			float resolution,
			int K,
			double r)
		{
			tree.setResolution(resolution);
			tree.setInputCloud(cloud.makeShared());
			tree.addPointsFromInputCloud();
			if (K > 1 && r < 1e-10)
			{
				if (tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) < 0);
				{
					WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "can not find k nearest point set of searchPoint from cloud.\n");
					exit(-1);
				}
			}

			if (r > 0 && K < 2)
			{
				if (tree.radiusSearch(searchPoint, r, pointIdxNKNSearch, pointNKNSquaredDistance) < 0);
				{
					WriteLog(DEBUG_LEVEL, __FILE__, __LINE__, "--------------------Call API: %s", "can not find r nearest point set of searchPoint from cloud.\n");
					exit(-1);
				}
			}
		}
		// 利用octree检测cloudB相对于cloudA的改变
		template <typename PointT>
		void octreecloudchangedetetor(pcl::PointCloud<PointT>& cloudA,
			pcl::PointCloud<PointT>& cloudB,
			std::vector<int> &newPoints,
			float resolution)
		{
			pcl::octree::OctreePointCloudChangeDetector<PointT> octree(resolution);
			// 为octree增加点云cloudA
			octree.setInputCloud(cloudA.makeShared());
			octree.addPointsFromInputCloud();
			// 该octree同时存储两个点云，在保持之前tree存在，我们需要重设缓冲
			octree.switchBuffers();
			octree.setInputCloud(cloudB.makeShared());
			octree.addPointsFromInputCloud();
			
			// 得到B中有，A中没有的点
			octree.getPointIndicesFromNewVoxels(newPoints);
		}
	private:
	};

	namespace zerosearch
	{
		// 利用kdtree进行邻域查询
		template <typename PointT>
		void KdtreeNKRsearch(pcl::KdTreeFLANN<PointT>& tree,
			pcl::PointCloud<PointT>& cloud,
			PointT &searchPoint,
			std::vector<int> &pointIdxNKNSearch,
			std::vector<float> &pointNKNSquaredDistance,
			int k = 20,
			double r = 0.0)
		{
			zero::kdtree kd;
			kd.zeronearestsearch(tree, cloud, searchPoint, pointIdxNKNSearch, pointNKNSquaredDistance, k, r);
		}
		// 利用Octree进行邻域查询
		template <typename PointT>
		void OctreeNKRsearch(pcl::octree::OctreePointCloudSearch<PointT> &tree,
			PointT &searchPoint,
			std::vector<int> &pointIdxNKNSearch,
			std::vector<float> &pointNKNSquaredDistance,
			float resolution = 128.0f,
			int K = 1,
			double r = 0.0)
		{
			zero::octree oc;
			oc.zeronearestsearch(tree, cloud, searchPoint, pointIdxNKNSearch, pointNKNSquaredDistance, resolution, k, r);
		}
		// 利用Octree压缩点云，compdata为包含压缩数据的二进制输出流
		template <typename PointT>
		void OctreeCompressCloud(pcl::PointCloud<PointT> &cloud,
			std::stringstream compdata)
		{
			pcl::io::OctreePointCloudCompression<PointT> octreecomp(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
			octreecomp.encodePointCloud(cloud, compdata);
		}
		// 利用Octree解压点云，compdata为包含压缩数据的二进制输入流
		template <typename PointT>
		void OctreeDecompressCloud(pcl::PointCloud<PointT> &cloud_compressed,
			std::stringstream compdata)
		{
			pcl::io::OctreePointCloudCompression<PointT> octreecomp(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
			octreecomp.decodePointCloud(cloud_compressed, compdata);
		}
		// 利用octree检测cloudB相对于cloudA的改变
		template <typename PointT>
		void OctreeCloudChangeDetetor(pcl::PointCloud<PointT>& cloudA,
			pcl::PointCloud<PointT>& cloudB,
			std::vector<int> &newPoints,
			float resolution = 128.0f)
		{
			zero::octree oc;
			oc.octreecloudchangedetetor(cloudA, cloudB, newPoints, resolution);
		}
	}
}

#endif