#include "stdafx.h"
#ifndef ZERO_COMMON_H_
#define ZERO_COMMON_H_

#include "Zero_exports.h"

namespace zero
{
	class ZERO_EXPORTS ZEROCommon
	{
	public:
		ZERO_EXPORTS ZEROCommon() {}
		~ZERO_EXPORTS ZEROCommon() {}
		// 计算极大值与极小值
		template<typename PointT>
		void MinMax(pcl::PointCloud<PointT>& cloud,
			PointT& min,
			PointT& max)
		{
			pcl::getMinMax3D(cloud, min, max);
		}
		// 计算点云包围盒中心
		template<typename PointT>
		void Center(pcl::PointCloud<PointT>& cloud,
			PointT& center)
		{
			PointT min, max;
			pcl::getMinMax3D(cloud, min, max);

			center.x = (min.x + max.x) / 2;
			center.y = (min.y + max.y) / 2;
			center.z = (min.z + max.z) / 2;
		}
		// 计算点云质心
		template<typename PointT>
		void Centroid(pcl::PointCloud<PointT>& cloud,
			Eigen::Vector4f &centroid)
		{
			pcl::compute3DCentroid(cloud, centroid);
		}
		// 单位化向量
		void unitvector(Eigen::Vector3f &vector)
		{
			vector.setIdentity();
		}
		// 根据索引号从点云中提取点集
		template<typename PointT>
		void extractindices(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<PointT>& cloud_extracted,
			pcl::PointIndices::Ptr indices,
			bool extract_flag)
		{
			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud(cloud.makeShared());
			extract.setIndices(indices);
			extract.setNegative(extract_flag);
			extract.filter(cloud_extracted);
		}
	};

	namespace zerocommon
	{
		// 计算极大值极小值
		template<typename PointT>
		void ComputeMinMax(pcl::PointCloud<PointT>& cloud,
			PointT& min,
			PointT& max)
		{
			zero::ZEROCommon common;
			common.MinMax(cloud, min, max);
		}
		//计算点云包围盒中心点
		template<typename PointT>
		void Compute3dCenter(pcl::PointCloud<PointT>& cloud,
			PointT &center)
		{
			zero::ZEROCommon common;
			common.Center(cloud, center);
		}
		//计算点云质心
		template<typename PointT>
		void Compute3dCentroid(pcl::PointCloud<PointT>& cloud,
			Eigen::Vector4f &centroid)
		{
			zero::ZEROCommon common;
			common.Centroid(cloud, centroid);
		}
		// 计算两点距离
		template<typename PointT>
		float ComputePDistance(PointT& first,
			PointT& second)
		{
			return (pcl::euclideanDistance(first, second));
		}
		// 根据索引号从点云中提取点集
		template<typename PointT>
		void Extractindices(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<PointT>& cloud_extracted,
			pcl::PointIndices::Ptr indices,
			bool extract_flag = true)
		{
			zero::ZEROCommon common;
			common.extractindices(cloud, cloud_extracted, indices, extract_flag);
		}
		//计算两个向量之间的欧氏距离
		double ComputeEUDistance(const Eigen::Vector4f& first,
			const Eigen::Vector4f& second);
	}
}

#endif