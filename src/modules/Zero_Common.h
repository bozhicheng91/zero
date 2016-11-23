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
		// ���㼫��ֵ�뼫Сֵ
		template<typename PointT>
		void MinMax(pcl::PointCloud<PointT>& cloud,
			PointT& min,
			PointT& max)
		{
			pcl::getMinMax3D(cloud, min, max);
		}
		// ������ư�Χ������
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
		// �����������
		template<typename PointT>
		void Centroid(pcl::PointCloud<PointT>& cloud,
			Eigen::Vector4f &centroid)
		{
			pcl::compute3DCentroid(cloud, centroid);
		}
		// ��λ������
		void unitvector(Eigen::Vector3f &vector)
		{
			vector.setIdentity();
		}
		// ���������Ŵӵ�������ȡ�㼯
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
		// ���㼫��ֵ��Сֵ
		template<typename PointT>
		void ComputeMinMax(pcl::PointCloud<PointT>& cloud,
			PointT& min,
			PointT& max)
		{
			zero::ZEROCommon common;
			common.MinMax(cloud, min, max);
		}
		//������ư�Χ�����ĵ�
		template<typename PointT>
		void Compute3dCenter(pcl::PointCloud<PointT>& cloud,
			PointT &center)
		{
			zero::ZEROCommon common;
			common.Center(cloud, center);
		}
		//�����������
		template<typename PointT>
		void Compute3dCentroid(pcl::PointCloud<PointT>& cloud,
			Eigen::Vector4f &centroid)
		{
			zero::ZEROCommon common;
			common.Centroid(cloud, centroid);
		}
		// �����������
		template<typename PointT>
		float ComputePDistance(PointT& first,
			PointT& second)
		{
			return (pcl::euclideanDistance(first, second));
		}
		// ���������Ŵӵ�������ȡ�㼯
		template<typename PointT>
		void Extractindices(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<PointT>& cloud_extracted,
			pcl::PointIndices::Ptr indices,
			bool extract_flag = true)
		{
			zero::ZEROCommon common;
			common.extractindices(cloud, cloud_extracted, indices, extract_flag);
		}
		//������������֮���ŷ�Ͼ���
		double ComputeEUDistance(const Eigen::Vector4f& first,
			const Eigen::Vector4f& second);
	}
}

#endif