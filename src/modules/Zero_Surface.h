#include "stdafx.h"
#ifndef ZERO_SURFACE_H_
#define ZERO_SURFACE_H_

#include "src/modules/Zero_exports.h"
#include "src/modules/Zero_Common.h"

namespace zero
{
	class ZERO_EXPORTS ZEROSurface
	{
	public:
		ZERO_EXPORTS ZEROSurface() {}
		~ZERO_EXPORTS ZEROSurface() {}
		// ������ƵĿ������ǻ�
		int zerospeedtriangulation(pcl::PointCloud<pcl::PointNormal>& cloud_with_normal,
			pcl::PolygonMesh& triangles,// �洢�������ǻ�������ģ��
			double mu,
			double k,
			double max_surface_angle,
			bool keep_normal);

		int PoissonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &mesh);

		int mesh_serrior(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig, pcl::PolygonMesh& mesh);

	private:
		double compute_clouds_mean_distance(pcl::PointCloud<pcl::PointNormal> &cloud,
			pcl::search::KdTree<pcl::PointNormal>& tree);
	};

	namespace zerosurface
	{
		// ������ƵĿ������ǻ�
		int SpeedTriangulation(pcl::PointCloud<pcl::PointNormal>& cloud_with_normal,
			pcl::PolygonMesh& mesh,// �洢�������ǻ�������ģ��
			double mu = 2.5,
			double k = 20,
			double max_surface_angle = M_PI / 4,
			bool keep_normal = false);

		// �����ؽ�
		
		int PCLPossionReconstruct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PolygonMesh& mesh);
	}
}

#endif