#include "stdafx.h"
#ifndef ZERO_SURFACE_H_
#define ZERO_SURFACE_H_

#include "src/modules/Zero_exports.h"
#include "src/modules/Zero_Common.h"
#include "src/modules/Zero_Pretreatment.h"

namespace zero
{
	class ZERO_EXPORTS ZEROSurface
	{
	public:
		ZERO_EXPORTS ZEROSurface() {}
		~ZERO_EXPORTS ZEROSurface() {}
		// 无序点云的快速三角化
		int zerospeedtriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PolygonMesh& triangles,// 存储最终三角化的网格模型
			int k,
			double r,
			double scale,
			double max_surface_angle,
			bool keep_normal);

		int PoissonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &mesh, int k, double r);

		int mesh_serrior(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig, pcl::PolygonMesh& mesh, double scale);
	};

	namespace zerosurface
	{
		// 无序点云的快速三角化
		int SpeedTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PolygonMesh& mesh,// 存储最终三角化的网格模型
			int k = 20,
			double r = 0.0,
			double scale = 1000,
			double max_surface_angle = M_PI / 4,
			bool keep_normal = true);

		// 泊松重建		
		int PCLPossionReconstruct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PolygonMesh& mesh,
			int k = 30,
			double r = 0,
			bool flag = false,
			double scale = 5.0);
	}
}

#endif