#include "stdafx.h"
#include "src/modules/Zero_Surface.h"

int zero::ZEROSurface::zerospeedtriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PolygonMesh& triangles,/* �洢�������ǻ�������ģ�� */ 
	int k, 
	double r, 
	double scale,
	double max_surface_angle, 
	bool keep_normal)
{
	if (cloud->size() <= 3)
	{
		return (1);
	}

	double min, max;
	zero::zerocommon::pointcloudmaxmind(*cloud, max, min);
	double d = max;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	zero::zeropretreatment::ComputeCloudNormal(*cloud, *normals, k, r);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normal);

	// ����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);
	kdtree->setInputCloud(cloud_with_normal);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;// �������ǻ�����
	// �������ӵ�֮��������루�����������߳���
	gp3.setSearchRadius(d * 100000 * scale);
	// ����������������ڵ����Զ����
	gp3.setMu(d * 10000 * scale);
	gp3.setMaximumNearestNeighbors(100);
	// �����ĽǶ�
	double angle = max_surface_angle / 180 * M_PI;
	gp3.setMaximumSurfaceAngle(angle);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normal);
	gp3.setSearchMethod(kdtree);
	gp3.reconstruct(triangles);

	return (0);
}

int zero::ZEROSurface::PoissonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PolygonMesh &mesh, 
	int k, 
	double r)
{
	if (cloud->size() == 0)
	{
		return (1);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	// ���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	zero::zeropretreatment::ComputeCloudNormal(*cloud, *normals, k, r);

	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	for (size_t i = 0; i < cloud_with_normals->size(); i++)
	{
		cloud_with_normals->points[i].normal_x *= -1;
		cloud_with_normals->points[i].normal_y *= -1;
		cloud_with_normals->points[i].normal_z *= -1;
	}

	pcl::Poisson<pcl::PointNormal>::Ptr poissonInstance(new pcl::Poisson<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	poissonInstance->setSearchMethod(tree2);
	poissonInstance->setInputCloud(cloud_with_normals);
	poissonInstance->setConfidence(false);
	poissonInstance->setManifold(false);
	poissonInstance->setOutputPolygons(false);
	poissonInstance->setIsoDivide(8);
	poissonInstance->setSamplesPerNode(4);
	poissonInstance->reconstruct(mesh);

	return 0;
}

int zero::ZEROSurface::mesh_serrior(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig, pcl::PolygonMesh& mesh, double scale)
{
	if (cloud_orig->size() == 0)
	{
		return (1);
	}

	//��ȡ mesh �� pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *meshcloud);

	//����������
	int K = 1;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	tree->setInputCloud(cloud_orig);

	//map ��¼���Ƿ��ٽ�ԭʼ����, 0:�ٽ� 1 ���ٽ�
	map<int, int> mapPointIn;

	//������ֵԽ��, ɾ���ĵ�Խ��
	double minDistance = zero::zerocommon::pointcloudmeand(*cloud_orig) * scale;
	for (int i = 0; i < meshcloud->size(); i++)
	{
		PT searchPoint = meshcloud->points[i];
		tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

		if (pointIdxNKNSearch.size() > 0 && pointNKNSquaredDistance[0] <= minDistance)
		{
			mapPointIn[i] = 1;
		}
		else
		{
			mapPointIn[i] = 0;
		}
	}

	//���ݵ��map  ɾ����Ƭ, ԭ������Ƭ������������һ���Ƿ��ٽ���, ��ɾ��
	for (std::vector< ::pcl::Vertices>::iterator iter = mesh.polygons.begin(); iter != mesh.polygons.end();)
	{
		std::vector<uint32_t> &v = iter->vertices;
		if (mapPointIn[v[0]] == 0 || mapPointIn[v[1]] == 0 || mapPointIn[v[2]] == 0)
		{
			//ɾ���ڵ�
			iter = mesh.polygons.erase(iter);
			if (iter == mesh.polygons.end())
			{
				break;
			}
			continue;
		}
		else
		{
			iter++;

			if (iter == mesh.polygons.end())
			{
				break;
			}
		}
	}

	return (0);
}

int zero::zerosurface::SpeedTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PolygonMesh& mesh,/* �洢�������ǻ�������ģ�� */ 
	int k,
	double r,
	double d,
	double max_surface_angle,
	bool keep_normal)
{
	zero::ZEROSurface surface;
	return surface.zerospeedtriangulation(cloud, mesh, k, r, d, max_surface_angle, keep_normal);
}

int zero::zerosurface::PCLPossionReconstruct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PolygonMesh& mesh, 
	int k /*= 30*/, 
	double r /*= 0*/, 
	bool flag /*= false*/, 
	double scale /*= 5.0*/)
{
	zero::ZEROSurface surface;
	if (surface.PoissonMesh(cloud, mesh, k, r) != 0)
	{
		return 1;
	}
	if (!flag)
	{
		return 0;
	}
	if (surface.mesh_serrior(cloud, mesh, scale) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
