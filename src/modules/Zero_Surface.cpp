#include "stdafx.h"
#include "src/modules/Zero_Surface.h"

int zero::ZEROSurface::zerospeedtriangulation(pcl::PointCloud<pcl::PointNormal>& cloud_with_normal, pcl::PolygonMesh& triangles,/* �洢�������ǻ�������ģ�� */ double mu, double k, double max_surface_angle, bool keep_normal)
{
	if (cloud_with_normal.size() <= 3)
	{
		return (1);
	}

	// ����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);
	kdtree->setInputCloud(cloud_with_normal.makeShared());

	double d = compute_clouds_mean_distance(cloud_with_normal, *kdtree);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;// �������ǻ�����
	// �������ӵ�֮��������루�����������߳���
	gp3.setSearchRadius(d * 1000);
	// ����������������ڵ����Զ����
	gp3.setMu(mu);

	gp3.setMaximumNearestNeighbors(k);

	gp3.setMaximumSurfaceAngle(max_surface_angle);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(M_PI / 3);
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normal.makeShared());
	gp3.setSearchMethod(kdtree);
	gp3.reconstruct(triangles);

	return (0);
}

int zero::ZEROSurface::PoissonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &mesh)
{
	if (cloud->size() == 0)
	{
		return (1);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);


	
	/*�������׶�*/
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setNumberOfThreads(8);
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(5);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	
	ne.compute(*normals);

	
	for (size_t i = 0; i < normals->size(); ++i){
		normals->points[i].normal_x *= -1;
		normals->points[i].normal_y *= -1;
		normals->points[i].normal_z *= -1;
	}
	
	//�洢���Ƶķ���
	/*
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n->setInputCloud(cloud);
	n->setSearchMethod(tree);
	n->setKSearch(50);
	n->compute(*normals);
	*/
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	qDebug() << "  >>> normal estimation is done !";

	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);
	pcl::Poisson<pcl::PointNormal>::Ptr poissonInstance(new pcl::Poisson<pcl::PointNormal>);
	//poissonInstance->setSearchMethod(tree2);
	poissonInstance->setInputCloud(cloud_with_normals);
	//poissonInstance->setConfidence(false);
	//poissonInstance->setManifold(false);
	poissonInstance->setOutputPolygons(false);
	//poissonInstance->setIsoDivide(8);
	//poissonInstance->setSamplesPerNode(4);
	poissonInstance->performReconstruction(mesh);
	qDebug() << "  >>> reconstruction step is done !";

	return 0;
}

int zero::ZEROSurface::mesh_serrior(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig, pcl::PolygonMesh& mesh)
{
	if (cloud_orig->size() == 0)
	{
		return (1);
	}
	qDebug() << "  >>> serrior step is beginning !";
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
	double minDistance = zero::zerocommon::pointcloudmeand(*cloud_orig) * 4.0;
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
	qDebug() << "  >>> serrior step is beginning !";
	return (0);
}

double zero::ZEROSurface::compute_clouds_mean_distance(pcl::PointCloud<pcl::PointNormal> &cloud,
	pcl::search::KdTree<pcl::PointNormal>& tree)
{
	double d = 0.0;
	int k = 2;
	std::vector<int> Idx(k);
	std::vector<float> Dis(k);
	for (size_t i = 0; i < cloud.size(); i++)
	{
		if (tree.nearestKSearch(cloud.points[i], k, Idx, Dis) > 0)
		{
			if (d < Dis[1])
				d += Dis[1];
			Idx.clear();
			Dis.clear();
		}
	}

	return (d / cloud.size());
}

int zero::zerosurface::SpeedTriangulation(pcl::PointCloud<pcl::PointNormal>& cloud_with_normal, pcl::PolygonMesh& mesh,/* �洢�������ǻ�������ģ�� */ double mu /*= 2.5*/, double k /*= 20*/, double max_surface_angle /*= M_PI / 4*/, bool keep_normal /*= false*/)
{
	zero::ZEROSurface surface;
	return surface.zerospeedtriangulation(cloud_with_normal, mesh, mu, k, max_surface_angle, keep_normal);
}

int zero::zerosurface::PCLPossionReconstruct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh& mesh)
{
	zero::ZEROSurface surface;
	if (surface.PoissonMesh(cloud, mesh) == 0)
	{
		if (surface.mesh_serrior(cloud, mesh) == 0)
		//if (1)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}
}
