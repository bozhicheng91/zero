#pragma once
#pragma execution_character_set("utf-8")

#include "Zero_Viewer.h"

void zero::ZEROViewer::viewerinit(std::string &title,
	const double bg_r, 
	const double bg_g, 
	const double bg_b,
	const bool coord_f)
{
	zeroviewer->setWindowName(title);
	zeroviewer->setBackgroundColor(bg_r, bg_g, bg_b);
	if (coord_f)
		zeroviewer->addCoordinateSystem(1.0);
	zeroviewer->initCameraParameters();
}

void zero::ZEROViewer::viewerrun()
{
	while (!zeroviewer->wasStopped())
	{
		zeroviewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

/*
void zero::ZEROViewer::MouseKeyboardEvent()
{
	zeroviewer->registerPointPickingCallback(&zero::ZEROViewer::zeroPointPickCallback, 
		(void *)viewer.get());
	viewer->registerAreaPickingCallback(&zero::ZEROViewer::zeroAeraPickCallback, *this);

	zeroviewer = viewer;
}

void zero::ZEROViewer::zeroPointPickCallback(const pcl::visualization::PointPickingEvent& event, void *cookie)
{
	int idx = event.getPointIndex();
	if (idx == -1)
	{
		std::string t = "未选中点";
		zeroviewer->addText(t, 200, 300);
		return;
	}

	//获取点的坐标
	float x, y, z;
	event.getPoint(x, y, z);
	std::ostringstream ss;
	ss << "(" << x << "," << y << "," << z << ")";
	pcl::PointXYZ position(x, y, z);
	zeroviewer->addText(ss.str(), 10, 15, 16, 1.0, 1.0, 1.0, "cloud text");
}

void zero::ZEROViewer::zeroAeraPickCallback(const pcl::visualization::AreaPickingEvent& event, void *cookie)
{
	std::vector<int> vec;
	if (event.getPointsIndices(vec) == -1)
		return;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < vec.size(); ++i)
	{
		cloud_tmp->points.push_back(cloud_.points[vec[i]]);
	}

	if (cloud_tmp->empty())
		return;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_tmp, 255, 0, 0);
	zeroviewer->addPointCloud(cloud_tmp, single_color, "Area point cloud");
	zeroviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Area point cloud");
}
*/