#pragma once
#pragma execution_character_set("utf-8")

#ifndef ZERO_VIEWER_H_
#define ZERO_VIEWER_H_

#include "Zero_exports.h"
#include "Zero_log.h"

#include <string>

#include <pcl/io/boost.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace zero
{
	class ZERO_EXPORTS ZEROViewer
	{
	public:
		ZEROViewer() { zeroviewer.reset(new pcl::visualization::PCLVisualizer); }
		~ZEROViewer() { cloud_.clear(); }
		// zeroviewer初始化
		void viewerinit(std::string &title,
			const double bd_r = 0.0,
			const double bd_g = 0.0,
			const double bd_b = 0.0,
			const bool coord_f = true);
		// 可视化一个点云
		template<typename PointT>
		void viewer_cloud(pcl::PointCloud<PointT>& cloud,
			bool Keep_color = true,
			double point_size = 1.0,
			double color_r = 255.0,
			double color_g = 255.0,
			double color_b = 255.0)
		{
			if (Keep_color)
				zeroviewer->addPointCloud<PointT>(cloud.makeShared(), "cloud");
			else
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud.makeShared(), color_r, color_g, color_b);
				zeroviewer->addPointCloud<PointT>(cloud.makeShared(), single_color, "cloud");
			}
			zeroviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
		}
		// 可视化两个点云
		template<typename PointT>
		void viewer_two_clouds(pcl::PointCloud<PointT>& cloud_f,
			pcl::PointCloud<PointT>& cloud_s,
			bool keep_color = true)
		{
			int v1(0);
			int v2(1);
			zeroviewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			zeroviewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			// 第一个点云为绿色
			if (keep_color)
				zeroviewer->addPointCloud(cloud_f.makeShared(), "cloud_first_v1", v1);
			else
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source_color(cloud_f.makeShared(), 0, 255, 0);
				zeroviewer->addPointCloud(cloud_f.makeShared(), cloud_source_color, "cloud_first_v1", v1);
			}
			std::stringstream ss;
			ss << cloud_f.points.size();
			std::string cloud_str = "the source cloud size is " + ss.str();
			zeroviewer->addText(cloud_str, 10, 15, 16, 1.0, 1.0, 1.0, "cloud_first", v1);

			// 第二个点云为红色
			if (keep_color)
				zeroviewer->addPointCloud(cloud_s.makeShared(), "cloud_first_v2", v2);
			else
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_color(cloud_s.makeShared(), 255, 0, 0);
				zeroviewer->addPointCloud(cloud_s.makeShared(), cloud_filtered_color, "cloud_second_v2", v2);
			}
			ss.str("");
			ss << cloud_s.points.size();
			std::string cloud_pretreated_str = "the cloud after pretreated size is " + ss.str();
			zeroviewer->addText(cloud_pretreated_str, 10, 15, 16, 1.0, 1.0, 1.0, "cloud_second", v2);
		}
		// 可视化带有法向量的点云
		template<typename PointT>
		void viewer_normals(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::PointNormal>& normals,
			int level = 100,
			float scale = 0.02,
			const std::string &id = "normals",
			int viewport = 0)
		{
			// 法向量为白色
			zeroviewer->addPointCloudNormals<PointT, pcl::PointNormal>(cloud.makeShared(), normals.makeShared(), level, scale, id);
			std::stringstream ss;
			ss << normals.points.size() / level;
			std::string cloud_pretreated_str = "the size of normals is " + ss.str();
			ss.str("");
			ss << scale;
			cloud_pretreated_str += "\nscale: " + ss.str();
			zeroviewer->addText(cloud_pretreated_str, 10, 15, 16, 1.0, 1.0, 1.0, "normal_txt");
		}
		// 可视化带有法向量的点云,渲染效率太慢，不知道为啥
		template<typename PointT>
		void zero_viewer_normals(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::Normal>& normals,
			int level = 100,
			float scale = 0.02,
			const std::string &id = "normals",
			int viewport = 0)
		{
			// 法向量为白色
			std::stringstream ss;
			PointT second;
			for (size_t i = 0; i < cloud.points.size();)
			{
				second.x = cloud.points[i].x - normals.points[i].normal_x * scale;
				second.y = cloud.points[i].y - normals.points[i].normal_y * scale;
				second.z = cloud.points[i].z - normals.points[i].normal_z * scale;
				ss << "line" << i;
				zeroviewer->addLine(cloud.points[i], second, 1.0, 0.0, 0.0, ss.str());
				//zeroviewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, ss.str());
				ss.str("");
				i += level;
			}
			ss << normals.points.size() / level;
			std::string cloud_pretreated_str = "the size of normals is " + ss.str();
			ss.str("");
			ss << scale;
			cloud_pretreated_str += "\nscale: " + ss.str();
			zeroviewer->addText(cloud_pretreated_str, 10, 15, 16, 1.0, 1.0, 1.0, id);
		}
		//void zeroAeraPickCallback(const pcl::visualization::AreaPickingEvent& event, void *cookie);
		//void zeroPointPickCallback(const pcl::visualization::PointPickingEvent& event, void *cookie);
		//void MouseKeyboardEvent();
		void viewerrun();

	private:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> zeroviewer;
		pcl::PointCloud<pcl::PointXYZ> cloud_;
		
	};
}

#endif