#include "stdafx.h"
#include "Zero_Viewer.h"
#include "Zero_Pretreatment.h"
#include "Zero_Registration.h"
#include "Zero_Search.h"




typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PCT;


// IICP
bool next_iteration = false;
void
print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}
// 定义一个旋转矩阵及平移向量
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
void vieweriicp(PCT &cloud_in, PCT& cloud_icp)
{
	pcl::console::TicToc time;
	PCT::Ptr cloud_tr(new PCT);
	*cloud_tr = cloud_icp;
	time.tic();
	zero::zeroregistration::PretreatSourceGICP(cloud_icp, cloud_in);
	std::cout << "\nregistration " << time.toc() << " ms\n" << std::endl;

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two verticaly separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// 源点云为白色
	pcl::visualization::PointCloudColorHandlerCustom<PT> cloud_in_color_h(cloud_in.makeShared(), (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in.makeShared(), cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in.makeShared(), cloud_in_color_h, "cloud_in_v2", v2);

	// 改变转换点云为绿色
	pcl::visualization::PointCloudColorHandlerCustom<PT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP配准点云为红色
	pcl::visualization::PointCloudColorHandlerCustom<PT> cloud_icp_color_h(cloud_icp.makeShared(), 180, 20, 20);
	viewer.addPointCloud(cloud_icp.makeShared(), cloud_icp_color_h, "cloud_icp_v2", v2);

	// 在每个视口添加文本说明
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	int iterations = 1;
	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// 设置背景色
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// 设置摄像机位置和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // 可视化窗口大下，并没有固定，还可调整

	// 注册键盘回掉函数
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

// passthrough
void passthrogh()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	//zero::zeropretreatment::FilterPassThrough(*cloud, *cloud_filtered, "z", 100.0, 800.0);
	zero::zeropretreatment::FilterConditional(*cloud, *cloud_filtered, "z", pcl::ComparisonOps::GT, 300, pcl::ComparisonOps::LT, 800.0, true);
	std::cerr << "Cloud after filtering: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << std::endl;
}

// 可视化一个点云
template<typename PointT>
void viewercloud(pcl::PointCloud<PointT>& cloud1, pcl::PointCloud<PointT>& cloud2)
{
	zero::ZEROViewer viewer;
	std::string title = "Zero Viewer";
	viewer.viewerinit(title, 0.5, 0.5, 1.0, false);
	viewer.viewer_two_clouds(cloud1, cloud2, false);
	//viewer.viewer_normals(*cloud, *normals, 50, 0.02);
	viewer.viewerrun();
}

int 
main(int argc, char **argv)
{
	
	if (argc < 2)
	{
		zero_error("There is no any file.pcd.\n");
		return (-1);
	}
	
	pcl::console::TicToc time;
	time.tic();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (zero::zeroio::LoadPCD(argv[1], *cloud) < 0)
	{
		printf("Failed to load data from %s", argv[1]);
		return (-1);
	}
	std::cout << "\nLoaded file " << argv[1] << " (" << cloud->size() << " points) in " << time.toc() << " ms\n" << std::endl;

	
	std::vector<int> invec;
	pcl::PointCloud<PT>::Ptr cloud_icp(new pcl::PointCloud<PT>);
	int iterations = 1;  // 默认的ICP迭代次数
	
	if (argc == 2)
	{
		//iterations = atoi(argv[2]);
		// 旋转矩阵(see https://en.wikipedia.org/wiki/Rotation_matrix)
		double theta = M_PI / 4;  // 角度转为弧度
		transformation_matrix(0, 0) = cos(theta);
		transformation_matrix(0, 1) = -sin(theta);
		transformation_matrix(1, 0) = sin(theta);
		transformation_matrix(1, 1) = cos(theta);

		// z轴的旋转 (0.4 meters)
		transformation_matrix(2, 3) = 0.7;
		print4x4Matrix(transformation_matrix);

		// 在终端显示旋转矩阵
		std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;

		// 执行转换
		pcl::transformPointCloud(*cloud, *cloud_icp, transformation_matrix);
	}
	if (argc == 3)
	{
		time.tic();
		if (pcl::io::loadPCDFile(argv[2], *cloud_icp) < 0)
		{
			PCL_ERROR("Error loading cloud %s.\n", argv[2]);
			return (-1);
		}
		pcl::removeNaNFromPointCloud(*cloud_icp, *cloud_icp, invec);
		std::cout << "\nLoaded file " << argv[2] << " (" << cloud_icp->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	}
	if (iterations < 1)
	{
		PCL_ERROR("Number of initial iterations must be >= 1\n");
		return (-1);
	}
	
	vieweriicp(*cloud, *cloud_icp);
	/*
	zero::ZEROViewer viewer;
	std::string title = "Zero Viewer";
	viewer.viewerinit(title, 0.5, 0.5, 1.0, false);
	viewer.viewer_cloud_single_color(*cloud);
	viewer.viewerrun();
	
	cout << "read ok" << endl;

	//提取索引号
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<PT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	seg.segment(*indices, *coefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZ>);
	zero::zerocommon::Extractindices(*cloud, *cloud_extracted, indices, false);

	for (size_t i = 0; i < cloud_extracted->points.size(); i++)
	{
		std::cout << cloud_extracted->points[i].x << " "
			<< cloud_extracted->points[i].y << " "
			<< cloud_extracted->points[i].z << std::endl;
	}
	std::cout << cloud_extracted->points.size() << std::endl;
	
	// 改变检测
	std::vector<int> newPoints;
	zero::zerosearch::OctreeCloudChangeDetetor(*cloud, *cloud_icp, newPoints);
	std::cout << newPoints.size() << std::endl;
	

	// passthrough
	//passthrogh();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	zero::zeropretreatment::UpSampling(*cloud, *cloud_filtered, 0.03, 0.03, 0.02);
	viewercloud(*cloud, *cloud_filtered);
	
	// 法向量估计及显示
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	std::cout << "Organiezed : " << cloud->isOrganized() << std::endl;
	if (cloud->isOrganized());
		//zero::zeropretreatment::ComputeIntegralImageNormal(*cloud, *normals);
	else
		zero::zeropretreatment::SmoothingNormal(*cloud, *normals, true, true, 0.05);
	

	cout << "pretreat ok" << endl;
	zero::ZEROViewer viewer;
	std::string title = "Zero Viewer";
	viewer.viewerinit(title, 0.5, 0.5, 1.0, false);
	viewer.viewer_cloud(*cloud, true, 2.0);
	viewer.viewer_normals(*cloud, *normals, 50, 0.02);
	viewer.viewerrun();
	*/

	return (0);
}