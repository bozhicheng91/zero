#include "stdafx.h"
#ifndef ZERO_SIMPLIFY_H_
#define ZERO_SIMPLIFY_H_

#include "src/modules/Zero_exports.h"

namespace zero
{
	class ZERO_EXPORTS ZEROPretreatment
	{
	public:
		ZERO_EXPORTS ZEROPretreatment() {}
		~ZERO_EXPORTS ZEROPretreatment() {}
		// 平面过滤器(已验证)
		template<typename PointT>
		int filterpassthrough(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::string name,
			float min, float max,
			bool LN_flag)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			pcl::PassThrough<PointT> pass;
			pass.setInputCloud(cloud_in.makeShared());
			pass.setFilterFieldName(name);//过滤的轴向
			pass.setFilterLimits(min, max);//过滤范围
			pass.setFilterLimitsNegative(LN_flag);//false得到范围内的点，true得到范围之外的点
			pass.filter(cloud_out);
			
			return 0;
		}
		// 条件过滤器,可指定坐标轴、条件(大于小于)、范围(已验证)
		template<typename PointT>
		int filterconditional(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::string name,
			pcl::ComparisonOps::CompareOp min_flag, float min,
			pcl::ComparisonOps::CompareOp max_flag, float max,
			bool keep_organized)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			pcl::ConditionAnd<PointT>::Ptr range_cond(new
				pcl::ConditionAnd<PointT>());
			range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
				pcl::FieldComparison<PointT>(name, min_flag, min)));
			range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
				pcl::FieldComparison<PointT>(name, max_flag, max)));
			pcl::ConditionalRemoval <PointT> condrem;
			condrem.setCondition(range_cond);
			condrem.setInputCloud(cloud_in.makeShared());
			condrem.setKeepOrganized(keep_organized);
			condrem.filter(cloud_out);

			return 0;
		}
		// 离群点去除，邻域法,从点云中删除r邻域范围内点数小于k的点(已验证)
		template<typename PointT>
		int radiusoutlierremoval(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double r,
			int k)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			pcl::RadiusOutlierRemoval<PointT> outrem;
			outrem.setInputCloud(cloud_in.makeShared());
			outrem.setRadiusSearch(r);
			outrem.setMinNeighborsInRadius(k);
			outrem.filter(cloud_out);

			return 0;
		}
		// 移除离群点，统计离群点去除能够更细化。(已验证)
		//首先，计算每个点的K近邻中值距离
		//然后，假想结果符合一个中值为u标准差为a的高斯分布
		// 删除中值距离在[u-a, u+a]范围外的点
		template<typename PointT>
		int staticaloutlierremoval(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			int k,
			double mt,
			bool outliter_flag)
		{
			if (cloud_in.size() == 0)
			{
				return (1);
			}

			pcl::StatisticalOutlierRemoval<PointT> sor;
			sor.setInputCloud(cloud_in.makeShared());
			sor.setMeanK(k);//设置中心距离估计时k近邻点集个数
			sor.setStddevMulThresh(mt);//为距离阈值计算设置标准的倍增器
			sor.setNegative(outliter_flag);//true得到离群点，false得到除离群点外的点
			sor.filter(cloud_out);
			//pcl::IndicesPtr indices;
			//indices = sor.getRemovedIndices();
			return 0;
		}
		// VoxeGrid下采样，均匀精简(已验证)
		template<typename PointT> 
		int voxelgridsimplify(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double scale)
		{
			if (cloud_in.size() == 0)
			{
				return (1);
			}

			// 极大值、极小值点
			PointT min, max;
			pcl::getMinMax3D(cloud_in, min, max);
			float xs = fabs(max.x - min.x) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			float ys = fabs(max.y - min.y) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			float zs = fabs(max.z - min.z) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			std::cout << xs << ys << zs << std::endl;
			// 精简
			pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize(xs, ys, zs);
			approximate_voxel_filter.setInputCloud(cloud_in.makeShared());
			approximate_voxel_filter.filter(cloud_out);

			return 0;
		}
		// 统一采样(已验证)
		template<typename PointT>
		int uniformsimplify(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double r)
		{
			if (cloud_in.size() == 0)
			{
				return (1);
			}

			pcl::UniformSampling<PointT> filter;
			filter.setInputCloud(cloud_in.makeShared());
			filter.setRadiusSearch(r);
			filter.filter(cloud_out);
			return 0;
		}
		// 上采样是曲面重建的一种形式，适用于点数较少的点云(已验证,未得到结果)
		// 该方法属于插值法，结果并不是100%正确，但是
		// PCL利用移动最小二乘曲面算法
		template<typename PointT>
		int upsampling(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<PointT>& cloud_filtered,
			double kr,
			double ur,
			double stepsize)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			pcl::MovingLeastSquares<PointT, PointT> filter;
			filter.setInputCloud(cloud.makeShared());
			pcl::search::KdTree<PointT>::Ptr kdtree;
			filter.setSearchMethod(kdtree);
			filter.setSearchRadius(kr);
			// NONE:无上采样，输入点云将映射到MLS曲面上
			//SAMPLE_LOCAL_PLANE:利用upsamping_radius_与upsampling_step_参数以环形方式上采样每个点的局部平面
			//RANDOM_UNIFORM_DENSITY:利用一个统一随机分布上采样每个点的局部平面，以至于遍历点云时每个点的密度为常量，该常量由desired_num_points_in_radius_参数设定
			//VOXEL_GRID_DILATION:输入点云插入到大小为voxel_size_的体素网格中；该体素网格将被扩大dilation_iteration_num_次，结果点云将被映射为输入点云最近点的MLS曲面；结果是一个带有填充孔与恒定点密度的点云
			filter.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
			filter.setUpsamplingRadius(ur);
			filter.setUpsamplingStepSize(stepsize);
			filter.process(cloud_filtered);

			return 0;
		}
		// 计算某个点的法向量(已验证)
		template<typename PointT>
		int computepointsnormal(const pcl::PointCloud<PointT> &cloud,
			PointT &point,
			pcl::Normal &normal,
			int k)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			std::vector<int> indices(k);
			std::vector<float> SDistance(k);

			pcl::KdTreeFLANN<PointT> kdtree;
			kdtree.setInputCloud(cloud.makeShared());

			kdtree.nearestKSearch(point, k, indices, SDistance);
			Eigen::Vector4f plane_parameters;
			float curvature;
			pcl::computePointNormal(cloud, indices, plane_parameters, curvature);

			//normal(plane_parameters[0], plane_parameters[1], plane_parameters[2]);
			normal.normal_x = plane_parameters[0];
			normal.normal_y = plane_parameters[1];
			normal.normal_z = plane_parameters[2];
			
			normal.curvature = curvature;
			std::vector<float>(SDistance).swap(SDistance);
			std::vector<int>(indices).swap(indices);

			return 0;
		}
		// 计算点云的法向量，PCA方法,任何类型点云(已验证)
		template<typename PointT>
		int computecloudnormal(const pcl::PointCloud<PointT> &cloud,
			pcl::PointCloud<pcl::Normal>& cloud_normal,
			int k,
			double r)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			if (k > 10)
			{
				r = 0.0;
			}
			if (r <= 1e-5 && k < 10)
			{
				k = 20;
			}

			pcl::NormalEstimation<PointT, pcl::Normal> ne;
			ne.setInputCloud(cloud.makeShared());
			pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
			ne.setSearchMethod(tree);
			if (k < 3)
				ne.setRadiusSearch(r);

			if (r <= 1e-10)
				ne.setKSearch(k);
			Eigen::Vector4f centriod;
			pcl::compute3DCentroid(cloud, centriod);
			ne.setViewPoint(centriod[0], centriod[1], centriod[2]);

			ne.compute(cloud_normal);

			return 0;
		}
		//计算点云法向量，积分图像法(已验证)
		template<typename PointT>
		int computeintegralimagenormal(const pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::Normal>& normal,
			float df,
			float smoothsize)
		{
			if (cloud.size() == 0)
			{
				return (1);
			}

			pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
			ne.setInputCloud(cloud.makeShared());
			// 其它估计方法: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT
			// COVARIANCE_MATRIX：创建9个积分图像计算一个邻域协方差矩阵中点的法向量
			// AVERAGE_3D_GRADIENT：创建6个积分图像计算水平和垂直的3D梯度的光滑版本，并利用这个两个梯度的叉乘计算法向量
			// AVERAGE_DEPTH_CHANGE：创建一个积分图像，并利用平均深度变化计算法向量
			// AVERAGE_3D_GRADIENT
			ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
			// 用于基于深度变化计算物体边界的深度阈值
			ne.setMaxDepthChangeFactor(df);
			// 用于光顺法向量的区域大小影响因子
			ne.setNormalSmoothingSize(smoothsize);
			ne.compute(normal);

			return 0;
		}
		//利用多项式重建光滑点云法向量(已验证，运行效率太慢)
		template<typename PointT>
		int smoothingnormal(const pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::PointXYZRGBNormal>& normals,/*BZC*/
			bool normal_f,
			bool polynomialfit_f,
			double r)
		{
			pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;
			mls.setComputeNormals(normal_f);
			mls.setInputCloud(cloud.makeShared());
			mls.setPolynomialFit(polynomialfit_f);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(r);

			mls.process(normals);

			return 0;
		}
	};

	namespace zeropretreatment
	{
		// 平面过滤器
		template<typename PointT>
		int FilterPassThrough(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::string name,
			float min, float max,
			const bool LN_flag = false)
		{
			zero::ZEROPretreatment p;
			if (p.filterpassthrough(cloud_in, cloud_out, name, min, max, LN_flag) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// 条件过滤器,可指定坐标轴、条件(大于小于)、范围
		template<typename PointT>
		int FilterConditional(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::string name,
			pcl::ComparisonOps::CompareOp min_flag, float min,
			pcl::ComparisonOps::CompareOp max_flag, float max,
			bool keep_organized = true)
		{
			zero::ZEROPretreatment p;
			if (p.filterconditional(cloud_in, cloud_out, name, min_flag, min, max_flag, max, keep_organized) != 0)
			{
				return 1;
			}
			else
				return 0;			
		}
		// 离群点去除，邻域法,从点云中删除r邻域范围内点数小于k的点
		template<typename PointT>
		int RadiusOutlierRemoval(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double r,
			int k)
		{
			zero::ZEROPretreatment p;
			if (p.radiusoutlierremoval(cloud_in, cloud_out, r, k) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// 移除离群点
		template<typename PointT>
		int StaticalPOutlierRemoval(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			const int K = 50,
			const double mt = 1.0,
			const bool outliter_flag = false)
		{
			zero::ZEROPretreatment p;
			if (p.staticaloutlierremoval(cloud_in, cloud_out, K, mt, outliter_flag) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// VoxeGrid下采样，均匀精简
		template<typename PointT>
		int VoxelGridSimplify(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double scale = 0.5)
		{
			zero::ZEROPretreatment p;
			if (p.voxelgridsimplify(cloud_in, cloud_out, scale) != 0)
			{
				return 1;
			}
			else
				return 0;			
		}
		// 统一采样
		template<typename PointT>
		int UniformSimplify(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double r)
		{
			zero::ZEROPretreatment p;
			if (p.uniformsimplify(cloud_in, cloud_out, r) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// 上采样是曲面重建的一种形式，适用于点数较少的点云
		// 该方法属于插值法，结果并不是100%正确，但是
		// PCL利用移动最小二乘曲面算法
		template<typename PointT>
		int UpSampling(pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<PointT>& cloud_filtered,
			double kr = 0.01,
			double ur = 0.01,
			double stepsize = 0.01)
		{
			zero::ZEROPretreatment p;
			if (p.upsampling(cloud, cloud_filtered, kr, ur, stepsize) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// 计算某个点的法向量
		template<typename PointT>
		int ComputePointsNormal(pcl::PointCloud<PointT> &cloud,
			PointT &point,
			pcl::Normal &normal,
			const int k = 20)
		{
			zero::ZEROPretreatment p;
			if (p.computepointsnormal(cloud, point, normal, k) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		// 计算点云的法向量
		template<typename PointT>
		int ComputeCloudNormal(const pcl::PointCloud<PointT> &cloud,
			pcl::PointCloud<pcl::Normal>& cloud_normal,
			int k = 1,
			double r = 0.0)
		{
			zero::ZEROPretreatment p;
			if (p.computecloudnormal(cloud, cloud_normal, k, r))
			{
				return 1;
			}
			else
				return 0;
		}
		// 利用积分图像法计算有序点云法向量
		template<typename PointT>
		int ComputeIntegralImageNormal(const pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::Normal>& normal,
			float df = 0.02f,
			float smoothsize = 10.0f)
		{
			zero::ZEROPretreatment p;
			if (p.computeintegralimagenormal(cloud, normal, df, smoothsize) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		//利用多项式重建光滑点云法向量
		template<typename PointT>
		int SmoothingNormal(const pcl::PointCloud<PointT>& cloud,
			pcl::PointCloud<pcl::PointXYZRGBNormal>& normals,
			bool normal_f = true,
			bool polynomialfit_f = true,
			double r = 0.03)
		{
			zero::ZEROPretreatment p;
			if (p.smoothingnormal(cloud, normals, normal_f, polynomialfit_f, r) != 0)
			{
				return 1;
			}
			else
				return 0;
		}
		//删除点云中的无效点
		template<typename PointT>
		void RemovaeNAN(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::vector<int>& mapping)
		{
			pcl::removeNaNFromPointCloud(cloud_in, cloud_out, mapping);
		}
		//删除点云中的无效法向量
		template<typename PointT>
		void RemovaeNANNormals(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::vector<int>& mapping)
		{
			pcl::removeNaNNormalsFromPointCloud(cloud_in, cloud_out, mapping);
		}
	}
}

#endif