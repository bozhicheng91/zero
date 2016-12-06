#include "stdafx.h"
#ifndef ZERO_REGISTRATION_H_
#define ZERO_REGISTRATION_H_

#include "src/modules/Zero_exports.h"
#include "src/modules/Zero_Common.h"

namespace zero
{
	class ZERO_EXPORTS ZERORegistration
	{
	public:
		ZERORegistration() {}
		~ZERORegistration() {}
		// ICP算法
		template<typename PointT>
		void zeroicp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::IterativeClosestPoint<PointT, PointT> &icp,
			int iterations,
			double corrd_dis)
		{
			icp.setMaximumIterations(iterations);
			icp.setInputSource(target.makeShared());
			icp.setInputTarget(source.makeShared());
			icp.setMaxCorrespondenceDistance(corrd_dis); // 设置对应点对之间的最大距离(此值对配准结果影响较大)
			icp.setEuclideanFitnessEpsilon(0.00001); // 设置收敛条件是均方误差和小于阈值，停止迭代，当迭代次数不为1时，才有用
			icp.setTransformationEpsilon(1e-10); // 设置两次变化矩阵之间的插值(一般设置为1e-10即可)
			icp.align(target);
		}
		// 预处理source与target后ICP算法
		template<typename PointT>
		void zeropretreatsourcetargeticp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(source, *simplify_source);
			}
			else
			{
				*simplify_source = source;
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(target, *simplify_target);
			}
			else
			{
				*simplify_target = target;
			}

			double corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);
			pcl::IterativeClosestPoint<PointT, PointT> icp;
			zero::zeroregistration::ICP(*simplify_source, *simplify_target, icp, 1, corrd_dis);

			int i = 0;
			double before_fit = icp.getFitnessScore();
			double after_fit = before_fit;
			while (i < iterations)
			{
				double dfit = icp.getFitnessScore();
				if (dfit <= 1e-6)
				{
					break;
				}
				if (i != 0 && i % 10 == 0)
				{
					after_fit = icp.getFitnessScore();
					if (before_fit - after_fit <= 1e-6)
					{
						break;
					}
					else
					{
						before_fit = after_fit;
					}
				}
				corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);
				icp.setMaxCorrespondenceDistance(corrd_dis);
				icp.align(*simplify_target);
				i++;
			}	
			pcl::PointCloud<PointT>::Ptr new_target(new pcl::PointCloud<PointT>);
			*new_target = target;
			target.clear();
			pcl::transformPointCloud(*new_target, target, icp.getFinalTransformation());
		}
		// NDT配准(效果不如ICP好)
		template<typename PointT>
		void zerondt(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::NormalDistributionsTransform<PointT, PointT> &ndt,
			int iterations,
			double resolution,
			double stepsize)
		{
			ndt.setMaximumIterations(iterations);
			// 设置voxel grid的分辨率，也就是体素大小
			ndt.setResolution(resolution);
			// 设置两次变换的最大允许差异
			ndt.setTransformationEpsilon(1e-10);
			// 设置或改变牛顿线搜索的最大步长
			ndt.setStepSize(stepsize);
			ndt.setInputSource(target.makeShared());
			ndt.setInputTarget(source.makeShared());
			ndt.align(target);
		}
		template<typename PointT>
		void zeropretreatsourcetargetndt(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations,
			double resolution,
			double stepsize)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(source, *simplify_source);
			}
			else
			{
				*simplify_source = source;
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(target, *simplify_target);
			}
			else
			{
				*simplify_target = target;
			}

			pcl::NormalDistributionsTransform<PointT, PointT> ndt;
			zero::zeroregistration::NDT(*simplify_source, *simplify_target, ndt, iterations, resolution, stepsize);
			pcl::transformPointCloud(target, target, ndt.getFinalTransformation());
		}
		// GICP广义ICP算法（效果时间都不如ICP好）
		template<typename PointT>
		void zeropretreatsourcetargetgicp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(source, *simplify_source);
			}
			else
			{
				*simplify_source = source;
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(target, *simplify_target);
			}
			else
			{
				*simplify_target = target;
			}
			
			double corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);

			pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
			gicp.setInputSource(simplify_target);
			gicp.setInputTarget(simplify_source);
			gicp.setMaximumIterations(1);
			gicp.setMaxCorrespondenceDistance(corrd_dis);
			gicp.align(*simplify_target);

			int i = 0;
			double before_fit = gicp.getFitnessScore();
			double after_fit = before_fit;
			while (i < iterations)
			{
				double dfit = gicp.getFitnessScore();
				if (dfit <= 1e-5)
				{
					break;
				}
				if (i != 0 && i % 10 == 0)
				{
					after_fit = gicp.getFitnessScore();
					if (before_fit - after_fit <= 1e-5)
					{
						break;
					}
					else
					{
						before_fit = after_fit;
					}
				}
				corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);
				gicp.setMaxCorrespondenceDistance(corrd_dis);
				gicp.align(*simplify_target);
				i++;
			}

			pcl::transformPointCloud(target, target, gicp.getFinalTransformation());
		}
		// ICP NONLINEAR算法（未测试）
		template<typename PointNT>
		void zeropretreatsourcetargeticpnonlinear(pcl::PointCloud<PointNT>& source,
			pcl::PointCloud<PointNT>& target,
			int iterations,
			double trans_eps)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(source, *simplify_source);
			}
			else
			{
				*simplify_source = source;
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(target, *simplify_target);
			}
			else
			{
				*simplify_target = target;
			}
			double corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);

			pcl::IterativeClosestPointNonLinear<PointNT, PointNT> icp;
			icp.setInputSource(simplify_target);
			icp.setInputTarget(simplify_source);
			icp.setMaximumIterations(1);
			icp.setMaxCorrespondenceDistance(corrd_dis);
			icp.align(*simplify_target);

			int i = 0;
			double before_fit = icp.getFitnessScore();
			double after_fit = before_fit;
			while (i < iterations)
			{
				double dfit = icp.getFitnessScore();
				if (dfit <= 1e-5)
				{
					break;
				}
				if (i != 0 && i % 10 == 0)
				{
					after_fit = icp.getFitnessScore();
					if (before_fit - after_fit <= 1e-5)
					{
						break;
					}
					else
					{
						before_fit = after_fit;
					}
				}
				corrd_dis = zero::zerocommon::computedisclouds(*simplify_source, *simplify_target);
				icp.setMaxCorrespondenceDistance(corrd_dis);
				icp.align(*simplify_target);
				i++;
			}

			pcl::transformPointCloud(target, target, icp.getFinalTransformation());
		}
	};

	namespace zeroregistration
	{
		// ICP算法
		template<typename PointT>
		void ICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::IterativeClosestPoint<PointT, PointT> &icp,
			int iterations = 30,
			double corrd_dis = 1.0)
		{
			zero::ZERORegistration r;
			r.zeroicp(source, target, icp, iterations, corrd_dis);
		}
		// 预处理source与target后ICP算法
		template<typename PointT>
		void PretreatSourceTargetICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations = 30)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcetargeticp(source, target, iterations);
		}
		// NDT配准
		template<typename PointT>
		void NDT(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::NormalDistributionsTransform<PointT, PointT> &ndt,
			int iterations = 40,
			double resolution = 1.0,
			double stepsize = 0.1)
		{
			zero::ZERORegistration r;
			r.zerondt(source, target, ndt, iterations, resolution, stepsize);
		}
		template<typename PointT>
		void PretreatSourceTargetNDT(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations = 40,
			double resolution = 1.0,
			double stepsize = 0.1)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcetargetndt(source, target, iterations, resolution, stepsize);
		}
		template<typename PointT>
		void PretreatSourceTargetGICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			int iterations = 30)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcetargetgicp(source, target, iterations);
		}
	}
}

#endif