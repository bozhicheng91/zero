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
		// ICP�㷨
		template<typename PointT>
		void zeroicp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::IterativeClosestPoint<PointT, PointT> &icp,
			double iterations,
			double ecul_fit,
			double trans_eps,
			double corrd_dis)
		{
			icp.setMaximumIterations(iterations);
			icp.setInputSource(source.makeShared());
			icp.setInputTarget(target.makeShared());
			icp.setMaxCorrespondenceDistance(corrd_dis); // ���ö�Ӧ���֮���������(��ֵ����׼���Ӱ��ϴ�)
			icp.setEuclideanFitnessEpsilon(ecul_fit); // �������������Ǿ�������С����ֵ��ֹͣ������������������Ϊ1ʱ��������
			icp.setTransformationEpsilon(trans_eps); // �������α仯����֮��Ĳ�ֵ(һ������Ϊ1e-10����)
			icp.align(source);
		}
		// Ԥ����source��ICP�㷨
		template<typename PointT>
		void zeropretreatsourceicp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations,
			double ecul_fit,
			double trans_eps)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr backup_source(new pcl::PointCloud<PointT>);
			*backup_source = source;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
				std::cout << "source simplify:" << simplify_source->points.size() << std::endl;
			}

			double corrd_dis = 1000 * compute_coord_dis(source, target);
			pcl::IterativeClosestPoint<PointT, PointT> icp;
			zero::zeroregistration::ICP(*simplify_source, target, icp, iterations, ecul_fit, trans_eps, corrd_dis);
			std::cout << "icp test\n";

			pcl::transformPointCloud(*backup_source, source, icp.getFinalTransformation());
		}
		// Ԥ����source��target��ICP�㷨
		template<typename PointT>
		void zeropretreatsourcetargeticp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations,
			double ecul_fit,
			double trans_eps)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr backup_source(new pcl::PointCloud<PointT>);
			*backup_source = source;
			pcl::PointCloud<PointT>::Ptr backup_target(new pcl::PointCloud<PointT>);
			*backup_target = target;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_target, *simplify_target);
			}

			double corrd_dis = 1000 * compute_coord_dis(*simplify_source, *simplify_target);
			pcl::IterativeClosestPoint<PointT, PointT> icp;
			zero::zeroregistration::ICP(*simplify_source, *simplify_target, icp, iterations, corrd_dis);
			
			pcl::transformPointCloud(*backup_source, source, icp.getFinalTransformation());
		}
		// NDT��׼(Ч������ICP��)
		template<typename PointT>
		void zerondt(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::NormalDistributionsTransform<PointT, PointT> &ndt,
			double iterations,
			double resolution,
			double trans_eps,
			double stepsize)
		{
			ndt.setMaximumIterations(iterations);
			// ����voxel grid�ķֱ��ʣ�Ҳ�������ش�С
			ndt.setResolution(resolution);
			// �������α任������������
			ndt.setTransformationEpsilon(trans_eps);
			// ���û�ı�ţ������������󲽳�
			ndt.setStepSize(stepsize);
			ndt.setInputSource(source.makeShared());
			ndt.setInputTarget(target.makeShared());
			ndt.align(source);
		}
		template<typename PointT>
		void zeropretreatsourcendt(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations,
			double resolution,
			double trans_eps,
			double stepsize)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr backup_source(new pcl::PointCloud<PointT>);
			*backup_source = source;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
			}

			pcl::NormalDistributionsTransform<PointT, PointT> ndt;
			zero::zeroregistration::NDT(*simplify_source, target, ndt, iterations, resolution, trans_eps, stepsize);

			pcl::transformPointCloud(*backup_source, source, ndt.getFinalTransformation());
		}
		template<typename PointT>
		void zeropretreatsourcetargetndt(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations,
			double resolution,
			double trans_eps,
			double stepsize)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr simplify_target(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr backup_source(new pcl::PointCloud<PointT>);
			*backup_source = source;
			pcl::PointCloud<PointT>::Ptr backup_target(new pcl::PointCloud<PointT>);
			*backup_target = target;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
			}
			if (target.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_target, *simplify_target);
			}

			pcl::NormalDistributionsTransform<PointT, PointT> ndt;
			zero::zeroregistration::NDT(*simplify_source, *simplify_target, ndt, iterations, resolution, trans_eps, stepsize);

			pcl::transformPointCloud(*backup_source, source, ndt.getFinalTransformation());
		}
		// GICP����ICP�㷨��Ч��ʱ�䶼����ICP�ã�
		template<typename PointT>
		void zeropretreatsourcegicp(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations)
		{
			pcl::PointCloud<PointT>::Ptr simplify_source(new pcl::PointCloud<PointT>);
			pcl::PointCloud<PointT>::Ptr backup_source(new pcl::PointCloud<PointT>);
			*backup_source = source;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
			}
			
			double corrd_dis = 1000 * compute_coord_dis(source, target);

			pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
			gicp.setInputSource(simplify_source);
			gicp.setInputTarget(target.makeShared());
			gicp.setMaximumIterations(iterations);
			gicp.setMaxCorrespondenceDistance(corrd_dis);
			gicp.align(*simplify_source);

			pcl::transformPointCloud(*backup_source, source, gicp.getFinalTransformation());
		}
		// ICP NONLINEAR�㷨��δ���ԣ�
		template<typename PointNT>
		void zeropretreatsourceicpnonlinear(pcl::PointCloud<PointNT>& source,
			pcl::PointCloud<PointNT>& target,
			double iterations,
			double trans_eps)
		{
			pcl::PointCloud<PointNT>::Ptr simplify_source(new pcl::PointCloud<PointNT>);
			pcl::PointCloud<PointNT>::Ptr backup_source(new pcl::PointCloud<PointNT>);
			*backup_source = source;
			if (source.points.size() > 10000)
			{
				zero::zeropretreatment::VoxelGridSimplify(*backup_source, *simplify_source);
			}
			double corrd_dis = 1000 * compute_coord_dis(source, target);

			pcl::IterativeClosestPointNonLinear<PointNT, PointNT> icp;
			icp.setInputSource(simplify_source);
			icp.setInputTarget(target.makeShared());
			icp.setMaximumIterations(iterations);
			icp.setMaxCorrespondenceDistance(corrd_dis);
			//icp.setPointRepresentation(mypointrepresentation);
			icp.align(*simplify_source);

			pcl::transformPointCloud(*backup_source, source, icp.getFinalTransformation());
		}

	private:
		template<typename PointT> 
		double compute_distance_between_clouds(pcl::PointCloud<PointT>& source, 
			pcl::PointCloud<PointT>& target)
		{
			PointT centroid_s, centroid_t;
			zero::zerocommon::Compute3dCenter(source, centroid_s);
			zero::zerocommon::Compute3dCenter(target, centroid_t);

			return (zero::zerocommon::ComputePDistance(centroid_s, centroid_t));
		}
	};

	namespace zeroregistration
	{
		// ICP�㷨
		template<typename PointT>
		void ICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::IterativeClosestPoint<PointT, PointT> &icp,
			double iterations = 30,
			double ecul_fit = 0.001,
			double trans_eps = 1e-10,
			double corrd_dis = 100)
		{
			zero::ZERORegistration r;
			r.zeroicp(source, target, icp, iterations, ecul_fit, trans_eps, corrd_dis);
		}
		// Ԥ����source��ICP�㷨
		template<typename PointT>
		void PretreatSourceICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations = 30,
			double ecul_fit = 0.001,
			double trans_eps = 1e-10)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourceicp(source, target, iterations, ecul_fit, trans_eps);
		}
		// Ԥ����source��target��ICP�㷨
		template<typename PointT>
		void PretreatSourceTargetICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations = 30,
			double ecul_fit = 0.001,
			double trans_eps = 1e-10)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcetargeticp(source, target, iterations, ecul_fit, trans_eps);
		}
		// NDT��׼
		template<typename PointT>
		void NDT(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			pcl::NormalDistributionsTransform<PointT, PointT> &ndt,
			double iterations = 40,
			double resolution = 1.0,
			double trans_eps = 0.01,
			double stepsize = 0.1)
		{
			zero::ZERORegistration r;
			r.zerondt(source, target, ndt, iterations, resolution, trans_eps, stepsize);
		}
		template<typename PointT>
		void PretreatSourceNDT(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations = 40,
			double resolution = 1.0,
			double trans_eps = 0.01,
			double stepsize = 0.1)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcendt(source, target, iterations, resolution, trans_eps, stepsize);
		}
		template<typename PointT>
		void PretreatSourceTargetNDT(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations = 40,
			double resolution = 1.0,
			double trans_eps = 0.01,
			double stepsize = 0.1)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcetargetndt(source, target, iterations, resolution, trans_eps, stepsize);
		}
		template<typename PointT>
		void PretreatSourceGICP(pcl::PointCloud<PointT>& source,
			pcl::PointCloud<PointT>& target,
			double iterations = 30)
		{
			zero::ZERORegistration r;
			r.zeropretreatsourcegicp(source, target, iterations);
		}
	}
}

#endif