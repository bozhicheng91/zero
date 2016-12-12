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
		// ƽ�������(����֤)
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
			pass.setFilterFieldName(name);//���˵�����
			pass.setFilterLimits(min, max);//���˷�Χ
			pass.setFilterLimitsNegative(LN_flag);//false�õ���Χ�ڵĵ㣬true�õ���Χ֮��ĵ�
			pass.filter(cloud_out);
			
			return 0;
		}
		// ����������,��ָ�������ᡢ����(����С��)����Χ(����֤)
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
		// ��Ⱥ��ȥ��������,�ӵ�����ɾ��r����Χ�ڵ���С��k�ĵ�(����֤)
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
		// �Ƴ���Ⱥ�㣬ͳ����Ⱥ��ȥ���ܹ���ϸ����(����֤)
		//���ȣ�����ÿ�����K������ֵ����
		//Ȼ�󣬼���������һ����ֵΪu��׼��Ϊa�ĸ�˹�ֲ�
		// ɾ����ֵ������[u-a, u+a]��Χ��ĵ�
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
			sor.setMeanK(k);//�������ľ������ʱk���ڵ㼯����
			sor.setStddevMulThresh(mt);//Ϊ������ֵ�������ñ�׼�ı�����
			sor.setNegative(outliter_flag);//true�õ���Ⱥ�㣬false�õ�����Ⱥ����ĵ�
			sor.filter(cloud_out);
			//pcl::IndicesPtr indices;
			//indices = sor.getRemovedIndices();
			return 0;
		}
		// VoxeGrid�²��������Ⱦ���(����֤)
		template<typename PointT> 
		int voxelgridsimplify(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			double scale)
		{
			if (cloud_in.size() == 0)
			{
				return (1);
			}

			// ����ֵ����Сֵ��
			PointT min, max;
			pcl::getMinMax3D(cloud_in, min, max);
			float xs = fabs(max.x - min.x) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			float ys = fabs(max.y - min.y) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			float zs = fabs(max.z - min.z) / pow(scale * cloud_in.points.size(), 1.0 / 3);
			std::cout << xs << ys << zs << std::endl;
			// ����
			pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize(xs, ys, zs);
			approximate_voxel_filter.setInputCloud(cloud_in.makeShared());
			approximate_voxel_filter.filter(cloud_out);

			return 0;
		}
		// ͳһ����(����֤)
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
		// �ϲ����������ؽ���һ����ʽ�������ڵ������ٵĵ���(����֤,δ�õ����)
		// �÷������ڲ�ֵ�������������100%��ȷ������
		// PCL�����ƶ���С���������㷨
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
			// NONE:���ϲ�����������ƽ�ӳ�䵽MLS������
			//SAMPLE_LOCAL_PLANE:����upsamping_radius_��upsampling_step_�����Ի��η�ʽ�ϲ���ÿ����ľֲ�ƽ��
			//RANDOM_UNIFORM_DENSITY:����һ��ͳһ����ֲ��ϲ���ÿ����ľֲ�ƽ�棬�����ڱ�������ʱÿ������ܶ�Ϊ�������ó�����desired_num_points_in_radius_�����趨
			//VOXEL_GRID_DILATION:������Ʋ��뵽��СΪvoxel_size_�����������У����������񽫱�����dilation_iteration_num_�Σ�������ƽ���ӳ��Ϊ�������������MLS���棻�����һ������������㶨���ܶȵĵ���
			filter.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
			filter.setUpsamplingRadius(ur);
			filter.setUpsamplingStepSize(stepsize);
			filter.process(cloud_filtered);

			return 0;
		}
		// ����ĳ����ķ�����(����֤)
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
		// ������Ƶķ�������PCA����,�κ����͵���(����֤)
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
		//������Ʒ�����������ͼ��(����֤)
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
			// �������Ʒ���: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT
			// COVARIANCE_MATRIX������9������ͼ�����һ������Э��������е�ķ�����
			// AVERAGE_3D_GRADIENT������6������ͼ�����ˮƽ�ʹ�ֱ��3D�ݶȵĹ⻬�汾����������������ݶȵĲ�˼��㷨����
			// AVERAGE_DEPTH_CHANGE������һ������ͼ�񣬲�����ƽ����ȱ仯���㷨����
			// AVERAGE_3D_GRADIENT
			ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
			// ���ڻ�����ȱ仯��������߽�������ֵ
			ne.setMaxDepthChangeFactor(df);
			// ���ڹ�˳�������������СӰ������
			ne.setNormalSmoothingSize(smoothsize);
			ne.compute(normal);

			return 0;
		}
		//���ö���ʽ�ؽ��⻬���Ʒ�����(����֤������Ч��̫��)
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
		// ƽ�������
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
		// ����������,��ָ�������ᡢ����(����С��)����Χ
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
		// ��Ⱥ��ȥ��������,�ӵ�����ɾ��r����Χ�ڵ���С��k�ĵ�
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
		// �Ƴ���Ⱥ��
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
		// VoxeGrid�²��������Ⱦ���
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
		// ͳһ����
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
		// �ϲ����������ؽ���һ����ʽ�������ڵ������ٵĵ���
		// �÷������ڲ�ֵ�������������100%��ȷ������
		// PCL�����ƶ���С���������㷨
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
		// ����ĳ����ķ�����
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
		// ������Ƶķ�����
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
		// ���û���ͼ�񷨼���������Ʒ�����
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
		//���ö���ʽ�ؽ��⻬���Ʒ�����
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
		//ɾ�������е���Ч��
		template<typename PointT>
		void RemovaeNAN(const pcl::PointCloud<PointT>& cloud_in,
			pcl::PointCloud<PointT>& cloud_out,
			std::vector<int>& mapping)
		{
			pcl::removeNaNFromPointCloud(cloud_in, cloud_out, mapping);
		}
		//ɾ�������е���Ч������
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