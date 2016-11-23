#include "stdafx.h"
#include "src/modules/Zero_Common.h"

double zero::zerocommon::ComputeEUDistance(const Eigen::Vector4f& first,
	const Eigen::Vector4f& second)
{
	return (pcl::distances::l2(first, second));
}