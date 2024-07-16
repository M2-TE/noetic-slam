#pragma once
#include <Eigen/Eigen>

// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
static Eigen::Vector3f normal_from_neighbourhood(std::vector<Eigen::Vector3f>& points) {
	// calculate centroid by through coefficient average
	Eigen::Vector3f centroid { 0, 0, 0 };
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		centroid += *p;
	}
	double recip = 1.0 / (double)points.size();
	centroid *= recip;

	// covariance matrix excluding symmetries
	double xx = 0.0; double xy = 0.0; double xz = 0.0;
	double yy = 0.0; double yz = 0.0; double zz = 0.0;
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		auto r = *p - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	xx *= recip;
	xy *= recip;
	xz *= recip;
	yy *= recip;
	yz *= recip;
	zz *= recip;

	// weighting linear regression based on square determinant
	Eigen::Vector3f weighted_dir = { 0, 0, 0 };

	// determinant x
	{
		double det_x = yy*zz - yz*yz;
		Eigen::Vector3f axis_dir = {
			(float)(det_x),
			(float)(xz*yz - xy*zz),
			(float)(xy*yz - xz*yy)
		};
		double weight = det_x * det_x;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant y
	{
		double det_y = xx*zz - xz*xz;
		Eigen::Vector3f axis_dir = {
			(float)(xz*yz - xy*zz),
			(float)(det_y),
			(float)(xy*xz - yz*xx)
		};
		double weight = det_y * det_y;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant z
	{
		double det_z = xx*yy - xy*xy;
		Eigen::Vector3f axis_dir = {
			(float)(xy*yz - xz*yy),
			(float)(xy*xz - yz*xx),
			(float)(det_z)
		};
		double weight = det_z * det_z;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}

	// return normalized weighted direction as surface normal
	weighted_dir.normalize();
	return weighted_dir;
}
static Eigen::Vector3f normal_from_neighbourhood(std::vector<Eigen::Vector4d>& points) {
	// calculate centroid by through coefficient average
	Eigen::Vector4d centroid { 0, 0, 0, 0 };
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		centroid += *p;
	}
	double recip = 1.0 / (double)points.size();
	centroid *= recip;

	// covariance matrix excluding symmetries
	double xx = 0.0; double xy = 0.0; double xz = 0.0;
	double yy = 0.0; double yz = 0.0; double zz = 0.0;
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		auto r = *p - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	xx *= recip;
	xy *= recip;
	xz *= recip;
	yy *= recip;
	yz *= recip;
	zz *= recip;

	// weighting linear regression based on square determinant
	Eigen::Vector4d weighted_dir = { 0, 0, 0, 0 };

	// determinant x
	{
		double det_x = yy*zz - yz*yz;
		Eigen::Vector4d axis_dir = {
			det_x,
			xz*yz - xy*zz,
			xy*yz - xz*yy,
			0.0
		};
		double weight = det_x * det_x;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant y
	{
		double det_y = xx*zz - xz*xz;
		Eigen::Vector4d axis_dir = {
			xz*yz - xy*zz,
			det_y,
			xy*xz - yz*xx,
			0.0
		};
		double weight = det_y * det_y;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant z
	{
		double det_z = xx*yy - xy*xy;
		Eigen::Vector4d axis_dir = {
			xy*yz - xz*yy,
			xy*xz - yz*xx,
			det_z,
			0.0
		};
		double weight = det_z * det_z;
		if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}

	// return normalized weighted direction as surface normal
	weighted_dir.normalize();
	return { (float)weighted_dir.x(), (float)weighted_dir.y(), (float)weighted_dir.z() };
}