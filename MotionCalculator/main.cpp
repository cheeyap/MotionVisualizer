#include <iostream>
#include <Eigen/Geometry>
#include "MotionCalculator.h"

#include <map>
#include <vector>
#include <algorithm>


// overload
float* GLMatFromEigenMat(Eigen::Matrix3f &mat)
{
	int n = 3; 
	Eigen::Matrix3f mat_t = mat.transpose();
	float* GLmat = new float[n*n];
	for (int i = 0; i < n*n; i++)
		GLmat[i] = mat_t((i - i % n) / n, i % n);
	return GLmat;
}


float* GLMatFromEigenMat(Eigen::Matrix4f &mat)
{
	int n = 4;
	Eigen::Matrix4f mat_t = mat.transpose();
	float* GLmat = new float[n*n];
	for (int i = 0; i < n*n; i++)
		GLmat[i] = mat_t((i - i % n) / n, i % n);
	return GLmat;
}


int main(int argc, char** argv)
{
	std::cout << "------------MotionCalculator--------------------------" << std::endl;
	std::cout << "------------1. Test basic Matrix Operation -----------" << std::endl;
	std::cout << "Tyler" << std::endl;
	
	std::cout << "Rotation R (Euler Angle)" << std::endl;
	Eigen::Matrix3f R_x; R_x = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX());
	Eigen::Matrix3f R_y; R_y = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY());
	Eigen::Matrix3f R_z; R_z = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
	Eigen::Matrix3f R = R_x*R_y*R_z;
	std::cout << R << std::endl;

	std::cout << "Translation t" << std::endl;
	Eigen::Vector3f t; t << 1, 2, 3;
	std::cout << t << std::endl;

	std::cout << "Transform M" << std::endl;
	Eigen::Matrix4f M; M.setZero();
	std::cout << M << std::endl;
	M.block<3, 3>(0, 0) = R;
	M.block<3, 1>(0, 3) = t;
	M(3, 3) = 1;
	std::cout << M << std::endl;

	std::cout << "Transform Chain M0 M1 M2" << std::endl;
	Eigen::Matrix4f M0; M0.setZero();
	Eigen::Matrix4f M1; M1.setZero();
	Eigen::Matrix4f M2; M2.setZero();

	Eigen::Matrix4f M10; M10 = M0;
	Eigen::Matrix4f M21; M21 = M1;
	Eigen::Matrix4f M32; M32 = M2;
	Eigen::Matrix4f M30; M30 = M10*M21*M32;
	// multiply (0,0,0,1) we get the origin
	// multiply (1,0,0,0) we get x axis
	// multiply (0,1,0,0) we get y axis
	// multiply (0,0,1,0) we get z axis

	std::cout << std::endl << std::endl;
	std::cout << "------------2. Test 2D Bundle Adjustment -----------" << std::endl;
	std::cout << "Tyler" << std::endl;

	// world points
	Points points;
	points.push_back(Point(0, 10));
	points.push_back(Point(10, 20));
	points.push_back(Point(0, 30));
	points.push_back(Point(-10, 20));
	points.push_back(Point(-20, 20));
	points.push_back(Point(20, 20));

	// ground truth camera
	std::vector<Camera> cameras;
	cameras.push_back(Camera(Point(-20, -50)));
	cameras.push_back(Camera(Point(-10, -50)));
	cameras.push_back(Camera(Point(0, -40)));
	cameras.push_back(Camera(Point(10, -40)));
	cameras.push_back(Camera(Point(20, -30)));

	// perturbation cameras' position
	std::vector<Camera> perturbed_cameras;
	perturbed_cameras.push_back(Camera(Point(-25, -51)));
	perturbed_cameras.push_back(Camera(Point(-12, -51)));
	perturbed_cameras.push_back(Camera(Point(-2, -38)));
	perturbed_cameras.push_back(Camera(Point(11, -42)));
	perturbed_cameras.push_back(Camera(Point(22, -31)));

	// take photos using ground truth cameras
	std::vector<Frame> photos;
	for (auto c : cameras)
		photos.push_back(c.capture(points));


	for (auto c : cameras)
		std::cout << c.position << std::endl << std::endl;
	std::cout << std::endl;

	for (auto c : perturbed_cameras)
		std::cout << c.position << std::endl << std::endl;
	std::cout << std::endl;

	for (auto f : photos)
	{
		std::cout << "A photo:" << std::endl << std::endl;
		for (auto p : f)
			std::cout << p << std::endl;
		std::cout << std::endl;
	}

	// randomly adjust camera one by one
	// use finite difference to estimate Jacobian
	for (int i = 0; i < perturbed_cameras.size(); i++)
	{
		// Construct Jacobian 
		Eigen::MatrixXf J(photos[i].size(), 2);
		Eigen::VectorXf E(photos[i].size());
		// for each visible points 
		int j = 0;
		for (auto p : photos[i])
		{
			// error
			ProjectedPoint U = perturbed_cameras[i].project(p);

			// calculate dUdt1
			Camera C_dt1 = Camera(perturbed_cameras[i].position);
			C_dt1.position += Translation(0.01f, 0.0f);
			float dU_t1 = C_dt1.project(p)-U;

			// calculate dUdt2
			Camera C_dt2 = Camera(perturbed_cameras[i].position);
			C_dt2.position += Translation(0.0f, 0.01f);
			float dU_t2 = C_dt2.project(p)-U;

			E(j) = U;
			J(j, 0) = dU_t1 / 0.01;
			J(j, 1) = dU_t2 / 0.01;
			j++;
		}

		std::cout << "E" << std::endl;
		std::cout << E << std::endl;

		std::cout << "J" << std::endl;
		std::cout << J << std::endl;

		Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
		std::cout << perturbed_cameras[i].position << "--->";
		perturbed_cameras[i].position += t;
		std::cout << perturbed_cameras[i].position << std::endl;
		std::cout << t << std::endl << std::endl;
	}



	// pertube points position
	// randomly adjust a point


	int wait;
	std::cin >> wait;
}