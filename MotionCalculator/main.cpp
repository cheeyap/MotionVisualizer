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

	/*
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
	*/
	std::cout << "------------2. Test 2D Bundle Adjustment -----------" << std::endl;

	// world points
	Points points;
	points.push_back(Point(0, 10));
	points.push_back(Point(10, 20));
	points.push_back(Point(0, 30));
	points.push_back(Point(-10, 20));
	points.push_back(Point(-20, 20));
	points.push_back(Point(20, 20));
	/*
	{{
		std::cout << "Point Cloud" << std::endl;
		for (auto p : points)
		{
			std::cout << p.transpose() << std::endl;
		}
		std::cout << std::endl;
	}}
	*/

	// perturbation points' position
	Translation PointPerturbation(3, 2);
	Eigen::MatrixXf PointPerturbations = Eigen::MatrixXf::Random(points.size(),2);
	Points perturbedPoints;
	int k = 0;
	for (auto p : points)
	{
		perturbedPoints.push_back(p + 3 * PointPerturbations.row(k).transpose());
		k = k+1;
	}
	{{
		std::cout << "Pertubed Points" << std::endl;
		for (int i = 0; i < perturbedPoints.size();i++)
		{
			auto p = perturbedPoints[i];
			std::cout <<"(" << points[i].transpose() << ") ---> ("<< p.transpose() << ")" << std::endl;
		}
		std::cout << std::endl;
	}}

	// global translation of camera group
	Translation cameraTranslation(0,-20);
	// ground truth camera
	std::vector<Camera> cameras;
	cameras.push_back(Camera(Point(-20, -20)+cameraTranslation));
	cameras.push_back(Camera(Point(-10, -20)+cameraTranslation));
	cameras.push_back(Camera(Point(0, -10)+cameraTranslation));
	cameras.push_back(Camera(Point(10, -10)+cameraTranslation));
	cameras.push_back(Camera(Point(20, -0)+cameraTranslation));
	/*
	{{
		std::cout << "Real Cameras" << std::endl;
		for (auto c : cameras)
		{
			std::cout << c.position.transpose() << std::endl;
		}
		std::cout << std::endl;
	}}
	*/

	// perturbation cameras' position
	Translation perturbation(0, 1);
	std::vector<Camera> perturbed_cameras;
	perturbed_cameras.push_back(Camera(Point(-20, -20)+cameraTranslation+Translation(1,2)));
	perturbed_cameras.push_back(Camera(Point(-10, -20) + cameraTranslation + Translation(-1, 1)));
	perturbed_cameras.push_back(Camera(Point(0, -10) + cameraTranslation + Translation(3, 1)));
	perturbed_cameras.push_back(Camera(Point(10, -10) + cameraTranslation + Translation(2, -2)));
	perturbed_cameras.push_back(Camera(Point(20, -0) + cameraTranslation + Translation(-1, 2)));
	{{
		std::cout << "Pertubed Cameras" << std::endl;
		for (int i = 0; i < perturbed_cameras.size();i++)
		{
			auto p = perturbed_cameras[i];
			std::cout << "(" << cameras[i].position.transpose() << ") ---> (" << p.position.transpose() << ")" << std::endl;
		}
		std::cout << std::endl;
	}}

	// take photos using ground truth cameras
	std::vector<Frame> photos;
	for (auto c : cameras)
		photos.push_back(c.capture(points));
	
	/*
	for (auto f : photos)
	{
		std::cout << "Points in a Photo:" << std::endl << std::endl;
		for (auto p : f)
			std::cout << p.transpose() << std::endl;
		std::cout << std::endl;
	}
	*/

	int numOfIteration = 3;

	/*
	// Adjust Cameras
	int numOfIteration = 6;
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		std::cout << std::endl;
		std::cout << "Iteration: " << iteration << std::endl;
		// randomly adjust camera one by one
		// use finite difference to estimate Jacobian
		for (int i = 0; i < perturbed_cameras.size(); i++)
		{
			// Construct Jacobian 
			Eigen::MatrixXf J(photos[i].size(), 2);
			Eigen::VectorXf E(photos[i].size());
			// for each visible points, compute one line of J and E
			int j = 0;
			for (auto p : photos[i])
			{
				// error: U-U0
				ProjectedPoint U0 = cameras[i].project(p);
				ProjectedPoint U = perturbed_cameras[i].project(p);

				// calculate dUdt1
				Camera C_dt1 = Camera(perturbed_cameras[i].position);
				C_dt1.position += Translation(0.01f, 0.0f);
				float dU_t1 = C_dt1.project(p) - U;

				// calculate dUdt2
				Camera C_dt2 = Camera(perturbed_cameras[i].position);
				C_dt2.position += Translation(0.0f, 0.01f);
				float dU_t2 = C_dt2.project(p) - U;

				E(j) = -(U - U0);// want to counter act against the difference
				J(j, 0) = dU_t1 / 0.01;
				J(j, 1) = dU_t2 / 0.01;
				j++;
			}

			//std::cout << "E" << std::endl;
			//std::cout << E << std::endl;

			//std::cout << "J" << std::endl;
			//std::cout << J << std::endl;

			Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
			//std::cout << "t:";
			//std::cout << t.transpose() << std::endl;
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ") ---> ";
			perturbed_cameras[i].position += t;
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ")";
			std::cout << " | Real Position: (" << cameras[i].position.transpose() << ")" << std::endl;
		}
	}

	// Adjust Points (There is no roation for point, since it is a single point)
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		std::cout << std::endl;
		std::cout << "Iteration: " << iteration << std::endl;
		// randomly adjust point one by one
		// use finite difference to estimate Jacobian
		for (int i = 0; i < perturbedPoints.size(); i++)
		{
			// Construct Jacobian 
			Eigen::MatrixXf J(cameras.size(), 2);
			Eigen::VectorXf E(cameras.size());
			// for each camera, compute one line of J and E
			int j = 0;
			for (int k = 0; k < cameras.size(); k++)
			{
				// error: U-U0
				ProjectedPoint U0 = cameras[k].project(points[i]);
				ProjectedPoint U = cameras[k].project(perturbedPoints[i]);

				// calculate dUdt1
				Point P_dt1 = perturbedPoints[i];
				P_dt1 += Translation(0.01f, 0.0f);
				float dP_t1 = cameras[k].project(P_dt1) - U;

				// calculate dUdt2
				Point P_dt2 = perturbedPoints[i];
				P_dt2 += Translation(0.0f, 0.01f);
				float dP_t2 = cameras[k].project(P_dt2) - U;

				E(j) = -(U - U0);// want to counter act against the difference
				J(j, 0) = dP_t1 / 0.01;
				J(j, 1) = dP_t2 / 0.01;
				j++;
			}

			//std::cout << "E" << std::endl;
			//std::cout << E << std::endl;

			//std::cout << "J" << std::endl;
			//std::cout << J << std::endl;

			Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
			//std::cout << "t:";
			//std::cout << t.transpose() << std::endl;
			std::cout << "(" << perturbedPoints[i].transpose().format(shortFmt) << ") ---> ";
			perturbedPoints[i] += t;
			std::cout << "(" << perturbedPoints[i].transpose().format(shortFmt) << ")";
			std::cout << " | Real Position: (" << points[i].transpose() << ")" << std::endl;
		}
	}
	*/

	// Adjust Points and Camera (There is no roation for point, since it is a single point)
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		std::cout << std::endl;
		std::cout << "Iteration: " << iteration << std::endl;

		int i = 0, ii = 0;
		while (i < perturbed_cameras.size() && ii < perturbedPoints.size())
		{
			if (i < perturbed_cameras.size())
			{
				std::cout << "Adjust Cameras" ;
				// Construct Jacobian 
				Eigen::MatrixXf J(photos[i].size(), 2);
				Eigen::VectorXf E(photos[i].size());
				// for each visible points, compute one line of J and E
				int j = 0;
				for (auto p : photos[i])
				{
					// serach for p
					int k;
					for (int m = 0; m < points.size(); m++)
					{
						if ((p - points[m]).norm() < 0.1)
							k = m;
					}
					// error: U-U0
					ProjectedPoint U0 = cameras[i].project(p);
					ProjectedPoint U = perturbed_cameras[i].project(perturbedPoints[k]);

					// calculate dUdt1
					Camera C_dt1 = Camera(perturbed_cameras[i].position);
					C_dt1.position += Translation(0.01f, 0.0f);
					float dU_t1 = C_dt1.project(p) - U;

					// calculate dUdt2
					Camera C_dt2 = Camera(perturbed_cameras[i].position);
					C_dt2.position += Translation(0.0f, 0.01f);
					float dU_t2 = C_dt2.project(p) - U;

					E(j) = -(U - U0);// want to counter act against the difference
					J(j, 0) = dU_t1 / 0.01;
					J(j, 1) = dU_t2 / 0.01;
					j++;
				}

				//std::cout << "E" << std::endl;
				//std::cout << E << std::endl;

				//std::cout << "J" << std::endl;
				//std::cout << J << std::endl;

				Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
				//std::cout << "t:";
				//std::cout << t.transpose() << std::endl;
				std::cout << "(" << perturbed_cameras[i].position.transpose() << ") ---> ";
				perturbed_cameras[i].position += t;
				std::cout << "(" << perturbed_cameras[i].position.transpose() << ")";
				std::cout << " | Real Position: (" << cameras[i].position.transpose() << ")" << std::endl;
				i = i + 1;
			}

			if (ii < perturbedPoints.size())
			{
				std::cout << "Adjust Points" ;
				// randomly adjust point one by one
				// use finite difference to estimate Jacobian
				// Construct Jacobian 
				Eigen::MatrixXf J(perturbed_cameras.size(), 2);
				Eigen::VectorXf E(perturbed_cameras.size());
				// for each camera, compute one line of J and E
				int j = 0;
				for (int k = 0; k < perturbed_cameras.size(); k++)
				{
					// error: U-U0
					ProjectedPoint U0 = cameras[k].project(points[ii]);
					ProjectedPoint U = perturbed_cameras[k].project(perturbedPoints[ii]);

					// calculate dUdt1
					Point P_dt1 = perturbedPoints[ii];
					P_dt1 += Translation(0.01f, 0.0f);
					float dP_t1 = perturbed_cameras[k].project(P_dt1) - U;

					// calculate dUdt2
					Point P_dt2 = perturbedPoints[ii];
					P_dt2 += Translation(0.0f, 0.01f);
					float dP_t2 = perturbed_cameras[k].project(P_dt2) - U;

					E(j) = -(U - U0);// want to counter act against the difference
					J(j, 0) = dP_t1 / 0.01;
					J(j, 1) = dP_t2 / 0.01;
					j++;
				}

				//std::cout << "E" << std::endl;
				//std::cout << E << std::endl;
				//std::cout << "J" << std::endl;
				//std::cout << J << std::endl;
				Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
				//std::cout << "t:";
				//std::cout << t.transpose() << std::endl;
				std::cout << "(" << perturbedPoints[ii].transpose() << ") ---> ";
				perturbedPoints[ii] += t;
				std::cout << "(" << perturbedPoints[ii].transpose() << ")";
				std::cout << " | Real Position: (" << points[ii].transpose() << ")" << std::endl;
				ii = ii + 1;
			}
		}
	}


	/*

	// Adjust Points and Camera (There is no roation for point, since it is a single point)
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		std::cout << std::endl;
		std::cout << "Iteration: " << iteration << std::endl;

		
		std::cout << "Adjust Cameras" << std::endl;
		for (int i = 0; i < perturbed_cameras.size(); i++)
		{
			// Construct Jacobian 
			Eigen::MatrixXf J(photos[i].size(), 2);
			Eigen::VectorXf E(photos[i].size());
			// for each visible points, compute one line of J and E
			int j = 0;
			for (auto p : photos[i])
			{
				// serach for p
				int k;
				for (int m = 0; m < points.size(); m++)
				{
					if ((p - points[m]).norm() < 0.1)
						k = m;
				}
				// error: U-U0
				ProjectedPoint U0 = cameras[i].project(p);
				ProjectedPoint U = perturbed_cameras[i].project(perturbedPoints[k]);

				// calculate dUdt1
				Camera C_dt1 = Camera(perturbed_cameras[i].position);
				C_dt1.position += Translation(0.01f, 0.0f);
				float dU_t1 = C_dt1.project(p) - U;

				// calculate dUdt2
				Camera C_dt2 = Camera(perturbed_cameras[i].position);
				C_dt2.position += Translation(0.0f, 0.01f);
				float dU_t2 = C_dt2.project(p) - U;

				E(j) = -(U - U0);// want to counter act against the difference
				J(j, 0) = dU_t1 / 0.01;
				J(j, 1) = dU_t2 / 0.01;
				j++;
			}

			//std::cout << "E" << std::endl;
			//std::cout << E << std::endl;

			//std::cout << "J" << std::endl;
			//std::cout << J << std::endl;

			Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
			//std::cout << "t:";
			//std::cout << t.transpose() << std::endl;
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ") ---> ";
			perturbed_cameras[i].position += t;
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ")";
			std::cout << " | Real Position: (" << cameras[i].position.transpose() << ")" << std::endl;
		}


		std::cout << "Adjust Points" << std::endl;
		// randomly adjust point one by one
		// use finite difference to estimate Jacobian
		for (int i = 0; i < perturbedPoints.size(); i++)
		{
			// Construct Jacobian 
			Eigen::MatrixXf J(perturbed_cameras.size(), 2);
			Eigen::VectorXf E(perturbed_cameras.size());
			// for each camera, compute one line of J and E
			int j = 0;
			for (int k = 0; k < perturbed_cameras.size(); k++)
			{
				// error: U-U0
				ProjectedPoint U0 = cameras[k].project(points[i]);
				ProjectedPoint U = perturbed_cameras[k].project(perturbedPoints[i]);

				// calculate dUdt1
				Point P_dt1 = perturbedPoints[i];
				P_dt1 += Translation(0.01f, 0.0f);
				float dP_t1 = perturbed_cameras[k].project(P_dt1) - U;

				// calculate dUdt2
				Point P_dt2 = perturbedPoints[i];
				P_dt2 += Translation(0.0f, 0.01f);
				float dP_t2 = perturbed_cameras[k].project(P_dt2) - U;

				E(j) = -(U - U0);// want to counter act against the difference
				J(j, 0) = dP_t1 / 0.01;
				J(j, 1) = dP_t2 / 0.01;
				j++;
			}

			//std::cout << "E" << std::endl;
			//std::cout << E << std::endl;
			//std::cout << "J" << std::endl;
			//std::cout << J << std::endl;
			Translation t = (J.transpose()*J).inverse()*(J.transpose()*E);
			//std::cout << "t:";
			//std::cout << t.transpose() << std::endl;
			std::cout << "(" << perturbedPoints[i].transpose() << ") ---> ";
			perturbedPoints[i] += t;
			std::cout << "(" << perturbedPoints[i].transpose() << ")";
			std::cout << " | Real Position: (" << points[i].transpose() << ")" << std::endl;
		}
	}
	*/

	// pertube points position
	// randomly adjust a point


	int wait;
	std::cin >> wait;
}
