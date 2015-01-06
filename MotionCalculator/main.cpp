#include <iostream>
#include <Eigen/Geometry>
#include "MotionCalculator.h"

#include "Reconstruction.h"

#include <map>
#include <vector>
#include <algorithm>
#include "scene.h"


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
	//Reconstruction r;
	//r.readLKTrajectories("C:\\Users\\Mocap-Vader\\Desktop\\Scene1.txt");

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
	Points points;	std::vector<Camera> cameras;
	Scene::readFile(points, cameras);
	
	/*
	points.push_back(Point(0, 10));
	points.push_back(Point(10, 20));
	points.push_back(Point(0, 30));
	points.push_back(Point(-10, 20));
	points.push_back(Point(-20, 20));
	points.push_back(Point(20, 20));
	*/
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
	float a = 5.0f;
	float b = 50.0f;
	int numOfIteration = 10;
	float step = 0.001;
	int fileNo = 0;

	// perturbation points' position
	Translation PointPerturbation(3, 2);
	Eigen::MatrixXf PointPerturbations = Eigen::MatrixXf::Random(points.size(),2);
	Points perturbedPoints;
	int k = 0;
	for (auto p : points)
	{
		perturbedPoints.push_back(p + a*PointPerturbations.row(k).transpose());
		k = k+1;
	}
	{{
		std::cout << "Perturb Points" << std::endl;
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

	/*
	cameras.push_back(Camera(Point(-20, -20)+cameraTranslation));
	cameras.push_back(Camera(Point(-10, -20)+cameraTranslation));
	cameras.push_back(Camera(Point(0, -10)+cameraTranslation));
	cameras.push_back(Camera(Point(10, -10)+cameraTranslation));
	cameras.push_back(Camera(Point(20, -0)+cameraTranslation));
	*/

	// accidental motion camera scene
	/*
	cameras.push_back(Camera(Point(0.4, -20.4) + cameraTranslation));
	cameras.push_back(Camera(Point(0.2, -20.2) + cameraTranslation));
	cameras.push_back(Camera(Point(0.4, -19.5) + cameraTranslation));
	cameras.push_back(Camera(Point(0.2, -19.8) + cameraTranslation));
	cameras.push_back(Camera(Point(0, -20) + cameraTranslation));
	*/

	// perturbation cameras' position
	std::vector<Camera> perturbed_cameras;

	// randomly init it
	Eigen::MatrixXf cameraPerturbations = Eigen::MatrixXf::Random(cameras.size(), 2);
	k = 0;
	for (auto c : cameras)
	{
		perturbed_cameras.push_back(Camera(c.position + b*cameraPerturbations.row(k).transpose()));
		k = k + 1;
	}

	// initialized with identity location
	/*
	for (auto c : cameras)
	{
		perturbed_cameras.push_back(Camera(Point(0,-40)));
		k = k + 1;
	}
	*/

	{{
		std::cout << "Perturb Cameras" << std::endl;
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

	Points originalPoints = perturbedPoints; std::vector<Camera> originalCameras = perturbed_cameras;
	Scene::writeFile(perturbedPoints, perturbed_cameras, fileNo); fileNo++;
	// adjust camera and points at the same time (Bundle Adjustment)
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		//std::cout << std::endl;
		//std::cout << "Iteration: " << iteration << std::endl;
		// use finite difference to estimate Jacobian
		// parameter space dimemsion: 2*m+2*n
		// output space dimenstion: m*n
		int m = perturbed_cameras.size();
		int n = perturbedPoints.size();
		Eigen::MatrixXf J(m*n, 2 * n + 2 * m); J.setZero();
		Eigen::VectorXf E(m*n);
		// fill in the row of Jacobian one by one
		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				int row = i*n + j;

				float pij0 = cameras[i].project(points[j]); // true 2d position

				float pij = perturbed_cameras[i].project(perturbedPoints[j]); // current 2d position
				Point P_dt1 = perturbedPoints[j]; P_dt1 += Translation(step, 0.0f);
				Point P_dt2 = perturbedPoints[j]; P_dt2 += Translation(0.0f, step);
				Camera C_dt1 = Camera(perturbed_cameras[i].position); C_dt1.position += Translation(step, 0.0f);
				Camera C_dt2 = Camera(perturbed_cameras[i].position); C_dt2.position += Translation(0.0f, step);

				E(row) = pij0 - pij;
				//std::cout << "E" << std::endl;
				//std::cout << E << std::endl;
				J(row, 2 * j) = (perturbed_cameras[i].project(P_dt1) - pij) / step;
				J(row, 2 * j + 1) = (perturbed_cameras[i].project(P_dt2) - pij) / step;
				J(row, 2 * n + 2 * i) = (C_dt1.project(perturbedPoints[j]) - pij) / step;
				J(row, 2 * n + 2 * i + 1) = (C_dt2.project(perturbedPoints[j]) - pij) / step;
			}
		}
		//std::cout << "E" << std::endl;
		//std::cout << E << std::endl;
		//std::cout << E.transpose().norm() << std::endl;

		//std::cout << "J" << std::endl;
		//std::cout << J << std::endl;

		Eigen::VectorXf Dparameter(2 * m + 2 * n);
		Dparameter = (J.transpose()*J).inverse()*(J.transpose()*E);

		//std::cout << "Dp" << std::endl;
		//std::cout << Dparameter << std::endl;


		//update points
		for (int j = 0; j < perturbedPoints.size(); j++)
		{
			std::cout << "(" << perturbedPoints[j].transpose() << ") ---> ";
			perturbedPoints[j] += Translation(Dparameter(2 * j), Dparameter(2 * j + 1));
			std::cout << "(" << perturbedPoints[j].transpose() << ")";
			std::cout << " | Real Position: (" << points[j].transpose() << ")" << std::endl;
		}


		//update cameras
		for (int i = 0; i < perturbed_cameras.size(); i++)
		{
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ") ---> ";
			perturbed_cameras[i].position += Translation(Dparameter(2 * n + 2 * i), Dparameter(2 * n + 2 * i + 1));
			std::cout << "(" << perturbed_cameras[i].position.transpose() << ")";
			std::cout << " | Real Position: (" << cameras[i].position.transpose() << ")" << std::endl;
		}

		Scene::writeFile(perturbedPoints, perturbed_cameras, fileNo);
		fileNo++;
	}

	std::cout << "Points" << std::endl;
	for (int j = 0; j < originalPoints.size(); j++)
	{
		std::cout << "(" << originalPoints[j].transpose() << ")";
		std::cout << " | Bundle Adjusted Position: (" << perturbedPoints[j].transpose() << ")" ;
		std::cout << " | Real Position: (" << points[j].transpose() << ")" << std::endl;
	}

	std::cout << "Cameras" << std::endl;
	for (int i = 0; i < originalCameras.size(); i++)
	{
		std::cout << "(" << originalCameras[i].position.transpose() << ")";
		std::cout << " | Bundle Adjusted Position: (" << perturbed_cameras[i].position.transpose() << ")";
		std::cout << " | Real Position: (" << cameras[i].position.transpose() << ")" << std::endl;
	}
	

	/*
	// Adjust Cameras
	for (int iteration = 0; iteration < numOfIteration; iteration++)
	{
		std::cout << std::endl;
		std::cout << "Iteration: " << iteration << std::endl;
		// randomly adjust camera one by one
		// use finite difference to estimate Jacobian
		for (int i = 0; i < perturbed_cameras.size(); i++)
		{
			// Construct Jacobian 
			Eigen::MatrixXf J(points.size(), 2);
			Eigen::VectorXf E(points.size());
			// for each visible points, compute one line of J and E
			for (int j = 0; j < points.size();j++)
			{
				// error: U-U0
				ProjectedPoint U0 = cameras[i].project(points[j]);
				ProjectedPoint U = perturbed_cameras[i].project(perturbedPoints[j]);

				// calculate dUdt1
				Camera C_dt1 = Camera(perturbed_cameras[i].position);
				C_dt1.position += Translation(0.01f, 0.0f);
				float dU_t1 = C_dt1.project(perturbedPoints[j]) - U;

				// calculate dUdt2
				Camera C_dt2 = Camera(perturbed_cameras[i].position);
				C_dt2.position += Translation(0.0f, 0.01f);
				float dU_t2 = C_dt2.project(perturbedPoints[j]) - U;

				E(j) = -(U - U0);// want to counter act against the difference
				J(j, 0) = dU_t1 / 0.01;
				J(j, 1) = dU_t2 / 0.01;
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
	/*
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
	*/

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

	std::cout << "------------2. Test 3D Bundle Adjustment -----------" << std::endl;
	

	int wait;
	std::cin >> wait;
}
