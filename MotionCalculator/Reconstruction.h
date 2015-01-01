#pragma once
#include <iostream>
#include <Eigen/Geometry>

#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <algorithm>

typedef std::vector<std::vector<Eigen::Vector2f>> Trajectories;
typedef std::vector<Eigen::Matrix3f> Rotations;
typedef std::vector<Eigen::Vector3f> Translations;
typedef std::vector<float> PointsInverseDepths;

typedef std::vector<float> X;
typedef std::vector<float> Y;

struct Parameter // parameter to be estimated in optimizer
{
	Rotations R;
	Translations T;
	PointsInverseDepths w;
};

class Reconstruction
{
	// this class is an implementation of the Paper:
	// 3D Reconstruction from Accidental Motion
public:
	Reconstruction();
	~Reconstruction();

	//-----------//
	// properties//
	//-----------//
	int N; // frame Number not including the reference frame
	int M; // Number of tracked points
	float f; // focal length
	int width; // width of each frame
	int height; // height of each frame
	Trajectories p; // usage p[i][j], frame i, point j, notice p is normalized by f
	Rotations R; // usage R[i]
	Translations T; // usage T[i]

	PointsInverseDepths w; // usage w[j]
	X x;// x, y location in reference frame, normalized, usage x[j]
	Y y;// x, y location in reference frame, normalized, usage y[j]

	//-----------//
	//APIs       //
	//-----------//
	// initialize p,R,T
	void init();
	
	// Fill in p, x and y, make sure to normalize it!!!!!(image space, local camera space), minues frame center and divide by f
	void readLKTrajectories(std::string path);
	
	// Fill in R,T with identity matrix, R = I, T = 0;
	void initMotion();
	
	// Fill in w with random numbers (suggested by the paper)
	void initStructure();
	
	// normalize image


	// evaluate distance to target pij,
	// - return a N*M vector, each element is D(wj,Ri,Tj) - pij
	Trajectories D(Parameter para, Trajectories p);
	// serialize the previous distance
	Eigen::VectorXf distanceSerialize(Trajectories dist);
	// restore distance
	Trajectories distanceRestore(Eigen::VectorXf dist);
	// serialize parameter
	Eigen::VectorXf parameterSerialize(Parameter para);
	// restore paramter
	Parameter parameterRestore(Eigen::VectorXf);
	// evaluate Jocobian
	Eigen::MatrixXf evalJ(Parameter para, Trajectories p);
	// Gaussian Newtown Opimizer (There are other method, e.g. LM)
	// - return a vector and it is used to update current parameters (w,R,T)
	Eigen::VectorXf GN(Eigen::MatrixXf J, Eigen::VectorXf x, Eigen::VectorXf b);

	//Bundle Adjustment
	// - return updated parameter
	Parameter ba(Parameter currParameter, Trajectories p);


};