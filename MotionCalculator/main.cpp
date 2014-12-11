#include <iostream>
#include <Eigen/Geometry>


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
	std::cout << "------------MotionCalculator-----------" << std::endl;
	std::cout << "Tyler" << std::endl;
	
	std::cout << "1.Euler Angle" << std::endl;

	Eigen::Matrix3f R_x; R_x = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX());
	Eigen::Matrix3f R_y; R_y = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY());
	Eigen::Matrix3f R_z; R_z = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());

	Eigen::Vector3f T; T << 1, 2, 3;
	std::cout << T << std::endl;
	Eigen::Matrix3f R = R_x*R_y*R_z;
	std::cout << R << std::endl;
	Eigen::Matrix4f M; M.setZero();
	std::cout << M << std::endl;
	M.block<3, 3>(0, 0) = R;
	M.block<3, 1>(0, 3) = T;
	M(3, 3) = 1;
	std::cout << M << std::endl;

	int wait;
	std::cin >> wait;
}