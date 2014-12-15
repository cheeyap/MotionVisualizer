#include "MotionCalculator.h"

Camera::Camera()
{
	f = 10;
	width = 10;
	position = Point(0, 0);
	direction = Direction(0.0f, 1.0f);
}

Camera::Camera(Point pos)
{
	f = 10;
	width = 10;
	position = Point(pos(0),pos(1));
	direction = Direction(0.0f, 1.0f);
}

Camera::~Camera()
{

}

ProjectedPoint
Camera::project(Point p)
{	
	float x = direction(0); float y = direction(1);
	float t1 = position(0); float t2 = position(1);
	// construct operator O (represent in w space) w frame to c frame
	// y x t1
	//-x y t2
	// 0 0  1
	Eigen::Matrix3f O_w;
	O_w << y, x, t1, -x, y, t2, 0, 0, 1;

	// construct T_wc, w(world) to c(camera) matrix
	Eigen::Matrix3f T_wc;
	T_wc = O_w.inverse();

	// take p into camera frame space
	Eigen::Vector3f p_h = Eigen::Vector3f(p(0),p(1),1);
	Eigen::Vector3f p_c = T_wc*p_h;

	// visibility test
	if (p_c(1) <= f)
		return -10000000; // invisible;

	// apply projection matrix
	// f 0 0
	// 0 0 0
	// 0 1 0
	Eigen::Matrix3f P;
	P << f, 0, 0, 0, 0, 0, 0, 1, 0;
	Eigen::Vector3f p_P = P*p_c;

	return p_P(0)/p_P(2);
}

Frame
Camera::capture(Points ps)
{
	Frame fs;
	for (auto p : ps)
	{
		ProjectedPoint p_P = project(p);
		// test if p_P is inside camera frame width
		if (-width / 2 < p_P && p_P < width / 2)
			fs.push_back(p);
	}
	return fs;
}