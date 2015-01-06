#pragma once
/*
// import procedure node
#include <iostream>
// import external Obj/procedure Node
#include <Eigen/Geometry>
// import container node
#include <map>
#include <vector>

typedef Eigen::Vector3f Point3D;
typedef Eigen::Vector3f Direction3D;
typedef Eigen::Vector3f Translation3D;

typedef  Eigen::Vector2f ProjectedPoint;
typedef  std::vector<Point3D> Frame;
typedef  std::vector<Point3D> Points;

class Camera
{
public:
	// APIs
	Camera();
	Camera(Point3D pos, Direction3D view, Direction3D up);
	virtual ~Camera();
	void setFrameResolution(float w, float h); // width and height, unit pixels
	void setFocalLength(float f); // unit pixels

	ProjectedPoint project(Point3D p);
	Frame capture(Points ps);

	// properties
	float f; // focal length
	float width; // horizontal resolution
	Point position; // camera position
	Direction direction; // camera direction
};
*/