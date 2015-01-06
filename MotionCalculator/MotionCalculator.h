// import procedure node
#pragma once
#include <iostream>
// import external Obj/procedure Node
#include <Eigen/Geometry>
// import container node
#include <map>
#include <vector>

typedef Eigen::Vector2f Point;
typedef Eigen::Vector2f Direction;
typedef Eigen::Vector2f Translation;

typedef float ProjectedPoint;
typedef int CamID;
typedef int PointID;

typedef  std::vector<Point> Frame;
typedef  std::vector<Point> Points;

class Camera
{
public:
	// APIs
	Camera();
	Camera(Point pos);
	virtual ~Camera();

	ProjectedPoint project(Point p);
	Frame capture(Points ps);

	// properties
	float f; // focal length
	float width; // horizontal resolution
	Point position; // camera position
	Direction direction; // camera direction
};



