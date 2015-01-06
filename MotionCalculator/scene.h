#pragma once
#include <vector>
#include <iostream>
#include "MotionCalculator.h"

class Scene
{
public:
	Scene();
	~Scene();
	static void writeFile(std::vector<Point> points, std::vector<Camera> cameras, int i);
	static void readFile(std::vector<Point> &points, std::vector<Camera> &cameras);
};

