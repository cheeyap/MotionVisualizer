#include "scene.h"


Scene::Scene()
{
}


Scene::~Scene()
{
}

void
Scene::readFile(std::vector<Point> &points, std::vector<Camera> &cameras)
{
	points.clear(); cameras.clear();
	// assume point and point normal are interlaced
	// this is a standard format exported by mesh lab!
	FILE * file = fopen("C:\\Users\\Mocap-Vader\\Desktop\\scene_o.txt", "r");
	{{
		if (file == NULL){
			std::cout << "Can't open file "<< std::endl;
		}
	}}
	while (1)
	{
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		{{
			if (res == EOF)
			break; // EOF = End Of File. Quit the loop.
		}}

		// else : parse lineHeader
		// point
		if (strcmp(lineHeader, "p") == 0){
			float x, y;
			fscanf(file, "%f %f\n", &x, &y);
			Point p = Point(x,y);
			points.push_back(p);
		}
		// camera
		else if (strcmp(lineHeader, "c") == 0){
			float x, y;
			fscanf(file, "%f %f\n", &x, &y);
			Camera c = Camera(Point(x, y));
			cameras.push_back(c);
		}
	}
	fclose(file);
}

void
Scene::writeFile(std::vector<Point> points, std::vector<Camera> cameras, int i)
{
	std::string path = "C:\\Users\\Mocap-Vader\\Desktop\\scene_r" + std::to_string(i) + ".txt";
	FILE * file = fopen(path.c_str(), "w+");
	{{
		if (file == NULL){
			std::cout << "Can't open file: " << "scene_o.txt" << std::endl;
		}
	}}

	for (int i = 0; i < points.size(); i++)
	{
		fprintf(file, "p %f %f\n", points[i][0], points[i][1]);
	}

	for (int i = 0; i < cameras.size(); i++)
	{
		fprintf(file, "c %f %f\n", cameras[i].position[0], cameras[i].position[1]);
	}

	fclose(file);

}
