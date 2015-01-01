#include "Reconstruction.h"


Reconstruction::Reconstruction()
{

}


Reconstruction::~Reconstruction()
{

}

void
Reconstruction::init()
{
}

void
Reconstruction::readLKTrajectories(std::string path)
{
	/*
	FILE * file = fopen(path.c_str(), "r");
	{{
		if (file == NULL){
			std::cout << "Can't open file: " << path << std::endl;
		}
	}}

	// read file Header and init N,M
	fscanf(file, "%d %d %d %d %d\n", &N, &M, &width, &height, &f);
	
	while (1){

		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		{{
			if (res == EOF)
			break; // EOF = End Of File. Quit the loop.
		}}

	
		// local data structures:
		int frameNumber = 0, trackedPointsNumber = 0;
		std::vector<Eigen::Vector2f> buffer;
		// else : parse lineHeader
		// new frame 
		if (strcmp(lineHeader, "fs") == 0){
			buffer.clear();
		}
		// end frame
		else if (strcmp(lineHeader, "fe") == 0){
			p.push_back(buffer);
		}
		else if (strcmp(lineHeader, "p") == 0) {
			float u, v;
			fscanf(file, "%f %f\n", &u, &v);
			float x, y;
			x = 
			buffer.push_back(Eigen::Vector2f(x, y));
		}
		// reference frame
		if (strcmp(lineHeader, "fs0") == 0){

		}
		// end reference frame
		else if (strcmp(lineHeader, "fe0") == 0){
		}
		// reference points
		else if (strcmp(lineHeader, "p0") == 0) {
			float u, v;
			fscanf(file, "%f %f\n", &u, &v);
			x.push_back(u)
		}
	}
	
	fclose(file);
	*/
}

void 
Reconstruction::initMotion()
{

}

void 
Reconstruction::initStructure()
{

}

Trajectories 
Reconstruction::D(Parameter para, Trajectories p)
{
	Trajectories result;

	return result;
}

Eigen::VectorXf 
Reconstruction::distanceSerialize(Trajectories dist)
{
	Eigen::VectorXf result;

	return result;
}

Trajectories 
Reconstruction::distanceRestore(Eigen::VectorXf dist)
{
	Trajectories result;

	return result;
}

Eigen::VectorXf 
Reconstruction::parameterSerialize(Parameter para)
{
	Eigen::Vector2f result;

	return result;
}

Parameter 
Reconstruction::parameterRestore(Eigen::VectorXf)
{
	Parameter result;

	return result;
}

Eigen::MatrixXf 
Reconstruction::evalJ(Parameter para, Trajectories p)
{
	Eigen::MatrixXf result;

	return result;
}

Eigen::VectorXf 
Reconstruction::GN(Eigen::MatrixXf J, Eigen::VectorXf x, Eigen::VectorXf b)
{
	Eigen::VectorXf result;

	return result;
}

Parameter 
Reconstruction::ba(Parameter currParameter, Trajectories p)
{
	Parameter result;

	return result;
}