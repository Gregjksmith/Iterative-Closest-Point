#include "ICP.h"

void gs::icp(std::vector<Point*> &dynamicPointCloud, std::vector<Point*> &staticPointCloud)
{
	float rotationMatrix[9];
	float translation[3];

	std::vector<Point*> staticPointCloudCopy;
	
	gs::Point dynamicMid(0.0,0.0,0.0);
	gs::Point staticMid(0.0,0.0,0.0);

	// copy the static point cloud
	for (int i = 0; i < staticPointCloud.size(); i++)
	{
		Point* pCopy = new Point(staticPointCloud[i]->pos[0], staticPointCloud[i]->pos[1], staticPointCloud[i]->pos[2]);
		staticPointCloudCopy.push_back(pCopy);
	}

	// create the kd tree
	KdTree* tree = new KdTree(staticPointCloudCopy);
	size_t numDynamicPoints = dynamicPointCloud.size();

	computeCloudMean(staticPointCloud, &staticMid);
	computeCloudMean(dynamicPointCloud, &dynamicMid);

	// initialize the translation vector
	clearTranslation(translation);

	// initialize the rotation matrix
	clearRotation(rotationMatrix);

	const int maxIterations = 400;
	const int numRandomSamples = 400;
	const float eps = 1e-8;
	gs::Point p;
	gs::Point x;

	float cost = 1.0;
	std::srand(std::time(0));

	gs::Point qd;
	gs::Point qs;

	float U[9];
	float w[9];
	float sigma[3];
	float V[9];

	float** uSvd = new float*[3];
	float** vSvd = new float*[3];
	uSvd[0] = new float[3];
	uSvd[1] = new float[3];
	uSvd[2] = new float[3];

	vSvd[0] = new float[3];
	vSvd[1] = new float[3];
	vSvd[2] = new float[3];

	for (int iter = 0; iter < maxIterations && abs(cost) > eps; iter++)
	{
		cost = 0.0;
		
		//clearRotation(rotationMatrix);
		clearMatrix(U);
		clearMatrix(V);
		clearMatrix(w);
		computeCloudMean(dynamicPointCloud, &dynamicMid);

		for (int i = 0; i < numRandomSamples; i++)
		{
			int randSample = std::rand() % dynamicPointCloud.size();
			// sample the dynamic point cloud
			p = *dynamicPointCloud[randSample];

			// get the closest point in the static point cloud
			tree->search(&p, &x);

			qd = p - dynamicMid;
			qs = x - staticMid;

			outerProduct(&qs, &qd, w);
			addMatrix(w, U, U);

			cost += error(&x, &p, rotationMatrix, translation);
		}

		copyMatToUV(U, uSvd);
		dsvd(uSvd, 3, 3, sigma, vSvd);
		copyUVtoMat(uSvd, U);
		copyUVtoMat(vSvd, V);

		transpose(V);
		matrixMult(U, V, rotationMatrix);

		gs::Point t(0.0, 0.0, 0.0);
		rotate(&dynamicMid, rotationMatrix, &t);
		translation[0] = staticMid.pos[0] - t.pos[0];
		translation[1] = staticMid.pos[1] - t.pos[1];
		translation[2] = staticMid.pos[2] - t.pos[2];

		//update the point cloud
		for (int i = 0; i < dynamicPointCloud.size(); i++)
		{
			rotate(dynamicPointCloud[i], rotationMatrix, &p);
			translate(&p, translation, dynamicPointCloud[i]);
		}
	}

	staticPointCloudCopy.clear();
	delete tree;
	
	delete[] uSvd[0];
	delete[] uSvd[1];
	delete[] uSvd[2];
	delete[] uSvd;

	delete[] vSvd[0];
	delete[] vSvd[1];
	delete[] vSvd[2];
	delete[] vSvd;
}