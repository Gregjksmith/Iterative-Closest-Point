#include "ICP.h"

using namespace gs;

/*
Create a test box of points.
*/
void createPoints(std::vector<Point*>& points)
{
	points.push_back(new Point(-1.0f, -1.0f, -1.0f));
	points.push_back(new Point(1.0f, -1.0f, -1.0f));
	points.push_back(new Point(-1.0f, 1.0f, -1.0f));
	points.push_back(new Point(1.0f, 1.0f, -1.0f));
	points.push_back(new Point(-1.0f, -1.0f, 1.0f));
	points.push_back(new Point(1.0f, -1.0f, 1.0f));
	points.push_back(new Point(-1.0f, 1.0f, 1.0f));
	points.push_back(new Point(1.0f, 1.0f, 1.0f));
}

/*
Apply an afine transofrm to a point cloud
*/
void applyAffineTransform(std::vector<Point*>& points, float* rotationMatrix, float* translation)
{
	Point pRot;
	for (int i = 0; i < points.size(); i++)
	{
		pRot.pos[0] = rotationMatrix[0] * points[i]->pos[0] + rotationMatrix[1] * points[i]->pos[1] + rotationMatrix[2] * points[i]->pos[2] + translation[0];
		pRot.pos[1] = rotationMatrix[3] * points[i]->pos[0] + rotationMatrix[4] * points[i]->pos[1] + rotationMatrix[5] * points[i]->pos[2] + translation[1];
		pRot.pos[2] = rotationMatrix[6] * points[i]->pos[0] + rotationMatrix[7] * points[i]->pos[1] + rotationMatrix[8] * points[i]->pos[2] + translation[2];

		*points[i] = pRot;
	}
}

/*
ICP is used to minimize the distance between 2 point clouds.

This example computes a point cloud of a unit box (Static point cloud)
and a second unit box with a linear transform applied (dynamic point cloud).
ICP is used to transform the dynamic point cloud to best match the static point cloud.
*/
void icpExample()
{
	//create a static box point cloud used as a reference.
	std::vector<Point*> staticPointCloud;
	createPoints(staticPointCloud);

	//create a dynamic box point cloud.
	//this point cloud is transformed to match the static point cloud.
	std::vector<Point*> dynamicPointCloud;
	createPoints(dynamicPointCloud);

	//apply an artitrary rotation and translation to the dynamic point cloud to misalign the point cloud.
	float rotation[] = { 1.0f, 0.0f, 0.0f,	0.0f, 0.70710678f, -0.70710678f,	0.0f, 0.70710678f, 0.70710678f };
	float translation[] = { -0.75f, 0.5f, -0.5f };
	applyAffineTransform(dynamicPointCloud, rotation, translation);

	printf("Static point Cloud: \n");
	for (int i = 0; i < staticPointCloud.size(); i++)
	{
		printf("%0.2f, %0.2f, %0.2f \n", staticPointCloud[i]->pos[0], staticPointCloud[i]->pos[1], staticPointCloud[i]->pos[2]);
	}
	printf("\n");

	printf("Dynamic point Cloud: \n");
	for (int i = 0; i < dynamicPointCloud.size(); i++)
	{
		printf("%0.2f, %0.2f, %0.2f \n", dynamicPointCloud[i]->pos[0], dynamicPointCloud[i]->pos[1], dynamicPointCloud[i]->pos[2]);
	}
	printf("\n");

	//use iterative closest point to transform the dynamic point cloud to best align the static point cloud.
	icp(dynamicPointCloud, staticPointCloud);

	printf("Dynamic point Cloud Transformed: \n");
	for (int i = 0; i < dynamicPointCloud.size(); i++)
	{
		printf("%0.2f, %0.2f, %0.2f \n", dynamicPointCloud[i]->pos[0], dynamicPointCloud[i]->pos[1], dynamicPointCloud[i]->pos[2]);
	}
	printf("\n");

	float alignmentError = 0.0f;
	for (int i = 0; i < dynamicPointCloud.size(); i++)
	{
		alignmentError += pow(dynamicPointCloud[i]->pos[0] - staticPointCloud[i]->pos[0], 2.0f);
		alignmentError += pow(dynamicPointCloud[i]->pos[1] - staticPointCloud[i]->pos[1], 2.0f);
		alignmentError += pow(dynamicPointCloud[i]->pos[2] - staticPointCloud[i]->pos[2], 2.0f);
	}

	alignmentError /= (float)dynamicPointCloud.size();

	printf("Alignment Error: %0.5f \n", alignmentError);
}

int main()
{
	icpExample();
	system("pause");
	return 0;
}