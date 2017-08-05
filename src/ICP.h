/*
Iterative Closest Point

Performs the iterative closest point algorithm. A point cloud 'dynamicPointCloud' is transformed such that
it best "matches" the point cloud 'staticPointCloud'.

this program uses the method outlined in [1].

1. for each point in the dynamic point cloud, find the closest point in the static point cloud.
2. solve for an affine transform which minimizes the distance between all dynamic points and their respective static points.
3. transform the dynamic points.
4. goto -> 1 until stopping conditions are met.

affine transforms are solved using SVD.

@author Greg Smith 2017

[1] Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. "Least-squares fitting of two 3-D point sets." 
	IEEE Transactions on pattern analysis and machine intelligence 5 (1987): 698-700.

*/

#pragma once

#include <stdio.h>
#include <vector>
#include <random>
#include "KdTree.h"
#include <ctime>
#include "svd.h"

namespace gs
{

	/*
	void icp
		transforms the point cloud 'dynamicPointCloud' such that it best matches 'staticPointCloud'

	@param std::vector<Point*> dynamicPointCloud : point cloud to be rotated and translated to match 'staticPointCloud'
	@param std::vector<Point*> staticPointCloud : reference point cloud.
	*/
	void icp(std::vector<Point*> &dynamicPointCloud, std::vector<Point*> &staticPointCloud);

	inline void computeCloudMean(std::vector<Point*> &cloud, gs::Point* mean)
	{
		mean->pos[0] = 0.0;
		mean->pos[1] = 0.0;
		mean->pos[2] = 0.0;
		for (int i = 0; i < cloud.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				mean->pos[j] += cloud[i]->pos[j];
			}
		}
		mean->pos[0] = mean->pos[0] / (float)cloud.size();
		mean->pos[1] = mean->pos[1] / (float)cloud.size();
		mean->pos[2] = mean->pos[2] / (float)cloud.size();
	}

	inline void clearTranslation(float* translation)
	{
		translation[0] = 0.0;
		translation[1] = 0.0;
		translation[2] = 0.0;
	}

	inline void clearRotation(float* rotation)
	{
		rotation[0] = 1.0;
		rotation[1] = 0.0;
		rotation[2] = 0.0;

		rotation[3] = 0.0;
		rotation[4] = 1.0;
		rotation[5] = 0.0;

		rotation[6] = 0.0;
		rotation[7] = 0.0;
		rotation[8] = 1.0;
	}

	inline void clearMatrix(float* mat)
	{
		mat[0] = 0.0;
		mat[1] = 0.0;
		mat[2] = 0.0;

		mat[3] = 0.0;
		mat[4] = 0.0;
		mat[5] = 0.0;

		mat[6] = 0.0;
		mat[7] = 0.0;
		mat[8] = 0.0;
	}

	inline void rotate(gs::Point* p, float* rotationMatrix, gs::Point* result)
	{
		result->pos[0] = p->pos[0] * rotationMatrix[0] + p->pos[1] * rotationMatrix[1] + p->pos[2] * rotationMatrix[2];
		result->pos[1] = p->pos[0] * rotationMatrix[3] + p->pos[1] * rotationMatrix[4] + p->pos[2] * rotationMatrix[5];
		result->pos[2] = p->pos[0] * rotationMatrix[6] + p->pos[1] * rotationMatrix[7] + p->pos[2] * rotationMatrix[8];
	}

	inline void translate(gs::Point* p, float* translationVector, gs::Point* result)
	{
		result->pos[0] = p->pos[0] + translationVector[0];
		result->pos[1] = p->pos[1] + translationVector[1];
		result->pos[2] = p->pos[2] + translationVector[2];
	}

	inline void outerProduct(gs::Point* a, gs::Point* b, float* mat)
	{
		mat[0] = a->pos[0] * b->pos[0];
		mat[1] = a->pos[0] * b->pos[1];
		mat[2] = a->pos[0] * b->pos[2];

		mat[3] = a->pos[1] * b->pos[0];
		mat[4] = a->pos[1] * b->pos[1];
		mat[5] = a->pos[1] * b->pos[2];

		mat[6] = a->pos[2] * b->pos[0];
		mat[7] = a->pos[2] * b->pos[1];
		mat[8] = a->pos[2] * b->pos[2];
	}

	inline void matrixMult(float* a, float* b, float* result)
	{
		result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
		result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
		result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

		result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
		result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
		result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

		result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
		result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
		result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
	}

	inline void transpose(float* a)
	{
		float temp;

		temp = a[1];
		a[1] = a[3];
		a[3] = temp;

		temp = a[2];
		a[2] = a[6];
		a[6] = temp;

		temp = a[5];
		a[5] = a[7];
		a[7] = temp;
	}

	inline void addMatrix(float* a, float* b, float* result)
	{
		result[0] = a[0] + b[0];
		result[1] = a[1] + b[1];
		result[2] = a[2] + b[2];

		result[3] = a[3] + b[3];
		result[4] = a[4] + b[4];
		result[5] = a[5] + b[5];

		result[6] = a[6] + b[6];
		result[7] = a[7] + b[7];
		result[8] = a[8] + b[8];
	}

	inline float error(Point* ps, Point* pd, float* r, float* t)
	{
		Point res;
		rotate(pd, r, &res);
		float err = pow(ps->pos[0] - res.pos[0] - t[0],2.0);
		err += pow(ps->pos[1] - res.pos[1] - t[1], 2.0);
		err += pow(ps->pos[2] - res.pos[2] - t[2], 2.0);
		return err;
	}

	inline void copyMatToUV(float* mat, float** result)
	{
		result[0][0] = mat[0];
		result[0][1] = mat[1];
		result[0][2] = mat[2];

		result[1][0] = mat[3];
		result[1][1] = mat[4];
		result[1][2] = mat[5];

		result[2][0] = mat[6];
		result[2][1] = mat[7];
		result[2][2] = mat[8];
	}

	inline void copyUVtoMat(float** mat, float* result)
	{
		result[0] = mat[0][0];
		result[1] = mat[0][1];
		result[2] = mat[0][2];

		result[3] = mat[1][0];
		result[4] = mat[1][1];
		result[5] = mat[1][2];

		result[6] = mat[2][0];
		result[7] = mat[2][1];
		result[8] = mat[2][2];
	}
}