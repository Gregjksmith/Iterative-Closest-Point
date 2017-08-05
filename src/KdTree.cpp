#include "KdTree.h"

gs::Point::Point()
{
	this->pos[0] = 0.0;
	this->pos[1] = 0.0;
	this->pos[2] = 0.0;
}
gs::Point::Point(float x, float y, float z)
{
	this->pos[0] = x;
	this->pos[1] = y;
	this->pos[2] = z;
}

gs::KdTree::KdTree(std::vector<Point*> &pointCloud)
{
	tempArray = new Point*[pointCloud.size()];
	build(pointCloud, 0, pointCloud.size(), 0);

	delete tempArray;
}
gs::KdTree::KdTree(std::vector<Point*> &pointCloud, int start, int end, int sortOn)
{
	build(pointCloud, start, end, sortOn);
}
gs::KdTree::~KdTree()
{
	delete __children[0];
	delete __children[1];
	delete __node;
}

void gs::KdTree::build(std::vector<Point*> &pointCloud, int start, int end, int sortOn)
{
	__children[0] = nullptr;
	__children[1] = nullptr;
	__node = nullptr;

	__sortOn = sortOn;
	__start = start;
	__end = end;

	if (end - start == 1)
	{
		__node = new Point(pointCloud[start]->pos[0], pointCloud[start]->pos[1], pointCloud[start]->pos[2]);
		return;
	}

	//insertionSort(pointCloud, start, end, sortOn);
	mergeSort(pointCloud, start, end);

	int numPoints = (end - start);
	int mid = (int)floor((float)numPoints*0.5) + start;

	__node = new Point(pointCloud[mid]->pos[0], pointCloud[mid]->pos[1], pointCloud[mid]->pos[2]);

	int numPointsHigh = (end - mid);
	int numPointsLow = (mid - start);

	if (numPointsHigh > 0)
	{
		__children[1] = new KdTree(pointCloud, mid, end, getNextSortOn(sortOn));
	}

	if (numPointsLow > 0)
	{
		__children[0] = new KdTree(pointCloud, start, mid, getNextSortOn(sortOn));
	}
}

void gs::KdTree::insertionSort(std::vector<Point*> &pointCloud, int start, int end, int sortOn)
{
	for (int i = start; i < end; i++)
	{
		for (int j = i + 1; j < end; j++)
		{
			if (pointCloud[j]->pos[sortOn] < pointCloud[i]->pos[sortOn])
			{
				Point* temp = pointCloud[i];
				pointCloud[i] = pointCloud[j];
				pointCloud[j] = temp;
			}
		}
	}
}

int gs::KdTree::getNextSortOn(int sortOn)
{
	switch (sortOn)
	{
	case SORT_ON_X:
		return SORT_ON_Y;
		break;
	case SORT_ON_Y:
		return SORT_ON_Z;
		break;
	case SORT_ON_Z:
		return SORT_ON_X;
		break;
	}
}

bool gs::KdTree::isLeaf()
{
	if (__children[0] == nullptr && __children[1] == nullptr)
		return true;
	
	return false;
}

float gs::KdTree::split()
{
	return __node->pos[__sortOn];
}

gs::KdTree* gs::KdTree::getChild(Point* searchPoint)
{
	if (searchPoint->pos[__sortOn] >= split())
	{
		return __children[1];
	}
	else
	{
		return __children[0];
	}
}

float gs::KdTree::nodeX()
{
	return __node->pos[0];
}
float gs::KdTree::nodeY()
{
	return __node->pos[1];
}
float gs::KdTree::nodeZ()
{
	return __node->pos[2];
}

void gs::KdTree::radiusSearch(Point* p, float* radius, Point* result)
{
	if (isLeaf())
	{
		float d = sqrt(pow(__node->pos[0] - p->pos[0], 2.0) + pow(__node->pos[1] - p->pos[1], 2.0) + pow(__node->pos[2] - p->pos[2], 2.0));
		if (d < *radius)
		{
			*radius = d;
			result->pos[0] = __node->pos[0];
			result->pos[1] = __node->pos[1];
			result->pos[2] = __node->pos[2];
			return;
		}
	}
	else
	{
		if (abs(__node->pos[__sortOn] - p->pos[__sortOn]) < *radius)
		{
			__children[0]->radiusSearch(p, radius, result);
			__children[1]->radiusSearch(p, radius, result);
		}
		else
		{
			if (p->pos[__sortOn] >= __node->pos[__sortOn])
			{
				__children[1]->radiusSearch(p, radius, result);
			}
			else
			{
				__children[0]->radiusSearch(p, radius, result);
			}
		}
	}
}

void gs::KdTree::search(Point* p, Point* result)
{
	// get closets node
	KdTree* tree = this;
	while (!tree->isLeaf())
	{
		tree = tree->getChild(p);
	}
	result->pos[0] = tree->nodeX();
	result->pos[1] = tree->nodeY();
	result->pos[2] = tree->nodeZ();

	float radius = sqrt(pow(p->pos[0] - result->pos[0], 2.0) + pow(p->pos[1] - result->pos[1], 2.0) + pow(p->pos[2] - result->pos[2], 2.0));

	radiusSearch(p, &radius, result);
}

void gs::KdTree::mergeSort(std::vector<Point*> &pointCloud, int start, int end)
{
	int mid;
	if (start > end)
	{
		mid = (int)floor((end + start)*0.5);
		mergeSort(pointCloud, start, mid);
		mergeSort(pointCloud, mid + 1, end);

		merge(pointCloud, start, mid + 1, end);
	}
}

void gs::KdTree::merge(std::vector<Point*> &pointCloud, int left, int mid, int right)
{
	int i, leftEnd, numElements, tempPos;

	leftEnd = mid;
	tempPos = 0;
	numElements = right - left;

	while (left < leftEnd && mid <= right)
	{
		if (pointCloud[left]->pos[__sortOn] <= pointCloud[mid]->pos[__sortOn])
		{
			tempArray[tempPos] = pointCloud[left];
			tempPos++;
			left++;
		}
		else
		{
			tempArray[tempPos] = pointCloud[mid];
			tempPos++;
			mid++;
		}
	}

	while (left < leftEnd)
	{
		tempArray[tempPos] = pointCloud[left];
		left++;
		tempPos++;
	}
	while (mid <= right)
	{
		tempArray[tempPos] = pointCloud[mid];
		mid++;
		tempPos++;
	}

	for (int i = tempPos - 1; i >= 0; i--)
	{
		pointCloud[right] = tempArray[i];
		right--;
	}
}