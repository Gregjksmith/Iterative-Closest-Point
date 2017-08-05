#pragma once

#include <stdio.h>
#include <vector>

#define SORT_ON_X 0
#define SORT_ON_Y 1
#define SORT_ON_Z 2

namespace gs
{

	class KdTree;
	class Point;


	class Point
	{
	public:
		Point();
		Point(float x, float y, float z);
		inline Point(Point &rhs)
		{
			this->pos[0] = rhs.pos[0];
			this->pos[1] = rhs.pos[1];
			this->pos[2] = rhs.pos[2];
		}

		inline Point operator+(const Point& rhs)
		{
			Point r;
			r.pos[0] = pos[0] + rhs.pos[0];
			r.pos[1] = pos[1] + rhs.pos[1];
			r.pos[2] = pos[2] + rhs.pos[2];
			return r;
		}
		inline Point operator-(const Point& rhs)
		{
			Point r;
			r.pos[0] = pos[0] - rhs.pos[0];
			r.pos[1] = pos[1] - rhs.pos[1];
			r.pos[2] = pos[2] - rhs.pos[2];
			return r;
		}
		inline Point& operator=(const Point& rhs)
		{
			this->pos[0] = rhs.pos[0];
			this->pos[1] = rhs.pos[1];
			this->pos[2] = rhs.pos[2];

			return *this;
		}

		float pos[3];
	};

	class KdTree
	{
	public:
		KdTree(std::vector<Point*> &pointCloud);
		KdTree(std::vector<Point*> &pointCloud, int start, int end, int sortOn);
		virtual ~KdTree();
		void build(std::vector<Point*> &pointCloud, int start, int end, int sortOn);
		
		bool isLeaf();
		float split();
		KdTree* getChild(Point* searchPoint);
		float nodeX();
		float nodeY();
		float nodeZ();

		void search(Point* p, Point* result);
		void radiusSearch(Point* p, float* searchRadius, Point* result);

	private:
		gs::KdTree* __children[2];
		Point* __node;
		int __sortOn;
		int __start;
		int __end;

		void insertionSort(std::vector<Point*> &pointCloud, int start, int end, int sortOn);
		void mergeSort(std::vector<Point*> &pointCloud, int start, int end);
		void merge(std::vector<Point*> &pointCloud, int left, int mid, int right);

		int getNextSortOn(int sortOn);
		Point** tempArray;
	};
}