# Iterative-Closest-Point

Implementation of the iterative closest point algorithm. Algorithm is based on the work outlined in [1]. A point cloud is transformed such that it best "matches" a reference point cloud. 

For each point in the dynamic point cloud, we search for its closest point in the static point cloud. We solve for an affine transform which minimized the distance between these point pairs. The dynamic point are updated with this affine transform and the process is repeated until a stopping condition is met.

Transforms are solved using SVD decomposition and searching is performed using kd-tree NN search.

## API

```
class Point
{
	float pos[3];	// x,y,z coordinates of a point in space.
}

ICP.h:

void icp(std::vector<Point*> &dynamicPointCloud, std::vector<Point*> &staticPointCloud);
```

## Install

build the following located in /src :

+ ICP.cpp
+ ICP.h
+ KdTree.cpp
+ KdTree.h
+ svd.cpp
+ svd.h
+ defs_and_types.h

## Results

iteration 0:
![Instant Radiosity](https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/images/ICP_iteration_0.png?raw=true)

iteration 20:
![Instant Radiosity](https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/images/ICP_iteration_20.png?raw=true)

iteration 40:
![Instant Radiosity](https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/images/ICP_iteration_40.png?raw=true)

iteration 100:
![Instant Radiosity](https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/images/ICP_iteration_100.png?raw=true)

iteration 400:
![Instant Radiosity](https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/images/ICP_iteration_400.png?raw=true)

[1] Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. "Least-squares fitting of two 3-D point sets." IEEE Transactions on pattern analysis and machine intelligence 5 (1987): 698-700.