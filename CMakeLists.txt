cmake_minimum_required(VERSION 3.0.0)
project(iterative_closest_point VERSION 0.1.0)

add_executable( iterative_closest_point 
                src/main.cpp 
                src/ICP.cpp 
                src/KdTree.cpp 
                src/svd.cpp )

