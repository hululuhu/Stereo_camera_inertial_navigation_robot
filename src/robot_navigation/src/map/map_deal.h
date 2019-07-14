#ifndef MAP_DEAL_H
#define MAP_DEAL_H

#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/opencv.hpp> 
#include "../route_plan_algorithm/A_star.h"


using namespace std;
using namespace cv;

uchar **  map_deal(int *rowl, int *coll, cv::String map_path, int *map_count); 
void printf_Route(cv::String map_path, pAStarNode * path_stack, int top, int iteration);    

#endif 
