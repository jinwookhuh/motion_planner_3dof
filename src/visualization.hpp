#pragma once
#include "rrt.hpp"
#include "opencv2/opencv.hpp"


using namespace kinematics;
using namespace cv;
using namespace std;

class Visualization
{
private:
    /* data */
public:
    int showWorkspace(RRT rrt, std::vector<VerticeList> vertices, ConfigList path);
};


