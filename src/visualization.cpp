#include "visualization.hpp"
#include <iostream>

int Visualization::showWorkspace(RRT rrt, std::vector<VerticeList> vertices, ConfigList path){
    // OpenCV
    IplImage *dstImage;
    dstImage = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U,3);

    cvSet(dstImage, CV_RGB(255,255,255));
 
    // Robot Configuration index
    int config_idx =0;
    //List of obstacles
    std::vector<CvPoint> list_pts;
    CvPoint ots1[4], ots2[4]; 
    //parts of robot body
    CvPoint rts1[4], rts2[4], rts3[4], rts4[4];

    while(config_idx <= path.size()-1)
    {
        int scale = 100;

        // obstacle locations
        for (int i = 0; i<4; i++) ots1[i] = {int(scale * vertices[0][i].x+100), 512-int(scale * vertices[0][i].y+100)};
        for (int i = 0; i<4; i++) ots2[i] = {int(scale * vertices[1][i].x+100), 512-int(scale * vertices[1][i].y+100)};

        CvPoint *polygon[2] = {ots1,ots2};

        int opts[2] = {4,4};
        cvPolyLine(dstImage, polygon,opts,2,1,CV_RGB(255,0,0));
        cvFillPoly(dstImage,polygon,opts,2,CV_RGB(255,0,0));

        ObstacleList robot_cofig = rrt.findRobotBodies(path[config_idx]);

        // Update the robot shape
        for (int i = 0; i<4; i++) rts1[i] = {int(scale * robot_cofig[0][i].x)+100, 512 -int(scale * robot_cofig[0][i].y+100)};
        for (int i = 0; i<4; i++) rts2[i] = {int(scale * robot_cofig[1][i].x)+100, 512 -int(scale * robot_cofig[1][i].y+100)};
        for (int i = 0; i<4; i++) rts3[i] = {int(scale * robot_cofig[2][i].x)+100, 512 -int(scale * robot_cofig[2][i].y+100)};
        for (int i = 0; i<4; i++) rts4[i] = {int(scale * robot_cofig[3][i].x)+100, 512 -int(scale * robot_cofig[3][i].y+100)};
       
        CvPoint *robot_polygon[4] = {rts1,rts2, rts3, rts4};

        // Draw the robot with the target box
        int npts[4] = {4, 4, 4, 4};
        cvPolyLine(dstImage, robot_polygon,npts,4,1,CV_RGB(0,0,255));
        cvFillPoly(dstImage,robot_polygon,npts,4,CV_RGB(0,255,255));
        cvNamedWindow("Drawing Graphics",CV_WINDOW_AUTOSIZE);
        cvShowImage("Drawing Graphics",dstImage);

        char c = (char)cv::waitKey(500);
        if (c == 'q') break;
        
        cvSet(dstImage, CV_RGB(255,255,255));
        config_idx += 1;

    }

    cv::waitKey(0);
    
    cvDestroyAllWindows();
    cvReleaseImage(&dstImage);

    return 0;


}
