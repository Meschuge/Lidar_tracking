/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered {pointProcessorI->FilterCloud(cloud,  0.27, Eigen::Vector4f(-15,-5.0,-3.8,1), Eigen::Vector4f(30,7,3,1))};
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filtered, 50, 0.3);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters {pointProcessorI->Clustering(segmentCloud.first, 0.4, 6, 1000)};

    auto itr = clusters.begin();
    int i {0};
    for(auto val : clusters){
        renderPointCloud(viewer, val, std::to_string(i), Color(1,0,0));
        Box box = pointProcessorI->BoundingBox(val);
        renderBox(viewer, box, i);
        i++;
    }

    renderPointCloud(viewer, segmentCloud.second, "plane", Color(0,1,0));
    renderPointCloud(viewer, segmentCloud.first, "obs", Color(0,0,1));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);


    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> *ppc {new ProcessPointClouds<pcl::PointXYZI>};
    std::vector<boost::filesystem::path> stream = ppc->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        cloud = ppc->loadPcd((*streamIterator).string());
        cityBlock(viewer, ppc, cloud);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}