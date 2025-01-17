/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <algorithm>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputcloud);
    //renderPointCloud(viewer, inputcloud, "pcd");
    ProcessPointClouds<pcl::PointXYZ> *pointcloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointcloudProcessor->SegmentPlane(inputcloud, 250, 0.2);
    //renderPointCloud(viewer, segResult.first, "plane", Color(0, 1, 0));
    //renderPointCloud(viewer, segResult.second, "obstacles", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointcloudProcessor->Clustering(segResult.second, 1.0, 3, 30);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointcloudProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        Box box = pointcloudProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

}

void cityBlockCustom(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, {-20, -6, -3, 1}, {30, 7, 7, 1});

    //renderPointCloud(viewer, filteredCloud, "fitleredCloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->RansacPlaneSegmentation(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segResult.first, "plane", Color(0, 1, 0));
    renderPointCloud(viewer, segResult.second, "obstacle", Color(1, 0, 0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanClusterCustom(segResult.second, 0.5, 10, 500);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(1,0.5,1), Color(1,1,1), Color(0.5,0.5,1)};
  
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    /*
    auto minMaxX = std::minmax_element(inputCloud->points.begin(), inputCloud->points.end(), [](const pcl::PointXYZI& a, const pcl::PointXYZI& b){
        return a.x < b.x;
    });
    auto minMaxY = std::minmax_element(inputCloud->points.begin(), inputCloud->points.end(), [](const pcl::PointXYZI& a, const pcl::PointXYZI& b){
        return a.y < b.y;
    });
    auto minMaxZ = std::minmax_element(inputCloud->points.begin(), inputCloud->points.end(), [](const pcl::PointXYZI& a, const pcl::PointXYZI& b){
        return a.z < b.z;
    });

    //std::cout << "x min : " << minMaxX.first->x << " y min : " << minMaxY.first->y << " z min : " << minMaxZ.first->z << std::endl;
    //std::cout << "x max : " << minMaxX.second->x << " y max : " << minMaxY.second->y << " z max : " << minMaxZ.second->z << std::endl;
    */

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, {-20, -6, -3, 1}, {30, 7, 7, 1});

    //renderPointCloud(viewer, filteredCloud, "fitleredCloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segResult.first, "plane", Color(0, 1, 0));
    renderPointCloud(viewer, segResult.second, "obstacle", Color(1, 0, 0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segResult.second, 0.5, 10, 500);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(1,0.5,1), Color(1,1,1), Color(0.5,0.5,1)};
  
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
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
    
    /*
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);
    */

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    
    //cityBlock(viewer, pointProcessorI, inputCloudI);
    cityBlockCustom(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        //cityBlock(viewer, pointProcessorI, inputCloudI);
        cityBlockCustom(viewer, pointProcessorI, inputCloudI);
            
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}