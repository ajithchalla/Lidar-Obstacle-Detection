/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
using namespace std;
#include <vector>
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
    Color color = Color(0,1,0);
    Color color2 = Color(1,0,0);
    
    // TODO:: Create lidar sensor 

    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud =  lidar->scan();
    // renderRays(viewer,lidar->position, pointcloud);
    // renderPointCloud(viewer, pointcloud, "firstpcd", color);
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* PointProcessor = new  ProcessPointClouds<pcl::PointXYZ>();
    auto pointclouds = PointProcessor->SegmentPlane(pointcloud,100,0.2);
    // renderPointCloud(viewer, pointclouds.first, "roadplane", color);
    // renderPointCloud(viewer, pointclouds.second, "obstacles", color2);

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = PointProcessor->Clustering(pointclouds.second, 1, 3, 100);
    vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr it:clusters){
        renderPointCloud(viewer, it,"obstacle"+to_string(clusterId), colors[clusterId%colors.size()]);

        Box box1 = PointProcessor->BoundingBox(it);
        renderBox(viewer, box1, clusterId, colors[clusterId%colors.size()]);
        clusterId++;
    }
    renderPointCloud(viewer, pointclouds.first, "roadplane", color);
}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
//     ProcessPointClouds<pcl::PointXYZI>* pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     // renderPointCloud(viewer, inputCloud, "inputCloud");

//     Color color = Color(0,1,0);
//     Color color2 = Color(1,0,0);
    
//     pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud = pointProcessorI->FilterCloud(inputCloud,0.2,Eigen::Vector4f(-10,-10,-2, 1), Eigen::Vector4f(10,10,2, 1));
//     // renderPointCloud(viewer, croppedCloud, "inputCloud");    

//     auto pointclouds = pointProcessorI->SegmentPlane(croppedCloud,100,0.2);
//     renderPointCloud(viewer, pointclouds.first, "roadplane", color);
//     renderPointCloud(viewer, pointclouds.second, "obstacles", color2);
    
//     vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(pointclouds.second, 1, 3, 1000);
//     vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

//     int clusterId = 0;
//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr it:clusters){
//         renderPointCloud(viewer, it,"obstacle"+to_string(clusterId), colors[clusterId%colors.size()]);

//         Box box1 = pointProcessorI->BoundingBox(it);
//         renderBox(viewer, box1, clusterId, colors[clusterId%colors.size()]);
//         clusterId++;
//     }
//     renderPointCloud(viewer, pointclouds.first, "roadplane2", color);
// }

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    Color color = Color(0,1,0);
    Color color2 = Color(1,0,0);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud = pointProcessorI->FilterCloud(inputCloud,0.2,Eigen::Vector4f(-10,-5,-2, 1), Eigen::Vector4f(10,8,2, 1));
    // renderPointCloud(viewer, croppedCloud, "inputCloud");    

    auto pointclouds = pointProcessorI->SegmentPlane(croppedCloud,100,0.2);
    renderPointCloud(viewer, pointclouds.first, "roadplane", color);
    renderPointCloud(viewer, pointclouds.second, "obstacles", color2);
    
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(pointclouds.second, 1, 3, 1000);
    vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr it:clusters){
        renderPointCloud(viewer, it,"obstacle"+to_string(clusterId), colors[clusterId%colors.size()]);

        Box box1 = pointProcessorI->BoundingBox(it);
        renderBox(viewer, box1, clusterId, colors[clusterId%colors.size()]);
        clusterId++;
    }
    renderPointCloud(viewer, pointclouds.first, "roadplane2", color);
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
    // simpleHighway(viewer);
    // cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}