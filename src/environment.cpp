/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <memory>

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

    auto lidar = std::make_unique<Lidar>(cars, 0);
    auto pc = lidar->scan();
    
    //renderRays(viewer, lidar->position, pc);
    //renderPointCloud(viewer, pc, "pc");
    
    ProcessPointClouds<pcl::PointXYZ> pc_proc{};
    auto segmented_pc = pc_proc.SegmentPlane(pc, 10, 0.3);
    
    //renderPointCloud(viewer, segmented_pc.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmented_pc.second, "planeCloud", Color(0,1,0));
    
    auto cloudClusters = pc_proc.Clustering(segmented_pc.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pc_proc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        Box box = pc_proc.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI> pc_proc{};
  auto inputCloud = pc_proc.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  auto filterCloud = pc_proc.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-5, -6.2, -3, 1), Eigen::Vector4f ( 25, 6.5, 2, 1));

    //renderPointCloud(viewer, filterCloud, "inputCloud");  
    auto segmented_pc = pc_proc.SegmentPlane(filterCloud, 10, 0.3);
    
    //renderPointCloud(viewer, segmented_pc.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmented_pc.second, "planeCloud", Color(0,1,0));
    
    auto cloudClusters = pc_proc.Clustering(segmented_pc.first, 0.5, 10, 200);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(auto cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pc_proc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        ++clusterId;
        //Box box = pc_proc.BoundingBox(cluster);
        //renderBox(viewer, box, clusterId);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
ProcessPointClouds<pcl::PointXYZI>& pc_proc, 
const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    auto filterCloud = pc_proc.FilterCloud(cloud, 0.2, Eigen::Vector4f (-10, -6.2, -3, 1), Eigen::Vector4f ( 25, 6.5, 2, 1));
    auto segmented_pc = pc_proc.SegmentPlane(filterCloud, 20, 0.3);
    renderPointCloud(viewer, segmented_pc.second, "planeCloud", Color(0,1,0));
    
    auto cloudClusters = pc_proc.Clustering(segmented_pc.first, 0.45, 10, 600);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(auto cluster : cloudClusters) {
        std::cout << "cluster size ";
        pc_proc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        ++clusterId;
        Box box = pc_proc.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    
    ProcessPointClouds<pcl::PointXYZI> pc_proc{};
    std::vector<boost::filesystem::path> stream = pc_proc.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        cloud = pc_proc.loadPcd((*streamIterator).string());
        cityBlock(viewer, pc_proc, cloud);

        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
}
