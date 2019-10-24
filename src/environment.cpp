/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    bool renderScene = false; //true;
    bool renderBoundingBox = true;
    bool renderCluster = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "abc", Color(0,1.0,0));
    ProcessPointClouds<pcl::PointXYZ> pclProcessor;
    auto segmentedCloud = pclProcessor.SegmentPlane(inputCloud, 100, 0.2);

    auto clusters = pclProcessor.Clustering(segmentedCloud.first, 1.0, 3, 30);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    int clusterID = 0;
    for (auto cluster : clusters) {
        if (renderCluster) {
            renderPointCloud(viewer, cluster, "obstacle"+std::to_string(clusterID), colors[clusterID % colors.size()]);
        }
        if (renderBoundingBox) {
            Box box = pclProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterID);
        }
        clusterID++;
    }

    //renderPointCloud(viewer, segmentedCloud.first, "obstacles", Color(1.0,0,0));
    //renderPointCloud(viewer, segmentedCloud.second, "road", Color(0,1.0,0));
    // Delete the lidar
    delete lidar;
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

void
cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
          ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
          const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud;
    filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f,
                                               Eigen::Vector4f(-10.0f,-5.0f,-2.0f,1),
                                               Eigen::Vector4f(30.0f,8.0f,1.0f, 1));

    // Segment cloud
    auto segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 200, 0.1);

    // First part of the segmented cloud set of points that is not on Plane
    // Cluster the point cloud via Euclidean Clustering algorithm
    auto clusters = pointProcessorI->Clustering(segmentedCloud.first, 0.50, 10, 500);

    // Draw bounding boxes around each cluster of points
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    int clusterID = 0;
    for (auto c : clusters) {
        renderPointCloud(viewer, c, "obstacle"+std::to_string(clusterID), colors[clusterID % colors.size()]);
        renderBox(viewer,pointProcessorI->BoundingBox(c), clusterID);
        clusterID++;
    }

    // Draw points of Plane (road) in green color to distinguish it from objects
    renderPointCloud(viewer, segmentedCloud.second, "plane", Color(0,1,0));
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    auto pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}
