/*
 * @ingroup environment
 * @brief Create simple 3d highway environment using PCL for exploring self-driving car sensors.
 *
 * \author Markus Mayer
 * \author Aaron Brown
 * */

#include <memory>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"  // using templates for processPointClouds so also include .cpp to help linker


#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantConditions"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    constexpr auto green = Color{0.365, 0.788, 0.290};
    constexpr auto blue = Color{0, 0.443, 0.73};

    Car egoCar{Vect3{0, 0, 0}, Vect3{4, 2, 2}, green, "egoCar"};
    Car car1{Vect3{15, 0, 0}, Vect3{4, 2, 2}, blue, "car1"};
    Car car2{Vect3{8, -4, 0}, Vect3{4, 2, 2}, blue, "car2"};
    Car car3{Vect3{-12, 4, 0}, Vect3{4, 2, 2}, blue, "car3"};

    std::vector<Car> cars { egoCar, car1, car2, car3 };

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    const auto renderScene = false;  // if false, only the point cloud is rendered
    const auto renderClusters = true;
    const auto renderBoxes = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Initialize LiDAR.
    const auto groundSlope = 0;
    const auto lidar = std::make_unique<Lidar>(cars, groundSlope);

    // Scan environment and render results.
    const auto pointCloud = lidar->scan();
#ifdef RENDER_RAYS
    renderRays(viewer, lidar->position, pointCloud);
#endif

    // Obtain point processor.
    const auto pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();

    const auto maxIterations = 100;
    const auto distanceTolerance = 0.5F;
    const auto [ planeCloud, obstacleCloud ] = pointProcessor->SegmentPlane(pointCloud, maxIterations, distanceTolerance);

    renderPointCloud(viewer, planeCloud, "plane", Color{0.623, 0.609, 0.591});

    // Cluster the obstacles
    const auto cloudClusters = pointProcessor->Clustering(obstacleCloud, 1.0, 3, 30);

    // Cycle through the colors ... all three of them.
    int clusterId = 0;
    std::vector<Color> colors = {
            Color(1.000,0.548,0.492),
            Color(0.953,0.792,1.000),
            Color(0.590,1.000,0.907)};

    for(const auto& cluster : cloudClusters)
    {
        const auto& clusterColor = colors[clusterId];

        if (renderClusters) {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacles" + std::to_string(clusterId), clusterColor);
        }

        if (renderBoxes) {
            // const auto box = pointProcessor->BoundingBoxAxisAligned(cluster);
            const auto box = pointProcessor->BoundingBoxOriented(cluster);
            renderBox(viewer, box, clusterId, clusterColor);
        }

        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    const auto pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
    renderPointCloud(viewer, inputCloud, "inputCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0.133, 0.133, 0.133);
    viewer->initCameraParameters();

    // distance away in meters
    const int distance = 16;

    switch (setAngle) {
        case XY:
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown:
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side:
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS:
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS) {
        viewer->addCoordinateSystem(1.0);
    }
}


int main(int argc, char **argv) {
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

#ifdef SIMPLE_HIGHWAY
    simpleHighway(viewer);
#else
    cityBlock(viewer);
#endif

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

#pragma clang diagnostic pop
