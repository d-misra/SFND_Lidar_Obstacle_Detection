/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(0));

    const auto numPoints = cloud->points.size();
    assert(numPoints >= 2);

    // For max iterations
    for (auto i = 0; i < maxIterations; ++i) {

        // Randomly sample subset and fit line.
        // We're using a set here to ensure we never sample the same point twice.
        std::unordered_set<int> inliers;
        while (inliers.size() < 2) {
            inliers.insert(rand() % numPoints);
        }

        // Get both points.
        auto itr = inliers.begin();
        const auto& p1 = cloud->points[*itr];
        itr++;
        const auto& p2 = cloud->points[*itr];

        // Determine the line through both points.
        const auto a = p1.y - p2.y;
        const auto b = p2.x - p1.x;
        const auto c = p1.x*p2.y - p2.x*p1.y;
        const auto normalizer = 1.0 / std::sqrt(a * a + b * b);

        // Measure distance between every point and fitted line
        for (auto j = 0; j < numPoints; ++j) {

            // Skip points we already know are inliers.
            if (inliers.count(j) > 0) {
                continue;
            }

            // Determine the distance of the current point to the line.
            const auto& pt = cloud->points[j];
            const auto px = pt.x;
            const auto py = pt.y;
            const auto d = std::abs(a*px + b*py + c) * normalizer;

            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol) {
                inliers.insert(j);
            }
        }

        // Return indices of inliers from fitted line with most inliers.
        // Specifically, we only keep the result if we found a better answer than
        // the one we already have.
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }

    }

    return inliersResult;

}

int main() {

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


    // Run RANSAC to find the line.
    const auto maxIterations = 10;
    const auto distanceTolerance = 1.0;
    std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceTolerance);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

}
