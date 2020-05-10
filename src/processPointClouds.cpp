// PCL lib Functions for processing point clouds

#include <pcl/common/pca.h>
#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Apply voxel point reduction.
    typename pcl::PointCloud<PointT>::Ptr filteredCloud{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(filterRes, filterRes, filterRes);
    grid.filter(*filteredCloud);

    // Apply region filter.
    typename pcl::PointCloud<PointT>::Ptr cloudRegion{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> region{true};
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    // Suppress points on the roof of the ego car.
    // Specifically, keep only points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f{-1.5, -1.7, -1., 1});
    roof.setMax(Eigen::Vector4f{2.6, 1.7, -0.4, 1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    // Obtain the indices of the roof points.
    pcl::PointIndices::Ptr roofPointIndices{new pcl::PointIndices};
    for (const auto idx : indices) {
        roofPointIndices->indices.push_back(idx);
    }

    // Use roof point indices to extract every _other_ (i.e., non-roof) point.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(roofPointIndices);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(const pcl::PointIndices::Ptr &inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) {

    typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud{new pcl::PointCloud<PointT>};

    // Creating the cloud containing plane points is trivial
    // since we already know the inliers.
    for (auto index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Next, obtain the obstacle cloud byt extracting the outliers.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // this is key
    extract.filter(*obstacleCloud);

    return std::make_pair(planeCloud, obstacleCloud);
}


/**
 * Perform planar segmentation of a point cloud.
 * @tparam PointT The point type.
 * @param cloud The point cloud to segment.
 * @param maxIterations The maximum number of RANSAC iterations.
 * @param distanceThreshold The RANSAC distance threshold.
 * @return A pair of plane points and non-plane points (outliers).
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         int maxIterations, float distanceThreshold) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Prepare the (random) sample consensus based point segmentation.
    // See e.g.
    // - https://pcl-tutorials.readthedocs.io/en/master/planar_segmentation.html
    // - https://pointcloudlibrary.github.io/documentation/classpcl_1_1_s_a_c_segmentation.html
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    // Configure segmentation.
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Determine planar points in the cloud.
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate planar model for the given point cloude." << std::endl;
    }

    // Separate the result into inliers and outliers.
    const auto planeAndObstacles = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " ms" << std::endl;

    return planeAndObstacles;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Create the cluster extraction object.
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Create a new cloud per cluster.
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        clusters.push_back(std::move(cloud_cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBoxAxisAligned(typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box{};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxOriented(typename pcl::PointCloud<PointT>::Ptr cluster) {
    // See: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

    BoxQ box{};

    // This block is debatable, but makes sense for street-bound cars (i.e., non-flying ones ...)
    // It suppresses the Z coordinate and forces the PCA to see X/Y extents only.
    // This, in turn, results in bounding boxes that are oriented in Z plane but aligned with XY and YZ.
    // TODO: Maybe there is a smarter way to do this; the copy isn't particularly fun.
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterXY{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::copyPointCloud(*cluster, *clusterXY);
    for (auto& pt : clusterXY->points) {
        pt.z = 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcaProjection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(clusterXY);
    pca.project(*cluster, *pcaProjection);

    const auto centroid = pca.getMean();
    const auto eigenvectors = pca.getEigenVectors();

    // Construct transformation matrix.
    Eigen::Matrix4f projectionTransform{Eigen::Matrix4f::Identity()};
    projectionTransform.block<3,3>(0,0) = eigenvectors.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());

    // Project point cloud to normalized space.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    box.bboxQuaternion = Eigen::Quaternionf{eigenvectors};
    box.bboxTransform = eigenvectors * meanDiagonal + centroid.head<3>();
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
