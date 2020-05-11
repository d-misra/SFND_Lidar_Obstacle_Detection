// PCL lib Functions for processing point clouds

#include <pcl/common/pca.h>
#include "processPointClouds.h"
#include "render/render.h"
#include "kdtree.h"

template<typename PointT>
pcl::PointIndices::Ptr
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) const {
    std::unordered_set<std::size_t> inliersResult;
    // srand(0);

    const auto numPoints = cloud->points.size();
    assert(numPoints >= 3); // ... and points are not collinear ...

    // For max iterations
    for (auto i = 0; i < maxIterations; ++i) {
        // Randomly sample subset and fit a plane.
        // We're using a set here to ensure we never sample the same point twice.
        std::unordered_set<std::size_t> inliers;
        while (inliers.size() < 3) {
            inliers.insert(rand() % numPoints);
        }

        // Get all points.
        auto itr = inliers.begin();
        const auto &p1 = cloud->points[*itr];
        itr++;
        const auto &p2 = cloud->points[*itr];
        itr++;
        const auto &p3 = cloud->points[*itr];

        // Determine the plane through three points.
        const auto v1 = Vect3{p2.x, p2.y, p2.z} - Vect3{p1.x, p1.y, p1.z};
        const auto v2 = Vect3{p3.x, p3.y, p3.z} - Vect3{p1.x, p1.y, p1.z};

        // Determine plane normal by taking the cross product.
        const auto normal = v1.cross(v2);
        const auto normalizer = 1.0 / normal.norm();

        // Plane coefficients
        const auto A = normal.x;
        const auto B = normal.y;
        const auto C = normal.z;
        const auto D = -(A * p1.x + B * p1.y + C * p1.z);

        // Measure distance between every point and fitted line
        for (auto j = 0; j < numPoints; ++j) {

            // Skip points we already know are inliers.
            if (inliers.count(j) > 0) {
                continue;
            }

            // Determine the distance of the current point to the line.
            const auto &pt = cloud->points[j];
            const auto d = std::abs(A * pt.x + B * pt.y + C * pt.z + D) * normalizer;

            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol) {
                inliers.insert(j);
            }
        }

        // Return indices of inliers from fitted line with most inliers.
        // Specifically, we only keep the result if we found a better answer than
        // the one we already have.
        if (inliers.size() > inliersResult.size()) {
            inliersResult = std::move(inliers);
        }
    }

    pcl::PointIndices::Ptr result{new pcl::PointIndices};
    for (const auto& idx : inliersResult) {
        result->indices.push_back(idx);
    }

    return result;
}


template<typename PointT>
void ProcessPointClouds<PointT>::printNumPoints(typename pcl::PointCloud<PointT>::Ptr cloud) const {
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) const {

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
                                           typename pcl::PointCloud<PointT>::Ptr cloud) const {

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
                                         int maxIterations, float distanceThreshold) const {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

#ifdef PLANE_RANSAC_PCL

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
        std::cerr << "Could not estimate planar model for the given point cloud." << std::endl;
    }

    // Separate the result into inliers and outliers.
    const auto planeAndObstacles = SeparateClouds(inliers, cloud);

#else

    const auto inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate planar model for the given point cloud." << std::endl;
    }

    const auto planeAndObstacles = SeparateClouds(inliers, cloud);

#endif

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " ms" << std::endl;

    return planeAndObstacles;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize,
                                       int maxSize) const {

    // Time clustering process
    const auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

#ifdef EUCLIDEAN_CLUSTERING_PCL

    // Creating the KdTree object for the search method of the extraction.
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Create the cluster extraction object.
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

#else

    // Build the Kd-tree.
    KdTree tree;
    const auto& points = cloud->points;
    for (auto i = 0; i < points.size(); ++i) {
        const auto& data = points[i].data;
        tree.insert(data, i);
    }

    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<bool> processed(points.size(), false);
    for (auto seedIndex = 0; seedIndex < points.size(); ++seedIndex) {
        if (processed[seedIndex]) continue;

        pcl::PointIndices cluster;

        // Boundary to explore nodes at that are candidates for the cluster
        std::stack<int> boundary{};
        boundary.push(seedIndex);

        // If we terminate here according to cluster size, things will get weird.
        while (!boundary.empty()) {
            const auto pointIndex = boundary.top();
            boundary.pop();
            if (processed[pointIndex]) {
                continue;
            }

            processed[pointIndex] = true;
            cluster.indices.push_back(pointIndex);

            const auto& data = points[pointIndex].data;
            const auto nearest = tree.search(data, clusterTolerance);
            for (const auto& neighborIndex : nearest) {
                if (!processed[neighborIndex]) {
                    boundary.push(neighborIndex);
                }
            }
        }

        const auto clusterSize = cluster.indices.size();
        if ((clusterSize >= minSize) && (clusterSize <= maxSize)) {
            clusterIndices.push_back(std::move(cluster));
        }
    }

#endif

    // Create a new cloud per cluster.
    for (const auto &clusterIndex : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        for (const auto& index : clusterIndex.indices) {
            cloud_cluster->points.push_back(points[index]);
        }

        clusters.push_back(std::move(cloud_cluster));
    }

    const auto endTime = std::chrono::steady_clock::now();
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBoxAxisAligned(typename pcl::PointCloud<PointT>::Ptr cluster) const {

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxOriented(typename pcl::PointCloud<PointT>::Ptr cluster) const {
    // See: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

    BoxQ box{};

    assert(cluster->points.size() >= 3); // ... and not collinear ...

    // This block is debatable, but makes sense for street-bound cars (i.e., non-flying ones ...)
    // It suppresses the Z coordinate and forces the PCA to see X/Y extents only.
    // This, in turn, results in bounding boxes that are oriented in Z plane but aligned with XY and YZ.
    // TODO: Maybe there is a smarter way to do this; the copy isn't particularly fun.
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterXY{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::copyPointCloud(*cluster, *clusterXY);
    for (auto &pt : clusterXY->points) {
        pt.z = 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcaProjection(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(clusterXY);
    pca.project(*clusterXY, *pcaProjection);

    const auto centroid = pca.getMean();
    const auto eigenvectors = pca.getEigenVectors();

    // Construct transformation matrix.
    Eigen::Matrix4f projectionTransform{Eigen::Matrix4f::Identity()};
    projectionTransform.block<3, 3>(0, 0) = eigenvectors.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * centroid.head<3>());

    // Project point cloud to normalized space.
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected{new pcl::PointCloud<PointT>};
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

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
