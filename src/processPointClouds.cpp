// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"
#include "debug.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        float filterRes,
                                        Eigen::Vector4f minPoint,
                                        Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //Downsized point cloud
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};

    // Point Cloud that falls into a certain region, everything else is thrown away
    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    // Voxel Grid that will downsize the point cloud
    pcl::VoxelGrid<PointT> vgf;

    // Crop Box that will only keep points at specified area
    pcl::CropBox<PointT> region;

    // Crop Box that will throw away the points at specified area
    pcl::CropBox<PointT> roof(true); //true means extract points within the area

    // Voxel Grid's input is original point cloud
    vgf.setInputCloud(cloud);
    vgf.setLeafSize(filterRes, filterRes, filterRes);
    vgf.filter(*cloudFiltered);

    // Only keep points we need within specified region
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    // Throw away points on the roof of our own car
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.setNegative(true);
    roof.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int>& inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacle {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};

    for (int i = 0; i < cloud->size(); i++) {
        if (inliers.end() == inliers.find(i)) {
            obstacle->push_back((*cloud)[i]);
        } else {
            planeCloud->push_back((*cloud)[i]);
        }
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, planeCloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateCloudsPCL(pcl::PointIndices::Ptr inliers,
                                              typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacle {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};

    for (auto index: inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Create filtering object
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, planeCloud);
    return segResult;
}

template <typename PointT>
std::unordered_set<int>
RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations 
    while (maxIterations--) {
        std::unordered_set<int> inliers(cloud->size());
        // Randomly sample subset and fit line
        while (inliers.size() < 3) {
            inliers.insert(rand() % cloud->size());
        }

        auto itr = inliers.begin();

        auto p1 = (*cloud)[*itr];
        itr++;
        auto p2 = (*cloud)[*itr];
        itr++;
        auto p3 = (*cloud)[*itr];

        // line:  Ax + By + C = 0
        // where: A = (y1-y2), B = (x2-x1), C = (x1*y2 - x2*y1)
        float A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        float B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
        float C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        float D = -(A * p1.x + B * p1.y + C * p1.z);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        for (int index = 0; index < cloud->size(); index++) {
            if (inliers.find(index) != inliers.end()) {
                continue;
            }
            PointT p = (*cloud)[index];
            float dist = fabs(A * p.x + B * p.y + C * p.z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));

            if (dist < distanceTol) {
                inliers.insert(index);
            }
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         int maxIterations,
                                         float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers;

    inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    if (inliers.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud,
                                            int maxIterations,
                                            float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // Create Segmentation object
    pcl::SACSegmentation<PointT> seg;

    // We will use SACMODEL_PLANE model and RANSAC algorithm
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateCloudsPCL(inliers,cloud);
    return segResult;
}

/**
 * Helper function to cluster points in specified point cloud.
 *
 * This function is part of Euclidean Clustering algorithm.
 *
 * @param coud
 *  Point Cloud containing all of the points to be clustered
 * @param pos
 *  Position of the target point in the cloud for searching nearest points
 * @param processed
 *  Vector of flags where processed[i] indicates whether cloud[i] has been processed
 * @param tree - KD-Tree being used for the clustering
 * @param distanceTol - Distance tolerance
 * @param cluster - resulting cluster for targetted point at cloud[pos]
 */
template <typename PointT>
void
clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud,
              uint pos,
              std::vector<bool>& processed,
              KdTree<PointT>* tree,
              float distanceTol,
              std::vector<int>& cluster)
{
    // mark the point as processed and add it to the cluster
    processed[pos] = true;
    cluster.push_back(pos);

    // search nearest points within distanceTol
    std::vector<int> nearest = tree->search((*cloud)[pos], distanceTol);

    // for each nearby points, if not processed, repeat recursively
    for (auto n : nearest) {
        if (!processed[n]) {
            clusterHelper<PointT>(cloud, n, processed, tree, distanceTol, cluster);
        }
    }
}

/**
 * Euclidean Clustering algorithm
 *
 * @param cloud
 *  Point Cloud containing all of the points to be clustered
 * @param tree - KD-Tree being used for the clustering
 * @param distanceTol - Distance tolerance
 */
template <typename PointT>
std::vector<std::vector<int>>
euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,
                 KdTree<PointT>* tree,
                 float distanceTol)
{
    // create vector of flags indicating whether point has been processed
    // initially all points are unprocessed
    std::vector<bool> processed (cloud->points.size(), false);
    // vector of clusters
    std::vector<std::vector<int>> clusters;

    // for every point in point cloud, if not processed, start processing
    for (auto i = 0; i < cloud->size(); i++) {
        if (!processed[i]) {
            // create a fresh empty cluster and use it for processing point at i
            std::vector<int> cluster;
            clusterHelper<PointT>(cloud, i, processed, tree, distanceTol, cluster);

            // save the crafter cluster into vector of clusters
            clusters.push_back(cluster);
        }
    }

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Actual Cluster is represented by Point Cloud
    // So, we need vector of Point Clouds for our result
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // Create a fresh empty KD-Tree for Euiclidean Clustering
    KdTree<PointT> *tree {new KdTree<PointT>()};
    // Insert entire cloud into the tree
    // NOTE: KD-Tree will balance itself by sorting the points in the cloud
    // based on the depth
    tree->insert(cloud);
    //for (int i = 0; i < cloud->size(); i++) {
    //    tree->insert((*cloud)[i], i);
    //}
    // Run the Euclidean clustering algorithm and collect groups of indicies
    std::vector<std::vector<int>> cluster_indices = euclideanCluster<PointT>(cloud, tree, clusterTolerance);
    // for every group of cluster indicies, create a point cloud and push
    // the corresponding points from original cloud
    for (auto cli : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster {new pcl::PointCloud<PointT>};
        for (auto index : cli) {
            cluster->push_back((*cloud)[index]);
        }
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    typename pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT>};
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster {new pcl::PointCloud<PointT>};
        for (auto index : indices.indices) {
            cluster->push_back((*cloud)[index]);
        }
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
