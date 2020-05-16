// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    // cropping ROI
    typename pcl::PointCloud<PointT>::Ptr ROIregion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*ROIregion);

    std::vector<int> roof_indices;
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roofFilter.setInputCloud(ROIregion);
    roofFilter.filter(roof_indices);

    pcl::PointIndices::Ptr roof_inliers (new pcl::PointIndices);

    for (auto point : roof_indices) {
        roof_inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (ROIregion);
    extract.setIndices(roof_inliers);
    extract.setNegative (true);
    extract.filter (*ROIregion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return ROIregion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstable(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*road);
    extract.setNegative (true);
    extract.filter (*obstable);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstable, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    /*** ========== PCL Implementation ============ ***/
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    // Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    seg.setMaxIterations(maxIterations);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (distanceThreshold);
//
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);

    /*** ========== Custom RANSAC Implementation ============ ***/
    srand(time(NULL));
    while (maxIterations--) {
        std::unordered_set<int> inliers_idx;
        while (inliers_idx.size() < 3) {
            inliers_idx.insert(rand()%cloud->points.size()); //store a number
        }
        PointT p1, p2, p3;
        auto itr = inliers_idx.begin();
        p1 = cloud->points[*itr];
        itr++;
        p2 = cloud->points[*itr];
        itr++;
        p3 = cloud->points[*itr];
        //fit a plane
        Eigen::Vector3f v1(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
        Eigen::Vector3f v2(p3.x-p1.x, p3.y-p1.y, p3.z-p1.z);
        auto coefficent = v1.cross(v2);
        float a = coefficent[0];
        float b = coefficent[1];
        float c = coefficent[2];
        float d = - (a*p1.x + b*p1.y + c*p1.z);


        pcl::PointIndices::Ptr inliers_candidate(new pcl::PointIndices);
        for (int i = 0; i < cloud->points.size(); i++){
            auto p = cloud->points[i];
            float distance = fabs(p.x*a + p.y*b + p.z*c + d) / sqrt(a*a + b*b + c*c);
            if (distance < distanceThreshold) {
                inliers_candidate->indices.push_back(i);
            }
        }
        if (inliers_candidate->indices.size() > inliers->indices.size() ) {
            inliers = inliers_candidate;
        }
    }
    if (inliers->indices.size () == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }
    std::cerr << "Ground inliers: " << inliers->indices.size () << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void  ProcessPointClouds<PointT>::proximity_check(int index, std::vector<PointT, Eigen::aligned_allocator<PointT>> points, KdTree* tree, std::vector<int>& cluster, std::vector<bool>& processed_mark, float distanceTol) {
    processed_mark[index] = true;
    cluster.push_back(index);
    Eigen::Vector3f point(points[index].x, points[index].y, points[index].z);
    auto search_result = tree->search(point, distanceTol);
    for (int idx : search_result) {
        if (!processed_mark[idx]) {
            proximity_check(idx, points, tree, cluster, processed_mark, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    /*** ========== PCL Implementation ============ ***/
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    tree->setInputCloud (cloud);
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance (clusterTolerance);
//    ec.setMinClusterSize (minSize);
//    ec.setMaxClusterSize (maxSize);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud);
//    ec.extract (cluster_indices);
    /*** ========== Custom Euclidain Clustering Implementation ============ ***/
    KdTree* kdtree = new KdTree;
    int i = 0;
    std::vector<int> idx(cloud->points.size());
    std::iota(idx.begin(), idx.end(), 0);
    while (i < cloud->points.size()) { // index the original cloud
        //using a balanced way to insert points
        int cmp_dim = i % 3;//asume 3d here;
        //sort based on current cmp_dim
        std::stable_sort(idx.begin(), idx.end(),[&cloud, cmp_dim](int i1, int i2) {return cloud->points[i1].data[cmp_dim] <  cloud->points[i2].data[cmp_dim];});
        int point_idx = idx[idx.size()/2]; // get the lower median instead rounding by integer division
        Eigen::Vector3f point(cloud->points[point_idx].x, cloud->points[point_idx].y, cloud->points[point_idx].z);
        kdtree->insert(point, point_idx);
        idx.erase(idx.begin() + idx.size()/2);
        i++;
    }

//    for (int i=0; i<cloud->points.size(); i++) {
//        Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//        kdtree->insert(point, i);
//    }

    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<bool> processed_mark(cloud->points.size(), false);
    for (int i = 0; i < cloud->points.size(); i++) {
        if (processed_mark[i]) {
            continue;
        }
        std::vector<int> cluster;
        proximity_check(i, cloud->points, kdtree, cluster, processed_mark, clusterTolerance);
        if (cluster.size() <= maxSize && cluster.size() >= minSize) {
            pcl::PointIndices::Ptr cluster_Indice(new pcl::PointIndices);
            cluster_Indice->indices = cluster;
            cluster_indices.push_back(*cluster_Indice);
        }
    }

    for (auto cluster_idx : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        for (auto idx : cluster_idx.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
