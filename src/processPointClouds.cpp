// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/voxel_grid.h>
#include <random>

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
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    auto cloud_filtered = boost::make_shared<pcl::PointCloud<PointT>>();
    
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    
    auto cloud_cropped = boost::make_shared<pcl::PointCloud<PointT>>();
    
    pcl::CropBox<PointT> box;
    box.setInputCloud (cloud_filtered);
    box.setMin(minPoint);
    box.setMax(maxPoint);
    box.filter(*cloud_cropped);
    
    auto cloud_wo_car = boost::make_shared<pcl::PointCloud<PointT>>();
    
    pcl::CropBox<PointT> box_car;
    box_car.setNegative(true);
    box_car.setInputCloud (cloud_cropped);
    box_car.setMin(Eigen::Vector4f(-2, -1.4, -2, 1));
    box_car.setMax(Eigen::Vector4f(2.7, 1.4, 2, 1));
    box_car.filter(*cloud_wo_car);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_wo_car;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    auto cloud_p = boost::make_shared<pcl::PointCloud<PointT>>();
    auto cloud_f = boost::make_shared<pcl::PointCloud<PointT>>();

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract{};
    
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::clog << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
              << " data points.\n";

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);

    return std::make_pair(cloud_f, cloud_p);
}

static std::vector<size_t>
samplePoints(std::vector<size_t>& idx, size_t n) {
    if (n > idx.size()) {
        throw std::invalid_argument("Number of samples cannot be bigger than size of the cloud");
    }
    
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::vector<size_t> result(n);
    
    for (size_t i = 0; i < n; ++i) {
        std::uniform_int_distribution<size_t> dis(0, idx.size() - i - 1);
        auto new_id = dis(gen);
        result[i] = idx[new_id];
        std::swap(idx[new_id], idx[idx.size() - i - 1]);
    }
    return result;
}

template<typename PointT>
std::unordered_set<int> 
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    
    const auto &cloud_ref = *cloud;
    auto cloud_size = cloud->size();
    
    std::vector<size_t> idx(cloud_size);
    std::iota(idx.begin(), idx.end(), 0);
        
	// For max iterations 
    for (int i = 0; i < maxIterations; ++i) {
        auto points = samplePoints(idx, 3);
        
        const auto &p1 = cloud_ref[points[0]];
        const auto &p2 = cloud_ref[points[1]];
        const auto &p3 = cloud_ref[points[2]];
        
        const auto x21 = p2.x - p1.x;
        const auto x31 = p3.x - p1.x;
        const auto y21 = p2.y - p1.y;
        const auto y31 = p3.y - p1.y;
        const auto z21 = p2.z - p1.z;
        const auto z31 = p3.z - p1.z;
        
        const auto A = y21 * z31 - z21 * y31;
        const auto B = z21 * x31 - x21 * z31;
        const auto C = x21 * y31 - y21 * x31;
        const auto D = -(A * p1.x + B * p1.y + C * p1.z);
        const auto E = std::sqrt(A * A + B * B + C * C);
        
        std::unordered_set<int> inliers;
        for (size_t j = 0; j < cloud_size; ++j) {
            const auto &p = cloud_ref[j];
            auto dist = std::fabs(A * p.x + B * p.y + C * p.z + D) / E;
            if (dist < distanceTol) {
                inliers.emplace(j);
            }
        }
        
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }
	return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	auto inliers = boost::make_shared<pcl::PointIndices>();
    auto inliers_idx = RansacPlane(cloud, maxIterations, distanceThreshold);
    for (const auto &idx : inliers_idx) {
        inliers->indices.push_back(idx);
    }

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return std::make_pair(typename pcl::PointCloud<PointT>::Ptr{}, typename pcl::PointCloud<PointT>::Ptr{});
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers, cloud);
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, KdTree<PointT>* tree, 
                float distanceTol, std::unordered_set<int>& processed_ids) {
    processed_ids.insert(id);
    cluster.push_back(id);
    auto neighbours = tree->search(cloud->points[id], distanceTol);
    for (int n : neighbours) {
        if (not processed_ids.count(n)) {
            proximity(n, cloud, cluster, tree, distanceTol, processed_ids);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> 
ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processed_ids;
    for (int id = 0; id < cloud->points.size(); ++id) {
        if (not processed_ids.count(id)) {
            std::vector<int> cluster;
            proximity(id, cloud, cluster, tree, distanceTol, processed_ids);
            clusters.emplace_back(std::move(cluster));
        }
    }
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    KdTree<PointT> tree;
    for (int i = 0; i < cloud->points.size(); ++i) { 
    	tree.insert(cloud->points[i], i); 
    }
    
    auto euc_clusters = euclideanCluster(cloud, &tree, clusterTolerance);
    for (const auto& cluster : euc_clusters) {
        if (cluster.size() >= minSize and cluster.size() <= maxSize) {
            auto cloud_cluster = boost::make_shared<pcl::PointCloud<PointT>>();
            for (auto id : cluster) {
                cloud_cluster->points.push_back(cloud->points[id]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            clusters.push_back(cloud_cluster);   
        }
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