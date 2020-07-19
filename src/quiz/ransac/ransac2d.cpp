/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <random>

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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    
    const auto &cloud_ref = *cloud;
    auto cloud_size = cloud->size();
    
    std::vector<size_t> idx(cloud_size);
    std::iota(idx.begin(), idx.end(), 0);
        
	// For max iterations 
    for (int i = 0; i < maxIterations; ++i) {
        auto points = samplePoints(idx, 2);
        
        const auto &p1 = cloud_ref[points[0]];
        const auto &p2 = cloud_ref[points[1]];
        
        const auto A = p2.y - p1.y;
        const auto B = p2.x - p1.x;
        const auto C = p2.x * p1.y - p2.y * p1.x;
        const auto D = std::sqrt(A * A + B * B);
        
        std::unordered_set<int> inliers;
        for (size_t j = 0; j < cloud_size; ++j) {
            const auto &p = cloud_ref[j];
            auto dist = std::fabs(A * p.x + B * p.y + C) / D;
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


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
