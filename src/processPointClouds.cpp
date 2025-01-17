// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"

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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> filter;  
    filter.setInputCloud(cloud);
    filter.setLeafSize(filterRes, filterRes, filterRes);
    filter.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roofRegion(true);
    roofRegion.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofRegion.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roofRegion.setInputCloud(cloudRegion);
    roofRegion.filter(indices);


    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (auto point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
    
    for (auto index : inliers->indices)
    {
        plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane, obstacles);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud (cloud);

    seg.segment (*inliers, *coefficients);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    auto startTime = std::chrono::steady_clock::now();

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;

		while(inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}
		
		auto it = inliers.begin();
		float x1 = cloud->points.at(*it).x;
		float y1 = cloud->points.at(*it).y;
		float z1 = cloud->points.at(*it).z;
		it++;
		float x2 = cloud->points.at(*it).x;
		float y2 = cloud->points.at(*it).y;
		float z2 = cloud->points.at(*it).z; 
		it++;
		float x3 = cloud->points.at(*it).x;
		float y3 = cloud->points.at(*it).y;
		float z3 = cloud->points.at(*it).z; 

		float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		int index = 0;
		for (auto point : cloud->points)
		{
			float planeModel = i * point.x + j * point.y + k * point.z - (i * x1 + j * y1 + k * z1);
			float distance =  std::abs(planeModel)/std::sqrt(i*i + j*j + k*k);

			if (distance <= distanceTol)
			{
				inliers.insert(index);
			}
			index++;
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}   

    pcl::PointIndices::Ptr inlierIndices (new pcl::PointIndices);
    for (auto inlier : inliersResult)
    {
        inlierIndices->indices.push_back(inlier);
    }

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac plane Segmenation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierIndices,cloud);
    return segResult;
}

template <typename PointT>
bool hasIntensity(const PointT& point) {
    return std::is_same<decltype(point.intensity), float>::value;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::vector<bool> &processed, int index, KdTree* tree, float distanceTol, std::vector<std::vector<float>> points, std::vector<int> &cluster)
{
	processed[index] = true;
	std::vector<int> nearestPoints;
	std::vector<float> point = points[index];
	cluster.push_back(index);
	nearestPoints = tree->search(point, distanceTol);
	for (auto id : nearestPoints)
	{
		if(processed[id] == false)
		{
			point = points[id];
			proximity(processed, id, tree, distanceTol, points, cluster);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClusterCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree *tree = new KdTree;
    std::vector<std::vector<float>> points;
    bool intensityInfo = hasIntensity(PointT());
    std::cout << "intensity : " << intensityInfo << std::endl;
    for(auto cloudPoint : cloud->points)
    {
        std::vector<float> point;
        point.push_back(cloudPoint.x);
        point.push_back(cloudPoint.y);
        point.push_back(cloudPoint.z);
        
        if(intensityInfo)
            point.push_back(cloudPoint.intensity);
        
        points.push_back(point);
    }

    int pointId = 0;
    for (auto point : points)
    {
        tree->insert(point, pointId);
        pointId++;
    }

	std::vector<std::vector<int>> clusters;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

	std::vector<bool> processed (points.size(), false);
	
	int index = 0;
	for(auto point : points)
	{
		if(processed[index] == false)
		{
			std::vector<int> cluster;
			proximity(processed, index, tree, clusterTolerance, points, cluster);
			clusters.push_back(cluster);
		}
		index++;
	}

    for (auto cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>());
        for(auto indice : cluster)
        {
            PointT point;
            point.x = points[indice][0];
            point.y = points[indice][1];
            point.z = points[indice][2];
            
            if(intensityInfo)
                point.intensity = points[indice][3];

            cloudCluster->points.push_back(point); 
        }
        if(cloudCluster->points.size() > minSize && cloudCluster->points.size() < maxSize)
            cloudClusters.push_back(cloudCluster);
    }
	return cloudClusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) 
        {
            cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
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