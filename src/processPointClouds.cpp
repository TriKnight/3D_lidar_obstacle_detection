// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// Code Filtering the real PCD Data with voxel Grid
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Downsampling --> create the voxel grid point reduction
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes); //Set the voxel grid leaf size [x,y,z]
    vg.filter(*cloudFiltered);
    //Creating cropbox
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint); //Set the minimum point of the box.
    region.setMax(maxPoint); //Set the maximum point of the box
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices; //Seting the point in side the roofbox

    //Creating roof bounding box to remove it
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.7, -2.0, -1.1  ,  1));
    roof.setMax(Eigen::Vector4f ( 3.0,  2.0, 0.1,  1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices); //

    //get inliner form indices
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    // Remove the roof inliers by (cloudRegion-inliers)
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

//Create two new point clouds, one cloud with obstacles and other with segmented plane
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

//Segmentation Plane using PCL library pcl::SACSegmentation
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // Fill the function to segme cloud into two part: driveable plane(road) & obstacles
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest plannar component
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // TODO:: Fill in this function to find inliers for the cloud.
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate plannar model" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}
//------------------Implemetation SegmentPlane by scratch-------------------
//--------------------------------------------------------------------------
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneScratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time the segmentation step
    auto startTime = std::chrono::steady_clock::now();

    // RANSAC algorithm for plane segmentation
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations--)
    {
        // Randomly sample subset and fit plane
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % cloud->points.size());
        }

        // Measure distance between every point and fitted plane
        // If distance is smaller than threshold count it as inlier
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float a, b, c, d;
        // v1 be a vector from point 1 to point 2 in the plane
        // v2 be a vector from point 1 to point 3 in the plane
        // v_cross is the cross product v1 x v2
        float v_cross[3] = {((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1)),
                      ((z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1)),
                      ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))};

        // This iteration's plane is modeled by the equation Ax + By + Cz + D = 0
        a = v_cross[0];
        b = v_cross[1];
        c = v_cross[2];

        d = -(v_cross[0] * x1 + v_cross[1] * y1 + v_cross[2] * z1);

        for (int index = 0; index < cloud->points.size(); index++)
        {
            // Skip if the considered point is already an inlier.
            if (inliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float distance = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

            if (distance <= distanceThreshold)
            {
                inliers.insert(index);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // Populate the newly allocated point clouds
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());  // The road plane points
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>()); // The not-plane points

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Return a std::pair of point clouds (obstacles, plane), first is obstacles plane (cloudOutliers) and second is road plane (cloudInliers)
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
}
//------------------END SegmentPlane by scratch-------------------
//--------------------------------------------------------------------------

//Clustering using PCL library pcl::EuclideanClusterExtraction
template <typename PointT>
//--------Clustering using the PCL library-----------------------
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
            cloud_cluster->points.push_back(cloud->points[index]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

//------------------Implemetation Clustering from scratch-------------------
//--------------------------------------------------------------------------
static void clusterProximity(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int idx : nearest) {
		if (!processed[idx]) {
			clusterProximity(idx, points, cluster, processed, tree, distanceTol);
		}
	}
}

static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;

	while (i < points.size()) {
		if (processed[i]) {
            i++;
            continue;
        }

		std::vector<int> cluster;
		clusterProximity(i, points, cluster, processed, tree, distanceTol);
        if (cluster.size() >= minSize && cluster.size() <= maxSize) {
            clusters.push_back(cluster);
        } else {
            for (int remove_index : cluster) {
                processed[remove_index] = false;
            }
        }
        i++;
	}

	return clusters;
}

template <typename PointT>
//--------Clustering from Scratch-----------------------
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringScratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
   // Time the clustering step
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; //list of clusters (P)
    std::vector<std::vector<float>> pts; //a queue if the point need to to check (Q)

    KdTree* tree = new KdTree; // initial kd-tree 
    // for each point (pt) inside the cloud dataset 
    for (int i = 0; i < cloud->points.size(); i++) {
        auto pt = cloud->points[i];
        pts.push_back(std::vector<float> {pt.x, pt.y, pt.z}); //add current point to queue (Q)
    	tree->insert(std::vector<float> {pt.x, pt.y, pt.z}, i); // insert this point to kd-tree
    }
    //inital and runing euclidean Cluster
    std::vector<std::vector<int>> clusterIndices = euclideanCluster(pts, tree, clusterTolerance, minSize, maxSize);
    // for each cluster Index (list of cluster)
    for (auto clusterIndex : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        // for each point inside the cluster
        for (auto pointIndex : clusterIndex) {
            cloudCluster->points.push_back (cloud->points[pointIndex]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
//------------------END Clustering from scratch-------------------
//--------------------------------------------------------------------------

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
