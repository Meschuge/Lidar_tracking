// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr region_cloud {new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> grid;

    grid.setInputCloud(cloud);
    grid.setLeafSize(filterRes, filterRes, filterRes);
    grid.filter(*filtered_cloud);

    pcl::CropBox<PointT> work_area {true};
    work_area.setInputCloud(filtered_cloud);
    work_area.setMax(maxPoint);
    work_area.setMin(minPoint);
    work_area.filter(*region_cloud);

    pcl::PointIndices::Ptr roof_indices {new pcl::PointIndices};
    std::vector<int> roof_points;
    pcl::CropBox<PointT> top {true};
    top.setInputCloud(region_cloud);
    top.setMax(Eigen::Vector4f(2.7, 1.6, -0.2, 1));
    top.setMin(Eigen::Vector4f(-1.5, -1.6, -1.0, 1));
    top.filter(roof_points);

    for(auto ind : roof_points)
        roof_indices->indices.push_back(ind);

    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(region_cloud);
    ext.setIndices(roof_indices);
    ext.setNegative(true);
    ext.filter(*region_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacle {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr plane {new pcl::PointCloud<PointT>()};

    for(int index {0} ; index < cloud->points.size(); ++index) {
        if (inliers.count(index)) {
            plane->points.push_back(cloud->points[index]);
        } else {
            obstacle->points.push_back(cloud->points[index]);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> tmp_inliers;
    srand(time(NULL));

    auto startTime = std::chrono::steady_clock::now();

    while(maxIterations--) {
        while(tmp_inliers.size() < 3){
            tmp_inliers.insert(rand() % cloud->points.size());
        }

        auto itr {tmp_inliers.begin()};
        PointT p1 {cloud->points[*itr]};
        PointT p2 {cloud->points[*(++itr)]};
        PointT p3 {cloud->points[*(++itr)]};

        float A {((p2.y - p1.y)*(p3.z - p1.z))-((p2.z-p1.z)*(p3.y -p1.y))};
        float B {((p2.z - p1.z )*(p3.x - p1.x))-((p2.x-p1.x)*(p3.z -p1.z))};
        float C {((p2.x - p1.x)*(p3.y - p1.y))-((p2.y-p1.y)*(p3.x -p1.x))};
        float D {A*p1.x + B*p1.y + C*p1.z};
        D = D - (2*D);

        for(int j {0}; j < cloud->points.size(); ++j){
            if(tmp_inliers.count(j) > 0)
                continue;

            PointT p {cloud->points[j]};
            float distance {std::fabs((A * p.x) + (B * p.y) + (C* p.z) + D) / std::sqrt((A*A)+(B*B) + (C*C))};
            if(distance <= distanceThreshold){
                tmp_inliers.insert(j);
            }
        }

        if(tmp_inliers.size() > inliersResult.size()){
            inliersResult = tmp_inliers;
        }

        tmp_inliers.clear();

    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}

template<typename PointT>
void proximity(const typename pcl::PointCloud<PointT>::Ptr cloud, int index, std::vector<int> &cluster, std::vector<bool> &processed, Tree<PointT>* tree, float &distanceTol){
    cluster.push_back(index);
    processed[index] = true;
    auto nearby = tree->search(cloud->points[index], distanceTol);
    for(auto i : nearby){
        if(!processed[i]){
            proximity(cloud, i, cluster, processed, tree, distanceTol);
        }
    }

}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, Tree<PointT>* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    auto cloud_size {cloud->points.size()};
    std::vector<bool> processedPoints(cloud_size, false);

    for(int j {0}; j < cloud_size; ++j){
        if(!processedPoints[j]){
            std::vector<int> cluster;
            ::proximity(cloud, j, cluster, processedPoints, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    ::Tree<PointT> *tree = new ::Tree<PointT>();

    for(int i {0}; i < cloud->points.size(); ++i){
        tree->insert(cloud->points[i], i);
    }

    std::vector<std::vector<int>> cluster_indices { ::euclideanCluster(cloud, tree, clusterTolerance) };

    for(auto ind {cluster_indices.begin()}; ind != cluster_indices.end(); ++ind){
        typename pcl::PointCloud<PointT>::Ptr cld {new pcl::PointCloud<PointT>};
        for(auto p {ind->begin()}; p != ind->end(); ++p){
            cld->points.push_back(cloud->points[*p]);
        }
        if(cld->points.size() <= maxSize && cld->points.size() >= minSize)
            clusters.push_back(cld);
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