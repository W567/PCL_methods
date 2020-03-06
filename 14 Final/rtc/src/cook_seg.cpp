#include "cook_seg.h"

/******************************************************************************
Function : SegmentPlane
Description : segment a plane out from the target cloud.
Input : target cloud, threshold for the thickness of the plane.
Output : coefficients of the plane and indices of the points in the cloud of the plane.
******************************************************************************/
void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane,float threshold)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2000);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
}

/******************************************************************************
Function : SegmentCircle3D
Description : segment a circle out from the target cloud.
Input : target cloud, r_min and r_max for circle detection and threshold as the width of the circle.
Output : coefficients of the circle and indices of points in the cloud of the circle.
******************************************************************************/
void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max,float threshold)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (threshold);
  seg.setRadiusLimits (r_min,r_max);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the circle inliers and coefficients
  seg.segment (*inliers_circle, *coefficients_circle);
  std::cerr << "Circle coefficients: " << *coefficients_circle << std::endl;
}


/******************************************************************************
Function : ExtractClusters
Description : Extract the clusters out from the target cloud and store them in PCD files.
Input : target cloud, tolerance for the distance between points, max and min of the size of clusters.
Output : PCD files of each cluster.
******************************************************************************/
int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float tolerance,int max_size,int min_size)
{
  if(cloud_in->points.size() == 0)
  {
    std::cout << "no points in the cloud! " << std::endl;
    return 0;
  }
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  return j;
}

/******************************************************************************
Function : ExtractClusters
Description : Extract clusters out from the target cloud and store them in a vector.
Input : target cloud, tolerance for distance between points, min and max of the size of clusters.
Output : vector of clusters.
******************************************************************************/
int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,float tolerance,int max_size,int min_size)
{
  if(cloud_in->points.size() == 0)
  {
    std::cout << "no points in the cloud ! " << std::endl;
    return 0;
  }
  clusters.clear();
  std::cout << "start clusters extracting" << std::endl;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    clusters.push_back(cloud_cluster);
    j++;
  }
  std::cout << "cluster extracting finished, totally " << j << " clusters" << std::endl;
  return j;
}



/******************************************************************************
Function : RadiusGrouping
Description : extract/delete the points inside of a ball out.
Input : target cloud, centroid of the ball, radius and state.
Output : new cloud.
Others :
        state:
            if true :
                  delete the points inside of the ball.
               false :
                  extract the points inside of the ball.
******************************************************************************/
void RadiusGrouping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float x,float y,float z,float radius,bool state)
{
  pcl::PointXYZ searchPoint;
  searchPoint.x = x;
  searchPoint.y = y;
  searchPoint.z = z;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_in);
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  if(state == true)  // extract the rest part
  {
    if(kdtree.radiusSearch(searchPoint,radius,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
      inliers->indices = pointIdxNKNSearch;
      ExtractCloud(cloud_in,inliers,cloud_out,state);  //true: rest   false: inliers
    }
    else
    {
      cloud_out = cloud_in;
      std::cout << "no points inside of the target area!" << std::endl;
    }
  }
  else  // extract the inner part
  {
    if(kdtree.radiusSearch(searchPoint,radius,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
      inliers->indices = pointIdxNKNSearch;
      ExtractCloud(cloud_in,inliers,cloud_out,state);  //true: rest   false: inliers
    }
    else
    {
      cloud_out->width = 0;
      cloud_out->height = 1;
      cloud_out->is_dense = true;
      std::cout << "no points inside of the target area!"<<std::endl;
    }
  }
}
