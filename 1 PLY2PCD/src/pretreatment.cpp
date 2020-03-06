#include "pretreatment.h"

void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDReader reader;
  reader.read (name,*cloud_in);
  std::cerr << "PointCloud has: " << cloud_in->points.size () << " data points." << std::endl;
}

void WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (name, *cloud_in);
}

void PrePassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float min,float max,std::string axis)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName (axis); //'x' 'y' 'z'
  pass.setFilterLimits (min,max);
  pass.filter(*cloud_out);
  std::cerr << "PointCloud after filtering(passthrough) has: " << cloud_out->points.size() << " data points." << std::endl;
}

void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.setLeafSize(size,size,size); //0.003f
  voxelGrid.filter(*cloud_out);
  std::cerr << "PointCloud after downsampling has: " << cloud_out->points.size() << " data points." << std::endl;
}

void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int meanK, float thresh)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setMeanK(meanK);  //50
  sor.setStddevMulThresh(thresh); //0.1
  sor.filter(*cloud_out);
  std::cerr << "cloud after StatisticalOutlierRemoval has: " << cloud_out->points.size() << std::endl;
}

void EstimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_out, int rangeK)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setKSearch (rangeK);
  ne.compute (*normal_out);
}

void ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,int coor_size,int R,int G,int B)
{
  if (cloud_in->points.empty())
    std::cerr << " No points exist." << std::endl;
  else
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Vierwer"));
    viewer->setBackgroundColor(R,G,B);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_in,"result");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"result");
    viewer->addCoordinateSystem(coor_size);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
}

void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.025);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
}

void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.01);
  seg.setRadiusLimits (r_min,r_max);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the circle inliers and coefficients
  seg.segment (*inliers_circle, *coefficients_circle);
  std::cerr << "Circle coefficients: " << *coefficients_circle << std::endl;
}

void ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointIndices::Ptr inliers_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,bool state)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers_in);
  extract.setNegative (state);   //true: rest   false: inliers
  extract.filter (*cloud_out);
}

void ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,std::vector<pcl::PointIndices> cluster_indices,float tolerance,int max_size,int min_size)
{
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
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
}

void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float x,float y,float z,std::string axis,float angle)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform.translation() << x,y,z; //0.033, -0.075, -0.45;

  //The same rotation matrix as before; theta radians around Z axis
  if(axis == "x")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitX()));}
  else if(axis == "y")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitY()));}
  else if(axis == "z")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitZ()));}
  else {std::cout << "input an illegal parameter" << std::endl;}

  // Print the transformation
  printf ("\nusing an Affine3f\n");
  std::cout << transform.matrix() << std::endl;

  pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
}
