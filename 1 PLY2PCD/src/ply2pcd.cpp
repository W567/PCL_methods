#include "pretreatment.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>


int loadCloud(int argc,char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

// This is the main function
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  loadCloud(argc,argv,cloud);

  ShowCloud(cloud,0.1);
  WriteCloud(cloud,"output.pcd");

  return 0;
}

int
loadCloud(int argc,char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if( argc < 2)
    {
        printf("No target Cloud files given!\n");
    }

    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

    if (filenames.size () != 1)  {
      filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

      if (filenames.size () != 1) {
        printf("input error!\n");
        return -1;
      } else {
        file_is_pcd = true;
      }
    }

    if (file_is_pcd) {
      if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    } else {
      if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    }
    return 0;
}
