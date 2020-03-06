// -*-C++-*-
/*!
 * @file  pcProcessPortSVC_impl.cpp
 * @brief Service implementation code of pcProcessPort.idl
 *
 */

#include "pcProcessPortSVC_impl.h"
#include "pcProcessLib.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcProcessLib.h>

#include "cook_basis.h"           // my library
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "alignment.h"
#include <pcl/console/time.h>
#include <time.h>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/registration/ndt.h>
/*
 * Example implementational code for IDL interface ComPcProcess
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr ComPcProcessSVC_impl::scene (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ComPcProcessSVC_impl::scene_nobase_ (new pcl::PointCloud<pcl::PointXYZ>);
float ComPcProcessSVC_impl::height_;

ComPcProcessSVC_impl::ComPcProcessSVC_impl() {
    // Please add extra constructor code here.
}
ComPcProcessSVC_impl::ComPcProcessSVC_impl(
        RTC::PointCloud *m_p,
        RTC::InPort<RTC::PointCloud> *m_p_i,
        RTC::PointCloud *m_p2,
        RTC::InPort<RTC::PointCloud> *m_p_i2,
        RTC::PointCloud *m_p3,
        RTC::InPort<RTC::PointCloud> *m_p_i3,
        RTC::PointCloud *m_p4,
        RTC::InPort<RTC::PointCloud> *m_p_i4) {

    m_pointCloud_in		= m_p;
    m_pointCloud_inIn	= m_p_i;
    m_pointCloud_in2	= m_p2;
    m_pointCloud_in2In	= m_p_i2;
    m_pointCloud_in3	= m_p3;
    m_pointCloud_in3In	= m_p_i3;
    m_pointCloud_in4	= m_p4;
    m_pointCloud_in4In	= m_p_i4;

    // Please add extra constructor code here.
}

ComPcProcessSVC_impl::~ComPcProcessSVC_impl() {
    // Please add extra destructor code here.
}


ComPcProcess::MultiPointCloud_slice* ComPcProcessSVC_impl::get_pointCloud(::CORBA::Short CameraBitMask, ::CORBA::Boolean& flag)
{
    ComPcProcess::MultiPointCloud_slice* pc = new RTC::PointCloud[4];

    std::cout << "exe get_pointCloud" << std::endl;

    // check new point cloud
    if (m_pointCloud_inIn->isNew()) {
        // read new point cloud
        m_pointCloud_inIn->read();

        std::cout << "discover new data" << std::endl;

        flag = true;
        pc[0] = *m_pointCloud_in;

        return pc;
    }

    std::cout << "not discover new data" << std::endl;

    flag = false;
    return pc;
}


static std::vector<std::string> StringSplit(const std::string &str, char sep)
{
	std::vector<std::string> v;
	std::stringstream ss(str);
	std::string buffer;
	while( std::getline(ss, buffer, sep) )
		v.push_back(buffer);
	return v;
}

/**
* @brief Combine the std::string that in std::vector, and seprated by [seprator]. \n
* @param[in] std::vector<std::string> src		Mask of which stream you want to setup and enable.
* @param[in] char seprator						Seprator. Default is none.
**/
static std::string StringCombine(std::vector<std::string> src, char seprator = '/')
{
	std::string res = "";
	for(std::string tmp : src)
		res += tmp + seprator;
	return res.erase(res.size()-1,1);
}

static std::string ProcessPathString(std::string src_path, std::string ChannelStr)
{
	char seprator = '/';

	auto path_list = StringSplit(src_path, seprator);
	std::string FileName = path_list[path_list.size() - 1];

	FileName.insert(0, ChannelStr);

	path_list[path_list.size() - 1] = FileName;
	return StringCombine(path_list, seprator);
}

::CORBA::Boolean ComPcProcessSVC_impl::save_pointCloud(::CORBA::Short CameraBitMask, const char* str)
{
    std::cout << "exe save_pointCloud" << std::endl;
    bool fSuccess = false;
    if (m_pointCloud_inIn->isNew()) {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::string Path(str);

        std::cout << "Channel1 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel1_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in2In->isNew()) {
        // read new point cloud
        m_pointCloud_in2In->read();
        std::string Path(str);

        std::cout << "Channel2 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in2, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel2_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in3In->isNew()) {
        // read new point cloud
        m_pointCloud_in3In->read();
        std::string Path(str);

        std::cout << "Channel3 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in3, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel3_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in4In->isNew()) {
        // read new point cloud
        m_pointCloud_in4In->read();
        std::string Path(str);

        std::cout << "Channel4 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in4, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel4_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if(fSuccess)
        return true;
    else
    {
        std::cout << "not discover new data" << std::endl;
        return false;
    }
}
/**********************************************************/


// End of example implementational code

//::CORBA::Boolean ComPcProcessSVC_impl::plate_recog(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::plate_info information)
::CORBA::Long ComPcProcessSVC_impl::plate_recog(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::plate_info information, ::CORBA::Double& r_max)
{
    int alig_model_count=0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cloud = scene_nobase_;
        //start my program-----------------------------------------------------------------------------
        pcl::console::TicToc tt;
        std::cerr << "Loading...\n", tt.tic ();
         // Load the object templates specified in the object_templates.txt file
        std::vector<FeatureCloud> object_templates;
        InputStream(object_templates,"../data/data_nobase/object_templates.txt");

        for(int i = 0;i < 10;i++)  //0.65, 0, 0.2, 0, 0, 0
        {
          information[i][0] = 0.65;    // x
          information[i][1] = 0;       // y
          information[i][2] = 0;       // radius
          information[i][3] = 0.2;     // top2desk
          information[i][4] = 0;       // bottom2deskC
          information[i][5] = 0.2;     // bottom2desk
          information[i][6] = 0;       // bottomself
          information[i][7] = 0;       //angle
          information[i][8] = 0;       //target
          information[i][9] = 0;       // bottomselfC
        }

        Eigen::Matrix4f matrix_h2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_h2b(i,j)=matrix_in[i][j];
          }
        }

        //ShowCloud(cloud);
        WriteCloud(cloud,"original.pcd");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud,cloud_sor_voxel,50,1);
        //*cloud_sor_voxel = *cloud;
        *cloud_angle = *cloud_sor_voxel;
        std::vector<PlateInformation> infor_pre;
        std::vector<PlateInformation> infor_food;
        std::vector<PlateInformation> infor;
        infor.clear();
        infor_pre.clear();
        infor_food.clear();
        alig_model_count=AlignAllModels(cloud_sor_voxel,object_templates,0.000010, infor_pre, infor_food);
        //ShowCloud(cloud_sor_voxel);

        std::cout << "information of plates (without food) (ordered) : " << std::endl;
        for(unsigned int i=0;i< infor_pre.size()-1;i++)
        {
          for(unsigned int j=i+1;j<infor_pre.size();j++)
          {
            if(infor_pre[i].radius < infor_pre[j].radius)
            {
              Swap(infor_pre[i].x,infor_pre[j].x);
              Swap(infor_pre[i].y,infor_pre[j].y);
              Swap(infor_pre[i].radius,infor_pre[j].radius);
              //Swap(infor_pre[i].food,infor_pre[j].food);
              Swap(infor_pre[i].top2desk,infor_pre[j].top2desk);
              Swap(infor_pre[i].bottom2deskC,infor_pre[j].bottom2deskC);
              Swap(infor_pre[i].bottom2desk, infor_pre[j].bottom2desk);
              Swap(infor_pre[i].bottomself, infor_pre[j].bottomself);
              Swap(infor_pre[i].bottomselfC, infor_pre[j].bottomselfC);
            }
          }
        }

        for(unsigned int i =0;i<infor_pre.size();i++)
        {
            std::cout << infor_pre[i].x << " ; " ;
            std::cout << infor_pre[i].y << " ; " ;
            std::cout << infor_pre[i].radius << " ; " ;
            std::cout << infor_pre[i].top2desk << " ; " ;
            std::cout << infor_pre[i].bottom2deskC << " ; " ;
            std::cout << infor_pre[i].bottom2desk << " ; " ;
            std::cout << infor_pre[i].bottomself << " ; " ;
            std::cout << infor_pre[i].bottomselfC << " ; " ;
            std::cout << std::endl;
        }
        r_max = infor_pre[0].radius;

        std::cout << "information of plates with food inside (not ordered) :" << std::endl;
        for(unsigned int i = 0; i < infor_food.size(); i ++ )
        {
          std::cout << infor_food[i].x << " ; " ;
          std::cout << infor_food[i].y << " ; " ;
          std::cout << infor_food[i].radius << " ; " ;
          std::cout << infor_food[i].top2desk << " ; " << std::endl;
        }

        std::cout << "the information vector size before checking the order with angles (without food): " << infor.size() <<std::endl;
        CheckOrderWithAngle(cloud_angle,infor_pre,infor);
        GetSizeOrder(infor);
        GetSequence(infor);
        std::cout << "the information vector size after checking the order wiht angles : " << infor.size() << std::endl;
        std::cout<<"the count of models that have been aligned: " << alig_model_count << std::endl;
        std::cout<<"plate information:" << std::endl;
        for(unsigned int i=0;i<infor.size();i++)
        {
          information[i][0]=infor[i].x;
          information[i][1]=infor[i].y;
          information[i][2]=infor[i].radius;
          information[i][3]=infor[i].top2desk;
          information[i][4]=infor[i].bottom2deskC;
          information[i][5]=infor[i].bottom2desk;
          information[i][6]=infor[i].bottomself;
          information[i][7]=infor[i].angle;
          information[i][8]=infor[i].target;
          information[i][9]=infor[i].bottomselfC;
        }


        for(unsigned int i =0;i<infor.size();i++)
        {
          for(int j=0;j<10;j++)
          {
            std::cout << information[i][j] << "; ";
          }
          std::cout<<std::endl;
        }

        std::cerr << ">> Done: " << tt.toc () << " ms\n";
        return (infor.size());
}




::CORBA::Long ComPcProcessSVC_impl::plate_modelling(const ::ComPcProcess::Matrix4X4F matrix_in)
{
  while(1)
  {
    if (m_pointCloud_inIn->isNew())
    {
      // read new point cloud
        m_pointCloud_inIn->read();


        std::cout << "Channel1 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, *cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }

        // Build a passthrough filter to remove spurious NaNs
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        PrePassThrough(cloud,cloud_passthrough,0,0.4,"z");

        //PointP cloud_downsample (new PointC);
        DownSampleCloud(cloud_passthrough,cloud,0.002f); //0.003

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud,cloud_removed,50,0.3);
        cloud = cloud_removed;
        ShowCloud(cloud);

        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
        EstimateNormal(cloud,normal,50);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        SegmentPlane(cloud,normal,coefficients_plane,inliers_plane,0.016); // 0.02
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
        ExtractCloud(cloud,inliers_plane,cloud_nobase,true);
        RemoveOutlier(cloud_nobase,cloud,50,0.3);
        ShowCloud(cloud);

        int num_cluster = 0;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        num_cluster = ExtractClusters(cloud,clusters,0.005,60000,3000);

        if(num_cluster < 1)
        {
          std::cout << "No suitable model given!" << std::endl;
          return 0;
        }
        else
        {
          int num_model = 0;
          for(int i=0;i<num_cluster;i++)
          {
	          DownSampleCloud(clusters[i],clusters[i],0.003f);
	          std::vector<float> min_max;
	          FindMinMax(*clusters[i],min_max);
	          float x_diff,y_diff,z_diff;
	          x_diff = min_max[1] - min_max[0];
	          y_diff = min_max[3] - min_max[2];
	          z_diff = min_max[5] - min_max[4];
	          std::cout << "check the target" << std::endl;
	          if(x_diff<0.3 && abs(x_diff - y_diff)<0.02 && z_diff < 0.06)
	          {
              pcl::transformPointCloud (*clusters[i], *clusters[i], matrix_c2b);
		          ShowCloud(clusters[i]);

	          	pcl::PointCloud<pcl::PointXYZ>::Ptr projection (new pcl::PointCloud<pcl::PointXYZ>);
		          PCACloud(clusters[i],projection);

	          	int index=0;
		          float temp = 0;
		          for(unsigned int k = 0;k< projection->points.size();k++)
		          {
                temp = temp > projection->points[k].x ? temp : projection->points[k].x;
		            index = temp > projection->points[k].x ? index : k;
		          }
              std::cout << "max_x = " << temp << "  index = " << index << std::endl;
		          if(projection->points[index].z < 0)
		          {
		            TransformCloud(projection,projection, 0, 0, 0, "x", 3.14);
                std::cout << "transform (upside down)" << std::endl;
		          }

		          std::stringstream ss;
		          ss << "model_" << num_model << ".pcd";
	          	WriteCloud(projection,ss.str());
		          num_model++;
		          std::cout << "find a model" << std::endl;
            }
          }
          return num_model;
        }
    }
  }
}

::CORBA::Boolean ComPcProcessSVC_impl::single_plate_modelling(const ::ComPcProcess::Matrix4X4F matrix_in)
{
  while(1)
  {
    if (m_pointCloud_inIn->isNew())
    {
      // read new point cloud
        m_pointCloud_inIn->read();


        std::cout << "Channel1 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, *cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }
        WriteCloud(cloud,"model_original.pcd");
        // Build a passthrough filter to remove spurious NaNs
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        PrePassThrough(cloud,cloud_passthrough,0,0.5,"z");

        WriteCloud(cloud_passthrough,"model_downsample.pcd");

        //PointP cloud_downsample (new PointC);
        DownSampleCloud(cloud_passthrough,cloud,0.0025f); //0.003

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud,cloud_removed,50,1);
        cloud = cloud_removed;
        ShowCloud(cloud);

        int num_cluster;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        num_cluster = ExtractClusters(cloud,clusters,0.006,200000,1000);
        WriteCloud(clusters[0],"cluster0.pcd");
        WriteCloud(clusters[1],"cluster1.pcd");

        if(num_cluster <2)
        {
          return 0;   // not successfully seperated
        }

        float diff = 0.0;
        float xmin,xmax;
        int index_large_diff;
        for (int i=0;i<num_cluster;i++)
        {
          //ShowCloud(clusters[i]);
          FindXMinMax(clusters[i],xmin,xmax);
          if(xmax-xmin > diff)
          {
            diff = xmax-xmin;
            index_large_diff=i;
          }
        }
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin() + index_large_diff;

        cloud = clusters[index_large_diff];
        float h_table;
        h_table = MeanZ(cloud);

        clusters.erase(it);
        cloud = clusters[0];

        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
        EstimateNormal(cloud,normal,50);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        SegmentPlane(cloud,normal,coefficients_plane,inliers_plane,0.01); // 0.02
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plate_bottom (new pcl::PointCloud<pcl::PointXYZ>);
        ExtractCloud(cloud,inliers_plane,cloud_nobase,true);
        ExtractCloud(cloud,inliers_plane,plate_bottom,false);
        RemoveOutlier(cloud_nobase,cloud,50,1);
        WriteCloud(plate_bottom,"model_bottom.pcd");
        WriteCloud(cloud_nobase,"model_nobase.pcd");

        float p_bottom;
        p_bottom = MeanZ(plate_bottom);

        std::vector<float> min_max;
        FindMinMax(*cloud,min_max);
        float x_diff,y_diff,z_diff;
        x_diff = min_max[1] - min_max[0];
        y_diff = min_max[3] - min_max[2];
        z_diff = min_max[5] - min_max[4];

        std::cout << "check the target" << std::endl;
        if(x_diff<0.3 && abs(x_diff - y_diff)<0.02 && z_diff < 0.06)
        {
          pcl::transformPointCloud (*cloud, *cloud, matrix_c2b);
          //ShowCloud(cloud);
          WriteCloud(cloud,"model_robot_base.pcd");

          pcl::PointCloud<pcl::PointXYZ>::Ptr projection (new pcl::PointCloud<pcl::PointXYZ>);
          PCACloud(cloud,projection);

          int index = 0;
          float temp = 0;
          for(unsigned int k = 0;k< projection->points.size();k++)
          {
            if(temp < projection->points[k].x)
            {
              temp = projection->points[k].x;
              index = k;
            }
          }
          if(projection->points[index].z < 0)
          {
            TransformCloud(projection,projection, 0, 0, 0, "x", 3.1415926);
            std::cout << "transform (upside down)" << std::endl;
          }

          float z_min,z_max;
          FindZMinMax(projection,z_min,z_max);
          float delta;
          delta = h_table - p_bottom + fabs(z_min);   // the former two variables are relative to the camera, not the base
          std::cout << "delta : " << delta << "z_min : " << z_min << std::endl;
          TransformCloud(projection,projection,0,0,delta,"x",0);
          //ShowCloud(projection);
          WriteCloud(projection,"model_pca.pcd");

          float dist_adj = 0;
          float thre_adj = 0;
          int index_far = 0;
          for(unsigned int i = 0; i < projection->points.size(); i++)
          {
            dist_adj = pow(fabs(projection->points[i].x), 2.0) + pow(fabs(projection->points[i].y), 2.0);
            if(dist_adj > thre_adj)
            {
              thre_adj = dist_adj;
              index_far = i;
            }
          }
//          std::cout << "the furthest point is [ " << projection->points[index_far].x << " , " << projection->points[index_far].y << " ] . " << std::endl;
//          std::cout << "the furthest distance is : " << sqrt(thre_adj) << std::endl;
          float angle;
//          std::cout << "y is equal to : " << projection->points[index_far].y << std::endl;
          angle = asin(fabs(projection->points[index_far].y / sqrt(thre_adj)));
//          std::cout << "sin is equal to : " << fabs(projection->points[index_far].y / sqrt(thre_adj)) << std::endl;
//          std::cout << "angle is equal to : " << angle << std::endl;
          if(projection->points[index_far].x >= 0)
          {
            TransformCloud(projection,projection,0,0,0,"z",-angle);
          }
          else
          {
            TransformCloud(projection,projection,0,0,0,"z",-(pi-angle));
          }
          FindMinMax(*projection,min_max);
          std::cout << "min_x : " << min_max[0] << std::endl;
          std::cout << "max_x : " << min_max[1] << std::endl;

          ShowCloud(projection);
          WriteCloud(projection,"model.pcd");
          std::cout << "find a model" << std::endl;
          return 1;
        }
        return 0;
      }
    }
}


::CORBA::Double ComPcProcessSVC_impl::check_destination(::CORBA::Double r, ::CORBA::Double x, ::CORBA::Double y)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cloud = scene_nobase_;
        std::cout << " show the cloud for destination checking " << std::endl;
        //ShowCloud(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud,cloud_removed,50,1);
        //PrePassThrough(cloud_removed,cloud_removed,x-1.2*r,x+1.2*r,"x");
        //PrePassThrough(cloud_removed,cloud_removed,y-1.2*r,y+1.2*r,"y");
        //ShowCloud(cloud);
        ExtractCircle(cloud_removed,cloud_removed,x,y,r,true);
        std::cout << " show the circle for destination checking" << std::endl;
        //ShowCloud(cloud_removed);

        int num_cluster = 0;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        num_cluster = ExtractClusters(cloud_removed,clusters,0.01,60000,100);

        if(num_cluster >0)
        {
          std::cout << "This place has been occupied! (condition1)" << std::endl;
          return 0;
        }
        else
        {
          std::cout << "This place has not been occupied." << std::endl;
          return 1;
        }
}

::CORBA::Double ComPcProcessSVC_impl::height_determine(::CORBA::Double r)
{
    double width_p;
    double theta;
    width_p = sqrt(r * r - 0.044 * 0.044);
    theta = asin((width_p - 0.00325) / 0.174);
    double height_h;
    height_h = 0.13 * cos(theta) + 0.02 * sin(theta + 3/180 * pi);
    std::cout << "width_p = " << width_p << std::endl;
    std::cout << "theta = " << theta << std::endl;
    std::cout << "height_h = " << height_h << std::endl;
    return height_h;
}

::CORBA::Double ComPcProcessSVC_impl::add_PointCloud(const ::ComPcProcess::Matrix4X4F matrix_in)
{
  while(1)
  {
    if (m_pointCloud_inIn->isNew())
    {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::cout << "Channel1 discover new data" << std::endl;
        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, *cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        PrePassThrough(cloud,cloud_passthrough,0,matrix_c2b(2,3),"z");
        pcl::transformPointCloud (*cloud_passthrough, *cloud, matrix_c2b);

        const float voxel_grid_size = 0.0026;//0.0026  //0.003
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
        DownSampleCloud(cloud,tempCloud,voxel_grid_size);

        // Remove Outliers from the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(tempCloud,cloud_sor_voxel,50,1);
        //cloud_sor_voxel = tempCloud;

        if(cloud_sor_voxel->points.size()>0)
        {
          WriteCloud(cloud_sor_voxel,"cloud2_original.pcd");
        }

        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
        EstimateNormal(cloud_sor_voxel,normal,50);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        SegmentPlane(cloud_sor_voxel,normal,coefficients_plane,inliers_plane,0.015);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
        ExtractCloud(cloud_sor_voxel,inliers_plane,cloud_nobase,true);
        ExtractCloud(cloud_sor_voxel,inliers_plane,plane,false);

        float height;
        height = MeanZ(plane);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after (new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud_nobase,cloud_after,50,1);
        //cloud_after = cloud_nobase;
        if(cloud_after->points.size()>0)
        {
          WriteCloud(cloud_after,"scene_toalign2.pcd");
        }

        if(scene_nobase_->points.size() <100)
        {
           *scene_nobase_ += *cloud_after;
           if(scene_nobase_->points.size()>0)
           {
             WriteCloud(scene_nobase_,"scene_nobase_2.pcd");
           }
           return height;
        }
        DeleteRedundency(scene_nobase_,cloud_after);
        *scene_nobase_ += *cloud_after;
        if(scene_nobase_->points.size()>0)
        {
          WriteCloud(scene_nobase_,"scene_nobase_2.pcd");
        }
        return height;

    }
  }
}


::CORBA::Double ComPcProcessSVC_impl::get_FirstCloud(const ::ComPcProcess::Matrix4X4F matrix_in)
{
  while(1)
  {
    if (m_pointCloud_inIn->isNew())
    {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::cout << "Channel1 discover new data" << std::endl;
        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, *cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        PrePassThrough(cloud,cloud_passthrough,0,matrix_c2b(2,3),"z");
        pcl::transformPointCloud (*cloud_passthrough, *cloud, matrix_c2b);

        const float voxel_grid_size = 0.0026;//0.0026  //0.003
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
        DownSampleCloud(cloud,tempCloud,voxel_grid_size);

        // Remove Outliers from the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(tempCloud,cloud_sor_voxel,50,1);
        //cloud_sor_voxel = tempCloud;
        if(cloud_sor_voxel->points.size()>0)
        {
          WriteCloud(cloud_sor_voxel,"cloud1_original.pcd");
        }

        pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
        EstimateNormal(cloud_sor_voxel,normal,50);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        SegmentPlane(cloud_sor_voxel,normal,coefficients_plane,inliers_plane,0.015);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nobase (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
        ExtractCloud(cloud_sor_voxel,inliers_plane,cloud_nobase,true);
        ExtractCloud(cloud_sor_voxel,inliers_plane,plane,false);

        float height;
        height = MeanZ(plane);
        height_ = height;
        std::cout << "height of the desktop is : " << height_ << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after (new pcl::PointCloud<pcl::PointXYZ>);
        RemoveOutlier(cloud_nobase,cloud_after,50,1);
        //cloud_after = cloud_nobase;
        if(cloud_after->points.size()>0)
        {
          WriteCloud(cloud_after,"scene_toalign_1.pcd");
        }

        *scene_nobase_ += *cloud_after;
        //WriteCloud(scene_nobase_,"scene_nobase_1.pcd");
        return height;
    }
  }
}
