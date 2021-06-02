#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"

#include <pcl/filters/voxel_grid.h>

#include "exercise6/Detection.h"

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;

ros::Publisher pub_detection;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointXYZRGB; //! wtf is up with that anyways??


void
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();

  pcl::PassThrough<PointXYZRGB> pass;
  pcl::NormalEstimation<PointXYZRGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointXYZRGB> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Downsample
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Read in the cloud data
  //pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.2, 0.2); //0, 1.5
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0, 4);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 3); //0, 1.5
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  /*
  pcl::PCLPointCloud2 outcloud_filtered;
  pcl::toPCLPointCloud2 (*cloud_filtered, outcloud_filtered);
  pubx.publish (outcloud_filtered);
  */

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);


  /*
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<PointXYZRGB> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2 (*cloud_plane, outcloud_plane);
  pubx.publish (outcloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);
  */

  cloud_filtered2 = cloud_filtered;
  cloud_normals2 = cloud_normals;

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.2); //0.1 0.3
  seg.setMaxIterations (5000);
  seg.setDistanceThreshold (0.005); //0.05 0.015
  seg.setRadiusLimits (0.105, 0.135); //0.06 0.2 0.10 0.15
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;


  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZRGB> ());
  extract.filter (*cloud_cylinder);
  int NUM_POINTS_THR = 180; //40 10
  if (cloud_cylinder->points.empty() || cloud_cylinder->points.size() < NUM_POINTS_THR){
    std::cerr << "Can't find the cylindrical component." <<g std::endl;
    //printf("         no cylinder          \n");
  }
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    //printf("         DETECT!!!!!!!          \n");

          pcl::compute3DCentroid (*cloud_cylinder, centroid);
          std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << std::endl;

	  //Create a point in the "camera_rgb_optical_frame"
          geometry_msgs::PointStamped point_camera;
          geometry_msgs::PointStamped point_map;
	      visualization_msgs::Marker marker;
          geometry_msgs::TransformStamped tss;

          point_camera.header.frame_id = "camera_rgb_optical_frame";
          point_camera.header.stamp = ros::Time::now();

	  	  point_map.header.frame_id = "map";
          point_map.header.stamp = ros::Time::now();

		  point_camera.point.x = centroid[0];
		  point_camera.point.y = centroid[1];
		  point_camera.point.z = centroid[2];



	  try{
		  time_test = ros::Time::now();

		  std::cerr << time_rec << std::endl;
		  std::cerr << time_test << std::endl;
  	      tss = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec);
          //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
	  }
          catch (tf2::TransformException &ex)
	  {
	       //ROS_WARN("Transform warning: %s\n", ex.what());
	  }

          //std::cerr << tss ;

          tf2::doTransform(point_camera, point_map, tss);

	      std::cerr << "point_camera: " << point_camera.point.x << " " <<  point_camera.point.y << " " <<  point_camera.point.z << std::endl;

	      std::cerr << "point_map: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;

	  	  marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();

          marker.ns = "cylinder";
          marker.id = 0;

          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = point_map.point.x;
          marker.pose.position.y = point_map.point.y;
          marker.pose.position.z = point_map.point.z;
          marker.pose.orientation.x = 0.0;
	      marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 0.1;
	      marker.scale.y = 0.1;
	      marker.scale.z = 0.1;

          marker.color.r=0.0f;
          marker.color.g=1.0f;
          marker.color.b=0.0f;
          marker.color.a=1.0f;

	      marker.lifetime = ros::Duration();

        printf("Points in detection: %ld\n", cloud_cylinder->points.size() );
        ROS_INFO("Points in detection: %ld\n", cloud_cylinder->points.size() );

	      pubm.publish (marker);

        /*
	      pcl::PCLPointCloud2 outcloud_cylinder;
          pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud_cylinder);
          puby.publish (outcloud_cylinder);
        */



        //  CREATE HISTOGRAM FROM POINT CLOUD SEGMENT
        int bins = 10; //bins per channel
        int mod = 5; //only use every mod-th pixel (for performance)
        int hist[30] = {}; //histogram
        int h = 16;
        int w = 16;

        int n = cloud_cylinder->points.size();

        int l = n/mod + 1;

        //std::vector<uint8_t> red;
        //std::vector<uint8_t> green;
        //std::vector<uint8_t> blue;

        std::vector<uint8_t> red(l,0);
        std::vector<uint8_t> green(l,0);
        std::vector<uint8_t> blue(l,0);

        int j = 0;
        for (int i = 0; i < n; i += mod){
          int r = cloud_cylinder->points[i].r;
          int g = cloud_cylinder->points[i].g;
          int b = cloud_cylinder->points[i].b;

          //red.push_back(r);
          //green.push_back(g);
          //blue.push_back(b);

          red[j] = r;
          green[j] = g;
          blue[j] = b;
          j += 1;
        }


        /*
        std::uint32_t* array = new std::uint32_t[n];
        for (int i = 0; i < n; i += mod){
          std::uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
          array[i] = rgb;
        }

        cv::Mat image = cv::Mat(w, h, CV_8UC3, (unsigned*)array );
        */

        exercise6::Detection detection;
        detection.position.x = point_map.point.x;
        detection.position.y = point_map.point.y;
        detection.position.z = point_map.point.z;
        detection.num_detections = n;

        detection.red = red;
        detection.green = green;
        detection.blue = blue;

        pub_detection.publish(detection);


        // -------------------------------------------

  }

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2> ("cylinder", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder",1);

  pub_detection = nh.advertise<exercise6::Detection>("detected_cylinder_info", 1);

  // Spin
  ros::spin ();
}
