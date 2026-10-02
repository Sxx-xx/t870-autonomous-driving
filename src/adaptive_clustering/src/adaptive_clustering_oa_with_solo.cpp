// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "adaptive_clustering/ClusterArray.h"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <string>
#include <algorithm>
#include <math.h>


#include "vehicle_msgs/Track.h"
#include "vehicle_msgs/TrackCone.h"

#include "vehicle_msgs/ObjectInfo.h"
#include <std_msgs/Bool.h>

using namespace std;

double roiTheta(double x, double y);

//#define LOG

ros::Publisher cluster_array_pub_;
ros::Publisher cloud_filtered_pub_; // cluster
ros::Publisher pose_array_pub_;   // pose
ros::Publisher marker_array_pub_; // bounding box
ros::Publisher cloud_center_pub_; // center
ros::Publisher nearest_one_pub_; // nearest one
ros::Publisher nearest_two_pub_; // nearest one
ros::Publisher track_array_pub_; // rubber cone track array

ros::Publisher object_info_pub_;
ros::Publisher box_drawn_pub_;
/////////////////////////////////////////
geometry_msgs::Point p[24];
geometry_msgs::Point q[24];


bool print_fps_;
float z_axis_min_;
float z_axis_max_;
int cluster_size_min_;
int cluster_size_max_;

float min_x = -0.4; // -0.4
float max_x = 5.5;    // 4.5
float min_y = -2.75;  // -2.25
float max_y = 2.25;  // 2.25   //////////////////////// kdh
float min_z = -0.6; //-0.65
float max_z = 0.0;   // 0float min_z = -0.65; //-0.825


// Default ROI Angle
float min_angle = 0; //-20    //////////////////////////// kdh
float max_angle = 180.0; //160


// Track Mission
int isTrackMission = 1;
float w = 0.0;  // 
float h = 0.0;
float box_width = 0.0;
float box_height = 0.0;

const int region_max_ = 10; // Change this value to match how far you want to detect.
int regions_[100];

int frames; clock_t start_time; bool reset = true;//fps

int count0 = 0;

double roiTheta(double x, double y) {
  double r;
  double theta;
  
  r = sqrt(pow(x, 2) + pow(y, 2));
  theta = acos(x/r) * 180 / 3.141592;
  
  return theta;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in) {   // velodyne input VLP::ros_pc2_in
  bool box_was_drawn = false;
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps  

  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);  // make PCL::pcl_pc_in
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);           // VLP::ros_pc2_in >> PCL::pcl_pc_in
  
  // // /*** Voxelization ***/

  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_v2 (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (pcl_pc_in);
  vg.setLeafSize (0.125f, 0.125f, 0.125f);    /// 0.125
  vg.filter (*pcl_pc_in);

  /*** Remove ground ***/
  pcl::IndicesPtr pc_indices(new std::vector<int>);         // make vec::pc_indices

  pcl::PassThrough<pcl::PointXYZI> pt;              // filter::ROI
  
  count0 = count0 + 1;
  // std::cout << "count0 : " << count0 << std::endl;
  // std::cout << "ROI value" << std::endl;
  // std::cout << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z << std::endl;
  pt.setInputCloud(pcl_pc_in);                // - input - PCL::pcl_pc_in
  pt.setFilterFieldName("z");
  pt.setFilterLimits(min_z, max_z);
  pt.filter(*pcl_pc_in);

  pt.setInputCloud(pcl_pc_in);
  pt.setFilterFieldName("x");
  pt.setFilterLimits(min_x, max_x);
  pt.filter(*pcl_pc_in);

  pt.setInputCloud(pcl_pc_in);
  pt.setFilterFieldName("y");
  pt.setFilterLimits(min_y, max_y);
  pt.filter(*pc_indices);     
  
      // - output - vec::pc_indices
  

  float angle = 0;
  for (unsigned int m = 0; m < pc_indices->size(); m++) {
    angle = roiTheta(pcl_pc_in->points[(*pc_indices)[m]].y, pcl_pc_in->points[(*pc_indices)[m]].x);
    // std::cout << "min_angle : " << min_angle << ", max_angle : " << max_angle << std::endl;
    if (angle < min_angle || angle > max_angle) {
      // std::cout << "angle < 45 or angle > 135, x : " << pcl_pc_in->points[(*pc_indices)[m]].x << ", y : " << pcl_pc_in->points[(*pc_indices)[m]].y << std::endl;
      pcl_pc_in->points[(*pc_indices)[m]].x = 0;
      pcl_pc_in->points[(*pc_indices)[m]].y = 0;
      pcl_pc_in->points[(*pc_indices)[m]].z = 0;
    }

  }

  /*** Divide the point cloud into nested circular regions ***/
  boost::array<std::vector<int>, region_max_> indices_array;        // make vec::indices_array
  // std::cout << "pc_indices.size() : " << pc_indices->size() << std::endl;

  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < region_max_; j++) {
      float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
  pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
  pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
      //std::cout << pcl_pc_in->points[(*pc_indices)[i]].intensity << std::endl;
      if(d2 > range * range && d2 <= (range+regions_[j]) * (range+regions_[j])) {
        indices_array[j].push_back((*pc_indices)[i]);         // push vec::pc_indices >> vec::indices_array
        break;
      }
      range += regions_[j];
    }
  }
  
  /*** Euclidean clustering ***/
  float tolerance = 0.3;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;
                      // make vec::clusters

  //int j = 0;
  for(int i = 0; i < region_max_; i++) {
    tolerance += 0.1;
    if(indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i])); // make vec::indices_array_ptr
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>); // make PCL::tree
      tree->setInputCloud(pcl_pc_in, indices_array_ptr);        // - input - PCL::pcl_pc_in & vec::indices_array_ptr
      
      std::vector<pcl::PointIndices> cluster_indices;         // make vec::cluster_indices

      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;       // filter::euclidean
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pcl_pc_in);              // - input - PCL::pcl_pc_in
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);              // - output - vec::cluster_indices
      
      //std::cout << "The number of clusters is " << cluster_indices.size() << std::endl;
      
      int j = 0;
      int count1 = 1;
      int count2 = 1;
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);  // make PCL::cluster
        count1 = 1;
        //std::cout << "count : " << count1 << std::endl;
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
          pcl::PointXYZI pt = pcl_pc_in->points[*pit];
          pcl::PointXYZI pt2;
          pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
          pt2.intensity = (float)(j + 1); 
          pt2.intensity = pt.intensity;
          // std::cout << pt.intensity << std::endl;
          // std::cout << "count2 : " << count2 << ", intensity : " << pt2.intensity << std::endl;
          count2++;
          cluster->points.push_back(pcl_pc_in->points[*pit]);       // push PCL::pcl_pc_in >> PCL::cluster
        }
        count1++;

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);              // push PCL::cluster >> vec::clusters
        //j++;
      }
    }
  }



  std::cout << "The number of clusters is " << clusters.size () << std::endl;

  
  /*** Output ***/
  if(cloud_filtered_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>); // make PCL::pcl_pc_out
    sensor_msgs::PointCloud2 ros_pc2_out;             // make msgs::ros_pc2_out
    pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);      // copy PCL::pcl_pc_in & vec::pc_indices >> PCL::pcl_pc_out
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);            // PCL::pcl_pc_out >> msgs::ros_pc2_out
    cloud_filtered_pub_.publish(ros_pc2_out);           // - publish - msgs::ros_pc2_out
  }
  
  adaptive_clustering::ClusterArray cluster_array;          // make msgs::cluster_array
  geometry_msgs::PoseArray pose_array;              // make msgs::pose_array
  visualization_msgs::MarkerArray marker_array;           // make msgs::marker_array
  visualization_msgs::MarkerArray center_array;           // make msgs::center_array
  visualization_msgs::MarkerArray one_array;              // make msgs::one_array
  visualization_msgs::MarkerArray two_array;              // make msgs::one_array
 
  vehicle_msgs::Track track_array; 


  
  float min_dist = 0.0, abs_x = 0.0, abs_y = 0.0, abs_dist = 0.0; 

  vehicle_msgs::ObjectInfo object_info;
  object_info.header = ros_pc2_in->header;
  object_info.objectCounts = 0;


  if(clusters.size() > 0){
 
  float dist_array[clusters.size()][2] = {};
  int nearest1 = 0, nearest2 = 0;


  

  for(int i = 0; i < clusters.size(); i++) {
    Eigen::Vector4f centroid;
    Eigen::Vector4f min_box, max_box;
    pcl::compute3DCentroid(*clusters[i], centroid);
    pcl::getMinMax3D(*clusters[i], min_box, max_box);

    float box_width = max_box[1] - min_box[1];
    float box_height = max_box[2] - min_box[2];

    object_info.centerX.push_back(centroid[0]);
    object_info.centerY.push_back(centroid[1]);
    object_info.lengthX.push_back(box_width);
    object_info.lengthY.push_back(box_height);
    object_info.objectCounts++;

    if(cluster_array_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*clusters[i], ros_pc2_out);
      cluster_array.clusters.push_back(ros_pc2_out);
    }

    if(track_array_pub_.getNumSubscribers() > 0) {
      std::cout << "**** publish : track_array_pub" << std::endl;
      
      vehicle_msgs::TrackCone cone;
      cone.x = centroid[0];
      cone.y = centroid[1];
      
      std::cout << "cone.x: " << cone.x << ", cone.y: " << cone.y << std::endl;

      if (isTrackMission == 1) {
        if(((0.10 <= box_width)&&(box_width <= 1.3)) && ((0.10<= box_height)&&(box_height <= 1.3))){  //0.6->3 hjh
          std::cout << "box width : " << box_width << ", box height : " << box_height << std::endl;
          track_array.cones.push_back(cone);
        } else {
          std::cout << "track mission but not rubber cone size" << std::endl;
        }
      } else if (isTrackMission == 0) {
        std::cout << "not track mission" << std::endl;
        track_array.cones.push_back(cone);
      }

      std::cout << "cone.x : " << cone.x << std::endl;
      std::cout << "cone.y : " << cone.y << std::endl;
      // std::cout << "cone.z : " << cone.z << std::endl;
    }

    if(pose_array_pub_.getNumSubscribers() > 0) {         // **** publish : pose_array_pub
      // std::cout << "**** publish : pose_array_pub"<< std::endl;        
      Eigen::Vector4f centroid;               // make Eigen::centroid
      pcl::compute3DCentroid(*clusters[i], centroid);         // compute vec::clusters >> Eigen::centroid
      
      geometry_msgs::Pose pose;               // make msgs::pose
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);           // push msgs::pose >> msgs::pose_array
      // std::cout << "pose.position.x : " << pose.position.x << std::endl;
      // std::cout << "pose.position.y : " << pose.position.y << std::endl;
      // std::cout << "pose.position.z : " << pose.position.z << std::endl;
    
      
#ifdef LOG
      Eigen::Vector4f min, max;               // make Eigen::min&max
      pcl::getMinMax3D(*clusters[i], min, max);           // get vec::clusters >> Eigen::min&max
      std::cerr << ros_pc2_in->header.seq << " "
    << ros_pc2_in->header.stamp << " "
    << min[0] << " "
    << min[1] << " "
    << min[2] << " "
    << max[0] << " "
    << max[1] << " "
    << max[2] << " "
    << std::endl;
#endif
    }
    
    if(marker_array_pub_.getNumSubscribers() > 0) {         // **** publish : marker_array_pub
      Eigen::Vector4f min, max;               // make Eigen::min&max
      pcl::getMinMax3D(*clusters[i], min, max);           // get vec::clusters >> Eigen::min&max
      
      visualization_msgs::Marker marker;            // make msgs::marker
      marker.header = ros_pc2_in->header;
      marker.ns = "adaptive_clustering";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;        // - marker - type - LINE_LIST
      
      // std::cout << "max[0] : " << max[0] << ", max[1] : " << max[1] << ", max[2] : " << max[2] << std::endl;
      // std::cout << "min[0] : " << min[0] << ", min[1] : " << min[1] << ", min[2] : " << min[2] << std::endl;
      
      geometry_msgs::Point p[24];             // make msgs::p[i]
      p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
      p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
      p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
      p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
      p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
      p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
      p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
      p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
      p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
      p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];

      w = max[1] - min[1];  // width. calculate with y coordinate
      h = max[2] - min[2];  // height. calculate with z coordinate
//1.5    //3.0
      if (isTrackMission == 1) {
        if(((0.10 <= w)&&(w <= 1.3)) && ((0.10 <= h)&&(h <=1.3))) {              
          box_was_drawn = true;
          for(int i = 0; i < 24; i++) {
            marker.points.push_back(p[i]);              // push msgs::p[i] >> msgs::marker
          }
          
          std::cout << "box width : " << w << ", box height : " << h << std::endl;
          std::cout << "max[0] : " << max[0] << ", max[1] : " << max[1] << ", max[2] : " << max[2] << std::endl;
          std::cout << "min[0] : " << min[0] << ", min[1] : " << min[1] << ", min[2] : " << min[2] << std::endl;
        }
        else {
          std::cout << "track mission but not rubber cone size" << std::endl;
        }
      }
      
      else if (isTrackMission == 0) {
        std::cout << "not track mission" << std::endl;
        marker.points.push_back(p[i]);
      }
      
      marker.scale.x = 0.02;                // - marker - style

      marker.color.a = 1;
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;

      marker.lifetime = ros::Duration(0.1);           // - marker - reset
      marker_array.markers.push_back(marker);           // push msgs::marker >> msgs::marker_array
    }

    if(cloud_center_pub_.getNumSubscribers() > 0) {         // **** publish : cloud_center_pub_
      Eigen::Vector4f center;               // make Eigen::center
      pcl::compute3DCentroid(*clusters[i], center);         // compute vec::clusters >> Eigen::center

      visualization_msgs::Marker marker_c;            // make msgs::marker_c
      marker_c.header = ros_pc2_in->header;
      marker_c.ns = "adaptive_clustering";
      marker_c.id = i;

      marker_c.type = visualization_msgs::Marker::LINE_LIST;        // - marker - type - LINE_LIST

      geometry_msgs::Point p[24];             // make msgs::p[i]
      p[0].x = center[0]-0.05;  p[0].y = center[1];  p[0].z = center[2];
      p[1].x = center[0];  p[1].y = center[1];  p[1].z = center[2];
      p[2].x = center[0]+0.05;  p[2].y = center[1];  p[2].z = center[2];
      p[3].x = center[0];  p[3].y = center[1]-0.05;  p[3].z = center[2];
      p[4].x = center[0];  p[4].y = center[1];  p[4].z = center[2];
      p[5].x = center[0];  p[5].y = center[1]+0.05;  p[5].z = center[2];
      p[6].x = center[0];  p[6].y = center[1];  p[6].z = center[2]-0.05;
      p[7].x = center[0];  p[7].y = center[1];  p[7].z = center[2];
      p[8].x = center[0];  p[8].y = center[1];  p[8].z = center[2]+0.05;
      p[9].x = center[0];  p[9].y = center[1];  p[9].z = center[2];
      p[10].x = center[0]; p[10].y = center[1]; p[10].z = center[2];
      p[11].x = center[0]; p[11].y = center[1]; p[11].z = center[2];
      p[12].x = center[0]; p[12].y = center[1]; p[12].z = center[2];
      p[13].x = center[0]; p[13].y = center[1]; p[13].z = center[2];
      p[14].x = center[0]; p[14].y = center[1]; p[14].z = center[2];
      p[15].x = center[0]; p[15].y = center[1]; p[15].z = center[2];
      p[16].x = center[0]; p[16].y = center[1]; p[16].z = center[2];
      p[17].x = center[0]; p[17].y = center[1]; p[17].z = center[2];
      p[18].x = center[0]; p[18].y = center[1]; p[18].z = center[2];
      p[19].x = center[0]; p[19].y = center[1]; p[19].z = center[2];
      p[20].x = center[0]; p[20].y = center[1]; p[20].z = center[2];
      p[21].x = center[0]; p[21].y = center[1]; p[21].z = center[2];
      p[22].x = center[0]; p[22].y = center[1]; p[22].z = center[2];
      p[23].x = center[0]; p[23].y = center[1]; p[23].z = center[2];

      for(int i = 0; i < 24; i++) {
        marker_c.points.push_back(p[i]);            // push msgs::p[i] >> msgs::marker
      }
      
      marker_c.scale.x = 0.01;                // - marker - style

      marker_c.color.a = 1;
      marker_c.color.r = 255;
      marker_c.color.g = 1;
      marker_c.color.b = 1;

      marker_c.lifetime = ros::Duration(10);            // - marker - reset
      center_array.markers.push_back(marker_c);           // push msgs::marker >> msgs::marker_array

    }
  }

  if(nearest_one_pub_.getNumSubscribers() > 0 || nearest_two_pub_.getNumSubscribers() > 0 ) {                                               // **** publish : nearest_one_pub_
    for(int i = 0; i < clusters.size(); i++) {

      Eigen::Vector4f distance;               // make Eigen::distance
      pcl::compute3DCentroid(*clusters[i], distance);         // compute vec::clusters >> Eigen::distance
      
      abs_x = (distance[0] < 0) ? -distance[0] : distance[0];
      abs_y = (distance[1] < 0) ? -distance[1] : distance[1];
    
      abs_dist = sqrt(pow(abs_x,2) + pow(abs_y,2));

      dist_array[i][0] = abs_dist;
      dist_array[i][1] = i;
      std::cout << "abs_x : " << abs_x << ", abs_y : "<< abs_y << ", abs_dist : " << abs_dist << std::endl;
    }
    
    for (int row = 0; row < clusters.size(); row++) {
    for (int col = 0; col < 2; col++) {
      std::cout << "dist_array[" << row << "][" << col << "] : " << dist_array[row][col] << std::endl;
      }
    }
  
    int i = 0, j = 0, key_index = 0;
    float key = 0.0;

      for(i=1; i < clusters.size(); i++){
          key = dist_array[i][0];
          key_index = dist_array[i][1]; 
          for(j=i-1; j>=0; j--){
              if(dist_array[j][0] > key){      
                  dist_array[j+1][0] = dist_array[j][0];
                  dist_array[j+1][1] = dist_array[j][1];
                  dist_array[j][1] = key_index;  
              }else{                  
                  break;
              }
          }
          dist_array[j+1][0] = key;            
      }
      
    for (int row = 0; row < clusters.size(); row++) {
    for (int col = 0; col < 2; col++) {
      std::cout << "dist_array[" << row << "][" << col << "] : " << dist_array[row][col] << std::endl;
    }
    }

    std::cout << "The index of nearest center is " << dist_array[0][1] << ", " << dist_array[1][1] << std::endl;
    
    nearest1 = dist_array[0][1];
    nearest2 = dist_array[1][1];

    Eigen::Vector4f center1;                // make Eigen::centroid
    pcl::compute3DCentroid(*clusters[nearest1], center1); 
    std::cout << "The nearest1 center : " << center1[0] << ", " << center1[1] << std::endl;
    
    Eigen::Vector4f center2;                // make Eigen::centroid
    pcl::compute3DCentroid(*clusters[nearest2], center2); 
    std::cout << "The nearest2 center : " << center2[0] << ", " << center2[1] << std::endl;

    Eigen::Vector4f min1, max1;               // make Eigen::min&max1
    pcl::getMinMax3D(*clusters[nearest1], min1, max1);            // get vec::clusters >> Eigen::min&max1

    Eigen::Vector4f min2, max2;               // make Eigen::min&max1
    pcl::getMinMax3D(*clusters[nearest2], min2, max2);            // get vec::clusters >> Eigen::min&max1
    
    visualization_msgs::Marker marker_one;            // make msgs::marker_one
    visualization_msgs::Marker marker_two;            // make msgs::marker_one

    for(int i = 0; i < clusters.size(); i++) {
      
      marker_one.header = ros_pc2_in->header;
      marker_one.ns = "adaptive_clustering";
      marker_one.id = i;
      marker_one.type = visualization_msgs::Marker::LINE_LIST;        // - marker - type - LINE_LIST

      marker_two.header = ros_pc2_in->header;
      marker_two.ns = "adaptive_clustering";
      marker_two.id = i;
      marker_two.type = visualization_msgs::Marker::LINE_LIST;        // - marker - type - LINE_LIST

      geometry_msgs::Point p[24];             // make msgs::p[i]
      p[0].x = max1[0];  p[0].y = max1[1];  p[0].z = max1[2];
      p[1].x = min1[0];  p[1].y = max1[1];  p[1].z = max1[2];
      p[2].x = max1[0];  p[2].y = max1[1];  p[2].z = max1[2];
      p[3].x = max1[0];  p[3].y = min1[1];  p[3].z = max1[2];
      p[4].x = max1[0];  p[4].y = max1[1];  p[4].z = max1[2];
      p[5].x = max1[0];  p[5].y = max1[1];  p[5].z = min1[2];
      p[6].x = min1[0];  p[6].y = min1[1];  p[6].z = min1[2];
      p[7].x = max1[0];  p[7].y = min1[1];  p[7].z = min1[2];
      p[8].x = min1[0];  p[8].y = min1[1];  p[8].z = min1[2];
      p[9].x = min1[0];  p[9].y = max1[1];  p[9].z = min1[2];
      p[10].x = min1[0]; p[10].y = min1[1]; p[10].z = min1[2];
      p[11].x = min1[0]; p[11].y = min1[1]; p[11].z = max1[2];
      p[12].x = min1[0]; p[12].y = max1[1]; p[12].z = max1[2];
      p[13].x = min1[0]; p[13].y = max1[1]; p[13].z = min1[2];
      p[14].x = min1[0]; p[14].y = max1[1]; p[14].z = max1[2];
      p[15].x = min1[0]; p[15].y = min1[1]; p[15].z = max1[2];
      p[16].x = max1[0]; p[16].y = min1[1]; p[16].z = max1[2];
      p[17].x = max1[0]; p[17].y = min1[1]; p[17].z = min1[2];
      p[18].x = max1[0]; p[18].y = min1[1]; p[18].z = max1[2];
      p[19].x = min1[0]; p[19].y = min1[1]; p[19].z = max1[2];
      p[20].x = max1[0]; p[20].y = max1[1]; p[20].z = min1[2];
      p[21].x = min1[0]; p[21].y = max1[1]; p[21].z = min1[2];
      p[22].x = max1[0]; p[22].y = max1[1]; p[22].z = min1[2];
      p[23].x = max1[0]; p[23].y = min1[1]; p[23].z = min1[2];

      geometry_msgs::Point q[24];             // make msgs::q[i]
      q[0].x = max2[0];  q[0].y = max2[1];  q[0].z = max2[2];
      q[1].x = min2[0];  q[1].y = max2[1];  q[1].z = max2[2];
      q[2].x = max2[0];  q[2].y = max2[1];  q[2].z = max2[2];
      q[3].x = max2[0];  q[3].y = min2[1];  q[3].z = max2[2];
      q[4].x = max2[0];  q[4].y = max2[1];  q[4].z = max2[2];
      q[5].x = max2[0];  q[5].y = max2[1];  q[5].z = min2[2];
      q[6].x = min2[0];  q[6].y = min2[1];  q[6].z = min2[2];
      q[7].x = max2[0];  q[7].y = min2[1];  q[7].z = min2[2];
      q[8].x = min2[0];  q[8].y = min2[1];  q[8].z = min2[2];
      q[9].x = min2[0];  q[9].y = max2[1];  q[9].z = min2[2];
      q[10].x = min2[0]; q[10].y = min2[1]; q[10].z = min2[2];
      q[11].x = min2[0]; q[11].y = min2[1]; q[11].z = max2[2];
      q[12].x = min2[0]; q[12].y = max2[1]; q[12].z = max2[2];
      q[13].x = min2[0]; q[13].y = max2[1]; q[13].z = min2[2];
      q[14].x = min2[0]; q[14].y = max2[1]; q[14].z = max2[2];
      q[15].x = min2[0]; q[15].y = min2[1]; q[15].z = max2[2];
      q[16].x = max2[0]; q[16].y = min2[1]; q[16].z = max2[2];
      q[17].x = max2[0]; q[17].y = min2[1]; q[17].z = min2[2];
      q[18].x = max2[0]; q[18].y = min2[1]; q[18].z = max2[2];
      q[19].x = min2[0]; q[19].y = min2[1]; q[19].z = max2[2];
      q[20].x = max2[0]; q[20].y = max2[1]; q[20].z = min2[2];
      q[21].x = min2[0]; q[21].y = max2[1]; q[21].z = min2[2];
      q[22].x = max2[0]; q[22].y = max2[1]; q[22].z = min2[2];
      q[23].x = max2[0]; q[23].y = min2[1]; q[23].z = min2[2];

      for(int i = 0; i < 24; i++) {
        marker_one.points.push_back(p[i]);            // push msgs::p[i] >> msgs::marker
        marker_two.points.push_back(q[i]);
      }
    }
    
    marker_one.scale.x = 0.03;                // - marker - style

    marker_one.color.a = 1;
    marker_one.color.r = 1;
    marker_one.color.g = 255;
    marker_one.color.b = 255;

    marker_one.lifetime = ros::Duration(0.1);           // - marker - reset
    one_array.markers.push_back(marker_one);            // push msgs::marker >> msgs::marker_array

    marker_two.scale.x = 0.03;                // - marker - style

    marker_two.color.a = 1;
    marker_two.color.r = 1;
    marker_two.color.g = 1;
    marker_two.color.b = 255;

    marker_two.lifetime = ros::Duration(0.1);           // - marker - reset
    two_array.markers.push_back(marker_two);            // push msgs::marker >> msgs::marker_array

  }
  }
  else{
      geometry_msgs::Pose pose;               // make msgs::pose
      pose.position.x = 100;
      pose.position.y = 100;
      pose.position.z = 0;
      pose.orientation.w = 0;
  }

  if(object_info_pub_.getNumSubscribers() > 0) {
    object_info_pub_.publish(object_info);
  }
  
  
  if(cluster_array.clusters.size()) {
    cluster_array.header = ros_pc2_in->header;
    cluster_array_pub_.publish(cluster_array);
  }

  if(pose_array.poses.size()) {
    pose_array.header = ros_pc2_in->header;
    pose_array_pub_.publish(pose_array);
  }
  
  if(track_array.cones.size()) {
    track_array_pub_.publish(track_array);
  }

  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }

  if(center_array.markers.size()) {
    cloud_center_pub_.publish(center_array);
  }

  if(one_array.markers.size()) {
    nearest_one_pub_.publish(one_array);
  }
  
  if(two_array.markers.size()) {
    nearest_two_pub_.publish(two_array);
  }
  if(print_fps_)if(++frames>10){std::cerr<<"[adaptive_clustering] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps

  std_msgs::Bool box_drawn_msg;
  box_drawn_msg.data = box_was_drawn;
  box_drawn_pub_.publish(box_drawn_msg);
}

int main(int argc, char **argv) {
  // Command line when using custom ROI  : "rosrun adaptive_clustering adaptive_clustering input:=/velodyne_points ROI 0 8 -4 4 -0.95 0"
  // Command line when using default ROI : "rosrun adaptive_clustering adaptive_clustering input:=/velodyne_points DEFAULT"
  ros::init(argc, argv, "adaptive_clustering");
  // std::cout << argc << std::endl;  // the number of arguments
  // std::cout << argv[1] << std::endl; 

  if (argc == 2 && strcmp(argv[1], "DEFAULT") == 0) {
    std::cout << "default ROI" << std::endl;
  }
  else if (argc > 2 && strcmp(argv[1], "ROI") == 0) {
    std::cout << "custom ROI" << std::endl;
    min_x = stof(argv[2]);
    max_x = stof(argv[3]);
    min_y = stof(argv[4]);
    max_y = stof(argv[5]);
    min_z = stof(argv[6]);
    max_z = stof(argv[7]);
    min_angle = stof(argv[8]);
    max_angle = stof(argv[9]);
  }

  if (argc == 11 && stoi(argv[10]) == 1) {  // track mission(rubber cone clustering)
    std::cout << "argc : " << argc << ", argv[10] : " << argv[10] << std::endl; 
    isTrackMission = 1;
  } 

  /*** Subscribers ***/
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hesai/pandar", 1, pointCloudCallback);



  /*** Publishers ***/
  ros::NodeHandle private_nh("~");
  cluster_array_pub_ = private_nh.advertise<adaptive_clustering::ClusterArray>("clusters", 100);
  cloud_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
  pose_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
  cloud_center_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers_center", 100);
  nearest_one_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("nearest_one", 100);
  nearest_two_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("nearest_two", 100);
  track_array_pub_ = nh.advertise<vehicle_msgs::Track>("track", 100);
  object_info_pub_ = nh.advertise<vehicle_msgs::ObjectInfo>("/object_info", 100);
  box_drawn_pub_ = nh.advertise<std_msgs::Bool>("/adaptive_clustering/box_drawn", 1);
  
  /*** Parameters ***/
  std::string sensor_model;

  private_nh.param<std::string>("sensor_model", sensor_model, "PandarXT-16"); // VLP-16, HDL-32E, HDL-64E
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<float>("z_axis_min", z_axis_min_, -0.8);
  private_nh.param<float>("z_axis_max", z_axis_max_, 2.0);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 3);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 2200000);
  
  // Divide the point cloud into nested circular regions centred at the sensor.
  
  if(sensor_model.compare("VLP-16") == 0) {
    regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
    regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
    regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
  } else if (sensor_model.compare("PandarXT-16") == 0) {
    regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
    regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
    regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
  } else if (sensor_model.compare("HDL-32E") == 0) {
    regions_[0] = 4; regions_[1] = 5; regions_[2] = 4; regions_[3] = 5; regions_[4] = 4;
    regions_[5] = 5; regions_[6] = 5; regions_[7] = 4; regions_[8] = 5; regions_[9] = 4;

    regions_[10]= 5; regions_[11]= 5; regions_[12]= 4; regions_[13]= 5;
  } else if (sensor_model.compare("HDL-64E") == 0) {
    regions_[0] = 14; regions_[1] = 14; regions_[2] = 14; regions_[3] = 15; regions_[4] = 14;
  } else {
    ROS_FATAL("Unknown sensor model!");
  }
  
  ros::spin();

  return 0;
}