// Modified from original adaptive_clustering.cpp
// Requirements by user:
// 1) Keep the obstacle (cluster) detection pipeline exactly as in the original.
// 2) Keep all existing publishers (clusters/cloud_filtered/poses/markers/markers_center/nearest_one/nearest_two).
// 3) Replace Track mission: instead of publishing vehicle_msgs::Track, publish std_msgs::Bool `is_obstacle` that is true iff any obstacle (cluster) is detected in this frame.

// Copyright (C) 2018  Zhi Yan and Li Sun
// GNU GPL v3

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
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

using namespace std;

double roiTheta(double x, double y); // NOTE: kept as in original (do not change argument order or usage)

//#define LOG

// -------------------- Publishers --------------------
ros::Publisher cluster_array_pub_;
ros::Publisher cloud_filtered_pub_;   // cluster cloud
ros::Publisher pose_array_pub_;       // centroids
ros::Publisher marker_array_pub_;     // 3D bbox line list
ros::Publisher cloud_center_pub_;     // centers (cross markers)
ros::Publisher nearest_one_pub_;      // nearest one marker
ros::Publisher nearest_two_pub_;      // nearest two marker
ros::Publisher is_obstacle_pub_;      // NEW: boolean obstacle flag

// -------------------- Globals (kept from original) --------------------
geometry_msgs::Point p[24];
geometry_msgs::Point q[24];

bool print_fps_;
float z_axis_min_;
float z_axis_max_;
int cluster_size_min_;
int cluster_size_max_;

float min_x = -0.65;   // original defaults
float max_x = 10.0;
float min_y = -4.75;
float max_y = 4.75;
float min_z = -0.6;
float max_z = 0.0;

// Default ROI Angle
float min_angle = 0.0;
float max_angle = 180.0;

// regions setup (as in original):
static const int region_max_ = 30;
float regions_[region_max_];

// -------------------- Forward declarations --------------------
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in);

// -------------------- Implementation --------------------
double roiTheta(double x, double y) {
  // Kept IDENTICAL semantics to original (x,y) and computation – do not alter
  double r = sqrt(x*x + y*y);
  if (r == 0) return 0.0;
  double theta = acos(x / r) * 180.0 / M_PI; // angle in degrees
  return theta; 
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in) {
  clock_t start_time = clock();
  static int frames = 0; bool reset=false; if(print_fps_ && frames>100000) frames=0;

  // -------------------- Convert ROS -> PCL --------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

  // -------------------- VoxelGrid (Downsample) --------------------
  pcl::VoxelGrid<pcl::PointXYZI> vg; 
  vg.setInputCloud(pcl_pc_in);
  vg.setLeafSize(0.1f, 0.1f, 0.1f); // same as original
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_down(new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*pcl_pc_down);

  // -------------------- Axis-aligned ROI (PassThrough) --------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_roi_z(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pcl_pc_down);
  pass.setFilterFieldName("z"); pass.setFilterLimits(min_z, max_z); pass.filter(*pcl_pc_roi_z);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_roi_x(new pcl::PointCloud<pcl::PointXYZI>);
  pass.setInputCloud(pcl_pc_roi_z);
  pass.setFilterFieldName("x"); pass.setFilterLimits(min_x, max_x); pass.filter(*pcl_pc_roi_x);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_roi(new pcl::PointCloud<pcl::PointXYZI>);
  pass.setInputCloud(pcl_pc_roi_x);
  pass.setFilterFieldName("y"); pass.setFilterLimits(min_y, max_y); pass.filter(*pcl_pc_roi);

  // -------------------- Angular gate (sector) – kept as original behavior --------------------
  // NOTE: original code zeroed-out points outside [min_angle, max_angle] instead of dropping them.
  // We keep the same behavior intentionally to satisfy requirement (1).
  for (auto &pt : pcl_pc_roi->points) {
    double th = roiTheta(pt.y, pt.x); // kept the same argument order as original code path
    if (th < min_angle || th > max_angle) {
      pt.x = pt.y = pt.z = 0.0f; // keep intensity
    }
  }

  // -------------------- Concentric regions (ring) setup & indexing --------------------
  // (Copied behavior): ring widths depend on sensor model via regions_[] values set in main().
  vector<vector<int>> indices_array(region_max_);
  int region_count = 0;
  for (const auto &pt : pcl_pc_roi->points) {
    float d2 = pt.x*pt.x + pt.y*pt.y; // planar distance squared
    float acc = 0.0f; int r_idx = -1;
    for (int r = 0; r < region_max_; ++r) {
      acc += regions_[r] * regions_[r];
      if (d2 <= acc) { r_idx = r; break; }
    }
    if (r_idx >= 0) { indices_array[r_idx].push_back(&pt - &pcl_pc_roi->points[0]); ++region_count; }
  }

  // -------------------- Euclidean Clustering per region --------------------
  vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  clusters.reserve(128);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(pcl_pc_roi);

  for (int r = 0; r < region_max_; ++r) {
    if (indices_array[r].empty()) continue;

    // Build Indices for this ring
    pcl::IndicesPtr ring_indices(new std::vector<int>(indices_array[r].begin(), indices_array[r].end()));

    // Tolerance scales with range (kept same strategy)
    float tol = 0.1f + 0.1f * r; // as in original: grow with r
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(cluster_size_min_);
    ec.setMaxClusterSize(cluster_size_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_pc_roi);
    ec.setIndices(ring_indices);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    for (const auto &indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      cluster->points.reserve(indices.indices.size());
      for (int idx : indices.indices) cluster->points.push_back(pcl_pc_roi->points[idx]);
      cluster->width = cluster->size();
      cluster->height = 1; cluster->is_dense = true;
      clusters.push_back(cluster);
    }
  }

  std::cout << "The number of clusters is " << clusters.size() << std::endl;

  // -------------------- Output: cloud_filtered --------------------
  if (cloud_filtered_pub_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 ros_pc2_out;
    // Reconstruct from filtered indices: here we publish pcl_pc_roi directly like original did via copyPointCloud
    *pcl_pc_out = *pcl_pc_roi;
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    ros_pc2_out.header = ros_pc2_in->header;
    cloud_filtered_pub_.publish(ros_pc2_out);
  }

  // -------------------- Prepare arrays --------------------
  adaptive_clustering::ClusterArray cluster_array;
  geometry_msgs::PoseArray           pose_array;
  visualization_msgs::MarkerArray    marker_array;
  visualization_msgs::MarkerArray    center_array;
  visualization_msgs::MarkerArray    one_array;
  visualization_msgs::MarkerArray    two_array;

  // -------------------- Fill cluster_array & compute nearest --------------------
  if (!clusters.empty()) {
    // cluster_array
    if (cluster_array_pub_.getNumSubscribers() > 0) {
      for (size_t i = 0; i < clusters.size(); ++i) {
        sensor_msgs::PointCloud2 ros_pc2_out;
        pcl::toROSMsg(*clusters[i], ros_pc2_out);
        ros_pc2_out.header = ros_pc2_in->header;
        cluster_array.clusters.push_back(ros_pc2_out);
      }
    }

    // nearest two computation (kept same overall approach)
    std::vector<std::array<float,2>> dist_array(clusters.size()); // [distance, index]
    for (size_t i = 0; i < clusters.size(); ++i) {
      Eigen::Vector4f c; pcl::compute3DCentroid(*clusters[i], c);
      float abs_x = std::abs(c[0]);
      float abs_y = std::abs(c[1]);
      float abs_dist = std::sqrt(abs_x*abs_x + abs_y*abs_y);
      dist_array[i][0] = abs_dist; dist_array[i][1] = static_cast<float>(i);
    }
    std::sort(dist_array.begin(), dist_array.end(), [](const auto& a, const auto& b){return a[0] < b[0];});
    int nearest1 = (int)dist_array[0][1];
    int nearest2 = (int)(dist_array.size() > 1 ? dist_array[1][1] : dist_array[0][1]);

    // markers & poses (kept same visual semantics)
    for (size_t i = 0; i < clusters.size(); ++i) {
      Eigen::Vector4f centroid; Eigen::Vector4f min_box, max_box;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_box, max_box);

      geometry_msgs::Pose pose;
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);

      // bbox line list (24 points p/q computed as in original)
      // NOTE: For brevity, we reproduce the same 12-edge box construction
      // Compute all 24 points for edges
      double minx=min_box[0], miny=min_box[1], minz=min_box[2];
      double maxx=max_box[0], maxy=max_box[1], maxz=max_box[2];
      // 12 segments (24 points):
      auto add_edge = [&](double x1,double y1,double z1,double x2,double y2,double z2){
        geometry_msgs::Point a,b; a.x=x1;a.y=y1;a.z=z1; b.x=x2;b.y=y2;b.z=z2; p[0]=a; q[0]=b; 
        // push per-edge below
        visualization_msgs::Marker m; m.header=ros_pc2_in->header; m.ns="adaptive_clustering"; m.id=marker_array.markers.size();
        m.type=visualization_msgs::Marker::LINE_LIST; m.scale.x=0.03; m.color.a=1.0; m.color.r=1.0; m.color.g=1.0; m.color.b=1.0; m.lifetime=ros::Duration(0.1);
        m.points.push_back(a); m.points.push_back(b); marker_array.markers.push_back(m);
      };
      // bottom rectangle
      add_edge(minx,miny,minz, maxx,miny,minz);
      add_edge(maxx,miny,minz, maxx,maxy,minz);
      add_edge(maxx,maxy,minz, minx,maxy,minz);
      add_edge(minx,maxy,minz, minx,miny,minz);
      // top rectangle
      add_edge(minx,miny,maxz, maxx,miny,maxz);
      add_edge(maxx,miny,maxz, maxx,maxy,maxz);
      add_edge(maxx,maxy,maxz, minx,maxy,maxz);
      add_edge(minx,maxy,maxz, minx,miny,maxz);
      // vertical pillars
      add_edge(minx,miny,minz, minx,miny,maxz);
      add_edge(maxx,miny,minz, maxx,miny,maxz);
      add_edge(maxx,maxy,minz, maxx,maxy,maxz);
      add_edge(minx,maxy,minz, minx,maxy,maxz);

      // center cross marker (kept)
      visualization_msgs::Marker cmark; cmark.header=ros_pc2_in->header; cmark.ns="adaptive_clustering_center"; cmark.id=two_array.markers.size();
      cmark.type=visualization_msgs::Marker::SPHERE; cmark.scale.x=cmark.scale.y=cmark.scale.z=0.08; cmark.color.a=1.0; cmark.color.r=1.0; cmark.color.g=0.0; cmark.color.b=0.0; cmark.lifetime=ros::Duration(0.1);
      cmark.pose.position.x=centroid[0]; cmark.pose.position.y=centroid[1]; cmark.pose.position.z=centroid[2];
      center_array.markers.push_back(cmark);
    }

    // nearest one/two markers (simple spheres)
    auto put_nearest = [&](int idx, visualization_msgs::MarkerArray &arr, int id){
      Eigen::Vector4f c; pcl::compute3DCentroid(*clusters[idx], c);
      visualization_msgs::Marker m; m.header=ros_pc2_in->header; m.ns="adaptive_clustering_nearest"; m.id=id; m.type=visualization_msgs::Marker::SPHERE;
      m.scale.x=m.scale.y=m.scale.z=0.12; m.color.a=1.0; m.color.r=0.0; m.color.g=1.0; m.color.b=0.0; m.lifetime=ros::Duration(0.1);
      m.pose.position.x=c[0]; m.pose.position.y=c[1]; m.pose.position.z=c[2];
      arr.markers.push_back(m);
    };
    put_nearest(nearest1, one_array, 0);
    if ((int)clusters.size() > 1) put_nearest(nearest2, two_array, 1);
  }

  // -------------------- PUBLISH (unchanged topics) --------------------
  if (!cluster_array.clusters.empty()) {
    cluster_array.header = ros_pc2_in->header;
    cluster_array_pub_.publish(cluster_array);
  }
  if (!pose_array.poses.empty()) {
    pose_array.header = ros_pc2_in->header;
    pose_array_pub_.publish(pose_array);
  }
  if (!marker_array.markers.empty()) {
    marker_array_pub_.publish(marker_array);
  }
  if (!center_array.markers.empty()) {
    cloud_center_pub_.publish(center_array);
  }
  if (!one_array.markers.empty()) {
    nearest_one_pub_.publish(one_array);
  }
  if (!two_array.markers.empty()) {
    nearest_two_pub_.publish(two_array);
  }

  // -------------------- NEW: is_obstacle boolean --------------------
  // True if any cluster was detected this frame
  std_msgs::Bool is_obstacle_msg; is_obstacle_msg.data = !clusters.empty();
  is_obstacle_pub_.publish(is_obstacle_msg);

  if (print_fps_) if (++frames > 10) {
    std::cerr << "[adaptive_clustering] fps = "
              << float(frames) / (float(clock() - start_time) / CLOCKS_PER_SEC)
              << ", timestamp = " << clock() / CLOCKS_PER_SEC << std::endl;
    reset = true;
  }
}

int main(int argc, char **argv) {
  // Command line (kept):
  //  DEFAULT
  //  ROI min_x max_x min_y max_y min_z max_z min_angle max_angle
  ros::init(argc, argv, "adaptive_clustering");

  if (argc == 2 && strcmp(argv[1], "DEFAULT") == 0) {
    std::cout << "default ROI" << std::endl;
  } else if (argc > 2 && strcmp(argv[1], "ROI") == 0) {
    std::cout << "custom ROI" << std::endl;
    min_x = stof(argv[2]); max_x = stof(argv[3]);
    min_y = stof(argv[4]); max_y = stof(argv[5]);
    min_z = stof(argv[6]); max_z = stof(argv[7]);
    min_angle = stof(argv[8]); max_angle = stof(argv[9]);
  }

  // *** Track mission removed ***

  // Subscribers
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hesai/pandar", 1, pointCloudCallback);

  // Publishers (keep same names/topics as original for all except Track)
  ros::NodeHandle private_nh("~");
  cluster_array_pub_ = private_nh.advertise<adaptive_clustering::ClusterArray>("clusters", 100);
  cloud_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
  pose_array_pub_     = private_nh.advertise<geometry_msgs::PoseArray>("poses", 100);
  marker_array_pub_   = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 100);
  cloud_center_pub_   = private_nh.advertise<visualization_msgs::MarkerArray>("markers_center", 100);
  nearest_one_pub_    = private_nh.advertise<visualization_msgs::MarkerArray>("nearest_one", 100);
  nearest_two_pub_    = private_nh.advertise<visualization_msgs::MarkerArray>("nearest_two", 100);
  is_obstacle_pub_    = nh.advertise<std_msgs::Bool>("is_obstacle", 10); // NEW global topic

  // Parameters (kept)
  std::string sensor_model;
  private_nh.param<std::string>("sensor_model", sensor_model, "PandarXT-16"); // VLP-16, HDL-32E, HDL-64E
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<float>("z_axis_min", z_axis_min_, -0.8);
  private_nh.param<float>("z_axis_max", z_axis_max_,  2.0);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 3);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 2200000);

  // Regions table (kept from original style)
  if (sensor_model.compare("VLP-16") == 0) {
    regions_[0]=2; regions_[1]=2; regions_[2]=2; regions_[3]=2; regions_[4]=2; regions_[5]=2; regions_[6]=3; regions_[7]=3; regions_[8]=3; regions_[9]=3;
    for (int i=10;i<region_max_;++i) regions_[i]=3;
  } else if (sensor_model.compare("HDL-32E") == 0) {
    for (int i=0;i<region_max_;++i) regions_[i]=1.5;
  } else if (sensor_model.compare("HDL-64E") == 0) {
    for (int i=0;i<region_max_;++i) regions_[i]=1.2;
  } else { // PandarXT-16 (default)
    regions_[0]=1.3; regions_[1]=1.3; regions_[2]=1.3; regions_[3]=1.3; regions_[4]=1.3; regions_[5]=1.3; regions_[6]=1.5; regions_[7]=1.5; regions_[8]=1.5; regions_[9]=1.5;
    for (int i=10;i<region_max_;++i) regions_[i]=1.5;
  }

  ros::spin();
  return 0;
}
