#include "voxel_mapping/tsdf.h"
#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/map_server.h"
namespace voxel_mapping {
void TSDF::inputPointCloudfromother(Transformation T_w_c,const vector<int> &points,int id,
  vector<float> values,vector<float> weights){
  if(points.empty()||(points.size()!=values.size()||points.size()!=weights.size())) return ;
  std::cout<<"sfcafsafaf1"<<id<<std::endl;
  Position sensor_position = T_w_c.getPosition();
  Rotation sensor_orientation = T_w_c.getRotation();

  // Initialize the update bounding box
  Position update_min = sensor_position;
  Position update_max = sensor_position;

  Position point_w, point_c, voxel_pos, raycast_start, raycast_end;
  VoxelIndex voxel_idx;
  VoxelAddress voxel_addr;
  FloatingPoint value, weight;
  for (int i=0;i<points.size();i++) {
      updateTSDFVoxel(points[i], values[i], weights[i]);
      if (occupancy_grid_->config_.mode_ == OccupancyGrid::Config::MODE::GEN_TSDF) {
        occupancy_grid_->updateOccupancyVoxel(points[i]);
      }
  
  }

  VoxelIndex update_min_idx = positionToIndex(update_min);
  VoxelIndex update_max_idx = positionToIndex(update_max);
  boundIndex(update_min_idx);
  boundIndex(update_max_idx);
  //esdf_->updateLocalESDF(update_min_idx, update_max_idx);

  // Bounding box for frontier update
  std::lock_guard<std::mutex> lock(bbox_mutex_);

  if (reset_updated_bbox_) {
    map_data_->update_bbox_min_ = sensor_position;
    map_data_->update_bbox_max_ = sensor_position;
    reset_updated_bbox_ = false;
  }

  for (int k = 0; k < 3; ++k) {
    map_data_->update_bbox_min_[k] = min(update_min[k], map_data_->update_bbox_min_[k]);
    map_data_->update_bbox_max_[k] = max(update_max[k], map_data_->update_bbox_max_[k]);
  }  
  std::cout<<"sfcafsafaf2"<<id<<std::endl;
  // if (points.empty()) {
  //   return;
  // }
  // std::cout<<"sfcafsafaf1"<<id<<std::endl;
  // Position sensor_position = T_w_c.getPosition();
  // Rotation sensor_orientation = T_w_c.getRotation();

  // // Initialize the update bounding box
  // Position update_min = sensor_position;
  // Position update_max = sensor_position;

  // Position point_w, point_c, voxel_pos, raycast_start, raycast_end;
  // VoxelIndex voxel_idx;
  // VoxelAddress voxel_addr;
  // FloatingPoint value, weight;
  // for (const auto &point : points) {
  //   point_w = addressToPosition(point);
  //   point_c = T_w_c.inverse() * point_w;

  //   double depth = 0.0;
  //   if (config_.depth_axis_ == Config::DepthAxis::X) {
  //     depth = point_c.x();
  //   } else if (config_.depth_axis_ == Config::DepthAxis::Y) {
  //     depth = point_c.y();
  //   } else if (config_.depth_axis_ == Config::DepthAxis::Z) {
  //     depth = point_c.z();
  //   }

  //   // raycast more behind the obstacle surface
  //   raycast_start = point_w + (point_w - sensor_position).normalized() * config_.truncated_dist_;
  //   // limit the raycast range
  //   if ((point_w - sensor_position).norm() > config_.raycast_max_)
  //     raycast_start =
  //         sensor_position + (point_w - sensor_position).normalized() * config_.raycast_max_;
  //   else if ((point_w - sensor_position).norm() < config_.raycast_min_)
  //     continue;
  //   raycast_end = sensor_position;

  //   // Get the closest point, i.e. the raycast line's intersection with map boundary
  //   if (!isInMap(raycast_start))
  //     raycast_start = closestPointInMap(raycast_start, sensor_position);

  //   for (int k = 0; k < 3; ++k) {
  //     update_min[k] = min(update_min[k], raycast_start[k]);
  //     update_max[k] = max(update_max[k], raycast_start[k]);
  //   }

  //   // Update each voxel on the ray from point to sensor
  //   raycaster_->input(raycast_start, raycast_end);
  //   while (raycaster_->nextId(voxel_idx)) {
  //     voxel_pos = indexToPosition(voxel_idx);
  //     voxel_addr = indexToAddress(voxel_idx);
  //     value = computeDistance(sensor_position, point_w, voxel_pos);
  //     weight = computeWeight(value, depth);
  //     updateTSDFVoxel(voxel_addr, value, weight);
  //     if (occupancy_grid_->config_.mode_ == OccupancyGrid::Config::MODE::GEN_TSDF) {
  //       occupancy_grid_->updateOccupancyVoxel(voxel_addr);
  //     }
  //   }
  // }

  // VoxelIndex update_min_idx = positionToIndex(update_min);
  // VoxelIndex update_max_idx = positionToIndex(update_max);
  // boundIndex(update_min_idx);
  // boundIndex(update_max_idx);
  // //esdf_->updateLocalESDF(update_min_idx, update_max_idx);

  // // Bounding box for frontier update
  // std::lock_guard<std::mutex> lock(bbox_mutex_);

  // if (reset_updated_bbox_) {
  //   map_data_->update_bbox_min_ = sensor_position;
  //   map_data_->update_bbox_max_ = sensor_position;
  //   reset_updated_bbox_ = false;
  // }

  // for (int k = 0; k < 3; ++k) {
  //   map_data_->update_bbox_min_[k] = min(update_min[k], map_data_->update_bbox_min_[k]);
  //   map_data_->update_bbox_max_[k] = max(update_max[k], map_data_->update_bbox_max_[k]);
  // }  
  // std::cout<<"sfcafsafaf2"<<id<<std::endl;
}
void TSDF::inputPointCloud(const PointCloudType &pointcloud) {
  if (pointcloud.empty()) {
    return;
  }

  Transformation T_w_c(pointcloud.sensor_orientation_.cast<FloatingPoint>(),
                       pointcloud.sensor_origin_.head<3>().cast<FloatingPoint>());
  Position sensor_position = T_w_c.getPosition();
  Rotation sensor_orientation = T_w_c.getRotation();

  // Initialize the update bounding box
  Position update_min = sensor_position;
  Position update_max = sensor_position;

  Position point_w, point_c, voxel_pos, raycast_start, raycast_end;
  VoxelIndex voxel_idx;
  VoxelAddress voxel_addr;
  FloatingPoint value, weight;
  int ct1=0,ct2=0,ct3=0;


  voxel_mapping::MapData tem;

  tem.time=ros::Time::now().toSec();
  Transformation t(pointcloud.sensor_orientation_.cast<FloatingPoint>(),
  pointcloud.sensor_origin_.head<3>().cast<FloatingPoint>());
  // 平移赋值
  tem.transform_w_c.translation.x = t.getPosition().x();
  tem.transform_w_c.translation.y = t.getPosition().y();
  tem.transform_w_c.translation.z = t.getPosition().z();

  // 四元数赋值（直接按 x, y, z, w 顺序写入）
  tem.transform_w_c.rotation.x = t.getRotation().x();
  tem.transform_w_c.rotation.y = t.getRotation().y();
  tem.transform_w_c.rotation.z = t.getRotation().z();
  tem.transform_w_c.rotation.w = t.getRotation().w();
  vector<int> ps;
  vector<float> wts,vls; 
  for (const auto &point : pointcloud.points) {
    point_w = Position(point.x, point.y, point.z);
    point_c = T_w_c.inverse() * point_w;
    ct1++;
    double depth = 0.0;
    if (config_.depth_axis_ == Config::DepthAxis::X) {
      depth = point_c.x();
    } else if (config_.depth_axis_ == Config::DepthAxis::Y) {
      depth = point_c.y();
    } else if (config_.depth_axis_ == Config::DepthAxis::Z) {
      depth = point_c.z();
    }

    // raycast more behind the obstacle surface
    raycast_start = point_w + (point_w - sensor_position).normalized() * config_.truncated_dist_;
    // limit the raycast range
    if ((point_w - sensor_position).norm() > config_.raycast_max_)
      raycast_start =
          sensor_position + (point_w - sensor_position).normalized() * config_.raycast_max_;
    else if ((point_w - sensor_position).norm() < config_.raycast_min_)
      continue;
    raycast_end = sensor_position;

    // Get the closest point, i.e. the raycast line's intersection with map boundary
    if (!isInMap(raycast_start))
      raycast_start = closestPointInMap(raycast_start, sensor_position);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], raycast_start[k]);
      update_max[k] = max(update_max[k], raycast_start[k]);
    }

    // Update each voxel on the ray from point to sensor
    raycaster_->input(raycast_start, raycast_end);
    while (raycaster_->nextId(voxel_idx)) {
      ct2++;
      voxel_pos = indexToPosition(voxel_idx);
      voxel_addr = indexToAddress(voxel_idx);
      value = computeDistance(sensor_position, point_w, voxel_pos);
      weight = computeWeight(value, depth);
      updateTSDFVoxel(voxel_addr, value, weight);
      if(occupancy_grid_->map_data_->data[voxel_addr].value==OccupancyType::UNKNOWN){
        ps.push_back(voxel_addr);
        vls.push_back(value);
        wts.push_back(weight);
        ct3++;
      }
      if (occupancy_grid_->config_.mode_ == OccupancyGrid::Config::MODE::GEN_TSDF) {
        occupancy_grid_->updateOccupancyVoxel(voxel_addr);
      }
    }
  }
  tem.points=ps;
  tem.values=vls;
  tem.weights=wts;
  tem.ct=countup;
  countup++;
  ms_->sendmap(tem);
  std::cout<<"sfasfglkhashgjkasg "<<ct1<<" "<<ct2<<" "<<ct3<<std::endl;
  VoxelIndex update_min_idx = positionToIndex(update_min);
  VoxelIndex update_max_idx = positionToIndex(update_max);
  boundIndex(update_min_idx);
  boundIndex(update_max_idx);
  esdf_->updateLocalESDF(update_min_idx, update_max_idx);

  // Bounding box for frontier update
  std::lock_guard<std::mutex> lock(bbox_mutex_);

  if (reset_updated_bbox_) {
    map_data_->update_bbox_min_ = sensor_position;
    map_data_->update_bbox_max_ = sensor_position;
    reset_updated_bbox_ = false;
  }

  for (int k = 0; k < 3; ++k) {
    map_data_->update_bbox_min_[k] = min(update_min[k], map_data_->update_bbox_min_[k]);
    map_data_->update_bbox_max_[k] = max(update_max[k], map_data_->update_bbox_max_[k]);
  }
}

FloatingPoint TSDF::computeDistance(const Position &origin, const Position &point,
                                    const Position &voxel) {
  const Position v_voxel_origin = voxel - origin;
  const Position v_point_origin = point - origin;

  const FloatingPoint dist_G = v_point_origin.norm();
  const FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  FloatingPoint sdf = static_cast<FloatingPoint>(dist_G - dist_G_V);
  sdf = (sdf > 0.0) ? min(config_.truncated_dist_, sdf) : max(-config_.truncated_dist_, sdf);

  return sdf;
}

FloatingPoint TSDF::computeWeight(const FloatingPoint &sdf, const FloatingPoint &depth) {
  FloatingPoint simple_weight, dropoff_weight;

  simple_weight = 1.0 / (depth * depth);

  if (sdf < -map_config_.resolution_) {
    dropoff_weight = simple_weight * (config_.truncated_dist_ + sdf) /
                     (config_.truncated_dist_ - map_config_.resolution_);
    dropoff_weight = std::max(dropoff_weight, 0.0);
  } else {
    dropoff_weight = simple_weight;
  }

  return dropoff_weight;
}

void TSDF::updateTSDFVoxel(const VoxelAddress &addr, const FloatingPoint &sdf,
                           const FloatingPoint &weight) {
  // Treated it as unknown if the weight is 0 after voxel update
  if (map_data_->data[addr].weight + weight < config_.epsilon_) {
    map_data_->data[addr].value = 1.0;
    map_data_->data[addr].weight = 0.0;
    return;
  }

  // Unknow voxel as weight is 0, reset the value to 0
  if (map_data_->data[addr].weight < config_.epsilon_)
    map_data_->data[addr].value = 0.0;

  map_data_->data[addr].value =
      (map_data_->data[addr].value * map_data_->data[addr].weight + sdf * weight) /
      (map_data_->data[addr].weight + weight);
  map_data_->data[addr].weight = map_data_->data[addr].weight + weight;

  map_data_->data[addr].value =
      (map_data_->data[addr].value > 0.0)
          ? std::min(config_.truncated_dist_, map_data_->data[addr].value)
          : std::max(-config_.truncated_dist_, map_data_->data[addr].value);
  map_data_->data[addr].weight = std::min(10000.0, map_data_->data[addr].weight);
}

void TSDF::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset) {
  std::lock_guard<std::mutex> lock(bbox_mutex_);

  bmin = map_data_->update_bbox_min_;
  bmax = map_data_->update_bbox_max_;

  for (int k = 0; k < 3; ++k) {
    bmin[k] = max(bmin[k], map_config_.box_min_[k]);
    bmax[k] = min(bmax[k], map_config_.box_max_[k]);
  }

  if (reset)
    reset_updated_bbox_ = true;
}

} // namespace voxel_mapping