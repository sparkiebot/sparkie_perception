include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- map frame name 
  tracking_frame = "base_footprint",  -- tracking frame name
  published_frame = "odom",  -- published frame name 
  odom_frame = "odom",  -- name of the odometer frame
  provide_odom_frame = false,  -- whether to provide the odometer frame
  publish_frame_projected_to_2d = true,  -- whether to publish 2d gesture  
  use_pose_extrapolator = false,
  use_odometry = true,  -- whether use odometry
  use_nav_sat = false,  -- whether use the navigation satellite 
  use_landmarks = false,  -- whether use the landmark
  num_laser_scans = 1,  -- LiDAR number  
  num_multi_echo_laser_scans = 0,  -- number of multi-echo LiDAR  
  num_subdivisions_per_laser_scan = 1,  -- number of subdivisions for each laser scan
  num_point_clouds = 0,  -- number of cloud points
  lookup_transform_timeout_sec = 0.2,  -- timeout for finding transformations (seconds)  
  pose_publish_period_sec = 20e-3,        -- da 5e-3 a 20e-3 (50Hz invece di 200Hz)
  trajectory_publish_period_sec = 100e-3,  -- da 30e-3 a 100e-3 (10Hz invece di 33Hz)
  submap_publish_period_sec = 0.5,        -- da 0.3 a 0.5 secondi
  rangefinder_sampling_ratio = 1.,  -- rangefinder sampling ratio
  odometry_sampling_ratio = 1.,  -- odometer sampling rate
  fixed_frame_pose_sampling_ratio = 1.,  -- fixed frame attitude sampling ratio  
  imu_sampling_ratio = 1.,  -- IMU sampling ratio
  landmarks_sampling_ratio = 1.,  -- landmarks sampling ratio
}

MAP_BUILDER.num_background_threads = 6  -- Number of background threads for map building
MAP_BUILDER.use_trajectory_builder_2d = true  -- whether use 2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- Number of range data for submaps in the 2D track builder  
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter.miss_probability = 0.47
TRAJECTORY_BUILDER_2D.min_range = 0.05  -- ignore anything smaller than the robot radius, limiting it to the minimum scan range of the lidar
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- the maximum scanning range of the lidar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- Restricted to maximum LiDAR scanning range  
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- whether use IMU data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Whether to scan for matches using real-time loopback detection

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Modify 1.0 to 0.1, increased sensitivity to movement
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Modify 0.55 to 0.65, the minium score of Fast csm, can be optimized above this score 

-- Fast correlative scan matcher meno intensivo
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}

TRAJECTORY_BUILDER.pure_localization = true

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- Ceres solver più veloce
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 6  -- importante!

return options