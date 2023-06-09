# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors. A plugin interface allows for the layers to be combined into the costmap and finally inflated via an inflation radius based on the robot footprint. The nav2 version of the costmap_2d package is mostly a direct ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) for additional parameter descriptions for the costmap and its included plugins. The [tutorials](https://navigation.ros.org/tutorials/index.html) and [first-time setup guides](https://navigation.ros.org/setup_guides/index.html) also provide helpful context for working with the costmap 2D package and its layers. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) is also provided to explain how to create costmap plugins.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## To visualize the voxels in RVIZ:
- Make sure `publish_voxel_map` in `voxel_layer` param's scope is set to `True`.
- Open a new terminal and run:
  ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```
    Here you can change `my_marker` to any topic name you like for the markers to be published on.

- Then add `my_marker` to RVIZ using the GUI.


### Errata:
- To see the markers in 3D, you will need to change the _view_ in RVIZ to a 3 dimensional view (e.g. orbit) from the RVIZ GUI.
- Currently due to some bug in rviz, you need to set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from a saved bag file while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## Costmap Filters

### Overview

Costmap Filters - is a costmap layer-based instrument which provides an ability to apply to map spatial-dependent raster features named as filter-masks. These features are used in plugin algorithms when filling costmaps in order to allow robots to change their trajectory, behavior or speed when a robot enters/leaves an area marked in a filter masks. Examples of costmap filters include keep-out/safety zones where robots will never enter, speed restriction areas, preferred lanes for robots moving in industries and warehouses. More information about design, architecture of the feature and how it works could be found on Nav2 website: https://navigation.ros.org.




local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["static_layer","decaying_obstacle_layer", "inflation_layer"]
      decaying_obstacle_layer:
        plugin:  "decaying_layer::DecayingObstacleLayer"
        enabled: True
        observation_sources: abc
        abc:
          topic: /test_cloud
          min_obstacle_height: -1.0
          max_obstacle_height: 2.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.2
          observation_persistence: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        # observation_sources: abc
        # abc:
        #   topic: /test_cloud2
        #   min_obstacle_height: -1.0
        #   max_obstacle_height: 2.0
        #   obstacle_max_range: 10.0
        #   obstacle_min_range: 0.2
        #   observation_persistence: 0.1
        #   clearing: True
        #   marking: True
        #   data_type: "PointCloud2"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      # voxel_layer:
      #   plugin: "nav2_costmap_2d::VoxelLayer"
      #   enabled: True
      #   publish_voxel_map: True
      #   origin_z: 0.0
      #   z_resolution: 0.05
      #   z_voxels: 16
      #   max_obstacle_height: 2.0
      #   mark_threshold: 0
      #   observation_sources: scan
      #   scan:
      #     topic: /scan
      #     max_obstacle_height: 2.0
      #     clearing: True
      #     marking: True
      #     data_type: "LaserScan"
      #     raytrace_max_range: 3.0
      #     raytrace_min_range: 0.0
      #     obstacle_max_range: 2.5
      #     obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05 
      track_unknown_space: true
      plugins: ["static_layer","decaying_obstacle_layer","inflation_layer"]
      decaying_obstacle_layer:
        plugin:  "decaying_layer::DecayingObstacleLayer"
        enabled: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan abc
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
        abc:
          topic: /test_cloud
          min_obstacle_height: -1.0
          max_obstacle_height: 2.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.2
          observation_persistence: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
