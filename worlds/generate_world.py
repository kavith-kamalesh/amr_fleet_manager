def generate_gazebo_world(grid, output_filename="sih_warehouse.world", block_size=1.0, wall_height=2.0):
    # Gazebo Harmonic (gz-sim) port. Two things changed from the Classic
    # version, both required, neither optional:
    #
    # 1. model://sun and model://ground_plane were Gazebo CLASSIC's
    #    built-in model-database shortcuts. Harmonic's Fuel-based asset
    #    system doesn't resolve them the same way, and depending on Fuel
    #    at all means needing internet connectivity to spawn a world --
    #    not something a live demo should depend on. Replaced with
    #    explicit <light>/<model> definitions that work fully offline.
    #
    # 2. Harmonic requires you to explicitly declare which system plugins
    #    a world uses (Physics, user commands, scene broadcasting,
    #    sensors). Classic auto-loaded these; Harmonic does not -- a
    #    world file without them will load with no physics stepping and
    #    no way to spawn entities into it at runtime.
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="sih_warehouse_world">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
    sdf_footer = """  </world>\n</sdf>\n"""

    with open(output_filename, 'w') as f:
        f.write(sdf_header)
        box_id = 0
        for y, row in enumerate(grid):
            for x, cell in enumerate(row):
                if cell == 1:
                    pos_x = x * block_size
                    pos_y = y * block_size
                    pos_z = wall_height / 2.0
                    box_sdf = f"""
    <model name='rack_{box_id}'>
      <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{block_size} {block_size} {wall_height}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{block_size} {block_size} {wall_height}</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>"""
                    f.write(box_sdf)
                    box_id += 1
        f.write(sdf_footer)
    print(f"Generated {box_id} racks -> {output_filename}")


sample_grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    [1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1],
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]

if __name__ == "__main__":
    generate_gazebo_world(sample_grid)
