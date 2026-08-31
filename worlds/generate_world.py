def generate_gazebo_world(grid, output_filename="sih_warehouse.world", block_size=1.0, wall_height=2.0):
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="sih_warehouse_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
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
