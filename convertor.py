import os
from lxml import etree


def convert_mz_to_world(mz_file, world_file, wall_height=2, wall_thickness=0.1, grid_size=1.0):
    with open(mz_file, 'r') as file:
        mz_data = file.read()

    rows = mz_data.strip().split('\n')
    num_rows = len(rows)
    num_cols = len(rows[0])

    sdf = etree.Element('sdf', version='1.6')
    world = etree.SubElement(sdf, 'world', name='default')
    include_sun(world)
    create_ground(world, num_cols, num_rows, grid_size)

    # Process vertical walls
    for i in range(num_rows):
        for j in range(num_cols):
            if rows[i][j] == '|':
                add_vertical_wall(world, i, i + 1, j, grid_size, wall_thickness, wall_height)

    # Process vertical walls at the rightmost edge
    # for i in range(num_rows):
    #     if rows[i][-1] == '|':
    #         add_vertical_wall(world, i, i + 1, num_cols, grid_size, wall_thickness, wall_height)

    # Process horizontal walls
    for i in range(len(rows)):
        j = 0
        while j < num_cols:
            if rows[i][j] == '_':
                start_j = j
                # Adjust start position if there is a vertical wall to the left
                if j > 0 and rows[i][j - 1] == '|':
                    start_j -= 1
                while j < num_cols and rows[i][j] == '_':
                    j += 1
                add_horizontal_wall(world, i, start_j, j - start_j, grid_size, wall_thickness, wall_height,
                                    is_top=False)
            elif rows[i][j] == '-':
                start_j = j
                # Adjust start position if there is a vertical wall to the left
                if j > 0 and rows[i][j - 1] == '|':
                    start_j -= 1
                while j < num_cols and rows[i][j] == '-':
                    j += 1
                add_horizontal_wall(world, i, start_j, j - start_j, grid_size, wall_thickness, wall_height,
                                    is_top=True)
            else:
                j += 1

    tree = etree.ElementTree(sdf)
    tree.write(world_file, pretty_print=True, xml_declaration=True, encoding='utf-8')


def include_sun(world):
    sun = etree.SubElement(world, 'include')
    uri = etree.SubElement(sun, 'uri')
    uri.text = 'model://sun'


def create_ground(world, num_cols, num_rows, grid_size):
    ground_size_x = num_cols * grid_size
    ground_size_y = num_rows * grid_size
    ground = etree.SubElement(world, 'model', name='ground_plane')
    static = etree.SubElement(ground, 'static')
    static.text = 'true'
    link = etree.SubElement(ground, 'link', name='link')
    collision = etree.SubElement(link, 'collision', name='collision')
    geometry = etree.SubElement(collision, 'geometry')
    plane = etree.SubElement(geometry, 'plane')
    size = etree.SubElement(plane, 'size')
    size.text = f'{ground_size_x} {ground_size_y}'
    visual = etree.SubElement(link, 'visual', name='visual')
    cast_shadows = etree.SubElement(visual, 'cast_shadows')
    cast_shadows.text = 'false'
    geometry = etree.SubElement(visual, 'geometry')
    plane = etree.SubElement(geometry, 'plane')
    size = etree.SubElement(plane, 'size')
    size.text = f'{ground_size_x} {ground_size_y}'
    material = etree.SubElement(visual, 'material')
    script = etree.SubElement(material, 'script')
    uri = etree.SubElement(script, 'uri')
    uri.text = 'file://media/materials/scripts/gazebo.material'
    name = etree.SubElement(script, 'name')
    name.text = 'Gazebo/Grey'

    # Set the pose so that the ground starts at (0, 0) and extends positively
    pose = etree.SubElement(ground, 'pose')
    pose.text = f'{ground_size_x / 2} {-ground_size_y / 2} 0 0 0 0'


def add_vertical_wall(world, start_i, end_i, j, grid_size, wall_thickness, wall_height):
    length = (end_i - start_i) * grid_size
    wall = etree.SubElement(world, 'model', name=f'vwall_{start_i}_{j}')
    pose = etree.SubElement(wall, 'pose')
    x = j * grid_size
    y = -(start_i * grid_size + length / 2)
    pose.text = f'{x} {y} {wall_height / 2} 0 0 0'
    create_wall_box(wall, wall_thickness, length, wall_height)


def add_horizontal_wall(world, i, start_j, wall_length, grid_size, wall_thickness, wall_height, is_top):
    length = wall_length * grid_size
    wall = etree.SubElement(world, 'model', name=f'hwall_{i}_{start_j}')
    pose = etree.SubElement(wall, 'pose')
    x = start_j * grid_size + length / 2
    y = -i * grid_size + (0 if is_top else -grid_size)
    pose.text = f'{x} {y} {wall_height / 2} 0 0 0'
    create_wall_box(wall, length, wall_thickness, wall_height)


def create_wall_box(wall, width, height, depth):
    link = etree.SubElement(wall, 'link', name='link')
    collision = etree.SubElement(link, 'collision', name='collision')
    geometry = etree.SubElement(collision, 'geometry')
    box = etree.SubElement(geometry, 'box')
    size = etree.SubElement(box, 'size')
    size.text = f'{width} {height} {depth}'
    visual = etree.SubElement(link, 'visual', name='visual')
    geometry = etree.SubElement(visual, 'geometry')
    box = etree.SubElement(geometry, 'box')
    size = etree.SubElement(box, 'size')
    size.text = f'{width} {height} {depth}'


if __name__ == '__main__':
    mz_file = 'sample_maze.mz'
    world_file = 'maze.world'
    convert_mz_to_world(mz_file, world_file)