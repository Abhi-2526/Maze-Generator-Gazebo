import secrets

def make_maze(w=10, h=10):
    wall = 'wall'
    cell = 'cell'
    maze = [[wall for _ in range(w)] for _ in range(h)]
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    stack = [(0, 0)]

    while stack:
        x, y = stack[-1]
        maze[y][x] = cell

        # Check potential directions
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and maze[ny][nx] == wall:
                if sum(1 for dxx, dyy in directions if 0 <= nx + dxx < w and 0 <= ny + dyy < h and maze[ny + dyy][nx + dxx] == cell) == 1:
                    neighbors.append((nx, ny))

        if neighbors:
            stack.append(secrets.choice(neighbors))
        else:
            stack.pop()

    # Convert maze to character representation with visual depth
    formatted_maze = []
    for y in range(h):
        row = []
        for x in range(w):
            if maze[y][x] == cell:
                # Check if the cell below is a wall to add an underscore
                if y + 1 < h and maze[y + 1][x] == wall:
                    row.append('_')
                else:
                    row.append(' ')
            else:
                row.append('|')
        formatted_maze.append(''.join(row))

    # Generate the top and bottom borders
    top_border = ''.join(['-' if formatted_maze[0][i] == ' ' else '|' for i in range(w)])
    bottom_border = ''.join(['_' if formatted_maze[-1][i] == ' ' else '|' for i in range(w)])

    # Adjust for consecutive vertical walls
    top_border = adjust_borders(top_border)
    bottom_border = adjust_borders(bottom_border)

    # Enclose the maze with borders
    enclosed_maze = ['|' + top_border + '|']
    enclosed_maze += ['|' + row + '|' for row in formatted_maze]
    enclosed_maze.append('|' + bottom_border + '|')

    return enclosed_maze

def adjust_borders(border):
    adjusted = []
    for i in range(len(border)):
        if i > 0 and border[i] == '|' and border[i-1] == '|':
            adjusted.append('-')
        else:
            adjusted.append(border[i])
    return ''.join(adjusted)

def save_maze(maze, filename):
    with open(filename, 'w') as f:
        for line in maze:
            f.write(line + '\n')

if __name__ == '__main__':
    for i in range(50):
        filename = f'mazes/maze_{i+1}.txt'
        maze = make_maze()
        save_maze(maze, filename)
        print(f'Maze {i+1} saved as {filename}')
