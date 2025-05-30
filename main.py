import serial
import time
from collections import deque

# Set up serial ports
ser_arduino = serial.Serial('/dev/ttyACM0', 9600)  # Arduino connection
ser_esp32 = serial.Serial('/dev/ttyUSB0', 115200)  # ESP32 connection (adjust port as needed)
time.sleep(2)

# Direction handling
DIRECTIONS = ['N', 'E', 'S', 'W']
BLUE_BIN = (0, 2)
RED_BIN = (5, 2)


def update_direction(current_dir, turn):
    idx = DIRECTIONS.index(current_dir)
    if turn == 'L':
        new_idx = (idx - 1) % 4
    else:
        new_idx = (idx + 1) % 4
    return DIRECTIONS[new_idx]


def turn_commands(current_dir, target_dir):
    idx_current = DIRECTIONS.index(current_dir)
    idx_target = DIRECTIONS.index(target_dir)
    diff = (idx_target - idx_current) % 4

    if diff == 0:
        return [], target_dir
    elif diff == 1:
        return ['R'], target_dir
    elif diff == 2:
        return ['U'], target_dir
    elif diff == 3:
        return ['L'], target_dir


def simulate_movement(start_x, start_y, start_dir, commands):
    current_x, current_y = start_x, start_y
    current_dir = start_dir
    for cmd in commands:
        if cmd == 'F':
            if current_dir == 'N':
                current_y += 1
            elif current_dir == 'E':
                current_x += 1
            elif current_dir == 'S':
                current_y -= 1
            elif current_dir == 'W':
                current_x -= 1
        elif cmd in ['L', 'R']:
            current_dir = update_direction(current_dir, cmd)
        elif cmd == 'U':  # Add U-turn handling
            current_dir = update_direction(current_dir, 'R')
            current_dir = update_direction(current_dir, 'R')
    return current_x, current_y, current_dir


def simulate_coordinates(start_x, start_y, start_dir, commands):
    coords = [(start_x, start_y)]
    current_x, current_y, current_dir = start_x, start_y, start_dir
    for cmd in commands:
        if cmd == 'F':
            if current_dir == 'N':
                current_y += 1
            elif current_dir == 'E':
                current_x += 1
            elif current_dir == 'S':
                current_y -= 1
            elif current_dir == 'W':
                current_x -= 1
            coords.append((current_x, current_y))
        elif cmd in ['L', 'R']:
            current_dir = update_direction(current_dir, cmd)
        elif cmd == 'U':  # Add U-turn handling
            current_dir = update_direction(current_dir, 'R')
            current_dir = update_direction(current_dir, 'R')
    return coords


def get_manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


def get_neighbors(x, y):
    """Get all 4 adjacent coordinates"""
    return [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]


def a_star_pathfind(start_x, start_y, target_x, target_y, obstacles):
    """A* pathfinding algorithm to avoid obstacles"""
    if (start_x, start_y) == (target_x, target_y):
        return [(start_x, start_y)]
    
    obstacles_set = set(obstacles)
    open_set = [(start_x, start_y)]
    came_from = {}
    g_score = {(start_x, start_y): 0}
    f_score = {(start_x, start_y): get_manhattan_distance(start_x, start_y, target_x, target_y)}
    
    while open_set:
        # Find node with lowest f_score
        current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
        
        if current == (target_x, target_y):
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append((start_x, start_y))
            return path[::-1]
        
        open_set.remove(current)
        current_x, current_y = current
        
        for next_x, next_y in get_neighbors(current_x, current_y):
            # Skip if this coordinate is an obstacle
            if (next_x, next_y) in obstacles_set:
                continue
            
            # Skip if out of reasonable bounds (adjust as needed for your grid)
            if next_x < -10 or next_x > 10 or next_y < -10 or next_y > 10:
                continue
            
            tentative_g_score = g_score[current] + 1
            
            if (next_x, next_y) not in g_score or tentative_g_score < g_score[(next_x, next_y)]:
                came_from[(next_x, next_y)] = current
                g_score[(next_x, next_y)] = tentative_g_score
                f_score[(next_x, next_y)] = tentative_g_score + get_manhattan_distance(next_x, next_y, target_x, target_y)
                
                if (next_x, next_y) not in open_set:
                    open_set.append((next_x, next_y))
    
    # No path found
    return None


def convert_path_to_commands(path, start_dir):
    """Convert coordinate path to movement commands"""
    if len(path) < 2:
        return []
    
    commands = []
    current_dir = start_dir
    
    for i in range(1, len(path)):
        prev_x, prev_y = path[i-1]
        curr_x, curr_y = path[i]
        
        # Determine required direction
        if curr_x > prev_x:
            required_dir = 'E'
        elif curr_x < prev_x:
            required_dir = 'W'
        elif curr_y > prev_y:
            required_dir = 'N'
        elif curr_y < prev_y:
            required_dir = 'S'
        else:
            continue  # No movement needed
        
        # Add turn commands if needed
        turns, new_dir = turn_commands(current_dir, required_dir)
        commands.extend(turns)
        current_dir = new_dir
        
        # Add forward command
        commands.append('F')
    
    return commands


def plan_path(curr_x, curr_y, curr_dir, tgt_x, tgt_y, forbidden_coords=[]):
    """Plan path using A* algorithm to avoid obstacles"""
    # Use A* pathfinding to find optimal path avoiding obstacles
    coordinate_path = a_star_pathfind(curr_x, curr_y, tgt_x, tgt_y, forbidden_coords)
    
    if coordinate_path is None:
        print("Warning: No path found avoiding all obstacles, trying simple path")
        # Fallback to original method if A* fails
        return plan_path_simple(curr_x, curr_y, curr_dir, tgt_x, tgt_y, forbidden_coords)
    
    # Convert coordinate path to movement commands
    commands = convert_path_to_commands(coordinate_path, curr_dir)
    
    # Verify the path doesn't go through forbidden coordinates
    coords = simulate_coordinates(curr_x, curr_y, curr_dir, commands)
    collision = any(coord in forbidden_coords for coord in coords[1:])  # Skip starting position
    
    if collision:
        print("Warning: Generated path still has collisions, using fallback")
        return plan_path_simple(curr_x, curr_y, curr_dir, tgt_x, tgt_y, forbidden_coords)
    
    return commands


def plan_path_simple(curr_x, curr_y, curr_dir, tgt_x, tgt_y, forbidden_coords=[]):
    """Original simple pathfinding as fallback"""
    def generate_path(x_first=True):
        path = []
        temp_x, temp_y = curr_x, curr_y
        temp_dir = curr_dir

        if x_first:
            # X movement
            if tgt_x > temp_x:
                desired_dir = 'E'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (tgt_x - temp_x)
                temp_x = tgt_x
                temp_dir = new_dir
            elif tgt_x < temp_x:
                desired_dir = 'W'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (temp_x - tgt_x)
                temp_x = tgt_x
                temp_dir = new_dir

            # Y movement
            if tgt_y > temp_y:
                desired_dir = 'N'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (tgt_y - temp_y)
            elif tgt_y < temp_y:
                desired_dir = 'S'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (temp_y - tgt_y)
        else:
            # Y movement
            if tgt_y > temp_y:
                desired_dir = 'N'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (tgt_y - temp_y)
                temp_y = tgt_y
                temp_dir = new_dir
            elif tgt_y < temp_y:
                desired_dir = 'S'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (temp_y - tgt_y)
                temp_y = tgt_y
                temp_dir = new_dir

            # X movement
            if tgt_x > temp_x:
                desired_dir = 'E'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (tgt_x - temp_x)
            elif tgt_x < temp_x:
                desired_dir = 'W'
                turns, new_dir = turn_commands(temp_dir, desired_dir)
                path += turns
                path += ['F'] * (temp_x - tgt_x)

        return path

    path_x_y = generate_path(x_first=True)
    path_y_x = generate_path(x_first=False)

    coords_x_y = simulate_coordinates(curr_x, curr_y, curr_dir, path_x_y)
    coords_y_x = simulate_coordinates(curr_x, curr_y, curr_dir, path_y_x)

    collision_x_y = any(coord in forbidden_coords for coord in coords_x_y[1:])  # Skip starting position
    collision_y_x = any(coord in forbidden_coords for coord in coords_y_x[1:])  # Skip starting position

    if not collision_x_y:
        return path_x_y
    elif not collision_y_x:
        return path_y_x
    else:
        print("Warning: Both simple paths pass through forbidden coordinates")
        print(f"Forbidden coordinates: {forbidden_coords}")
        print(f"Path X-Y coordinates: {coords_x_y}")
        print(f"Path Y-X coordinates: {coords_y_x}")
        return path_x_y  # Return one anyway, but with warning


def plan_bin_path(curr_x, curr_y, curr_dir, bin_x, bin_y, color, forbidden_coords=[]):
    if color.lower() == 'blue':
        approach_x, approach_y = bin_x + 1, bin_y
        final_dir = 'W'
    elif color.lower() == 'red':
        approach_x, approach_y = bin_x - 1, bin_y
        final_dir = 'E'
    else:
        return []

    # Plan path to approach point
    path = plan_path(curr_x, curr_y, curr_dir, approach_x, approach_y, forbidden_coords)

    # Simulate to get final position/direction
    final_x, final_y, final_dir_after = simulate_movement(curr_x, curr_y, curr_dir, path)

    # Add turn to face bin
    turns, new_dir = turn_commands(final_dir_after, final_dir)
    path += turns

    # Add final move into bin
    path.append('F')

    return path


def send_esp32_command(command):
    """Send command to ESP32 and print confirmation"""
    ser_esp32.write(command.encode())
    print(f"Sent to ESP32: {command}")


def execute_commands(commands, current_x, current_y, current_dir):
    for command in commands:
        ser_arduino.write(command.encode())
        print(f"Sent to Arduino: {command}")

        # Update position and direction
        if command == 'F':
            if current_dir == 'N':
                current_y += 1
            elif current_dir == 'E':
                current_x += 1
            elif current_dir == 'S':
                current_y -= 1
            elif current_dir == 'W':
                current_x -= 1
        elif command in ['L', 'R']:
            current_dir = update_direction(current_dir, command)
        elif command == 'U':  # THIS IS THE CRITICAL FIX
            # U-turn = 180 degrees = two right turns
            current_dir = update_direction(current_dir, 'R')
            current_dir = update_direction(current_dir, 'R')

        # Wait for Arduino confirmation
        while True:
            if ser_arduino.in_waiting > 0:
                response = ser_arduino.readline().decode().strip()
                if response == "DONE":
                    print("Arduino: DONE")
                    break
                else:
                    print(f"Arduino says: {response}")

    return current_x, current_y, current_dir


def main():
    send_esp32_command('J')
    print("Enter initial coordinates (x y):")
    curr_x, curr_y = map(int, input().split())

    print("Enter initial direction (N, E, S, W):")
    curr_dir = input().strip().upper()

    objects = []
    for i in range(4):
        print(f"Enter coordinates and color for object {i + 1} (x y color):")
        x, y, color = input().split()
        objects.append((int(x), int(y), color))

    # Initial startup sequence
    startup_sequence = ['C']
    for command in startup_sequence:
        if command == 'WAIT':
            print("Waiting 2 seconds...")
            time.sleep(2)
        else:
            ser_arduino.write(command.encode())
            print(f"Startup - Sent to Arduino: {command}")

            # Update simulated position and direction
            if command == 'F':
                if curr_dir == 'N':
                    curr_y += 1
                elif curr_dir == 'E':
                    curr_x += 1
                elif curr_dir == 'S':
                    curr_y -= 1
                elif curr_dir == 'W':
                    curr_x -= 1
            elif command in ['L', 'R']:
                curr_dir = update_direction(curr_dir, command)

            # Wait for Arduino DONE response
            while True:
                if ser_arduino.in_waiting > 0:
                    response = ser_arduino.readline().decode().strip()
                    if response == "DONE":
                        print("Arduino: DONE")
                        break
                    else:
                        print(f"Arduino says: {response}")

    # Keep track of visited objects
    visited_objects = set()

    for i, (obj_x, obj_y, color) in enumerate(objects):
        # Create forbidden coordinates list: all unvisited objects
        forbidden = []
        for j, (x, y, c) in enumerate(objects):
            if j != i and j not in visited_objects:
                forbidden.append((x, y))

        print(f"\nObject {i + 1}: Navigating to {color} object at ({obj_x}, {obj_y})")
        print(f"Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")
        print(f"Avoiding unvisited objects at: {forbidden}")

        # Navigate to object
        commands = plan_path(curr_x, curr_y, curr_dir, obj_x, obj_y, forbidden)
        print("Path to object:", commands)

        curr_x, curr_y, curr_dir = execute_commands(commands, curr_x, curr_y, curr_dir)
        print(f"Reached object. Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")

        # Mark this object as visited
        visited_objects.add(i)

        # Send 'K' to ESP32 after completing path to object
        send_esp32_command('K')

        # Update forbidden coordinates for bin path (exclude current object location, but keep other unvisited objects)
        forbidden_for_bin = []
        for j, (x, y, c) in enumerate(objects):
            if j != i and j not in visited_objects:
                forbidden_for_bin.append((x, y))

        # Navigate to bin
        bin_coords = BLUE_BIN if color.lower() == 'blue' else RED_BIN
        print(f"\nNavigating to {color} bin at {bin_coords}")
        print(f"Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")
        print(f"Avoiding unvisited objects at: {forbidden_for_bin}")
        commands = plan_bin_path(curr_x, curr_y, curr_dir, bin_coords[0], bin_coords[1], color, forbidden_for_bin)
        print("Path to bin:", commands)

        curr_x, curr_y, curr_dir = execute_commands(commands, curr_x, curr_y, curr_dir)
        print(f"Reached bin. Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")

        # Send 'J' to ESP32 after completing path to bin
        send_esp32_command('J')

    # Close both serial connections
    ser_arduino.close()
    ser_esp32.close()
    print("\nAll 4 deliveries completed!")


if __name__ == "__main__":
    main()