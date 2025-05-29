import serial
import time

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

def plan_path(curr_x, curr_y, curr_dir, tgt_x, tgt_y, forbidden_coords=[]):
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

    collision_x_y = any(coord in forbidden_coords for coord in coords_x_y)
    collision_y_x = any(coord in forbidden_coords for coord in coords_y_x)

    if not collision_x_y:
        return path_x_y
    elif not collision_y_x:
        return path_y_x
    else:
        print("Warning: Path may pass through forbidden coordinates")
        return path_x_y


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
    print("Enter initial coordinates (x y):")
    curr_x, curr_y = map(int, input().split())
    send_esp32_command('J')
    print("Enter initial direction (N, E, S, W):")
    curr_dir = input().strip().upper()

    objects = []
    for i in range(2):
        print(f"Enter coordinates and color for object {i + 1} (x y color):")
        x, y, color = input().split()
        objects.append((int(x), int(y), color))

    for i, (obj_x, obj_y, color) in enumerate(objects):
        # Get other object's coordinates for collision avoidance
        forbidden = [(x, y) for j, (x, y, c) in enumerate(objects) if j != i]

        # Navigate to object
        print(f"\nNavigating to {color} object at ({obj_x}, {obj_y})")
        commands = plan_path(curr_x, curr_y, curr_dir, obj_x, obj_y, forbidden)
        print("Path to object:", commands)

        curr_x, curr_y, curr_dir = execute_commands(commands, curr_x, curr_y, curr_dir)
        print(f"Reached object. Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")

        # Send 'K' to ESP32 after completing path to object
        send_esp32_command('K')

        # Navigate to bin
        bin_coords = BLUE_BIN if color.lower() == 'blue' else RED_BIN
        print(f"\nNavigating to {color} bin at {bin_coords}")
        commands = plan_bin_path(curr_x, curr_y, curr_dir, bin_coords[0], bin_coords[1], color, forbidden)
        print("Path to bin:", commands)

        curr_x, curr_y, curr_dir = execute_commands(commands, curr_x, curr_y, curr_dir)
        print(f"Reached bin. Current position: ({curr_x}, {curr_y}), Direction: {curr_dir}")

        # Send 'J' to ESP32 after completing path to bin
        send_esp32_command('J')

    # Close both serial connections
    ser_arduino.close()
    ser_esp32.close()
    print("\nAll deliveries completed!")


if __name__ == "__main__":
    main()