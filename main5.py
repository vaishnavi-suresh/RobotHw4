# main5.py

import asyncio
from viam.components.base import Base, Vector3
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
import numpy as np

async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

async def get_position(slam):
    return await slam.get_position()

def get_dist(curr_x, curr_y, want_x, want_y):
    return np.sqrt((want_x - curr_x)**2 + (want_y - curr_y)**2)

async def closest_to_path(curr_x, curr_y, slam, arr_pos):
    wp_index = await find_waypoint(curr_x, curr_y, slam, arr_pos)
    return wp_index

async def move_angle(base, slam, to_move, target_angle):
    while abs(to_move) > 1:
        await base.spin(to_move/2, 45)
        curr_pos = await slam.get_position()
        curr_theta = curr_pos.theta
        to_move = (target_angle - curr_theta + 180) % 360 - 180
        await asyncio.sleep(0.1)  # Added sleep to prevent too rapid updates

async def move_to_pos(base, slam, x, y, theta):
    print(f"Moving to point: x={x}, y={y}, theta={theta}")
    curr_pos = await slam.get_position()
    curr_x = curr_pos.x
    curr_y = curr_pos.y
    curr_theta = curr_pos.theta
    
    # Calculate angle to target
    target_angle_rad = np.arctan2(y - curr_y, x - curr_x)
    target_angle = np.degrees(target_angle_rad)
    to_move = (target_angle - curr_theta + 175) % 360 - 180
    
    # Calculate distance
    dist = get_dist(curr_x, curr_y, x, y)
    
    # First rotate to face the target
    await move_angle(base, slam, to_move, target_angle)
    
    # Then move straight to the target
    if dist > 0:
        await base.move_straight(int(dist), 400)
    
    # Finally rotate to desired final orientation
    final_rotation = (theta - target_angle + 180) % 360 - 180
    await move_angle(base, slam, final_rotation, theta)

async def find_waypoint(x, y, slam, arr_pos):
    print(f"Finding closest waypoint from position x={x}, y={y}")
    min_dist = get_dist(x, y, arr_pos[0][0], arr_pos[0][1])
    min_index = 0
    
    for i, (wp_x, wp_y, _) in enumerate(arr_pos):
        dist = get_dist(x, y, wp_x, wp_y)
        if dist < min_dist:
            min_dist = dist
            min_index = i
    
    return min_index

async def go_through_path(orig, base, slam, wp_index, pos_arr):
    while True:
        next_index = (wp_index + 1) % len(pos_arr)
        if next_index == orig:
            break
            
        await asyncio.sleep(0.5)
        
        pos = await slam.get_position()
        curr_x = pos.x
        curr_y = pos.y
        
        # Check if we're still close to our current waypoint
        current_wp = pos_arr[wp_index]
        if get_dist(curr_x, curr_y, current_wp[0], current_wp[1]) > 100:
            # We've drifted too far, find closest waypoint
            wp_index = await find_waypoint(curr_x, curr_y, slam, pos_arr)
            next_index = (wp_index + 1) % len(pos_arr)
        
        # Move to next waypoint
        next_wp = pos_arr[next_index]
        await move_to_pos(base, slam, next_wp[0], next_wp[1], next_wp[2])
        wp_index = next_index

async def main():
    try:
        robot = await connect()
        print('Resources:', robot.resource_names)
        
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-2')
        motion = MotionClient.from_robot(robot, name="builtin")
        
        # Set initial power
        await base.set_power(
            linear=Vector3(x=0, y=1, z=0),
            angular=Vector3(x=0, y=0, z=0.75)
        )
        
        # Define waypoints (square path)
        waypoints = [
            [0, 0, 0],
            [400, 0, 90],
            [400, 400, 180],
            [0, 400, -90]
        ]
        
        # Get initial position and start navigation
        pos = await slam.get_position()
        wp_index = await closest_to_path(pos.x, pos.y, slam, waypoints)
        
        # Navigate through waypoints
        await go_through_path(wp_index, base, slam, wp_index, waypoints)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())