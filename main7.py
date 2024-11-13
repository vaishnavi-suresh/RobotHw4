

# main7.py


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

async def get_current_position(slam):
    pos = await slam.get_position()
    return pos.x, pos.y, pos.theta

def calculate_distance(x1, y1, x2, y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

async def rotate_to_target(base, slam, target_x, target_y, current_x, current_y, current_theta):
    # Calculate target angle in degrees
    target_angle_rad = np.arctan2(target_y - current_y, target_x - current_x)
    target_angle_deg = np.degrees(target_angle_rad)
    
    # Calculate the shortest rotation needed
    rotation_needed = (target_angle_deg - current_theta + 180) % 360 - 180
    
    # Rotate in smaller increments for smoother motion
    while abs(rotation_needed) > 2:  # 2 degree tolerance
        rotation_step = np.clip(rotation_needed, -45, 45)
        await base.spin(rotation_step, 45)
        current_x, current_y, current_theta = await get_current_position(slam)
        target_angle_rad = np.arctan2(target_y - current_y, target_x - current_x)
        target_angle_deg = np.degrees(target_angle_rad)
        rotation_needed = (target_angle_deg - current_theta + 180) % 360 - 180

async def move_to_waypoint(base, slam, target_x, target_y, target_theta, threshold_distance=50):
    current_x, current_y, current_theta = await get_current_position(slam)
    
    # First rotate to face the target
    await rotate_to_target(base, slam, target_x, target_y, current_x, current_y, current_theta)
    
    # Move straight to target
    distance = calculate_distance(current_x, current_y, target_x, target_y)
    if distance > threshold_distance:
        await base.move_straight(int(distance), 400)
    
    # Final rotation to achieve target orientation
    current_x, current_y, current_theta = await get_current_position(slam)
    final_rotation = (target_theta - current_theta + 180) % 360 - 180
    if abs(final_rotation) > 2:
        await base.spin(final_rotation, 45)

async def find_closest_waypoint(slam, waypoints):
    current_x, current_y, _ = await get_current_position(slam)
    min_dist = float('inf')
    closest_idx = 0
    
    for idx, (wx, wy, _) in enumerate(waypoints):
        dist = calculate_distance(current_x, current_y, wx, wy)
        if dist < min_dist:
            min_dist = dist
            closest_idx = idx
    
    return closest_idx

async def follow_course(base, slam, waypoints, threshold_distance=50):
    current_waypoint_idx = 0
    
    while True:
        current_x, current_y, _ = await get_current_position(slam)
        
        # Check if we're off course and need to recover
        closest_idx = await find_closest_waypoint(slam, waypoints)
        current_waypoint = waypoints[closest_idx]
        
        # If we're too far from our current waypoint, go to the closest one
        if calculate_distance(current_x, current_y, 
                            current_waypoint[0], current_waypoint[1]) > threshold_distance:
            print(f"Off course! Recovering to waypoint {closest_idx}")
            current_waypoint_idx = closest_idx
        
        # Move to current waypoint
        target = waypoints[current_waypoint_idx]
        print(f"Moving to waypoint {current_waypoint_idx}: {target}")
        await move_to_waypoint(base, slam, target[0], target[1], target[2], threshold_distance)
        
        # Check if we've reached the waypoint
        current_x, current_y, _ = await get_current_position(slam)
        if calculate_distance(current_x, current_y, target[0], target[1]) < threshold_distance:
            current_waypoint_idx = (current_waypoint_idx + 1) % len(waypoints)
        
        await asyncio.sleep(0.1)

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)
    
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')
    
    # Get initial position
    pos = await slam.get_position()
    base_origin_x = pos.x + 50
    base_origin_y = pos.y + 50
    
    # Define square course waypoints [x, y, theta]
    waypoints = [
        [0, 0, 0],
        [600, 0, 90],
        [600, 600, 180],
        [0, 600, -90]
    ]
    
    # Adjust waypoints relative to starting position
    for wp in waypoints:
        wp[0] += base_origin_x
        wp[1] += base_origin_y
    
    # Start following course
    await follow_course(base, slam, waypoints)
    
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
