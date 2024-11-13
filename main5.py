# main5.py

import asyncio
import numpy as np
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient

async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

async def get_position(slam):
    return await slam.get_position()

def get_distance(curr_x, curr_y, want_x, want_y):
    return np.sqrt((want_x - curr_x)**2 + (want_y - curr_y)**2)

async def move_to_position(base, slam, x, y, theta, tolerance=50):
    while True:
        curr_pos = await slam.get_position()
        curr_x, curr_y, curr_theta = curr_pos.x, curr_pos.y, curr_pos.theta
        
        distance = get_distance(curr_x, curr_y, x, y)
        if distance < tolerance:
            break
        
        target_angle_rad = np.arctan2(y - curr_y, x - curr_x)
        target_angle = np.degrees(target_angle_rad)
        angle_to_turn = (target_angle - curr_theta + 180) % 360 - 180
        
        # Adjust rotation speed based on the angle difference
        rotation_speed = min(45, max(10, abs(angle_to_turn)))
        await base.spin(angle_to_turn, rotation_speed)
        
        # Move forward
        move_distance = min(distance, 200)  # Move in smaller increments
        await base.move_straight(int(move_distance), 100)
        
        await asyncio.sleep(0.1)  # Small delay to allow for position update

async def follow_path(base, slam, waypoints):
    for wp in waypoints:
        await move_to_position(base, slam, wp[0], wp[1], wp[2])
    print("Path completed")

async def recover_and_resume(base, slam, waypoints):
    current_pos = await slam.get_position()
    closest_wp_index = min(range(len(waypoints)), 
                           key=lambda i: get_distance(current_pos.x, current_pos.y, 
                                                      waypoints[i][0], waypoints[i][1]))
    print(f"Recovering to waypoint {closest_wp_index}")
    await follow_path(base, slam, waypoints[closest_wp_index:])

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')
    
    # Define the course (square in this case)
    initial_pos = await slam.get_position()
    side_length = 1000  # 1 meter
    waypoints = [
        [initial_pos.x, initial_pos.y, 0],
        [initial_pos.x + side_length, initial_pos.y, 90],
        [initial_pos.x + side_length, initial_pos.y + side_length, 180],
        [initial_pos.x, initial_pos.y + side_length, -90],
        [initial_pos.x, initial_pos.y, 0]  # Return to start
    ]

    print("Starting course following")
    await follow_path(base, slam, waypoints)
    
    print("Simulating perturbation...")
    # Simulate picking up and putting down the rover
    await asyncio.sleep(5)  # Wait for 5 seconds to simulate displacement
    
    print("Recovering and resuming course")
    await recover_and_resume(base, slam, waypoints)

    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())