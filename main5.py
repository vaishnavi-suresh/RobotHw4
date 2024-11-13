# main2.py
import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
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
    position = await slam.get_position()
    return position

def getDist(currX, currY, wantX, wantY):
    return np.sqrt((wantX - currX)**2 + (wantY - currY)**2)

async def findWaypt(slam, arrPos):
    print("Finding the closest waypoint to current position...")
    pos = await get_position(slam)
    x, y, theta = pos.x, pos.y, pos.theta
    print(f'Current Position -> x: {x}, y: {y}, theta: {theta}')

    minDist = getDist(x, y, arrPos[0][0], arrPos[0][1])
    minIndex = 0
    for i in range(1, len(arrPos)):
        wpX, wpY = arrPos[i][0], arrPos[i][1]
        dist = getDist(x, y, wpX, wpY)
        if dist < minDist:
            minDist = dist
            minIndex = i

    print(f'Closest waypoint index: {minIndex}')
    print(f'Waypoint coordinates -> x: {arrPos[minIndex][0]}, y: {arrPos[minIndex][1]}, theta: {arrPos[minIndex][2]}')
    return minIndex

async def moveToPos(base, slam, x, y, theta):
    print("Initiating movement to target position...")
    pos = await get_position(slam)
    currX, currY, currTheta = pos.x, pos.y, pos.theta
    print(f'Current Position -> x: {currX}, y: {currY}, theta: {currTheta}')
    print(f'Target Position -> x: {x}, y: {y}, theta: {theta}')

    # Calculate the angle to the target
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 - 180
    print(f'Target Angle: {target_angle} degrees')
    print(f'Angle to Move: {toMove} degrees')

    # Rotate to face the target
    while np.abs(toMove) > 1:
        spin_angle = np.clip(toMove, -30, 30)  # Limit spin rate for stability
        await base.spin(spin_angle, 45)
        pos = await get_position(slam)
        currTheta = pos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180
        print(f'Adjusting Angle -> Current Theta: {currTheta}, Remaining to Move: {toMove}')

    # Move straight to the target
    dist = getDist(currX, currY, x, y)
    print(f'Distance to Target: {dist} units')
    await base.move_straight(int(dist), 400)
    print("Movement command issued.")

async def monitor_and_recover(base, slam, posArr, current_wp_index):
    while True:
        pos = await get_position(slam)
        closest_wp_index = await findWaypt(slam, posArr)
        dist = getDist(pos.x, pos.y, posArr[closest_wp_index][0], posArr[closest_wp_index][1])
        if dist > 150:  # Threshold for recovery
            print("Significant deviation detected. Initiating recovery...")
            await moveToPos(base, slam, posArr[closest_wp_index][0], posArr[closest_wp_index][1], posArr[closest_wp_index][2])
            current_wp_index = closest_wp_index
        await asyncio.sleep(1)  # Check every second

async def goThroughPath(base, slam, posArr):
    wpIndex = await findWaypt(slam, posArr)
    total_waypoints = len(posArr)
    
    # Start the monitor task
    monitor_task = asyncio.create_task(monitor_and_recover(base, slam, posArr, wpIndex))
    
    for _ in range(total_waypoints):
        print(f"Navigating to waypoint {wpIndex}: {posArr[wpIndex]}")
        await moveToPos(base, slam, posArr[wpIndex][0], posArr[wpIndex][1], posArr[wpIndex][2])
        
        # Wait until the rover is close to the waypoint
        while True:
            pos = await get_position(slam)
            dist = getDist(pos.x, pos.y, posArr[wpIndex][0], posArr[wpIndex][1])
            if dist < 50:  # Threshold can be adjusted
                print(f"Reached waypoint {wpIndex}")
                break
            await asyncio.sleep(0.5)
        
        # Move to next waypoint
        wpIndex = (wpIndex + 1) % total_waypoints
    
    # Cancel the monitor task once done
    monitor_task.cancel()
    try:
        await monitor_task
    except asyncio.CancelledError:
        pass

async def main():
    robot = await connect()
    print('Connected to Robot. Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    motion = MotionClient.from_robot(robot, name="builtin")

    # Get initial position
    pos = await slam.get_position()
    x, y, theta = pos.x, pos.y, pos.theta
    print(f'Initial Position -> x: {x}, y: {y}, theta: {theta}')

    # Define waypoints (example: square path)
    wp = [
        [0, 0, 0],
        [600, 0, 90],
        [600, 600, 180],
        [0, 600, -90]
    ]

    # Adjust waypoints based on initial position
    base_origin_x = x + 50
    base_origin_y = y + 50
    for waypoint in wp:
        waypoint[0] += base_origin_x
        waypoint[1] += base_origin_y
    print("Adjusted Waypoints:", wp)

    # Start navigating through waypoints
    await goThroughPath(base, slam, wp)

    await robot.close()
    print("Robot connection closed.")

if __name__ == '__main__':
    asyncio.run(main())
