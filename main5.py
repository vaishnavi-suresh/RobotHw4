# main5.py
import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
import scipy
import numpy as np

async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

async def get_position(slam):
    position = await slam.get_position()
    print(f"Current position: x={position.x:.1f}mm, y={position.y:.1f}mm, θ={position.theta:.1f}°")
    return position

def getDist(currX, currY, wantX, wantY):
    dist = np.sqrt((wantX-currX)**2 + (wantY-currY)**2)
    print(f"Distance calculation: {dist:.1f}mm")
    return dist

async def moveToPos(base, slam, x, y, theta):
    print("\nMove to point call")
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print(f'Current: x={currX:.1f}, y={currY:.1f}, θ={currTheta:.1f}°')
    print(f'Target: x={x:.1f}, y={y:.1f}, θ={theta:.1f}°')
    
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 - 180
    
    print(f'Initial rotation needed: {toMove:.1f}°')
    while np.abs(toMove) > 2:  # Increased threshold from 1 to 2
        await base.spin(toMove/2, 45)
        currPos = await slam.get_position()
        currTheta = currPos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180
        
    dist = getDist(currX, currY, x, y)
    print(f'Moving forward: {dist:.1f}mm')
    
    # Move in smaller steps if distance is large
    max_step = 300  # Maximum step size
    remaining_dist = dist
    
    while remaining_dist > 0:
        step = min(remaining_dist, max_step)
        await base.move_straight(int(step), 300)  # Reduced speed from 400 to 300
        await asyncio.sleep(0.5)  # Small pause between movements
        
        # Update position and remaining distance
        currPos = await slam.get_position()
        remaining_dist = getDist(currPos.x, currPos.y, x, y)
        if remaining_dist < 100:  # If close enough, consider it reached
            break

async def findWaypt(base, slam, arrPos):
    print("\nFinding closest waypoint...")
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    
    minDist = float('inf')
    minIndex = 0
    
    for i, wp in enumerate(arrPos):
        dist = getDist(x, y, wp[0], wp[1])
        print(f"Distance to waypoint {i}: {dist:.1f}mm")
        if dist < minDist:
            minDist = dist
            minIndex = i
            print(f"New closest waypoint: {i}")
    
    print(f'Selected waypoint {minIndex}: x={arrPos[minIndex][0]:.1f}, y={arrPos[minIndex][1]:.1f}, θ={arrPos[minIndex][2]:.1f}°')
    return minIndex

async def goThroughPath(orig, base, slam, wpIndex, posArr):
    visited = set()  # Track visited waypoints
    max_attempts = 12  # Maximum attempts
    attempts = 0
    
    while len(visited) < len(posArr) and attempts < max_attempts:
        attempts += 1
        print(f"\nNavigation attempt {attempts}/{max_attempts}")
        print(f"Visited waypoints: {visited}")
        
        next_index = (wpIndex + 1) % len(posArr)
        pos = await get_position(slam)
        currX = pos.x
        currY = pos.y
        
        # Check distance to current waypoint
        curr_dist = getDist(currX, currY, posArr[wpIndex][0], posArr[wpIndex][1])
        print(f"Distance to current waypoint {wpIndex}: {curr_dist:.1f}mm")
        
        if curr_dist > 150:  # Increased threshold from 120 to 150
            print(f"Too far from waypoint {wpIndex}, recalculating...")
            new_index = await findWaypt(base, slam, posArr)
            await moveToPos(base, slam, posArr[new_index][0], posArr[new_index][1], posArr[new_index][2])
            wpIndex = new_index
        else:
            print(f"Moving to waypoint {next_index}")
            await moveToPos(base, slam, posArr[next_index][0], posArr[next_index][1], posArr[next_index][2])
            visited.add(wpIndex)
            wpIndex = next_index
            
        # If we're close to the final waypoint, add it to visited
        if curr_dist < 150:
            visited.add(wpIndex)
            
    if len(visited) == len(posArr):
        print("Successfully completed the square!")
    else:
        print(f"Navigation ended after {attempts} attempts. Visited {len(visited)} of {len(posArr)} waypoints")

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')
    motion = MotionClient.from_robot(robot, name="builtin")
    
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    theta = pos.theta
    
    base_origin_x = x + 50
    base_origin_y = y + 50

    # Define square waypoints (reduced size for better reliability)
    wp = [[0,0,0],
          [500,0,90],    # Changed from 600
          [500,500,180], # Changed from 600
          [0,500,-90]]   # Changed from 600
          
    # Adjust waypoints relative to starting position
    for i in wp:
        i[0] += base_origin_x
        i[1] += base_origin_y

    print("\nSquare corners:")
    for i, point in enumerate(wp):
        print(f"Corner {i}: x={point[0]:.1f}, y={point[1]:.1f}, θ={point[2]}°")

    wpIndex = await findWaypt(base, slam, wp)
    print(f"\nStarting navigation from waypoint {wpIndex}")
    await goThroughPath(wpIndex, base, slam, wpIndex, wp)
    
    print("\nNavigation completed")
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
