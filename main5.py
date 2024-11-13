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
    while np.abs(toMove) > 1:
        await base.spin(toMove/2, 45)
        currPos = await slam.get_position()
        currTheta = currPos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180
        
    dist = getDist(currX, currY, x, y)
    print(f'Moving forward: {dist:.1f}mm')
    await base.move_straight(int(dist), 400)

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

async def goThroughPath(base, slam, wpIndex, posArr):
    total_waypoints = len(posArr)
    visited_count = 0
    
    while visited_count < total_waypoints:
        print(f"\nProcessing waypoint {wpIndex} of {total_waypoints-1}")
        next_index = (wpIndex + 1) % total_waypoints
        
        # Get current position
        pos = await get_position(slam)
        currX = pos.x
        currY = pos.y
        
        # Distance to current waypoint
        dist_to_current = getDist(currX, currY, posArr[wpIndex][0], posArr[wpIndex][1])
        print(f"Distance to current waypoint: {dist_to_current:.1f}mm")
        
        if dist_to_current > 120:
            print("Too far from waypoint, finding closest point")
            new_index = await findWaypt(base, slam, posArr)
            if new_index != wpIndex:
                wpIndex = new_index
                continue
                
        # Move to next waypoint
        print(f"Moving to waypoint {next_index}")
        await moveToPos(base, slam, posArr[next_index][0], posArr[next_index][1], posArr[next_index][2])
        
        wpIndex = next_index
        visited_count += 1
        print(f"Completed waypoint {wpIndex}, visited {visited_count} of {total_waypoints}")

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

    # Define square waypoints
    wp = [[0,0,0],
          [600,0,90],
          [600,600,180],
          [0,600,-90]]
          
    # Adjust waypoints relative to starting position
    for i in wp:
        i[0] += base_origin_x
        i[1] += base_origin_y

    print("\nSquare corners:")
    for i, point in enumerate(wp):
        print(f"Corner {i}: x={point[0]:.1f}, y={point[1]:.1f}, θ={point[2]}°")

    # Find starting waypoint and begin navigation
    wpIndex = await findWaypt(base, slam, wp)
    print(f"\nStarting navigation from waypoint {wpIndex}")
    await goThroughPath(base, slam, wpIndex, wp)
    
    print("\nNavigation completed")
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())

