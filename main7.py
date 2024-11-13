

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

async def get_position(slam):
    position = await slam.get_position()
    return position

def getDist(currX, currY, wantX, wantY):
    return np.sqrt((wantX-currX)**2 + (wantY-currY)**2)

async def moveToPos(base, slam, x, y, theta):
    print("move to point call")
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print(f'x={currX}')
    print(f'y={currY}')
    print(f'theta={currTheta}')
    print(f'want x={x}')
    print(f'want y={y}')
    
    # Calculate initial angle to target
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    
    # Adjust rotation calculation to account for drift
    # Using -200 instead of -180 as in your working version
    toMove = (target_angle - currTheta + 180) % 360 - 200
    
    print(f'moving to angle: {target_angle}')
    print(f'rotation needed: {toMove}')
    
    # Perform rotation in smaller increments with position checks
    while abs(toMove) > 1:
        # Use smaller rotation steps
        rotation_step = toMove / 2
        if abs(rotation_step) > 30:  # Limit maximum rotation speed
            rotation_step = 30 * np.sign(rotation_step)
            
        await base.spin(rotation_step, 45)
        await asyncio.sleep(0.1)  # Small delay to allow position update
        
        # Update current position and recalculate needed rotation
        currPos = await slam.get_position()
        currTheta = currPos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180
    
    # Calculate distance to target
    dist = getDist(currX, currY, x, y)
    
    # Move in smaller segments to reduce drift
    segment_size = 200  # mm
    remaining_dist = dist
    
    while remaining_dist > 50:  # Continue until within 50mm of target
        move_dist = min(remaining_dist, segment_size)
        await base.move_straight(int(move_dist), 300)  # Reduced speed from 400 to 300
        
        # Update position and recalculate remaining distance
        currPos = await slam.get_position()
        currX = currPos.x
        currY = currPos.y
        remaining_dist = getDist(currX, currY, x, y)
        
        # Check if we need to correct heading
        if remaining_dist > 50:
            target_angle_rad = np.arctan2(y - currY, x - currX)
            target_angle = np.degrees(target_angle_rad)
            toMove = (target_angle - currTheta + 180) % 360 - 200  # Using -200 compensation
            if abs(toMove) > 5:  # Only correct if off by more than 5 degrees
                await base.spin(toMove/2, 45)

async def findWaypt(x, y, slam, arrPos):
    print("going to new position")
    print(f'currently at x = {x}')
    print(f'currently at y = {y}')
    minDist = getDist(x, y, arrPos[0][0], arrPos[0][1])
    minIndex = 0
    for i in range(len(arrPos)):
        wpX = arrPos[i][0]
        wpY = arrPos[i][1]
        dist = getDist(x, y, wpX, wpY)
        if dist < minDist:
            minDist = dist
            minIndex = i
    
    print(f'trying to go to: x= {arrPos[minIndex][0]}')
    print(f'trying to go to: y= {arrPos[minIndex][1]}')
    print(f'trying to go to: theta= {arrPos[minIndex][2]}')
    return minIndex

async def goThroughPath(orig, base, slam, wpIndex, posArr):
    next = wpIndex + 1
    while next != orig:
        next = 0
        if wpIndex + 1 < len(posArr):
            next = wpIndex + 1
        
        await asyncio.sleep(0.5)
        pos = await slam.get_position()
        currX = pos.x
        currY = pos.y
        
        # Find closest waypoint
        c = await findWaypt(currX, currY, slam, posArr)
        
        # Check if we're off course or too far from current waypoint
        if c != wpIndex or getDist(currX, currY, posArr[wpIndex][0], posArr[wpIndex][1]) > 200:
            print("NOT CLOSEST")
            print(c)
            await moveToPos(base, slam, posArr[c][0], posArr[c][1], posArr[c][2])
            wpIndex = c
            next = c + 1
        else:
            print("next wp")
            print(next)
            await moveToPos(base, slam, posArr[next][0], posArr[next][1], posArr[next][2])
            wpIndex += 1

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')
    
    # Initialize position and power
    internal_state = await slam.get_internal_state()
    await base.set_power(
        linear=Vector3(x=0, y=1, z=0),
        angular=Vector3(x=0, y=0, z=0.75)
    )
    
    # Get starting position
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    base_origin_x = x + 50
    base_origin_y = y + 50
    
    # Define waypoints (smaller square than original)
    wp = [
        [0, 0, 0],
        [500, 0, 90],    # Using 500mm instead of 600mm
        [500, 500, 180],
        [0, 500, -90]
    ]
    
    # Adjust waypoints relative to starting position
    for i in wp:
        i[0] += base_origin_x
        i[1] += base_origin_y
    print(wp)
    
    # Get initial position and start path following
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(f'currently at x={x}')
    print(f'currently at y={y}')
    print(f'currently at theta={theta}')
    
    wpIndex = await findWaypt(x, y, slam, wp)
    try:
        await goThroughPath(wpIndex, base, slam, wpIndex, wp)
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())