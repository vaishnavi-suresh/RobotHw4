# main7.py 
# bascically main2 without 5 discrete steps

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

async def closestToPath(currX, currY, slam, arrPos):
    wpIndex = await findWaypt(currX, currY, slam, arrPos)
    baseX = arrPos[wpIndex][0]
    baseY = arrPos[wpIndex][1]
    baseTheta = arrPos[wpIndex][2]
    return wpIndex

async def moveAngle(base, slam, toMove, target_angle):
    while np.abs(toMove) > 1:
        await base.spin(toMove/2, 45)
        currPos = await slam.get_position()
        currTheta = currPos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180

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
    
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 - 173
    print(f'moving to angle: {target_angle}')
    dist = getDist(currX, currY, x, y)
    
    await moveAngle(base, slam, toMove, target_angle)
    await base.move_straight(int(dist), 400)

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
    while True:  # Changed from while next != orig
        # Handle waypoint indexing
        if wpIndex >= len(posArr):
            wpIndex = 0
            next = 1
        
        # Check if we've completed a loop
        if next == orig:
            break

        pos = await slam.get_position()
        currX = pos.x
        currY = pos.y
        c = await findWaypt(currX, currY, slam, posArr)
        
        # Check if we're too far from current waypoint
        if np.abs(currX-posArr[wpIndex][0]) > 170 or np.abs(currY-posArr[wpIndex][1]) > 170:
            print("NOT CLOSEST")
            print(c)
            await moveToPos(base, slam, posArr[c][0], posArr[c][1], posArr[c][2])
            await asyncio.sleep(0.5)
            
            wpIndex = c
            next = c + 1
        else:
            print("next wp")
            print(next)
            await moveToPos(base, slam, posArr[next][0], posArr[next][1], posArr[next][2])
            await asyncio.sleep(0.5)
            wpIndex += 1
            next = wpIndex + 1

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')
    motion = MotionClient.from_robot(robot, name="builtin")
    
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
    base_origin_x = x
    base_origin_y = y

    # Define square path waypoints
    wp = [
        [0, 0, 0],
        [600, 0, 90],
        [600, 600, 180],
        [0, 600, -90]
    ]
    print(wp)

    # Adjust waypoints relative to starting position
    for i in wp:
        i[0] += base_origin_x
        i[1] += base_origin_y

    # Get initial position
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(f'currently at x={x}')
    print(f'currently at y={y}')
    print(f'currently at theta={theta}')
    
    # Move to first waypoint
    await moveToPos(base, slam, wp[0][0], wp[0][1], wp[0][2])
    
    # Start path following
    wpIndex = await closestToPath(x, y, slam, wp)
    try:
        await goThroughPath(wpIndex, base, slam, wpIndex, wp)
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())