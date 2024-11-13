import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
import scipy
from scipy.spatial import distance
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
    curr = (currX, currY)
    want = (wantX, wantY)
    return np.sqrt((wantX - currX)**2 + (wantY - currY)**2)

async def closestToPath(base, slam, arrPos):
    wpIndex = await findWaypt(base, slam, arrPos)
    baseX = arrPos[wpIndex][0]
    baseY = arrPos[wpIndex][1]
    baseTheta = arrPos[wpIndex][2]
    await moveToPos(base, slam, baseX, baseY, baseTheta)
    return wpIndex

def getAngle(x, y):
    arctan = np.arctan2(y, x)
    return arctan

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
    toMove = (target_angle - currTheta + 180) % 360 - 180
    print(f'moving to angle: {toMove}')
    dist = getDist(currX, currY, x, y)
    await base.spin(toMove, 45)
    await base.move_straight(int(dist), 100)

async def findWaypt(base, slam, arrPos):
    print("going to new position")
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(f'currently at x = {x}')
    print(f'currently at y = {y}')
    print(f'currently at theta = {theta}')
    minDist = np.sqrt((x - arrPos[0][0])**2 + (y - arrPos[0][1])**2)
    minIndex = 0
    for i, wp in enumerate(arrPos):
        wpX = arrPos[i][0]
        wpY = arrPos[i][1]
        wpTheta = arrPos[i][2]
        dist = np.sqrt((x - wpX)**2 + (y - wpY)**2)
        if dist < minDist:
            minDist = dist
            minIndex = i

    print(f'trying to go to: x= {arrPos[minIndex][0]}')
    print(f'trying to go to: y= {arrPos[minIndex][1]}')
    print(f'trying to go to: theta= {arrPos[minIndex][2]}')
    return minIndex

async def goThroughPath(base, slam, wpIndex, posArr):
    while wpIndex < len(posArr):
        next_wp = wpIndex + 1
        if next_wp >= len(posArr):
            next_wp = 0  # Optionally loop back to the first waypoint
        c = await closestToPath(base, slam, posArr)
        pos = await slam.get_position()
        currX = pos.x
        currY = pos.y
        if getDist(currX, currY, posArr[wpIndex][0], posArr[wpIndex][1]) > 300:
            print("NOT CLOSEST")
            await moveToPos(base, slam, posArr[c][0], posArr[c][1], posArr[c][2])
            wpIndex = c
        else:
            print("next wp")
            print(next_wp)
            await moveToPos(base, slam, posArr[next_wp][0], posArr[next_wp][1], posArr[next_wp][2])
            wpIndex = next_wp

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    motion = MotionClient.from_robot(robot, name="builtin")
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    base_origin_x = x + 200
    base_origin_y = y + 200

    # Get a set of waypoints to track and populate them
    wp = [
        [0, 0, 0],
        [1000, 0, 90],
        [1000, 1000, 180],
        [0, 1000, -90]
    ]
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

    wpIndex = await closestToPath(base, slam, wp)

    await goThroughPath(base, slam, wpIndex, wp)

    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
