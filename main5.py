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
    position = await slam.get_position()  # This depends on your specific SLAM client
    return position


def getDist(currX, currY, wantX, wantY):
    return np.sqrt((wantX - currX)**2 + (wantY - currY)**2)


async def findWaypt(x, y, slam, arrPos):
    print("Finding the closest waypoint to current position...")
    print(f'currently at x = {x}')
    print(f'currently at y = {y}')
    # Note: theta is retrieved but not used
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


async def closestToPath(currX, currY, slam, arrPos):
    wpIndex = await findWaypt(currX, currY, slam, arrPos)
    return wpIndex


async def moveToPos(base, slam, x, y, theta):
    print("move to point call")
    currPos = await get_position(slam)
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print(f'x={currX}')
    print(f'y={currY}')
    print(f'theta={currTheta}')
    print(f'want x={x}')
    print(f'want y={y}')
    
    # Calculate the angle to the target
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 - 180
    print(f'moving to angle: {target_angle} degrees')
    
    # Rotate to face the target
    while np.abs(toMove) > 1:
        spin_angle = toMove / 2  # Consider limiting the spin angle for stability
        await base.spin(spin_angle, 45)
        currPos = await get_position(slam)
        currTheta = currPos.theta
        toMove = (target_angle - currTheta + 180) % 360 - 180
        print(f'Adjusting Angle -> Current Theta: {currTheta}, Remaining to Move: {toMove} degrees')
    
    # Move straight to the target
    dist = getDist(currX, currY, x, y)
    print(f'Distance to Target: {dist} units')
    await base.move_straight(int(dist), 400)
    print("Movement command issued.")


async def goThroughPath(base, slam, wpIndex, posArr):
    while wpIndex < len(posArr):
        current_wp = posArr[wpIndex]
        print(f"Navigating to waypoint {wpIndex}: {current_wp}")
        await moveToPos(base, slam, current_wp[0], current_wp[1], current_wp[2])

        # Get current position
        pos = await get_position(slam)
        currX = pos.x
        currY = pos.y
        distance = getDist(currX, currY, current_wp[0], current_wp[1])
        print(f'Distance to waypoint {wpIndex}: {distance} units')

        # Wait until the rover is close to the waypoint
        while distance > 50:  # Threshold can be adjusted
            print(f"Still {distance} units away from waypoint {wpIndex}. Adjusting...")
            await asyncio.sleep(1)  # Wait before re-checking
            pos = await get_position(slam)
            currX = pos.x
            currY = pos.y
            distance = getDist(currX, currY, current_wp[0], current_wp[1])

        print(f"Reached waypoint {wpIndex}")

        # Perturbation Recovery: Use Euclidean distance
        if distance > 110:
            print("NOT CLOSEST")
            closest_wp = await findWaypt(currX, currY, slam, posArr)
            print(f"Closest waypoint index: {closest_wp}")
            await moveToPos(base, slam, posArr[closest_wp][0], posArr[closest_wp][1], posArr[closest_wp][2])
            await asyncio.sleep(0.5)
            wpIndex = closest_wp
        else:
            wpIndex += 1


async def main():
    try:
        robot = await connect()
        print('Resources:', robot.resource_names)

        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
        motion = MotionClient.from_robot(robot, name="builtin")
        pos = await slam.get_position()
        x = pos.x
        y = pos.y
        theta = pos.theta
        base_origin_x = x + 50
        base_origin_y = y + 50

        # Define waypoints (example: square path)
        wp = [
            [0, 0, 0],
            [600, 0, 90],
            [600, 600, 180],
            [0, 600, -90]
        ]
        for waypoint in wp:
            waypoint[0] += base_origin_x
            waypoint[1] += base_origin_y
        print("Adjusted Waypoints:", wp)

        # Remove redundant position retrieval
        # pos = await slam.get_position()
        # x = pos.x
        # y = pos.y
        # theta = pos.theta
        # print(f'currently at x={x}')
        # print(f'currently at y={y}')
        # print(f'currently at theta={theta}')

        # Find the closest waypoint to start
        wpIndex = await closestToPath(x, y, slam, wp)
        print(f"Starting navigation from waypoint index: {wpIndex}")

        # Start navigating through waypoints
        await goThroughPath(base, slam, wpIndex, wp)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        await robot.close()
        print("Robot connection closed.")


if __name__ == '__main__':
    asyncio.run(main())
