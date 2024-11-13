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
    try:
        robot = await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
        print("Successfully connected to the robot.")
        return robot
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")
        raise e


async def get_position(slam):
    try:
        position = await slam.get_position()
        print(f"Retrieved position -> x: {position.x}, y: {position.y}, theta: {position.theta}")
        return position
    except Exception as e:
        print(f"Error retrieving position: {e}")
        raise e


def getDist(currX, currY, wantX, wantY):
    distance = np.sqrt((wantX - currX)**2 + (wantY - currY)**2)
    print(f"Calculated distance from ({currX}, {currY}) to ({wantX}, {wantY}): {distance:.2f} units")
    return distance


async def findWaypt(x, y, slam, arrPos):
    print("\n--- Finding Closest Waypoint ---")
    print(f"Current Position -> x: {x}, y: {y}")
    
    minDist = getDist(x, y, arrPos[0][0], arrPos[0][1])
    minIndex = 0
    for i, waypoint in enumerate(arrPos):
        wpX, wpY, wpTheta = waypoint
        dist = getDist(x, y, wpX, wpY)
        if dist < minDist:
            minDist = dist
            minIndex = i
    
    closest_wp = arrPos[minIndex]
    print(f"Closest Waypoint Found -> Index: {minIndex}, x: {closest_wp[0]}, y: {closest_wp[1]}, theta: {closest_wp[2]}")
    print("--------------------------------\n")
    return minIndex


async def closestToPath(currX, currY, slam, arrPos):
    wpIndex = await findWaypt(currX, currY, slam, arrPos)
    return wpIndex


async def moveToPos(base, slam, x, y, theta):
    print("\n--- Initiating Movement to Target Position ---")
    try:
        currPos = await get_position(slam)
        currX = currPos.x
        currY = currPos.y
        currTheta = currPos.theta
        print(f"Current Position -> x: {currX}, y: {currY}, theta: {currTheta}")
        print(f"Target Position -> x: {x}, y: {y}, theta: {theta}")
        
        # Calculate the angle to the target
        target_angle_rad = np.arctan2(y - currY, x - currX)
        target_angle = np.degrees(target_angle_rad)
        toMove = (target_angle - currTheta + 180) % 360 - 180
        print(f"Calculated Target Angle: {target_angle:.2f} degrees")
        
        # Rotate to face the target
        while np.abs(toMove) > 1:
            spin_angle = toMove / 2  # Consider limiting the spin angle for stability
            print(f"Spinning by {spin_angle:.2f} degrees to adjust orientation.")
            await base.spin(spin_angle, 45)
            currPos = await get_position(slam)
            currTheta = currPos.theta
            toMove = (target_angle - currTheta + 180) % 360 - 180
            print(f"Adjusted Angle -> Current Theta: {currTheta:.2f}, Remaining to Move: {toMove:.2f} degrees")
        
        # Move straight to the target
        dist = getDist(currX, currY, x, y)
        print(f"Moving straight for {int(dist)} units at speed 400.")
        await base.move_straight(int(dist), 400)
        print("Movement command issued successfully.")
    except Exception as e:
        print(f"Error during movement: {e}")
        raise e
    print("--------------------------------------------\n")


async def goThroughPath(base, slam, wpIndex, posArr):
    print("\n--- Starting Path Navigation ---")
    while wpIndex < len(posArr):
        current_wp = posArr[wpIndex]
        print(f"\nNavigating to Waypoint {wpIndex}: {current_wp}")
        await moveToPos(base, slam, current_wp[0], current_wp[1], current_wp[2])

        # Get current position
        pos = await get_position(slam)
        currX = pos.x
        currY = pos.y
        distance = getDist(currX, currY, current_wp[0], current_wp[1])
        print(f"Distance to Waypoint {wpIndex}: {distance:.2f} units")

        # Wait until the rover is close to the waypoint
        while distance > 50:  # Threshold can be adjusted
            print(f"Still {distance:.2f} units away from Waypoint {wpIndex}. Re-adjusting...")
            await asyncio.sleep(1)  # Wait before re-checking
            pos = await get_position(slam)
            currX = pos.x
            currY = pos.y
            distance = getDist(currX, currY, current_wp[0], current_wp[1])

        print(f"Successfully reached Waypoint {wpIndex}.\n")

        # Perturbation Recovery: Use Euclidean distance
        if distance > 110:
            print("Significant deviation detected from Waypoint.")
            closest_wp = await findWaypt(currX, currY, slam, posArr)
            print(f"Navigating back to Closest Waypoint {closest_wp}: {posArr[closest_wp]}")
            await moveToPos(base, slam, posArr[closest_wp][0], posArr[closest_wp][1], posArr[closest_wp][2])
            await asyncio.sleep(0.5)
            wpIndex = closest_wp
        else:
            wpIndex += 1
    print("--- Completed Path Navigation ---\n")


async def main():
    try:
        robot = await connect()
        print('Resources:', robot.resource_names)

        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
        motion = MotionClient.from_robot(robot, name="builtin")

        # Get initial position
        pos = await get_position(slam)
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

        # Find the closest waypoint to start
        wpIndex = await closestToPath(x, y, slam, wp)
        print(f"Starting navigation from Waypoint index: {wpIndex}, Position: {wp[wpIndex]}")

        # Start navigating through waypoints
        await goThroughPath(base, slam, wpIndex, wp)

    except Exception as e:
        print(f"An error occurred in main: {e}")

    finally:
        await robot.close()
        print("Robot connection closed.")


if __name__ == '__main__':
    asyncio.run(main())
