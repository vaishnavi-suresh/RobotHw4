# main2.py
import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
from viam.proto.common import Pose
import numpy as np
async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
def getDist (currX, currY, wantX, wantY):
    return np.sqrt((wantX-currX)**2+(wantY-currY)**2)
async def moveToPos(base, slam, x, y, theta):
    # Retrieve current position from SLAM
    currPos = await slam.get_position()
    currX, currY, currTheta = currPos.x, currPos.y, currPos.theta

    # Print current position for debugging
    print(f'Initial position: x={currX}, y={currY}, theta={currTheta}')

    # Step 1: Calculate the angle to face the target position
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 - 180  # Normalize to [-180, 180]

    print(f"Calculated angle to move: {toMove} degrees")

    # Step 2: Rotate to face the target position
    await base.spin(toMove, 45)

    # Step 3: Move straight to the target position
    dist = getDist(currX, currY, x, y)
    await base.move_straight(int(dist), 50)

    # Step 4: Recheck the current position and orientation
    currPos = await slam.get_position()
    currX, currY, currTheta = currPos.x, currPos.y, currPos.theta
    print(f'Position after movement: x={currX}, y={currY}, theta={currTheta}')

    # Step 5: Rotate to the final target orientation
    final_angle_diff = (theta - currTheta + 180) % 360 - 180
    print(f"Rotating to final orientation by {final_angle_diff} degrees")
    await base.spin(final_angle_diff, 45)

    # Final check of position and orientation
    currPos = await slam.get_position()
    currX, currY, currTheta = currPos.x, currPos.y, currPos.theta
    print(f'Final position: x={currX}, y={currY}, theta={currTheta}')

"""


async def moveToPos(base, slam, x,y,theta):
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print (f'x={currX}')
    print (f'y={currY}')
    print (f'theta={currTheta}')
    toMove = np.degrees(np.arctan2((y-currY),(x-currX)))-currTheta
    if toMove >180:
        toMove = 180-toMove
    print(f'moving to angle: {toMove}')
    dist = getDist(currX,currY,x,y)
    await base.spin(toMove,45)

    await base.move_straight(int(dist),50)

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    baseName = base.get_resource_name('viam_base')
    slamName = slam.get_resource_name('slam-2')
    await moveToPos(base,slam,0,0,0)
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print (f'x={currX}')
    print (f'y={currY}')
    print (f'theta={currTheta}')


if __name__ == '__main__':
    asyncio.run(main())
"""