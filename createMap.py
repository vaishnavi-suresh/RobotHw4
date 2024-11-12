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

async def moveToPos(base, slam, x,y,theta):
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print (f'x={currX}')
    print (f'y={currY}')
    print (f'theta={currTheta}')
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    toMove = (target_angle - currTheta + 180) % 360 -180
    print(f'moving to angle: {toMove}')
    dist = getDist(currX,currY,x,y)

    while np.abs(target_angle-currTheta)>5:
        if currTheta>target_angle:
            await base.spin(10,45)
        else:
            await base.spin(-10,45)
        currPos = await slam.get_position()
        currTheta = currPos.theta
        
    await base.move_straight(int(dist),100)

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    internal_state = await slam.get_internal_state()

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
