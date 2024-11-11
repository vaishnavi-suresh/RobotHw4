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

async def moveToPos(baseName, slamName, x, y, theta):
    toMove = Pose(x=x,y=y,theta=theta)
    movement = await move.move_on_map(baseName,toMove,slamName)

async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    move = MotionClient.from_robot(robot,name="builtin")
    baseName = base.get_resource_name('viam_base')
    slamName = slam.get_resource_name('slam-2')
    await moveToPos(baseName,slamName,0,0,0)

if __name__ == '__main__':
    asyncio.run(main())
