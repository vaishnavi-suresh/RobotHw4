import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
from viam.proto.common import Pose
import numpy as np

# Initialize Viam client to communicate with the robot
async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

# Define your target coordinates (x_target, y_target) in the robot’s coordinate system
x_target, y_target = 5.0, 5.0  # Replace with desired coordinates

# Function to calculate distance and direction
def calculate_distance_and_angle(current_x, current_y, target_x, target_y):
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    distance = (delta_x**2 + delta_y**2) ** 0.5
    angle = np.arctan2(delta_y, delta_x)  # Angle in radians
    return distance, angle

# Function to move robot to target
async def move_to_target(x_target, y_target, tolerance=0.1):
    while True:
        robot = await connect()

        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
        internal_state = await slam.get_internal_state()

        baseName = base.get_resource_name('viam_base')
        slamName = slam.get_resource_name('slam-2')
        # Request the internal state


        # Extract current position from internal state (mock example, adjust according to actual data structure)
        current_x = internal_state.position.x
        current_y = internal_state.position.y

        # Calculate distance and angle to target
        distance, angle = calculate_distance_and_angle(current_x, current_y, x_target, y_target)

        # Check if we are within tolerance
        if distance <= tolerance:
            print("Arrived at target!")
            break

        # Send movement commands to robot
        # Here you would issue commands for rotation and translation to align and move toward target
        # Example: Rotate to angle, then move forward by distance (adjust to your API)
        await MotionClient.robot.move(distance, angle)

        # Small delay to prevent overload
        await asyncio.sleep(0.5)

# Call the move function
await move_to_target(x_target, y_target)

"""# main2.py
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
    toMove = (target_angle - currTheta + 180) % 360
    print(f'moving to angle: {toMove}')
    dist = getDist(currX,currY,x,y)

    while dist>300:
        await base.spin(toMove,50)
        currPos = await slam.get_position()
        currX = currPos.x
        currY = currPos.y
        currTheta = currPos.theta
        target_angle_rad = np.arctan2(y - currY, x - currX)
        target_angle = np.degrees(target_angle_rad)
        toMove = (target_angle - currTheta + 180) % 360
        await base.move_straight(int(dist),50)

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
"""