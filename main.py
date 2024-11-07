import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)


async def get_position(slam):
    position = await slam.get_position()
    return position.x, position.y
"""
async def moveInSquare(base, slam, base_coords):
    for _ in range(4):
        await base.move_straight(velocity=500, distance=500)
        print("move straight")
        await base.spin(velocity=100, angle=90)
        print("spin 90 degrees")

        # Retrieve and print current position
        x, y = await get_position(slam)
        print(f"Current position: ({x}, {y})")
        
        # If far from base, calculate and adjust path to return
        if abs(x - base_coords[0]) > threshold or abs(y - base_coords[1]) > threshold:
            print("Rover is off-course, recalculating path to base.")
            # Here, implement logic to guide the rover back to base_coords
"""
async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    roverBase = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-1')  # Initialize SLAM

    # Set the base starting coordinates
    x,y = await get_position(slam)
    print(x)
    print(y)
    
    #base_coords = (x0, y0)
    #print(f"Base coordinates: ({x0}, {y0})")
"""
    # Move in a square and maintain base coordinates
    await moveInSquare(roverBase, slam, base_coords)

    await robot.close()
"""

if __name__ == '__main__':
    asyncio.run(main())