import asyncio
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient

# Define a function to connect to the robot
async def connect_to_robot():
    # Replace with your robot's address and secret
    robot = await RobotClient.at_address("your_robot_address", "your_robot_secret")
    return robot

# Define the function to save the internal state
async def save_internal_state():
    # Connect to the robot
    robot = await connect_to_robot()

    # Create the SLAM service client
    slam_service = SLAMClient.from_robot(robot=robot, name="my_slam_service")

    # Retrieve the internal state
    internal_state_chunks = await slam_service.get_internal_state()

    # Concatenate the byte chunks
    internal_state_data = b''.join(internal_state_chunks)

    # Save to a file
    with open('map_internal_state.pbstream', 'wb') as file:
        file.write(internal_state_data)

    # Close the robot connection
    await robot.close()

# Run the async function
asyncio.run(save_internal_state())
