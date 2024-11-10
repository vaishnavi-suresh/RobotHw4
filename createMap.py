import asyncio
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient

# Define a function to connect to the robot
async def connect_to_robot():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

# Define the function to save the internal state
async def save_internal_state():
    # Connect to the robot
    robot = await connect_to_robot()

    # Create the SLAM service client
    slam_service = SLAMClient.from_robot(robot=robot, name="slam-1")

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
