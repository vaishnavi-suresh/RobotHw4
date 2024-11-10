from viam.services.slam import SLAMClient

# Assume 'robot' is your connected RobotClient instance
slam_service = SLAMClient.from_robot(robot=robot, name="my_slam_service")

# Retrieve the internal state
internal_state_chunks = await slam_service.get_internal_state()

# Concatenate the byte chunks
internal_state_data = b''.join(internal_state_chunks)

# Save to a file
with open('map_internal_state.pbstream', 'wb') as file:
    file.write(internal_state_data)

