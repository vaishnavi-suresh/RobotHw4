
# import asyncio
# from viam.components.base import Base, Vector3
# from viam.robot.client import RobotClient
# from viam.rpc.dial import Credentials, DialOptions
# from viam.services.slam import SLAMClient
# from viam.services.motion import MotionClient
# # import scipy  # Removed unused import

# import numpy as np

# async def connect():
#     opts = RobotClient.Options.with_api_key(
#         api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
#         api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
#     )
#     return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

# async def get_position(slam):
#     position = await slam.get_position()  # This depends on your specific SLAM client
#     return position

# def getDist(currX, currY, wantX, wantY):
#     return np.sqrt((wantX - currX)**2 + (wantY - currY)**2)

# async def closestToPath(currX, currY, slam, arrPos):
#     wpIndex = await findWaypt(currX, currY, slam, arrPos)
#     return wpIndex

# async def moveAngle(base, slam, toMove, target_angle):
#     max_spin_attempts = 10  # Prevent infinite spinning
#     spin_attempts = 0
#     while np.abs(toMove) > 1 and spin_attempts < max_spin_attempts:
#         spin_direction = toMove / 2  # Spin half the required angle for stability
#         print(f'Spining by {spin_direction:.2f} degrees.')
#         await base.spin(spin_direction, 45)
#         currPos = await slam.get_position()
#         currTheta = currPos.theta
#         toMove = (target_angle - currTheta + 180) % 360 - 180
#         print(f'Adjusted angle: {toMove:.2f} degrees.')
#         spin_attempts += 1
#     if spin_attempts >= max_spin_attempts:
#         print("Max spin attempts reached. Proceeding with current orientation.")

# async def moveToPos(base, slam, x, y, theta):
#     print("move to point call")
#     currPos = await slam.get_position()
#     currX = currPos.x
#     currY = currPos.y
#     currTheta = currPos.theta
#     print(f'x={currX}')
#     print(f'y={currY}')
#     print(f'theta={currTheta}')
#     print(f'want x={x}')
#     print(f'want y={y}')
    
#     # Calculate target angle
#     target_angle_rad = np.arctan2(y - currY, x - currX)
#     target_angle = np.degrees(target_angle_rad)
#     toMove = (target_angle - currTheta + 180) % 360 - 180 
#     print(f'moving to angle: {target_angle:.2f} degrees.')
#     dist = getDist(currX, currY, x, y)
#     print(f'Distance to target: {dist:.2f} units.')
    
#     # Spin to target angle
#     await moveAngle(base, slam, toMove, target_angle)
    
#     # Move straight to target position
#     print(f'Moving straight for {int(dist)} units at speed 400.')
#     await base.move_straight(int(dist), 400)
    
#     # Optionally, verify if the rover reached the position
#     await asyncio.sleep(1)  # Wait for movement to complete
#     currPos = await slam.get_position()
#     new_dist = getDist(currPos.x, currPos.y, x, y)
#     print(f'Post-move distance to target: {new_dist:.2f} units.')
#     if new_dist > 50:
#         print("Warning: Rover did not reach the target position accurately.")

# async def goThroughPath(base, slam, wpIndex, posArr):
#     iterations = 0
#     max_iterations = 20  # Safety limit to prevent infinite loops
    
#     while iterations < max_iterations and wpIndex < len(posArr):
#         next_wp = wpIndex + 1
#         if next_wp >= len(posArr):
#             next_wp = 0  # Loop back to the first waypoint
    
#         await asyncio.sleep(0.5)  # Brief pause between actions
    
#         pos = await slam.get_position()
#         currX = pos.x
#         currY = pos.y
#         c = await closestToPath(currX, currY, slam, posArr)
        
#         # More robust position check using Euclidean distance
#         distance = getDist(currX, currY, posArr[wpIndex][0], posArr[wpIndex][1])
#         print(f'Current distance to waypoint {wpIndex}: {distance:.2f} units.')
#         if distance > 200:
#             print("NOT CLOSEST")
#             print(f'Closest waypoint index: {c}')
#             await moveToPos(base, slam, posArr[c][0], posArr[c][1], posArr[c][2])
#             wpIndex = c
#             next_wp = wpIndex + 1
#             if next_wp >= len(posArr):
#                 next_wp = 0
#         else:
#             print("Next waypoint")
#             print(f'Next waypoint index: {next_wp}')
#             await moveToPos(base, slam, posArr[next_wp][0], posArr[next_wp][1], posArr[next_wp][2])
#             wpIndex = next_wp
        
#         iterations += 1
    
#     if iterations >= max_iterations:
#         print("Maximum iterations reached - stopping for safety")

# async def findWaypt(x, y, slam, arrPos):
#     print("going to new position")
   
#     print(f'currently at x = {x}')
#     print(f'currently at y = {y}')
#     minDist = getDist(x, y, arrPos[0][0], arrPos[0][1])
#     minIndex = 0
#     for i in range(len(arrPos)):
#         wpX = arrPos[i][0]
#         wpY = arrPos[i][1]
#         dist = getDist(x, y, wpX, wpY)
#         if dist < minDist:
#             minDist = dist
#             minIndex = i

#     print(f'trying to go to: x= {arrPos[minIndex][0]}')
#     print(f'trying to go to: y= {arrPos[minIndex][1]}')
#     print(f'trying to go to: theta= {arrPos[minIndex][2]}')
#     return minIndex

# async def main():
#     robot = await connect()
#     print('Resources:', robot.resource_names)
#     base = Base.from_robot(robot, 'viam_base')
#     slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
#     motion = MotionClient.from_robot(robot, name="builtin")
    
#     # Set power settings (ensure these values are appropriate for your rover)
#     await base.set_power(
#         linear=Vector3(x=0, y=1, z=0),
#         angular=Vector3(x=0, y=0, z=0.75)
#     )

#     pos = await slam.get_position()
#     x = pos.x
#     y = pos.y
#     theta = pos.theta
#     base_origin_x = x + 50
#     base_origin_y = y + 50

#     # Define waypoints
#     wp = [
#         [0, 0, 0],
#         [600, 0, 90],
#         [600, 600, 180],
#         [0, 600, -90]
#     ]
#     print("Original Waypoints:", wp)

#     for i in wp:
#         i[0] += base_origin_x
#         i[1] += base_origin_y
#     print("Adjusted Waypoints:", wp)

#     # Get initial position
#     pos = await slam.get_position()
#     x = pos.x
#     y = pos.y
#     theta = pos.theta
#     print(f'currently at x={x}')
#     print(f'currently at y={y}')
#     print(f'currently at theta={theta}')
    
#     # Move to the first waypoint
#     await moveToPos(base, slam, wp[0][0], wp[0][1], wp[0][2])
    
#     # Find the closest waypoint after initial movement
#     wpIndex = await closestToPath(x, y, slam, wp)

#     # Start navigating through waypoints
#     await goThroughPath(base, slam, wpIndex, wp)

#     await robot.close()

# if __name__ == '__main__':
#     asyncio.run(main())
