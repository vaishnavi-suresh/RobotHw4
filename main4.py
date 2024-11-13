# # # main4.py
# # import asyncio
# # from viam.components.base import Base
# # from viam.robot.client import RobotClient
# # from viam.services.slam import SLAMClient
# # import numpy as np
# # import math

# # async def connect():
# #     opts = RobotClient.Options.with_api_key(
# #         api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
# #         api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
# #     )
# #     return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

# # async def get_position(slam):
# #     position = await slam.get_position()
# #     print(f"current position: x={position.x:.1f}mm, y={position.y:.1f}mm, θ={position.theta:.1f}°")
# #     return position

# # def get_dist(currX, currY, wantX, wantY):
# #     dist = np.sqrt((wantX-currX)**2+(wantY-currY)**2)
# #     print(f"distance to target: {dist:.1f}mm")
# #     return dist

# # def normalize_angle(angle):
# #     return ((angle+180) % 360) -180 #normalizing angle to [-180, 180]

# # async def moveToPos(base, slam, x, y, theta, last_pos=None):
# #     currPos =await get_position(slam)

# #     if last_pos and await detect_perbutation(slam, last_pos):
# #         print("perbutation detected, relocalizing...")
# #         await asyncio.sleep(0.5)
# #         return False

# #     currX = currPos.x
# #     currY = currPos.y
# #     currTheta = currPos.theta
    
# #     dx = x -currX # calculating angle to target here
# #     dy = y -currY
# #     target_angle = math.degrees(math.atan2(dy, dx))
# #     angle_diff = normalize_angle(target_angle -currTheta)
# #     print(f"target position: x={x:.1f}mm, y={y:.1f}mm, θ={theta}°")
# #     print(f"need to turn: {angle_diff:.1f}°")
    
# #     await base.spin(angle_diff, 20)   # turn towards target
# #     await asyncio.sleep(abs(angle_diff)/20 + 0.5)  # waiting for turn to complete
    
# #     dist = get_dist(currX, currY, x, y) # moving it forward
# #     if dist > 200:  # limit maximum movement distance to avoid hitting wall
# #         dist = 200
# #     print(f"moving forward: {dist:.1f}mm")
# #     await base.move_straight(int(dist), 100)
# #     await asyncio.sleep(dist/100 + 0.5)  # sleep 
    
# #     currPos = await get_position(slam) #orientating it to the target angle
# #     final_angle_diff = normalize_angle(theta-currPos.theta)
# #     if abs(final_angle_diff)>10:  # only adjust if off by more than 10 degrees
# #         print(f"final rotation: {final_angle_diff:.1f}°")
# #         await base.spin(final_angle_diff,20)
# #         await asyncio.sleep(abs(final_angle_diff)/20+0.5)

# #     final_dis = get_dist(currPos.x, currPos.y, x, y)  
# #     return final_dis < 50 # returns here if close to target
    
# # async def findWaypt(base, slam, arrPos):
# #     print("\nfinding closest waypoint...")
# #     pos = await get_position(slam)
# #     x = pos.x
# #     y = pos.y
    
# #     minDist = float('inf')
# #     minIndex = 0
# #     for i, wp in enumerate(arrPos):
# #         wpX = wp[0]
# #         wpY = wp[1]
# #         dist = get_dist(x, y, wpX, wpY)
# #         print(f"waypoint {i}: distance = {dist:.1f}mm")
# #         if dist < minDist:
# #             minDist = dist
# #             minIndex = i
# #     print(f"closest waypoint is {minIndex}")
# #     return minIndex

# # async def goThroughPath(base, slam, wpIndex, posArr, last_pos=None):
# #     if wpIndex >= len(posArr):
# #         print("path completed!!!")
# #         return
# #     pos = await get_position(slam)

# #     last_pos = pos 
# #     # currX = pos.x
# #     # currY = pos.y
# #     wpX = posArr[wpIndex][0]
# #     wpY = posArr[wpIndex][1]
# #     wpTheta = posArr[wpIndex][2]
    
# #     print(f"\nmoving to waypoint {wpIndex}")
# #     print(f"target: x={wpX:.1f}, y={wpY:.1f}, θ={wpTheta}")
    
# #     success_attempt = await moveToPos(base, slam, wpX, wpY, wpTheta, last_pos)
# #     if success_attempt:
# #         print(f"successfully reached waypoint {wpIndex}")
# #         await goThroughPath(base, slam, wpIndex + 1, posArr, last_pos)
# #     else:
# #         print("pos error detected or perturbation detected, retrying...")
# #         new_index = await findWaypt(base, slam, posArr)
# #         if new_index != wpIndex:
# #             print("switching to closer waypoint: {new_index}")
# #         await goThroughPath(base, slam, new_index, posArr, last_pos)

# #     # dist = get_dist(currX, currY, wpX, wpY)
# #     # if dist < 50:  # Within threshold of waypoint
# #     #     print("at waypoint now, moving to next")
# #     #     await moveToPos(base, slam, wpX, wpY, wpTheta)
# #     #     await goThroughPath(base, slam, wpIndex + 1, posArr)
# #     # else:
# #     #     print("moving to waypoint...")
# #     #     await moveToPos(base, slam, wpX, wpY, wpTheta)
# #     #     await asyncio.sleep(0.5) 
# #     #     await goThroughPath(base, slam, wpIndex, posArr)

# # # perturbation recovery
# # async def detect_perbutation(slam, last_pos, threshold=250):
# #     curr = await get_position(slam)
# #     dist = get_dist(curr.x, curr.y, last_pos.x, last_pos.y) 
# #     angle_diff = abs(normalize_angle(curr.theta - last_pos.theta))
# #     if dist > threshold or angle_diff > 30:
# #         print(f"PERTURBATION DETECTED!!!!: dist changed ={dist:.1f}mm, angle changed={angle_diff:.1f}°")
# #         return True
# #     return False

# # async def main():
# #     robot = await connect()
# #     print('resources:', robot.resource_names)
# #     base = Base.from_robot(robot, 'viam_base')
# #     slam = SLAMClient.from_robot(robot, 'slam-2')
    
# #     pos = await get_position(slam) #get initial position
# #     base_origin_x = pos.x
# #     base_origin_y = pos.y
    
# #     square_size = 500 # move in square with length 500mm 
# #     wp = np.zeros((4, 3))  # here have 4 waypoints for a simple square
# #     # defined square corners
# #     wp[0] = [base_origin_x, base_origin_y,0]                    # start
# #     wp[1] = [base_origin_x+square_size, base_origin_y,90]     # right
# #     wp[2] = [base_origin_x+square_size, base_origin_y+square_size,180]  # top-right
# #     wp[3] = [base_origin_x, base_origin_y+square_size,270]    # top-left

# #     print("\nstarting square navigation...")
# #     wpIndex = await findWaypt(base, slam, wp)
# #     await goThroughPath(base, slam, wpIndex, wp)
    
# #     await robot.close()

# # if __name__ == '__main__':
# #     asyncio.run(main())


# import asyncio
# from viam.components.base import Base
# from viam.robot.client import RobotClient
# from viam.services.slam import SLAMClient
# import numpy as np
# import math

# async def connect():
#     opts = RobotClient.Options.with_api_key(
#         api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
#         api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
#     )
#     return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

# async def get_position(slam):
#     position = await slam.get_position()
#     print(f"current position: x={position.x:.1f}mm, y={position.y:.1f}mm, θ={position.theta:.1f}°")
#     return position

# def get_dist(currX, currY, wantX, wantY):
#     dist = np.sqrt((wantX-currX)**2+(wantY-currY)**2)
#     print(f"distance to target: {dist:.1f}mm")
#     return dist

# def normalize_angle(angle):
#     return ((angle+180) % 360) -180 #normalizing angle to [-180, 180]

# async def moveToPos(base, slam, x, y, theta, last_pos=None):
#     currPos = await get_position(slam)

#     if last_pos and await detect_perbutation(slam, last_pos):
#         print("perbutation detected, relocalizing...")
#         await asyncio.sleep(0.5)
#         return False

#     currX = currPos.x
#     currY = currPos.y
#     currTheta = currPos.theta
    
#     dx = x - currX # calculating angle to target here
#     dy = y - currY
#     target_angle = math.degrees(math.atan2(dy, dx))
#     angle_diff = normalize_angle(target_angle - currTheta)
#     print(f"target position: x={x:.1f}mm, y={y:.1f}mm, θ={theta}°")
#     print(f"need to turn: {angle_diff:.1f}°")
    
#     await base.spin(angle_diff, 45)   # turn towards target
#     await asyncio.sleep(abs(angle_diff)/45 + 0.5)  # waiting for turn to complete
    
#     dist = get_dist(currX, currY, x, y) # moving it forward
#     if dist > 200:  # limit maximum movement distance to avoid hitting wall
#         dist = 200
#     print(f"moving forward: {dist:.1f}mm")
#     await base.move_straight(int(dist), 300)
#     await asyncio.sleep(dist/300 + 0.5)  # sleep 
    
#     currPos = await get_position(slam) #orientating it to the target angle
#     final_angle_diff = normalize_angle(theta-currPos.theta)
#     if abs(final_angle_diff)>10:  # only adjust if off by more than 10 degrees
#         print(f"final rotation: {final_angle_diff:.1f}°")
#         await base.spin(final_angle_diff, 45)
#         await asyncio.sleep(abs(final_angle_diff)/45 + 0.5)

#     final_dist = get_dist(currPos.x, currPos.y, x, y)  
#     return final_dist < 50 # returns here if close to target
    
# async def findWaypt(base, slam, arrPos):
#     print("\nfinding closest waypoint...")
#     pos = await get_position(slam)
#     x = pos.x
#     y = pos.y
    
#     minDist = float('inf')
#     minIndex = 0
#     for i, wp in enumerate(arrPos):
#         wpX = wp[0]
#         wpY = wp[1]
#         dist = get_dist(x, y, wpX, wpY)
#         print(f"waypoint {i}: x={wpX:.1f}, y={wpY:.1f}, distance={dist:.1f}mm")
#         if dist < minDist:
#             minDist = dist
#             minIndex = i
#     print(f"closest waypoint is {minIndex} at distance {minDist:.1f}mm")
#     return minIndex

# async def goThroughPath(base, slam, wpIndex, posArr, last_pos=None):
#     if wpIndex >= len(posArr):
#         print("path completed!!!")
#         return
#     pos = await get_position(slam)

#     last_pos = pos 
#     wpX = posArr[wpIndex][0]
#     wpY = posArr[wpIndex][1]
#     wpTheta = posArr[wpIndex][2]
    
#     print(f"\nmoving to waypoint {wpIndex}")
#     print(f"target: x={wpX:.1f}, y={wpY:.1f}, θ={wpTheta}")
    
#     success_attempt = await moveToPos(base, slam, wpX, wpY, wpTheta, last_pos)
#     if success_attempt:
#         print(f"successfully reached waypoint {wpIndex}")
#         await goThroughPath(base, slam, wpIndex + 1, posArr, last_pos)
#     else:
#         print("pos error detected or perturbation detected, retrying...")
#         new_index = await findWaypt(base, slam, posArr)
#         if new_index != wpIndex:
#             print(f"switching to closer waypoint: {new_index}")
#         await goThroughPath(base, slam, new_index, posArr, last_pos)

# # perturbation recovery
# async def detect_perbutation(slam, last_pos, threshold=200):
#     curr = await get_position(slam)
#     dist = get_dist(curr.x, curr.y, last_pos.x, last_pos.y) 
#     angle_diff = abs(normalize_angle(curr.theta - last_pos.theta))
#     if dist > threshold or angle_diff > 30:
#         print(f"PERTURBATION DETECTED!!!!: dist changed={dist:.1f}mm, angle changed={angle_diff:.1f}°")
#         return True
#     return False

# async def main():
#     robot = await connect()
#     print('Resources:', robot.resource_names)
#     base = Base.from_robot(robot, 'viam_base')
#     slam = SLAMClient.from_robot(robot, 'slam-2')  # Changed to SLAM-2
    
#     pos = await get_position(slam) #get initial position
#     base_origin_x = pos.x + 50  # Added offset
#     base_origin_y = pos.y + 50  # Added offset
    
#     square_size = 300  # Reduced square size to 300mm
#     wp = np.zeros((4, 3))  # here have 4 waypoints for a simple square
    
#     # defined square corners
#     wp[0] = [base_origin_x, base_origin_y, 0]                    # start
#     wp[1] = [base_origin_x + square_size, base_origin_y, 90]     # right
#     wp[2] = [base_origin_x + square_size, base_origin_y + square_size, 180]  # top-right
#     wp[3] = [base_origin_x, base_origin_y + square_size, 270]    # top-left

#     print("\nstarting square navigation...")
#     print("Square corner positions:")
#     for i, point in enumerate(wp):
#         print(f"Point {i}: x={point[0]:.1f}, y={point[1]:.1f}, θ={point[2]}°")
    
#     wpIndex = await findWaypt(base, slam, wp)
#     await goThroughPath(base, slam, wpIndex, wp)
    
#     await robot.close()

# if __name__ == '__main__':
#     asyncio.run(main()) 


import asyncio
from viam.components.base import Base, Vector3
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
import numpy as np

# Connection and initialization
async def connect():
    """Establish connection to the robot."""
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

async def get_position(slam):
    """Get current position from SLAM."""
    return await slam.get_position()

def getDist(curr_x, curr_y, want_x, want_y):
    """Calculate distance between two points."""
    return np.sqrt((want_x - curr_x)**2 + (want_y - curr_y)**2)

async def moveAngle(base, slam, to_move, target_angle):
    """
    Rotate the robot to a specific angle.
    Accounts for drift with smaller movements.
    """
    try:
        while abs(to_move) > 1:
            # Use smaller movements to prevent overshooting
            rotation_amount = to_move/2 if abs(to_move) > 10 else to_move
            await base.spin(rotation_amount, 45)
            
            curr_pos = await slam.get_position()
            curr_theta = curr_pos.theta
            to_move = (target_angle - curr_theta + 180) % 360 - 180
            await asyncio.sleep(0.1)  # Prevent too rapid updates
    except Exception as e:
        print(f"Rotation error: {e}")
        raise

async def moveToPos(base, slam, x, y, theta):
    """
    Move to a specific position and orientation.
    Compensates for drift with 175-degree adjustment.
    """
    try:
        print(f"Moving to: x={x}, y={y}, theta={theta}")
        curr_pos = await slam.get_position()
        curr_x = curr_pos.x
        curr_y = curr_pos.y
        curr_theta = curr_pos.theta
        
        # Calculate target angle with drift compensation
        target_angle_rad = np.arctan2(y - curr_y, x - curr_x)
        target_angle = np.degrees(target_angle_rad)
        to_move = (target_angle - curr_theta + 175) % 360 - 180  # 175 for drift compensation
        
        # Calculate distance
        dist = getDist(curr_x, curr_y, x, y)
        
        print(f"Current position: x={curr_x}, y={curr_y}, theta={curr_theta}")
        print(f"Distance to target: {dist}mm, Angle to move: {to_move}°")
        
        # Execute movement
        await moveAngle(base, slam, to_move, target_angle)
        if dist > 0:
            await base.move_straight(int(dist/2), 400)  # Half distance for drift compensation
        
        # Final orientation adjustment
        curr_pos = await slam.get_position()
        final_rotation = (theta - curr_pos.theta + 180) % 360 - 180
        if abs(final_rotation) > 5:  # Only adjust if difference is significant
            await moveAngle(base, slam, final_rotation, theta)
            
    except Exception as e:
        print(f"Movement error: {e}")
        raise

async def findWaypt(x, y, slam, arr_pos):
    """Find the closest waypoint to current position."""
    try:
        print(f"Finding closest waypoint from position x={x}, y={y}")
        min_dist = getDist(x, y, arr_pos[0][0], arr_pos[0][1])
        min_index = 0
        
        for i, (wp_x, wp_y, _) in enumerate(arr_pos):
            dist = getDist(x, y, wp_x, wp_y)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        print(f"Closest waypoint: {min_index} at distance {min_dist}mm")
        return min_index
    except Exception as e:
        print(f"Waypoint finding error: {e}")
        raise

async def closestToPath(curr_x, curr_y, slam, arr_pos):
    """Find closest point on path and return its index."""
    try:
        wp_index = await findWaypt(curr_x, curr_y, slam, arr_pos)
        return wp_index
    except Exception as e:
        print(f"Path proximity error: {e}")
        raise

async def goThroughPath(orig, base, slam, wp_index, pos_arr):
    """
    Navigate through a series of waypoints with drift correction and error handling.
    """
    try:
        current_index = wp_index
        path_complete = False
        max_retries = 3
        position_tolerance = 100  # mm
        
        while not path_complete:
            try:
                pos = await slam.get_position()
                curr_x, curr_y = pos.x, pos.y
            except Exception as e:
                print(f"Position reading error: {e}")
                await asyncio.sleep(1)
                continue

            next_index = (current_index + 1) % len(pos_arr)
            
            if next_index == orig:
                print("Path complete - reached final waypoint")
                path_complete = True
                break

            current_wp = pos_arr[current_index]
            distance_to_current = getDist(curr_x, curr_y, current_wp[0], current_wp[1])
            
            if distance_to_current > position_tolerance:
                print(f"Drift detected: {distance_to_current}mm from waypoint")
                closest_wp = await findWaypt(curr_x, curr_y, slam, pos_arr)
                
                if closest_wp != current_index:
                    print(f"Repositioning to waypoint {closest_wp}")
                    retry_count = 0
                    
                    while retry_count < max_retries:
                        try:
                            await moveToPos(base, slam, 
                                          pos_arr[closest_wp][0],
                                          pos_arr[closest_wp][1],
                                          pos_arr[closest_wp][2])
                            
                            pos = await slam.get_position()
                            if getDist(pos.x, pos.y, 
                                     pos_arr[closest_wp][0],
                                     pos_arr[closest_wp][1]) <= position_tolerance:
                                current_index = closest_wp
                                break
                        except Exception as e:
                            print(f"Movement error: {e}")
                            retry_count += 1
                            await asyncio.sleep(1)
                    
                    if retry_count >= max_retries:
                        raise Exception("Failed to reach waypoint after maximum retries")
            else:
                print(f"Moving to next waypoint {next_index}")
                retry_count = 0
                
                while retry_count < max_retries:
                    try:
                        await moveToPos(base, slam,
                                      pos_arr[next_index][0],
                                      pos_arr[next_index][1],
                                      pos_arr[next_index][2])
                        
                        pos = await slam.get_position()
                        if getDist(pos.x, pos.y,
                                 pos_arr[next_index][0],
                                 pos_arr[next_index][1]) <= position_tolerance:
                            current_index = next_index
                            break
                    except Exception as e:
                        print(f"Movement error: {e}")
                        retry_count += 1
                        await asyncio.sleep(1)
                
                if retry_count >= max_retries:
                    raise Exception("Failed to reach waypoint after maximum retries")
            
            await asyncio.sleep(0.5)
            
    except Exception as e:
        print(f"Navigation error: {e}")
        raise
    
    return current_index

async def main():
    """Main execution function."""
    robot = None
    try:
        # Initialize robot connection
        robot = await connect()
        print('Resources:', robot.resource_names)
        
        # Initialize components
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-2')
        motion = MotionClient.from_robot(robot, name="builtin")
        
        # Set initial power
        await base.set_power(
            linear=Vector3(x=0, y=1, z=0),
            angular=Vector3(x=0, y=0, z=0.75)
        )
        
        # Get initial position
        pos = await slam.get_position()
        x, y, theta = pos.x, pos.y, pos.theta
        print(f"Starting position: x={x}, y={y}, theta={theta}")
        
        # Define waypoints for square path
        wp = [
            [0, 0, 0],
            [400, 0, 90],
            [400, 400, 180],
            [0, 400, -90]
        ]
        print("Waypoints:", wp)
        
        # Move to initial position
        await moveToPos(base, slam, wp[0][0], wp[0][1], wp[0][2])
        
        # Find closest waypoint and execute path
        wp_index = await closestToPath(x, y, slam, wp)
        await goThroughPath(wp_index, base, slam, wp_index, wp)
        
        print("Navigation completed successfully")
        
    except Exception as e:
        print(f"Error in main execution: {e}")
        raise
    finally:
        if robot:
            await robot.close()

if __name__ == '__main__':
    asyncio.run(main())

