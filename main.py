import numpy as np

async def moveToPos(base, slam, x, y, theta):
    currPos = await get_position(slam)
    currX, currY, currTheta = currPos.x, currPos.y, currPos.theta

    # Step 1: Rotate to face the target direction
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    angle_diff = (target_angle - currTheta + 180) % 360 - 180  # Normalize to [-180, 180]

    if abs(angle_diff) > 5:  # Only rotate if angle difference is significant
        print(f"Rotating by {angle_diff} degrees to face the target.")
        await base.spin(angle_diff, 20)

    # Step 2: Move forward to the target position
    dist = getDist(currX, currY, x, y)
    if dist > 100:  # Move only if the distance to target is significant
        print(f"Moving forward, distance to target: {dist}")
        await base.move_straight(int(dist), 50)

    # Step 3: Rotate to the final desired orientation
    final_angle_diff = (theta - currTheta + 180) % 360 - 180
    if abs(final_angle_diff) > 5:  # Only rotate if final angle difference is significant
        print(f"Rotating to final orientation by {final_angle_diff} degrees.")
        await base.spin(final_angle_diff, 20)

async def follow_path(base, slam, waypoints):
    print("Starting path traversal.")
    for target_x, target_y, target_theta in waypoints:
        await moveToPos(base, slam, target_x, target_y, target_theta)
        print(f"Arrived at waypoint: ({target_x}, {target_y}, {target_theta})")

# Example for using the function
async def main():
    robot = await connect()
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')

    # Define waypoints
    waypoints = [
        [-1093.937, -745.430, 0],
        # Add more waypoints as needed
    ]
    
    await follow_path(base, slam, waypoints)
    
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
