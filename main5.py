import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
import numpy as np
import math

class CourseFollower:
    def __init__(self, base, slam):
        """
        Initializes the CourseFollower with base and SLAM components.
        Sets up control parameters and defines the waypoints for the course.
        """
        self.base = base
        self.slam = slam
        
        # Control parameters
        self.position_threshold = 100   # Maximum allowable deviation from the path in mm
        self.angle_threshold = 15       # Minimum angle difference to trigger rotation in degrees
        self.move_speed = 150           # Forward movement speed in mm/s
        self.rotation_speed = 30        # Rotation speed in degrees/s
        self.max_distance = 200         # Maximum distance to move in one command in mm
        
        # Track starting position and orientation
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        
        # Define waypoints for a square course (300mm sides)
        # Each waypoint is a tuple: (x, y, theta)
        self.waypoints = [
            (300, 0, 0),     # Move to (300mm, 0mm) facing 0°
            (300, 300, 90),  # Move to (300mm, 300mm) facing 90°
            (0, 300, 180),   # Move to (0mm, 300mm) facing 180°
            (0, 0, 270)      # Move back to (0mm, 0mm) facing 270°
        ]
        self.current_wp_index = 0    # Index of the current waypoint

    async def initialize_position(self):
        """
        Initializes the rover's starting position and orientation using SLAM.
        """
        pos = await self.slam.get_position()
        self.start_x = pos.x
        self.start_y = pos.y
        self.start_theta = pos.theta
        print(f"[INIT] Starting Position: x={pos.x:.1f}mm, y={pos.y:.1f}mm, θ={pos.theta:.1f}°")

    async def get_relative_position(self):
        """
        Retrieves the rover's current position relative to the starting point.
        If starting position is not initialized, it initializes it.
        Returns:
            rel_x (float): Relative X position in mm
            rel_y (float): Relative Y position in mm
            rel_theta (float): Current orientation in degrees
        """
        pos = await self.slam.get_position()
        if self.start_x is None:
            await self.initialize_position()
            return 0, 0, pos.theta
        
        rel_x = pos.x - self.start_x
        rel_y = pos.y - self.start_y
        rel_theta = pos.theta
        print(f"[POSITION] Relative Position: x={rel_x:.1f}mm, y={rel_y:.1f}mm, θ={rel_theta:.1f}°")
        return rel_x, rel_y, rel_theta

    async def turn_to_angle(self, target_angle):
        """
        Rotates the rover to face a specific absolute angle.
        Args:
            target_angle (float): The desired orientation in degrees.
        """
        _, _, curr_theta = await self.get_relative_position()
        angle_diff = target_angle - curr_theta
        # Normalize the angle difference to [-180, 180]
        angle_diff = ((angle_diff + 180) % 360) - 180
        
        if abs(angle_diff) > self.angle_threshold:
            print(f"[TURN] Rotating {angle_diff:.1f}° at {self.rotation_speed}°/s to reach {target_angle}°")
            await self.base.spin(angle_diff, self.rotation_speed)
            # Calculate time to rotate plus buffer
            rotation_time = abs(angle_diff) / self.rotation_speed + 0.5
            await asyncio.sleep(rotation_time)
            print(f"[TURN] Rotation to {target_angle}° completed")
        else:
            print(f"[TURN] Current angle {curr_theta:.1f}° within threshold of target {target_angle}°. No rotation needed.")

    async def move_forward(self, distance):
        """
        Moves the rover forward by a specified distance with safety checks.
        Args:
            distance (float): Distance to move forward in mm.
        """
        if distance > self.max_distance:
            step_distance = self.max_distance
            print(f"[MOVE] Requested distance {distance}mm exceeds max {self.max_distance}mm. Moving {step_distance}mm instead.")
        else:
            step_distance = distance
            print(f"[MOVE] Moving forward {step_distance}mm at {self.move_speed}mm/s")
        
        await self.base.move_straight(int(step_distance), self.move_speed)
        # Calculate time to move plus buffer
        move_time = step_distance / self.move_speed + 0.5
        await asyncio.sleep(move_time)
        print(f"[MOVE] Moved forward {step_distance}mm")

    async def move_square_side(self, target_angle, side_length):
        """
        Moves the rover along one side of the square course.
        Args:
            target_angle (float): The desired orientation in degrees.
            side_length (float): Length of the square side in mm.
        """
        print(f"\n[MOVE_SIDE] Starting movement: angle={target_angle}°, length={side_length}mm")
        
        # Rotate to the target angle
        await self.turn_to_angle(target_angle)
        
        # Move forward in steps to ensure precise control
        remaining_distance = side_length
        while remaining_distance > 0:
            step_distance = min(remaining_distance, self.max_distance)
            await self.move_forward(step_distance)
            remaining_distance -= step_distance
            print(f"[MOVE_SIDE] Remaining distance for this side: {remaining_distance}mm")
            await asyncio.sleep(0.5)  # Brief pause between movements
        
        print(f"[MOVE_SIDE] Completed movement: angle={target_angle}°, length={side_length}mm")

    async def navigate_square(self, side_length):
        """
        Navigates the rover through the entire square course.
        Args:
            side_length (float): Length of each side of the square in mm.
        """
        print(f"\n[NAVIGATE] Starting square navigation with {side_length}mm sides")
        
        # Initialize starting position
        await self.initialize_position()
        
        # Iterate through each waypoint to navigate the square
        for idx, waypoint in enumerate(self.waypoints):
            target_angle = waypoint[2]
            print(f"\n[NAVIGATE] Navigating to waypoint {idx + 1}: Target Angle={target_angle}°")
            await self.move_square_side(target_angle, side_length)
            print(f"[NAVIGATE] Arrived at waypoint {idx + 1}")
            await asyncio.sleep(1)  # Pause between sides
        
        print(f"\n[NAVIGATE] Completed square navigation")

    async def check_deviation(self):
        """
        Checks if the rover has deviated from the course beyond the position threshold.
        Returns:
            deviation (float): The distance from the intended path in mm.
        """
        rel_x, rel_y, _ = await self.get_relative_position()
        deviation = self.calculate_distance_from_path(rel_x, rel_y)
        print(f"[DEVIATION] Current deviation from path: {deviation:.1f}mm")
        return deviation

    def calculate_distance_from_path(self, x, y):
        """
        Calculates the shortest distance from the rover's current position to the current path segment.
        Args:
            x (float): Current relative X position in mm.
            y (float): Current relative Y position in mm.
        Returns:
            distance (float): Shortest distance from the path in mm.
        """
        # Determine which path segment the rover is currently on
        if self.current_wp_index == 0:
            # Path segment from (0,0) to (300,0) - along positive X-axis
            distance = abs(y)
        elif self.current_wp_index == 1:
            # Path segment from (300,0) to (300,300) - along positive Y-axis
            distance = abs(x - 300)
        elif self.current_wp_index == 2:
            # Path segment from (300,300) to (0,300) - along negative X-axis
            distance = abs(y - 300)
        elif self.current_wp_index == 3:
            # Path segment from (0,300) to (0,0) - along negative Y-axis
            distance = abs(x)
        else:
            distance = 0
        return distance

    async def recover_path(self):
        """
        Recovers the rover's path by navigating back to the nearest point on the intended path.
        """
        print("[RECOVERY] Initiating path recovery...")
        rel_x, rel_y, _ = await self.get_relative_position()
        
        # Calculate the nearest path segment based on current waypoint index
        nearest_segment = self.current_wp_index
        print(f"[RECOVERY] Nearest path segment index: {nearest_segment}")
        
        # Determine the target position on the nearest path segment
        if nearest_segment == 0:
            # Along positive X-axis
            target_x = 300
            target_y = 0
        elif nearest_segment == 1:
            # Along positive Y-axis
            target_x = 300
            target_y = 300
        elif nearest_segment == 2:
            # Along negative X-axis
            target_x = 0
            target_y = 300
        elif nearest_segment == 3:
            # Along negative Y-axis
            target_x = 0
            target_y = 0
        else:
            target_x = self.waypoints[0][0]
            target_y = self.waypoints[0][1]
        
        print(f"[RECOVERY] Target position to recover: x={target_x}mm, y={target_y}mm")
        
        # Calculate the desired angle to face the target position
        desired_angle = math.degrees(math.atan2(target_y, target_x)) % 360
        print(f"[RECOVERY] Desired angle to face recovery target: {desired_angle}°")
        
        # Rotate to the desired angle
        await self.turn_to_angle(desired_angle)
        
        # Calculate distance to the target position
        distance = self.get_distance(0, 0, target_x, target_y)
        print(f"[RECOVERY] Calculated distance to target position: {distance:.1f}mm")
        
        # Move to the target position
        await self.move_forward(distance)
        print("[RECOVERY] Path recovery completed. Resuming course following.")

    def get_distance(self, currX, currY, wantX, wantY):
        """
        Calculates the Euclidean distance between two points.
        Args:
            currX (float): Current X position in mm.
            currY (float): Current Y position in mm.
            wantX (float): Target X position in mm.
            wantY (float): Target Y position in mm.
        Returns:
            distance (float): Distance between the two points in mm.
        """
        return np.sqrt((wantX - currX)**2 + (wantY - currY)**2)

    async def monitor_course(self, side_length):
        """
        Monitors the rover's adherence to the course and initiates recovery if necessary.
        Args:
            side_length (float): Length of each side of the square in mm.
        """
        print("\n[MONITOR] Starting course monitoring...")
        
        while self.current_wp_index < len(self.waypoints):
            deviation = await self.check_deviation()
            if deviation > self.position_threshold:
                print(f"[MONITOR] Deviation {deviation:.1f}mm exceeds threshold {self.position_threshold}mm")
                await self.recover_path()
                # After recovery, reset the starting position to the recovery point
                await self.initialize_position()
            else:
                print(f"[MONITOR] Deviation {deviation:.1f}mm within acceptable range.")
            await asyncio.sleep(1)  # Check deviation every second

    async def navigate_square_with_recovery(self, side_length):
        """
        Navigates the square course with continuous monitoring for perturbations.
        Args:
            side_length (float): Length of each side of the square in mm.
        """
        print(f"\n[NAVIGATE_RECOVERY] Starting square navigation with {side_length}mm sides and perturbation recovery")
        
        # Initialize starting position
        await self.initialize_position()
        
        # Start monitoring task
        monitor_task = asyncio.create_task(self.monitor_course(side_length))
        
        # Iterate through each waypoint to navigate the square
        for idx, waypoint in enumerate(self.waypoints):
            target_angle = waypoint[2]
            print(f"\n[NAVIGATE_RECOVERY] Navigating to waypoint {idx + 1}: Target Angle={target_angle}°")
            await self.move_square_side(target_angle, side_length)
            print(f"[NAVIGATE_RECOVERY] Arrived at waypoint {idx + 1}")
            await asyncio.sleep(1)  # Pause between sides
        
        # Cancel monitoring after navigation is complete
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            print("[MONITOR] Course monitoring task cancelled after navigation completion.")
        
        print(f"\n[NAVIGATE_RECOVERY] Completed square navigation with perturbation recovery")

async def main():
    """
    Main function to initialize the robot, create a CourseFollower instance,
    and start the navigation with perturbation recovery.
    """
    # Replace with your actual API credentials and robot address
    api_key = 'YOUR_API_KEY'
    api_key_id = 'YOUR_API_KEY_ID'
    robot_address = 'your_robot_address'  # e.g., 'rover6-main.9883cqmu1w.viam.cloud'
    
    # Configure robot client with API credentials
    opts = RobotClient.Options.with_api_key(
        api_key=api_key,
        api_key_id=api_key_id
    )
    
    # Connect to the robot
    print("[MAIN] Connecting to the robot...")
    robot = await RobotClient.at_address(robot_address, opts)
    print("[MAIN] Connected to the robot.")
    
    try:
        # Initialize components
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-1')
        print("[MAIN] Initialized Base and SLAM components.")
        
        # Create CourseFollower instance
        follower = CourseFollower(base, slam)
        print("[MAIN] Created CourseFollower instance.")
        
        # Define the side length of the square in mm
        side_length = 300
        
        # Start navigating the square with perturbation recovery
        await follower.navigate_square_with_recovery(side_length)
        
    except Exception as e:
        print(f"[ERROR] An error occurred: {e}")
    
    finally:
        # Ensure the robot connection is closed properly
        await robot.close()
        print("[MAIN] Robot connection closed.")

if __name__ == '__main__':
    asyncio.run(main())
