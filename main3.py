import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
import numpy as np
import math

class CourseFollower:
    def __init__(self, base, slam):
        self.base = base
        self.slam = slam
        
        # Modified parameters to improve stopping behavior
        self.position_threshold = 150   # Increased to be more lenient with stopping
        self.angle_threshold = 20       # Reduced for better direction control
        self.move_speed = 300          # Reduced speed for better control
        self.rotation_speed = 45       # Reduced for more precise turning
        self.scale_factor = 1          # Changed to 1 to use direct mm measurements

    async def get_position(self):
        """Get current position from SLAM"""
        position = await self.slam.get_position()
        print(f"\nCurrent Position: x={position.x:.2f}mm, y={position.y:.2f}mm, θ={position.theta:.2f}°")
        return position.x, position.y, position.theta

    def get_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        print(f"Distance calculation: ({x1:.1f}, {y1:.1f}) to ({x2:.1f}, {y2:.1f}) = {dist:.1f}mm")
        return dist

    def get_angle_to_target(self, curr_x, curr_y, target_x, target_y, curr_theta):
        """Calculate angle needed to turn towards target"""
        dx = target_x - curr_x
        dy = target_y - curr_y
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = target_angle - curr_theta
        
        # Normalize angle to [-180, 180]
        normalized_angle = (angle_diff + 180) % 360 - 180
        print(f"Angle calculation: target={target_angle:.2f}°, current={curr_theta:.2f}°, needed turn={normalized_angle:.2f}°")
        return normalized_angle

    async def move_to_position(self, target_point):
        """Move to target position with position verification"""
        curr_x, curr_y, curr_theta = await self.get_position()
        
        # No scaling needed anymore
        target_x = target_point[0]
        target_y = target_point[1]
        target_theta = target_point[2]
        
        print(f"\nMoving to target: x={target_x:.2f}mm, y={target_y:.2f}mm, θ={target_theta:.2f}°")
        
        # Calculate distance and angle to target
        distance = self.get_distance(curr_x, curr_y, target_x, target_y)
        print(f"Distance to target: {distance:.2f}mm")
        
        # Check if we're already at the target
        if distance < self.position_threshold:
            print(f"Already within position threshold ({self.position_threshold}mm)")
            if abs(target_theta - curr_theta) > self.angle_threshold:
                angle_adjust = target_theta - curr_theta
                print(f"Adjusting final orientation by {angle_adjust:.2f}°")
                await self.base.spin(angle_adjust, self.rotation_speed)
            return True

        # Calculate and execute turn
        angle_to_target = self.get_angle_to_target(curr_x, curr_y, target_x, target_y, curr_theta)
        if abs(angle_to_target) > self.angle_threshold:
            print(f"Turning {angle_to_target:.2f}° at {self.rotation_speed}°/s")
            await self.base.spin(angle_to_target, self.rotation_speed)
            await asyncio.sleep(abs(angle_to_target/self.rotation_speed))  # Wait for turn to complete

        # Move forward with a shorter distance
        move_distance = min(distance, 200)  # Move in smaller steps
        print(f"Moving forward {move_distance:.2f}mm at {self.move_speed}mm/s")
        await self.base.move_straight(int(move_distance), self.move_speed)
        await asyncio.sleep(move_distance/self.move_speed + 0.5)  # Wait for movement to complete

        # Verify new position
        new_x, new_y, new_theta = await self.get_position()
        final_dist = self.get_distance(new_x, new_y, target_x, target_y)
        print(f"After movement - Distance to target: {final_dist:.2f}mm")

        success = final_dist < self.position_threshold
        print(f"Movement {'successful' if success else 'failed'}")
        return success

    def generate_square_path(self, size, points_per_side):
        """Generate a square path with given size and density"""
        path = []
        
        # Define corner points (in mm)
        corners = [
            (0, 0, 0),
            (size, 0, 90),
            (size, size, 180),
            (0, size, 270),
            (0, 0, 0)
        ]
        
        print("\nGenerating square path:")
        for i in range(len(corners) - 1):
            start = corners[i]
            end = corners[i + 1]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                theta = start[2]
                path.append((x, y, theta))
                print(f"Added waypoint: ({x:.2f}, {y:.2f}, {theta})")
        
        return path

    async def follow_path(self, path):
        """Follow the given path with recovery capability"""
        current_index = 0  # Start from the beginning
        
        while current_index < len(path):
            print(f"\n--- Following path: Point {current_index}/{len(path)-1} ---")
            success = await self.move_to_position(path[current_index])
            
            if success:
                print(f"Successfully reached point {current_index}")
                current_index += 1
            else:
                print("Failed to reach point, retrying...")
                # Optional: Add a small delay before retrying
                await asyncio.sleep(1)

async def main():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    robot = await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
    
    try:
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-1')
        
        follower = CourseFollower(base, slam)
        
        print("\nStarting square path navigation...")
        path = follower.generate_square_path(300, 5)  # 300mm sides, 5 points per side (reduced points)
        await follower.follow_path(path)
        
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
