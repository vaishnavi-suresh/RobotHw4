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
        
        # Adjustable parameters - you may need to tune these
        self.position_threshold = 100  # mm - increased for more lenient positioning
        self.angle_threshold = 30      # degrees - increased for more lenient rotation
        self.move_speed = 800          # mm/s - increased for faster movement
        self.rotation_speed = 90       # degrees/s - increased for faster rotation
        self.scale_factor = 30         # Scale factor for converting path coordinates to mm

    async def get_position(self):
        """Get current position from SLAM"""
        position = await self.slam.get_position()
        print(f"\nCurrent Position: x={position.x:.2f}mm, y={position.y:.2f}mm, θ={position.theta:.2f}°")
        return position.x, position.y, position.theta

    def get_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
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
        
        # Scale target coordinates
        target_x = target_point[0] * self.scale_factor
        target_y = target_point[1] * self.scale_factor
        target_theta = target_point[2]
        
        print(f"\nMoving to target: x={target_x:.2f}mm, y={target_y:.2f}mm, θ={target_theta:.2f}°")
        
        # Calculate distance and angle to target
        distance = self.get_distance(curr_x, curr_y, target_x, target_y)
        print(f"Distance to target: {distance:.2f}mm")
        
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
            await asyncio.sleep(1)  # Give time to complete rotation

        # Move forward
        print(f"Moving forward {distance:.2f}mm at {self.move_speed}mm/s")
        await self.base.move_straight(int(distance), self.move_speed)
        await asyncio.sleep(1)  # Give time to complete movement

        # Verify new position
        new_x, new_y, new_theta = await self.get_position()
        final_dist = self.get_distance(new_x, new_y, target_x, target_y)
        print(f"After movement - Distance to target: {final_dist:.2f}mm")
        
        # Final orientation adjustment
        if abs(target_theta - new_theta) > self.angle_threshold:
            final_angle_adjust = target_theta - new_theta
            print(f"Final orientation adjustment: {final_angle_adjust:.2f}°")
            await self.base.spin(final_angle_adjust, self.rotation_speed)
            await asyncio.sleep(1)

        success = final_dist < self.position_threshold
        print(f"Movement {'successful' if success else 'failed'}")
        return success

    async def find_closest_waypoint(self, path):
        """Find the closest waypoint in the path"""
        curr_x, curr_y, _ = await self.get_position()
        
        min_dist = float('inf')
        min_index = 0
        
        print("\nFinding closest waypoint:")
        for i, point in enumerate(path):
            scaled_x = point[0] * self.scale_factor
            scaled_y = point[1] * self.scale_factor
            dist = self.get_distance(curr_x, curr_y, scaled_x, scaled_y)
            print(f"Waypoint {i}: ({scaled_x:.2f}, {scaled_y:.2f}) - Distance: {dist:.2f}mm")
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        print(f"Closest waypoint is {min_index} at distance {min_dist:.2f}mm")
        return min_index

    def generate_square_path(self, size, points_per_side):
        """Generate a square path with given size and density"""
        path = []
        size = size / self.scale_factor  # Convert mm to path units
        
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
        current_index = await self.find_closest_waypoint(path)
        
        while current_index < len(path):
            print(f"\n--- Following path: Point {current_index}/{len(path)-1} ---")
            success = await self.move_to_position(path[current_index])
            
            if success:
                print(f"Successfully reached point {current_index}")
                current_index += 1
            else:
                print("Failed to reach point, finding closest waypoint for recovery")
                current_index = await self.find_closest_waypoint(path)
            
            curr_x, curr_y, _ = await self.get_position()
            target_x = path[current_index][0] * self.scale_factor
            target_y = path[current_index][1] * self.scale_factor
            
            if self.get_distance(curr_x, curr_y, target_x, target_y) > self.position_threshold * 2:
                print("Large deviation detected - recalculating path")
                current_index = await self.find_closest_waypoint(path)

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
        
        # Adjustable parameters at the top of the class can be modified here
        # follower.position_threshold = 100  # Increase for more lenient positioning
        # follower.angle_threshold = 10      # Increase for more lenient rotation
        # follower.move_speed = 200          # Adjust movement speed
        # follower.rotation_speed = 20       # Adjust rotation speed
        # follower.scale_factor = 30         # Adjust scaling of coordinates
        
        print("\nStarting square path navigation...")
        path = follower.generate_square_path(300, 10)  # 300mm sides, 10 points per side
        await follower.follow_path(path)
        
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
