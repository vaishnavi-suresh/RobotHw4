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
        self.position_threshold = 40  # mm
        self.angle_threshold = 5  # degrees
        self.move_speed = 50  # mm/s
        self.rotation_speed = 10  # degrees/s

    async def get_position(self):
        """Get current position from SLAM"""
        position = await self.slam.get_position()
        return position.x, position.y, position.theta

    def get_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle_to_target(self, curr_x, curr_y, target_x, target_y, curr_theta):
        """Calculate angle needed to turn towards target"""
        target_angle = math.degrees(math.atan2(target_y - curr_y, target_x - curr_x))
        angle_diff = target_angle - curr_theta
        
        # Normalize angle to [-180, 180]
        return (angle_diff + 180) % 360 - 180

    async def move_to_position(self, target_point):
        """Move to target position with position verification"""
        curr_x, curr_y, curr_theta = await self.get_position()
        
        # Extract target coordinates
        target_x, target_y, target_theta = target_point
        
        # Calculate distance and angle to target
        distance = self.get_distance(curr_x, curr_y, target_x, target_y)
        angle_to_target = self.get_angle_to_target(curr_x, curr_y, target_x, target_y, curr_theta)

        # If we're close enough, only adjust orientation
        if distance < self.position_threshold:
            if abs(target_theta - curr_theta) > self.angle_threshold:
                await self.base.spin(target_theta - curr_theta, self.rotation_speed)
            return True

        # First, orient towards target
        if abs(angle_to_target) > self.angle_threshold:
            await self.base.spin(angle_to_target, self.rotation_speed)

        # Move towards target
        await self.base.move_straight(int(distance), self.move_speed)

        # Final orientation
        curr_x, curr_y, curr_theta = await self.get_position()
        if abs(target_theta - curr_theta) > self.angle_threshold:
            await self.base.spin(target_theta - curr_theta, self.rotation_speed)

        return self.get_distance(curr_x, curr_y, target_x, target_y) < self.position_threshold

    async def find_closest_waypoint(self, path):
        """Find the closest waypoint in the path"""
        curr_x, curr_y, _ = await self.get_position()
        
        min_dist = float('inf')
        min_index = 0
        
        for i, point in enumerate(path):
            dist = self.get_distance(curr_x, curr_y, point[0], point[1])
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        return min_index

    def generate_square_path(self, size, points_per_side):
        """Generate a square path with given size and density"""
        path = []
        
        # Define corner points
        corners = [
            (0, 0, 0),
            (size, 0, 90),
            (size, size, 180),
            (0, size, 270),
            (0, 0, 0)  # Close the square
        ]
        
        # Interpolate points between corners
        for i in range(len(corners) - 1):
            start = corners[i]
            end = corners[i + 1]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                theta = start[2]  # Maintain orientation until corner
                path.append((x, y, theta))
        
        return path

    async def follow_path(self, path):
        """Follow the given path with recovery capability"""
        current_index = await self.find_closest_waypoint(path)
        
        while current_index < len(path):
            success = await self.move_to_position(path[current_index])
            
            if success:
                current_index += 1
            else:
                # Recovery behavior - find closest point and continue
                current_index = await self.find_closest_waypoint(path)
            
            # Verify we're still on track
            curr_x, curr_y, _ = await self.get_position()
            if self.get_distance(curr_x, curr_y, path[current_index][0], path[current_index][1]) > self.position_threshold * 2:
                print("Significant deviation detected, recalculating path...")
                current_index = await self.find_closest_waypoint(path)

async def main():
    # Connect to robot
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    robot = await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
    
    try:
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-1')
        
        # Initialize course follower
        follower = CourseFollower(base, slam)
        
        # Generate square path (30cm x 30cm)
        path = follower.generate_square_path(300, 10)  # 300mm sides, 10 points per side
        
        # Follow path
        await follower.follow_path(path)
        
    finally:
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
