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
        
        # Control parameters
        self.angle_threshold = 15       # degrees
        self.move_speed = 150           # mm/s - slower for better control
        self.rotation_speed = 30        # degrees/s
        self.max_distance = 200         # Maximum distance to move in one go
        
        # Track starting position
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        
        # Define waypoints for square (e.g., 300mm sides)
        self.waypoints = [
            (800, 0, 0),
            (800, 800, 90),
            (0, 800, 180),
            (0, 0, 270)
        ]
        self.current_wp_index = 0

    async def initialize_position(self):
        """Store initial position as reference point"""
        pos = await self.slam.get_position()
        self.start_x = pos.x
        self.start_y = pos.y
        self.start_theta = pos.theta
        print(f"Initialized at: x={pos.x:.1f}mm, y={pos.y:.1f}mm, θ={pos.theta:.1f}°")

    async def get_relative_position(self):
        """Get position relative to starting point"""
        pos = await self.slam.get_position()
        if self.start_x is None:
            await self.initialize_position()
            return 0, 0, pos.theta
        
        rel_x = pos.x - self.start_x
        rel_y = pos.y - self.start_y
        print(f"Current relative position: x={rel_x:.1f}mm, y={rel_y:.1f}mm, θ={pos.theta:.1f}°")
        return rel_x, rel_y, pos.theta

    async def turn_to_angle(self, target_angle):
        """Turn to absolute angle"""
        _, _, curr_theta = await self.get_relative_position()
        angle_diff = target_angle - curr_theta
        # Normalize to [-180, 180]
        angle_diff = ((angle_diff + 180) % 360) - 180
        
        if abs(angle_diff) > self.angle_threshold:
            print(f"Turning {angle_diff:.1f}° at {self.rotation_speed}°/s")
            await self.base.spin(angle_diff, self.rotation_speed)
            await asyncio.sleep(abs(angle_diff) / self.rotation_speed + 0.5)

    async def move_forward(self, distance):
        """Move forward with safety check"""
        if distance > self.max_distance:
            distance = self.max_distance
        
        print(f"Moving forward {distance:.1f}mm at {self.move_speed}mm/s")
        await self.base.move_straight(int(distance), self.move_speed)
        await asyncio.sleep(distance / self.move_speed + 0.5)

    async def move_square_side(self, target_angle, side_length):
        """Move one side of the square"""
        print(f"\nMoving square side: angle={target_angle}°, length={side_length}mm")
        
        # First turn to the correct angle
        await self.turn_to_angle(target_angle)
        
        # Move in smaller steps if necessary
        remaining_distance = side_length
        while remaining_distance > 0:
            step_distance = min(remaining_distance, self.max_distance)
            await self.move_forward(step_distance)
            remaining_distance -= step_distance
            
            # Brief pause between movements
            await asyncio.sleep(0.5)

    async def navigate_square(self, side_length):
        """Navigate a square pattern"""
        print(f"\nStarting square navigation with {side_length}mm sides")
        
        # Initialize starting position
        await self.initialize_position()
        
        # Execute each movement
        for idx, waypoint in enumerate(self.waypoints):
            target_angle, length = waypoint[2], side_length
            print(f"\n--- Navigating to waypoint {idx + 1} ---")
            await self.move_square_side(target_angle, length)
            await asyncio.sleep(1)  # Pause between sides

async def main():
    # Configure robot client with API credentials
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    robot = await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
    
    try:
        # Initialize components
        base = Base.from_robot(robot, 'viam_base')
        slam = SLAMClient.from_robot(robot, 'slam-1')
        
        # Create course follower instance
        follower = CourseFollower(base, slam)
        
        # Navigate a square with 300mm sides
        await follower.navigate_square(300)
        
    finally:
        # Ensure the robot connection is closed properly
        await robot.close()

if __name__ == '__main__':
    asyncio.run(main())


