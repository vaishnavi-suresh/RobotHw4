# main4.py

# without perbutation recovery added yet
import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.services.slam import SLAMClient
import numpy as np
import math

async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)

async def get_position(slam):
    position = await slam.get_position()
    print(f"current position: x={position.x:.1f}mm, y={position.y:.1f}mm, θ={position.theta:.1f}°")
    return position

def get_dist(currX, currY, wantX, wantY):
    dist = np.sqrt((wantX-currX)**2+(wantY-currY)**2)
    print(f"distance to target: {dist:.1f}mm")
    return dist

def normalize_angle(angle):
    return ((angle+180) % 360) -180 #normalizing angle to [-180, 180]

async def moveToPos(base, slam, x, y, theta):
    currPos =await get_position(slam)
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    
    dx = x -currX # calculating angle to target here
    dy = y -currY
    target_angle = math.degrees(math.atan2(dy, dx))
    angle_diff = normalize_angle(target_angle -currTheta)
    print(f"target position: x={x:.1f}mm, y={y:.1f}mm, θ={theta}°")
    print(f"need to turn: {angle_diff:.1f}°")
    
    await base.spin(angle_diff, 20)   # turn towards target
    await asyncio.sleep(abs(angle_diff)/20 + 0.5)  # waiting for turn to complete
    
    dist = get_dist(currX, currY, x, y) # moving it forward
    if dist > 200:  # limit maximum movement distance to avoid hitting wall
        dist = 200
    print(f"moving forward: {dist:.1f}mm")
    await base.move_straight(int(dist), 100)
    await asyncio.sleep(dist/100 + 0.5)  # sleep 
    
    currPos = await get_position(slam) #orientating it to the target angle
    final_angle_diff = normalize_angle(theta-currPos.theta)
    if abs(final_angle_diff)>10:  # only adjust if off by more than 10 degrees
        print(f"final rotation: {final_angle_diff:.1f}°")
        await base.spin(final_angle_diff,20)
        await asyncio.sleep(abs(final_angle_diff)/20+0.5)

async def findWaypt(base, slam, arrPos):
    print("\nfinding closest waypoint...")
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    
    minDist = float('inf')
    minIndex = 0
    for i, wp in enumerate(arrPos):
        wpX = wp[0]
        wpY = wp[1]
        dist = get_dist(x, y, wpX, wpY)
        print(f"waypoint {i}: distance = {dist:.1f}mm")
        if dist < minDist:
            minDist = dist
            minIndex = i
    print(f"closest waypoint is {minIndex}")
    return minIndex

async def goThroughPath(base, slam, wpIndex, posArr):
    if wpIndex >= len(posArr):
        print("path completed!!!")
        return
    pos = await get_position(slam)
    currX = pos.x
    currY = pos.y
    wpX = posArr[wpIndex][0]
    wpY = posArr[wpIndex][1]
    wpTheta = posArr[wpIndex][2]
    
    print(f"\nmoving to waypoint {wpIndex}")
    print(f"target: x={wpX:.1f}, y={wpY:.1f}, θ={wpTheta}")
    
    dist = get_dist(currX, currY, wpX, wpY)
    if dist < 50:  # Within threshold of waypoint
        print("at waypoint now, moving to next")
        await moveToPos(base, slam, wpX, wpY, wpTheta)
        await goThroughPath(base, slam, wpIndex + 1, posArr)
    else:
        print("moving to waypoint...")
        await moveToPos(base, slam, wpX, wpY, wpTheta)
        await asyncio.sleep(0.5) 
        await goThroughPath(base, slam, wpIndex, posArr)

async def main():
    robot = await connect()
    print('resources:', robot.resource_names)
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-1')
    
    pos = await get_position(slam) #get initial position
    base_origin_x = pos.x
    base_origin_y = pos.y
    
    square_size = 500 # move in square with length 500mm 
    wp = np.zeros((4, 3))  # here have 4 waypoints for a simple square
    # defined square corners
    wp[0] = [base_origin_x, base_origin_y,0]                    # start
    wp[1] = [base_origin_x+square_size, base_origin_y,90]     # right
    wp[2] = [base_origin_x+square_size, base_origin_y+square_size,180]  # top-right
    wp[3] = [base_origin_x, base_origin_y+square_size,270]    # top-left

    print("\nstarting square navigation...")
    wpIndex = await findWaypt(base, slam, wp)
    await goThroughPath(base, slam, wpIndex, wp)
    
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())

