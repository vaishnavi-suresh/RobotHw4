import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
import numpy as np
async def connect():
    opts = RobotClient.Options.with_api_key(
        api_key='i11ph4btwvdp1kixh3oveex92tmvdtx2',
        api_key_id='8b19e462-949d-4cf3-9f7a-5ce0854eb7b8'
    )
    return await RobotClient.at_address('rover6-main.9883cqmu1w.viam.cloud', opts)
async def get_position(slam):
    position = await slam.get_position()  # This depends on your specific SLAM client
    return position


async def checkPosition (posArr,i,slam,base):
    a = posArr[i][0]*100
    b = posArr[i][1]*100
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    if a-x<50 and b-y <50:
        return i 

    closest = np.sqrt((x-i[0]*100)**2 +(y-i[1]*100)**2)
    closestIndex = 0
    for j,i in enumerate(posArr):
        dist = np.sqrt((x-i[0]*100)**2 +(y-i[1]*100)**2)
        if dist<closest:
            closest = dist
            closestIndex = j
    toRotate = np.arccos((posArr[closestIndex][1]-x)/closest)-theta
    await base.spin(toRotate, 50) #set velocity as needed
    await base.move_straight(int(closest),100)
    return closestIndex

async def goToZero(x,y,theta,base):
    if x<0:
        await base.spin(-theta,50) #adjust velocity as needed
        await base.move_straight(int(np.abs(x)),100)
        if y<0:
            await base.spin(-90,50)
            await base.move_straight(int(np.abs(y)),100)
        else:
            await base.spin(90,50)
            await base.move_straight(int(np.abs(y)),100)



    else:
        await base.spin(180-theta,50)#adjust velocity as needed
        await base.move_straight(int(np.abs(x)),100)
        if y<0:
            await base.spin(90,50)
            await base.move_straight(int(np.abs(y)),100)
        else:
            await base.spin(-90,50)
            await base.move_straight(int(np.abs(y)),100)





async def navigate_path(posArr,i,slam,base):
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    k = await checkPosition(posArr,i,slam,base)

    while k < len(posArr):
        #function to check if the robot is in the correct position
        if theta != posArr[k][2]:
            await base.spin(posArr[k][2]-theta,50) #set velocity as needed
        await base.move_straight(100,100) #scale up distance by 100
        k = await checkPosition(posArr,i,slam,base)

        





    


"""
async def moveInSquare(base, slam, base_coords):
    for _ in range(4):
        await base.move_straight(velocity=500, distance=500)
        print("move straight")
        await base.spin(velocity=100, angle=90)
        print("spin 90 degrees")

        # Retrieve and print current position
        x, y = await get_position(slam)
        print(f"Current position: ({x}, {y})")
        
        # If far from base, calculate and adjust path to return
        if abs(x - base_coords[0]) > threshold or abs(y - base_coords[1]) > threshold:
            print("Rover is off-course, recalculating path to base.")
            # Here, implement logic to guide the rover back to base_coords
"""
async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-1')  # Initialize SLAM

    # Set the base starting coordinates
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(x)
    print(y)

    #get a set of waypoints to track and populate them
    wp = np.zeros((40,3))
    for i in range(10):
        wp[i][0]=i
        wp[i][2]=90
        wp[i+10][0]=10
        wp[i+10][1]=i
        wp[i+10][2]=180
        wp[i+20][0]=10
        wp[i+20][1]=10-i
        wp[i+20][2]=270
        wp[i+30][0]=0
        wp[i+30][1]=10-i
        wp[i+30][2]=0
    
    await goToZero(x,y,theta,base)
    await navigate_path(wp,0,slam,base)

    await robot.close()

    
    #base_coords = (x0, y0)
    #print(f"Base coordinates: ({x0}, {y0})")
"""
    # Move in a square and maintain base coordinates
    await moveInSquare(roverBase, slam, base_coords)

    await robot.close()
"""

if __name__ == '__main__':
    asyncio.run(main())