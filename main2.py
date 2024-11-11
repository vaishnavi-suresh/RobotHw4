# main2.py
import asyncio
from viam.components.base import Base
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
from viam.proto.common import Pose

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

def getDist (currX, currY, wantX, wantY):
    return np.sqrt((wantX-currX)**2+(wantY-currY)**2)

async def closestToPath(move,base,slam, arrPos):
    wpIndex = await findWaypt(base,slam,arrPos)
    baseX = arrPos[wpIndex][0]
    baseY = arrPos[wpIndex][1]
    baseTheta = arrPos[wpIndex][2]
    await moveToPos(move,base,slam,baseX,baseY,baseTheta)
    return wpIndex

def normalize_angle(angle):
    """Normalize an angle to be within the range [-180, 180] degrees."""
    return (angle + 180) % 360 - 180

async def moveToPos(move,base, slam, x, y, theta):
    toMove = Pose(x=x,y=y,theta=theta)
    baseName = base.get_resource_name('viam_base')
    slamName = slam.get_resource_name('slam-2')
    movement = await move.move_on_map(baseName,toMove,slamName)

    """# Get the current position
    currPos = await get_position(slam)
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta

    # Calculate the angle to move towards the target
    target_angle = np.arctan2(y - currY, x - currX)
    toMove = normalize_angle(target_angle - currTheta)
    print(f'Rotating to angle: {toMove} radians')

    # Calculate the distance to the target position
    dist = getDist(currX, currY, x, y)

    # Rotate towards the target direction
    await base.spin(toMove, 45)  # Adjust speed if necessary

    # Move forward the required distance
    await base.move_straight(int(dist), 50)  # Avoid converting distance to an integer

    # Get the current orientation after moving to the target position
    finalPos = await get_position(slam)
    finalTheta = finalPos.theta

    # Calculate the final rotation needed to achieve the desired orientation
    final_rotation = normalize_angle(theta - finalTheta)
    print(f'Final rotation to adjust to orientation: {final_rotation} radians')

    # Rotate to the final orientation
    await base.spin(final_rotation, 20)  # Adjust speed if necessary
"""

"""async def moveToPos(base, slam, x,y,theta):
    currPos = await get_position(slam)
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    toMove = np.arctan2((y-currY),(x-currX))-currTheta
    print(f'moving to angle: {toMove}')
    dist = getDist(currX,currY,x,y)
    if x-currX <0:
        toMove+= 90
    await base.spin(toMove,45)
    await base.move_straight(int(dist),50)


    await base.spin(theta-toMove,20)"""

async def findWaypt(base,slam, arrPos):
    print("going to new position")
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(f'currently at x = {x}')
    print(f'currently at y = {y}')
    print(f'currently at theta = {theta}')
    minDist = np.sqrt((y-arrPos[0][0])**2+(x-arrPos[0][1])**2)
    minIndex = 0
    for i, wp in enumerate(arrPos):
        wpX = arrPos[i][0]
        wpY = arrPos[i][1]
        wpTheta = arrPos[i][2]
        dist = np.sqrt((y-wpY)**2+(x-wpX)**2)
        if dist<minDist:
            minDist = dist
            minIndex = i

    print(f'trying to go to: x= {arrPos[minIndex][0]}')
    print(f'trying to go to: y= {arrPos[minIndex][1]}')
    print(f'trying to go to: theta= {arrPos[minIndex][2]}')
    return minIndex

async def goThroughPath(move,base,slam,wpIndex, posArr):
    #iterate through waypoints
    #check if the next point is within 50 mm of the current pos
    #if not, use the goto function on the closest position
    # if so, use the go to function on the next position in the posArr 
    pos = await get_position(slam)
    currX = pos.x
    currY = pos.y
    wpX = posArr[wpIndex][0]
    wpY = posArr[wpIndex][1]
    wpTheta = posArr[wpIndex][2]
    dist = getDist(currX,currY,wpX,wpY)
    if dist <150:
        await moveToPos(move,base,slam,wpX,wpY,wpTheta)
        wpIndex+=1
        if wpIndex <len(posArr):
            await goThroughPath(move,base,slam,wpIndex,posArr)
    else:
        wpIndex = await closestToPath(move,base,slam,posArr)
        await goThroughPath(move,base,slam,wpIndex,posArr)

    


        





async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    move = MotionClient.from_robot(robot,name="builtin")
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    base_origin_x = x+200
    base_origin_y = y+200


    #get a set of waypoints to track and populate them
    #wp = np.zeros((40,3))
    wp = [[0,0,0],
          [1000,0,90],
          [1000,1000,180],
          [0,1000,-90]]
    for i in wp:
        i[0]+=base_origin_x
        i[1] += base_origin_y
    """
    for i in range(10):
        wp[i][0]=i*100 + base_origin_x
        wp[i][2]=0 
        wp[i+10][0]=1000 +base_origin_x
        wp[i+10][1]=i*100 + base_origin_y
        wp[i+10][2]=90
        wp[i+20][1]=1000+base_origin_y
        wp[i+20][0]=1000-i*100 + base_origin_x
        wp[i+20][2]=180
        wp[i+30][0]=0
        wp[i+30][1]=1000-i*100 + base_origin_y
        wp[i+30][2]=270
    """
    #get initial position
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(f'currently at x={x}')
    print(f'currently at y={y}')
    print(f'currently at theta={theta}')
    
    wpIndex = await closestToPath(move,base,slam,wp)
    await goThroughPath(move,base,slam,wpIndex,wp)

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

