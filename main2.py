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

def getDist (currX, currY, wantX, wantY):
    return np.sqrt((wantX-currY)**2+(wantY-currX)**2)

async def closestToPath(base,slam, arrPos):
    wpIndex = await findWaypt(base,slam,arrPos)
    baseX = arrPos[wpIndex][0]
    baseY = arrPos[wpIndex][1]
    baseTheta = arrPos[wpIndex][2]
    await moveToPos(base,slam,baseX,baseY,baseTheta)
    return wpIndex
    

async def moveToPos(base, slam, x,y,theta):
    currPos = await get_position(slam)
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    toMove = np.arctan((y-currY)/(x-currX))-currTheta
    print(toMove)
    dist = np.sqrt((y-currY)**2+(x-currX)**2)
    if x-currX <0:
        toMove+= 90
    await base.spin(toMove,10)

    await base.move_straight(int(dist),50)
    await base.spin(theta-toMove,50)

async def findWaypt(base,slam, arrPos):
    pos = await get_position(slam)
    x = pos.x
    y = pos.y
    theta = pos.theta
    minDist = np.sqrt((y-arrPos[0][0])**2+(x-arrPos[0][1])**2)
    minIndex = 0
    for i, wp in enumerate(arrPos):
        wpX = arrPos[i][0] *30
        wpY = arrPos[i][1] *30
        wpTheta = arrPos[i][2]
        dist = np.sqrt((y-wpY)**2+(x-wpX)**2)
        if dist<minDist:
            minDist = dist
            minIndex = i

    print(arrPos[minIndex][0] *30)
    print(arrPos[minIndex][1] *30)
    print(arrPos[minIndex][2])
    return minIndex

async def goThroughPath(base,slam,wpIndex, posArr):
    #iterate through waypoints
    #check if the next point is within 50 mm of the current pos
    #if not, use the goto function on the closest position
    # if so, use the go to function on the next position in the posArr 
    pos = await get_position(slam)
    currX = pos.x
    currY = pos.y
    wpX = posArr[wpIndex][0]*30
    wpY = posArr[wpIndex][1]*30
    wpTheta = posArr[wpIndex][2]
    dist = getDist(currX,currY,wpX,wpY)
    if dist <40:
        await moveToPos(base,slam,wpX,wpY,wpTheta)
        wpIndex+=1
        if wpIndex <len(posArr):
            await goThroughPath(base,slam,wpIndex,posArr)
    else:
        wpIndex = await closestToPath(base,slam,posArr)
        await goThroughPath(base,slam,wpIndex,posArr)

    


        





async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)

    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-1')  # Initialize SLAM



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
    
    #get initial position
    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    print(x)
    print(y)
    print(theta)
    
    wpIndex = await closestToPath(base,slam,wp)
    await goThroughPath(base,slam,wpIndex,wp)

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

