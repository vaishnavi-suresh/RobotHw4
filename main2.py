# main2.py
import asyncio
from viam.components.base import Base
from viam.components.base import Vector3
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.services.slam import SLAMClient
from viam.services.motion import MotionClient
import scipy


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
    curr = (currX,currY)
    want = (wantX,wantY)
    return np.sqrt((wantX-currX)**2+(wantY-currY)**2)

async def closestToPath(currX,currY,slam, arrPos):
    wpIndex = await findWaypt(currX,currY,slam,arrPos)
    baseX = arrPos[wpIndex][0]
    baseY = arrPos[wpIndex][1]
    baseTheta = arrPos[wpIndex][2]
    #await moveToPos(base,slam,baseX,baseY,baseTheta)
    return wpIndex

async def computeAng(slam, target_angle):
    currPos = await slam.get_position()
    currTheta = currPos.theta
    toMove = (target_angle - currTheta + 180) % 360 -180
    return toMove
    
async def moveAngle(base,slam,target_angle):
    toMove = await computeAng(slam,target_angle)
    while np.abs(toMove)>1:
        toMove = await computeAng(slam,target_angle)
        await base.spin(toMove/2, 45)



async def moveToPos(base, slam, x,y,theta):
    print("move to point call")
    currPos = await slam.get_position()
    currX = currPos.x
    currY = currPos.y
    currTheta = currPos.theta
    print (f'x={currX}')
    print (f'y={currY}')
    print (f'theta={currTheta}')
    print (f'want x={x}')
    print (f'want y={y}')
    target_angle_rad = np.arctan2(y - currY, x - currX)
    target_angle = np.degrees(target_angle_rad)
    #toMove = (target_angle - currTheta + 180) % 360 -180 
    print(f'moving to angle: {target_angle}')
    dist = getDist(currX,currY,x,y)
    """    for i in range (5):
        await moveAngle(base,slam,target_angle)
        await base.move_straight(int(dist/5),400)
        """
    while dist >70:
        await moveAngle(base,slam,target_angle)
        await base.move_straight(int(dist/2),400)
        currPos = await slam.get_position()
        currX = currPos.x
        currY = currPos.y
        dist = getDist(currX,currY,x,y)







        
    #await base.spin(toMove, 45)
    #await base.move_straight(int(dist),200)
"""    while np.abs(currTheta-target_angle)>7:
        if currTheta<theta:
            await base.spin(5,45)
        else:
            await base.spin(-5,45)
        currPos = await slam.get_position()
        currTheta = currPos.theta
    while np.abs(target_angle-currTheta)>5:
        toMove = (target_angle - currTheta + 180) % 360 -180
        print(currTheta)
        await base.spin(toMove, 45)
        currPos = await slam.get_position()
        await asyncio.sleep(0.1)
        currTheta = currPos.theta"""
        
   

async def findWaypt(x,y,slam, arrPos):
    print("going to new position")
   
    print(f'currently at x = {x}')
    print(f'currently at y = {y}')
    minDist = getDist(x,y,arrPos[0][0],arrPos[0][1])
    minIndex = 0
    for i in range(len (arrPos)):

        wpX = arrPos[i][0]
        wpY = arrPos[i][1]
        dist = getDist(x,y,wpX,wpY)
        print (f'calculating distance x: {x} y = {y}, WPx = {wpX}, WPy = {wpY}, dist = {dist}')
        if dist<minDist:
            minDist = dist
            minIndex = i

    print(f'trying to go to: x= {arrPos[minIndex][0]}')
    print(f'trying to go to: y= {arrPos[minIndex][1]}')
    print(f'trying to go to: theta= {arrPos[minIndex][2]}')
    return minIndex

async def goThroughPath(orig,base,slam,wpIndex, posArr):
    next = wpIndex+1
    while True:
        
        if wpIndex >= len(posArr):
            next = 1
            wpIndex = 0
        elif next >= len(posArr):
            next = 0

        #await asyncio.sleep(0.1)

        pos = await slam.get_position()
        currX = pos.x
        currY = pos.y
        c = await findWaypt(currX,currY,slam,posArr)
        if np.abs(currX-posArr[wpIndex][0])>200 or np.abs(currY-posArr[wpIndex][1])>200:
            print("NOT CLOSEST")
                

            print(c)
            await moveToPos(base,slam,posArr[c][0],posArr[c][1],posArr[c][2])
            await asyncio.sleep(0.5)

            wpIndex = c
            next = c+1
        else:
            print("next wp")
            print(next)
            await moveToPos(base,slam,posArr[next][0],posArr[next][1],posArr[next][2])
            await asyncio.sleep(0.5)
            wpIndex+=1
            if wpIndex ==orig:
                break
            next = wpIndex+1
    
        
"""
async def goThroughPath(base,slam,wpIndex, posArr):
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
    await moveToPos(base,slam,wpX,wpY,wpTheta)
    currX = pos.x
    currY = pos.y
    if dist <300:
        await moveToPos(base,slam,wpX,wpY,wpTheta)
        wpIndex+=1
        if wpIndex <len(posArr):
            await goThroughPath(base,slam,wpIndex,posArr)
    else:
        wpIndex = await closestToPath(base,slam,posArr)
        await goThroughPath(base,slam,wpIndex,posArr)"""

    


        





async def main():
    robot = await connect()
    print('Resources:', robot.resource_names)
    base = Base.from_robot(robot, 'viam_base')
    slam = SLAMClient.from_robot(robot, 'slam-2')  # Initialize SLAM
    motion = MotionClient.from_robot(robot,name="builtin")
    internal_state = await slam.get_internal_state()
    await base.set_power(
    linear=Vector3(x=0, y=1, z=0),
    angular=Vector3(x=0, y=0, z=0.75))

    pos = await slam.get_position()
    x = pos.x
    y = pos.y
    theta = pos.theta
    base_origin_x = x
    base_origin_y = y


    #get a set of waypoints to track and populate them
    #wp = np.zeros((40,3))
    wp = [[0,0,0],
          [250,0,90],
          [250,250,180],
          [0,250,-90]]
    print(wp)

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
    await moveToPos(base,slam,wp[0][0],wp[0][1],wp[0][2])
    
    wpIndex = 0
    orig = 0

    await goThroughPath(orig,base,slam,wpIndex,wp)

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

