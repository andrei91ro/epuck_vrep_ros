-- This is the Epuck principal control script. It is threaded

actualizeLEDs=function()
    if (relLedPositions==nil) then
        relLedPositions={{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        simRemoveDrawingObject(drawingObject)
    end
    type=sim_drawing_painttag+sim_drawing_followparentvisibility+sim_drawing_spherepoints+
        sim_drawing_50percenttransparency+sim_drawing_itemcolors+sim_drawing_itemsizes+
        sim_drawing_backfaceculling+sim_drawing_emissioncolor
    drawingObject=simAddDrawingObject(type,0,0,bodyElements,27)
    m=simGetObjectMatrix(ePuckBase,-1)
    itemData={0,0,0,0,0,0,0}
    simSetLightParameters(ledLight,0)
    for i=1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p=simMultiplyVector(m,relLedPositions[i])
            itemData[1]=p[1]
            itemData[2]=p[2]
            itemData[3]=p[3]
            itemData[4]=ledColors[i][1]
            itemData[5]=ledColors[i][2]
            itemData[6]=ledColors[i][3]
            simSetLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j=1,3,1 do
                itemData[7]=j*0.003
                simAddDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

getLightSensors=function()
    data=simReceiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=simUnpackFloatTable(data)
    end
    return lightSens
end

threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        st=simGetSimulationTime()
        velLeft=0
        velRight=0
        opMode=simGetScriptSimulationParameter(sim_handle_self,'opMode')
        lightSens=getLightSensors()
        s=simGetObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=simReadProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end
        if (opMode==0) then -- We wanna follow the line
            if (math.mod(st,2)>1.5) then
                intensity=1
            else
                intensity=0
            end
            for i=1,9,1 do
                ledColors[i]={intensity,0,0} -- red
            end
            -- Now make sure the light sensors have been read, we have a line and the 4 front prox. sensors didn't detect anything:
            if lightSens and ((lightSens[1]<0.5)or(lightSens[2]<0.5)or(lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                if (lightSens[1]>0.5) then 
                    velLeft=maxVel
                else
                    velLeft=maxVel*0.25
                end
                if (lightSens[3]>0.5) then 
                    velRight=maxVel
                else
                    velRight=maxVel*0.25
                end
            else
                velRight=maxVel
                velLeft=maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front. Use Braitenberg to avoid it
                    for i=1,4,1 do
                        velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                        velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                    end
                end
            end
        end
        if (opMode==1) then -- We wanna follow something!
            index=math.floor(1+math.mod(st*3,9))
            for i=1,9,1 do
                if (index==i) then
                    ledColors[i]={0,0.5,1} -- light blue
                else
                    ledColors[i]={0,0,0} -- off
                end
            end
            velRightFollow=maxVel
            velLeftFollow=maxVel
            minDist=1000
            for i=1,8,1 do
                velLeftFollow=velLeftFollow+maxVel*braitAllSensFollow_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightFollow=velRightFollow+maxVel*braitAllSensFollow_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                if (proxSensDist[i]<minDist) then
                    minDist=proxSensDist[i]
                end
            end

            velRightAvoid=0
            velLeftAvoid=0
            for i=1,8,1 do
                velLeftAvoid=velLeftAvoid+maxVel*braitAllSensAvoid_leftMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
                velRightAvoid=velRightAvoid+maxVel*braitAllSensAvoid_rightMotor[i]*(1-(proxSensDist[i]/noDetectionDistance))
            end
            if (minDist>0.025*s) then minDist=0.025*s end
            t=minDist/(0.025*s)
            velLeft=velLeftFollow*t+velLeftAvoid*(1-t)
            velRight=velRightFollow*t+velRightAvoid*(1-t)
        end
        simSetJointTargetVelocity(leftMotor,velLeft)
        simSetJointTargetVelocity(rightMotor,velRight)
        actualizeLEDs()
        simSwitchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

-- Put some initialization code here:
simSetThreadSwitchTiming(200) -- We will manually switch in the main loop
bodyElements=simGetObjectHandle('ePuck_bodyElements')
leftMotor=simGetObjectHandle('ePuck_leftJoint')
rightMotor=simGetObjectHandle('ePuck_rightJoint')
ePuck=simGetObjectHandle('ePuck')
ePuckBase=simGetObjectHandle('ePuck_base')
ledLight=simGetObjectHandle('ePuck_ledLight')
proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=simGetObjectHandle('ePuck_proxSensor'..i)
end
maxVel=120*math.pi/180
ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

-- Braitenberg weights for the 4 front prox sensors (avoidance):
braitFrontSens_leftMotor={1,2,-2,-1}
-- Braitenberg weights for the 2 side prox sensors (following):
braitSideSens_leftMotor={-1,0}
-- Braitenberg weights for the 8 sensors (following):
braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Put some clean-up code here:

for i=1,9,1 do
    ledColors[i]={0,0,0} -- no light
end
actualizeLEDs()
