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

rosGetCameraData=function()
    local data,w,h=simGetVisionSensorCharImage(handleCamera)
    d={}
    d['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="a"}
    d['height']=h
    d['width']=w
    d['encoding']='rgb8'
    d['is_bigendian']=1
    d['step']=w*3
    d['data']=data
    return d
end

rosGetProximityData=function(proxData)
    d={{}, {}, {}, {}, {}, {}, {}, {}}

    for i=1,8,1 do
        d[i]['header']={seq=0,stamp=simExtRosInterface_getTime(), frame_id="base_prox" .. (i-1)}
        d[i]['radiation_type']=1 --infrared
        d[i]['field_of_view']=0.52359878 -- 30 deg
        d[i]['min_range']=0.0015
        d[i]['max_range']=0.04
        d[i]['range']= proxData[i]
    end
    return d
end

rosCallbackCmdLed=function(msg)
    for i=1,9,1 do
        if (msg.data[i] == 1) then
            ledColors[i] = {1, 0, 0} --red
        else
            ledColors[i] = {0, 0, 0} --off
        end
    end
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

        if (opMode==2) then -- ROS interface
            -- publish camera
            simExtRosInterface_publish(pubCamera,rosGetCameraData())

            -- publish proximity
            proximityData = rosGetProximityData(proxSensDist)
            for i=1,8,1 do
                simExtRosInterface_publish(pubProx[i], proximityData[i])
            end
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

-- ROS interface initialization
ePuckName = simGetObjectName(ePuck)
simAddStatusbarMessage('epuck '.. ePuckName ..' initializing')
-- Check if the required RosInterface is there:
-- http://www.coppeliarobotics.com/helpFiles/en/rosTutorialIndigo.htm
moduleName=0
index=0
rosInterfacePresent=false
while moduleName do
    moduleName=simGetModuleName(index)
    if (moduleName=='RosInterface') then
        rosInterfacePresent=true
    end
    index=index+1
end

--setup handles for various components
handleCamera=simGetObjectHandle('ePuck_camera')

--create publishers (proximity, imu, camera, ground_truth)
pubCamera = simExtRosInterface_advertise('/' .. ePuckName ..'/camera', 'sensor_msgs/Image')
simExtRosInterface_publisherTreatUInt8ArrayAsString(pubCamera) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)

pubProx = {}
pubProx[1] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity1', 'sensor_msgs/Range')
pubProx[2] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity2', 'sensor_msgs/Range')
pubProx[3] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity3', 'sensor_msgs/Range')
pubProx[4] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity4', 'sensor_msgs/Range')
pubProx[5] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity5', 'sensor_msgs/Range')
pubProx[6] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity6', 'sensor_msgs/Range')
pubProx[7] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity7', 'sensor_msgs/Range')
pubProx[8] = simExtRosInterface_advertise('/' .. ePuckName .. '/proximity8', 'sensor_msgs/Range')

--create subscribers (leds, movement)
subLED=simExtRosInterface_subscribe('/' .. ePuckName .. '/cmd_led','std_msgs/UInt8MultiArray','rosCallbackCmdLed')

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
