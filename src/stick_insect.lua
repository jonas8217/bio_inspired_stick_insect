function sysCall_init()

    -- test variable
    max_distance_x = 0
    distance_found = false

    -- Settings
    use_muscle_model = true
    log_finish_distance = false
    maxTorque = 10.0 -- The maximum torque the joints are allowed to exitbit
    
    -- joint offset values for force control (muscle model)
    jointOffets = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    offsetSet = false

    jointName = {'TR0','TR1','TR2','TL0','TL1','TL2','CR0','CR1','CR2','CL0','CL1','CL2','FR0','FR1','FR2','FL0','FL1','FL2'}
    bodypartName = {'abdomen', 'thorax_hind','thorax_front','caput'}


    -- Init Handles
    jointHandles = {}
    bodypartHandles = {}

    for i = 1, 4, 1 do -- 4
        bodypartHandles[i] = sim.getObjectHandle(bodypartName[i])
    end
    for i = 1, 18, 1 do 
        jointHandles[i] = sim.getObjectHandle(jointName[i])
    end
    
    
    -- camera variables
    first = 0
    camera_bodyTrack_3D = sim.getObjectHandle('Camera_3D')
    camera_bodyTrack_top = sim.getObjectHandle('Camera_top')
    camTop_init_pos = {0,0,0}
    
    
    -- Create Publisher/subscriber
    jointTargetPos = simROS.subscribe('target/joint_positions', 'std_msgs/Float64MultiArray', 'setJointTarget')
    jointNeutralPosition = simROS.subscribe('target/joint_offsets', 'std_msgs/Float64MultiArray', 'setjointNeutralPosition')
    simState_pub = simROS.advertise('simulation_state','std_msgs/Float32')
    
    -- global var for simulation_state publisher
    lastSimState = sim.getSimulationState()
    
    -- Run ROS node
    local rosnode = {'stick_insect_sim_controller'}
    for i = 1,table.getn(rosnode),1 do
        result=sim.launchExecutable( sim.getStringParameter(sim.stringparam_scene_path) .. '/../../../devel/lib/stick_insect_sim_pkg/'..rosnode[i])
    end
    if (result==false) then
        sim.displayDialog('Error','External ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

end

function simulation_test_function()
    -- test if x distance is bigger than the last
    x_distance = math.abs(sim.getObjectPosition(bodypartHandles[2], -1)[1])
    t = sim.getSimulationTime()
    if (max_distance_x < x_distance) then
        max_distance_x = x_distance
    end
    end_time = 10 -- seconds
    if(t > end_time) then
        if(not distance_found) then -- makes sure it's unly run once
            distance_found = true
            print("$$  Robot distance = " .. max_distance_x)
            print("time " .. t)

            -- write to file
            local file = io.open("" .. "distance_02_slip_muscle.csv", "a")
            io.output(file)
            io.write(max_distance_x .. '\n')
            io.close(file)

            sim.stopSimulation()
        end
    end

end

function setjointNeutralPosition(msg)
    if offsetSet == false then
        offsets = msg.data
        if table.getn(offsets) == 18 then
            for i = 1, 18, 1 do
                jointOffets[i] = offsets[i]
                sim.setJointPosition(jointHandles[i], offsets[i])
            end
        end
        offsetSet = true
    end
end




function setJointTarget(msg)
    if not use_muscle_model then
        target = msg.data
        for i = 1, 18, 1 do
            sim.setJointTargetPosition(jointHandles[i], target[i])
        end
    else
        local r = 0.05
        local K = 0
        local D = 0

        local neuron_activation = 0


        target = msg.data
        for i = 1, 18, 1 do
            local current_pos = sim.getJointPosition(jointHandles[i])
            local current_vel = sim.getJointVelocity(jointHandles[i])

            if i <= 6 then -- horizontal shoulder joints
                neuron_activation = target[i]*120
                K = 4000
                D = 50
            else -- vertical shoulder joints & elbow joints
                neuron_activation = target[i]*300
                K = 8000
                D = 70
            end

            local neutral_position = jointOffets[i]
            local error =  current_pos - neutral_position

            local tau = r*neuron_activation - r*(2*K*error*r + 2*D*current_vel*r)

            if tau > maxTorque then
                tau = maxTorque
            elseif tau < -maxTorque then
                tau = -maxTorque
            end

            sim.setJointTargetForce(jointHandles[i], tau, true)
        end


    end
end

function publishSimState()
    local simulationState = sim.getSimulationState()
    if simulationState ~= lastSimState then -- only publish simulation state if it has changed
        simROS.publish(simState_pub,{data=simulationState})
        lastSimState = simulationState
    end
end


function sysCall_sensing()

    -- Set camera_track to tracking robot body
    if first == 0 then
        first = 1
        camTop_init_pos = sim.getObjectPosition(camera_bodyTrack_top, -1)
    end

    -- SET CAMERA tracking
    body_pos = sim.getObjectPosition(bodypartHandles[2], -1)
    sim.setObjectPosition(camera_bodyTrack_3D, -1, {body_pos[1]-0.6556, body_pos[2]-0.4406  , body_pos[3] + 0.4175})
    sim.setObjectPosition(camera_bodyTrack_top, -1, {body_pos[1], body_pos[2], camTop_init_pos[3]})


    -- publish sim state (To check for sim stop in controller)
    publishSimState()

    if log_finish_distance then
        simulation_test_function()
    end
end


function sysCall_cleanup()
    simROS.shutdownSubscriber(jointTargetPos)
end