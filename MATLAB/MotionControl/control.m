function state = control(robot_config, robot_motion, sim, clientID, total_steps, sensor_data, slamObj, axMap, state, akfObj)
    num_loops = total_steps; 
    for i = 1 : num_loops
        robot_motion.step = 1; 
        sensor_data = control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state, akfObj); 
        [state, sensor_data] = correction_AKF(clientID, sim, sensor_data, slamObj, axMap, state, akfObj);
        drawnow limitrate;
    end
end