function theta_i = Gait(robot_motion, robot_config)
    switch robot_motion.gait
        case "ZERO"
            [x, y, z] = ForwardKinematics(0, 0, 0, robot_config);
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, 0.15, -0.24];
                case "left-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, 0.15, -0.24];
                case "right-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, -0.15, -0.24];
                case "right-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, -0.15, -0.24];
                otherwise
            end
        case "FORWARD"
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [0.25, 0.15, -0.24];
                    pos_B = [0.25, 0.15, -0.20];
                    pos_C = [0.29, 0.15, -0.20];
                    pos_D = [0.29, 0.15, -0.24];
                case "left-behind"
                    pos_A = [-0.25, 0.15, -0.24];
                    pos_B = [-0.25, 0.15, -0.20];
                    pos_C = [-0.21, 0.15, -0.20];
                    pos_D = [-0.21, 0.15, -0.24];
                case "right-front"
                    pos_A = [0.25, -0.15, -0.24];
                    pos_B = [0.25, -0.15, -0.20];
                    pos_C = [0.29, -0.15, -0.20];
                    pos_D = [0.29, -0.15, -0.24];
                case "right-behind"
                    pos_A = [-0.25, -0.15, -0.24];
                    pos_B = [-0.25, -0.15, -0.20];
                    pos_C = [-0.21, -0.15, -0.20];
                    pos_D = [-0.21, -0.15, -0.24];
                otherwise
            end
        case "BACKWARD"
        case "TURN_LEFT"
        case "TURN_RIGHT"
        otherwise
    end
    
    waypoint_n = 10;
    waypoint_n_zero = 500;
    if robot_motion.gait == "ZERO"
        theta_iA{1} = [0, 0, 0];
        theta_iA{2} = [0, 0, 0];
        theta_iA{3} = [0, 0, 0];
        theta_iA{4} = [0, 0, 0];
        theta_iB{1} = [0, -pi/2,  pi/4];
        theta_iB{2} = [0, -pi/2,  pi/4];
        theta_iB{3} = [0,  pi/2, -pi/4];
        theta_iB{4} = [0,  pi/2, -pi/4];
        waypoint_jointspace_AB1{1} = linspace(theta_iA{1}(1), theta_iB{1}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{2} = linspace(theta_iA{2}(1), theta_iB{2}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{3} = linspace(theta_iA{3}(1), theta_iB{3}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{4} = linspace(theta_iA{4}(1), theta_iB{4}(1), waypoint_n_zero);
        waypoint_jointspace_AB2{1} = linspace(theta_iA{1}(2), theta_iB{1}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{2} = linspace(theta_iA{2}(2), theta_iB{2}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{3} = linspace(theta_iA{3}(2), theta_iB{3}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{4} = linspace(theta_iA{4}(2), theta_iB{4}(2), waypoint_n_zero);
        waypoint_jointspace_AB3{1} = linspace(theta_iA{1}(3), theta_iB{1}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{2} = linspace(theta_iA{2}(3), theta_iB{2}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{3} = linspace(theta_iA{3}(3), theta_iB{3}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{4} = linspace(theta_iA{4}(3), theta_iB{4}(3), waypoint_n_zero);
        
        theta_i = cell(1, 4);
        for i = 1 : waypoint_n_zero
            theta_i{i} = zeros(waypoint_n_zero, 3);
        end
        for i = 1 : waypoint_n_zero
            theta_i{1}(i, :) = [waypoint_jointspace_AB1{1}(i), waypoint_jointspace_AB2{1}(i), waypoint_jointspace_AB3{1}(i)];
            theta_i{2}(i, :) = [waypoint_jointspace_AB1{2}(i), waypoint_jointspace_AB2{2}(i), waypoint_jointspace_AB3{2}(i)];
            theta_i{3}(i, :) = [waypoint_jointspace_AB1{3}(i), waypoint_jointspace_AB2{3}(i), waypoint_jointspace_AB3{3}(i)];
            theta_i{4}(i, :) = [waypoint_jointspace_AB1{4}(i), waypoint_jointspace_AB2{4}(i), waypoint_jointspace_AB3{4}(i)];
        end
    else         
        waypoint_AB1 = linspace(pos_A(1), pos_B(1), waypoint_n);
        waypoint_AB2 = linspace(pos_A(2), pos_B(2), waypoint_n);
        waypoint_AB3 = linspace(pos_A(3), pos_B(3), waypoint_n);
        waypoint_BC1 = linspace(pos_B(1), pos_C(1), waypoint_n);
        waypoint_BC2 = linspace(pos_B(2), pos_C(2), waypoint_n);
        waypoint_BC3 = linspace(pos_B(3), pos_C(3), waypoint_n);
        waypoint_CD1 = linspace(pos_C(1), pos_D(1), waypoint_n);
        waypoint_CD2 = linspace(pos_C(2), pos_D(2), waypoint_n);
        waypoint_CD3 = linspace(pos_C(3), pos_D(3), waypoint_n);
        waypoint_DA1 = linspace(pos_D(1), pos_A(1), waypoint_n);
        waypoint_DA2 = linspace(pos_D(2), pos_A(2), waypoint_n);
        waypoint_DA3 = linspace(pos_D(3), pos_A(3), waypoint_n);
        waypoint_AB = [waypoint_AB1' waypoint_AB2' waypoint_AB3'];
        waypoint_BC = [waypoint_BC1' waypoint_BC2' waypoint_BC3'];
        waypoint_CD = [waypoint_CD1' waypoint_CD2' waypoint_CD3'];
        waypoint_DA = [waypoint_DA1' waypoint_DA2' waypoint_DA3'];
        waypoint = [waypoint_AB; waypoint_BC; waypoint_CD; waypoint_DA];

        theta_i = zeros(size(waypoint, 1), 3);
        for i = 1 : size(waypoint, 1)
            px = waypoint(i, 1);
            py = waypoint(i, 2);
            pz = waypoint(i, 3);
            [theta1, theta2, theta3] = InverseKinematics(px, py, pz, robot_config);
            theta_i(i, :) = [theta1 theta2 theta3]; 
        end
    end         
end