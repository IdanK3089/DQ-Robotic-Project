classdef GeneralRobot < DQ_VrepRobot
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = GeneralRobot(robot_name,vrep_interface,jointNumber)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
          robot_index = '';
     
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:jointNumber
                current_joint_name = {robot_name,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end
        
   function kin = kinematics(obj,numerOfJoints,DH_theta_Vector,DH_d_Vector,DH_a_Vector,DH_alpha_Vector)

            theta_count_columns=width(DH_theta_Vector);
            d_count_columns=width(DH_d_Vector);
            a_count_columns=width(DH_a_Vector);
            alpha_count_columns=width(DH_alpha_Vector);

            if(numerOfJoints~=theta_count_columns | numerOfJoints~=d_count_columns | numerOfJoints~=a_count_columns | numerOfJoints~=alpha_count_columns)
                  error('DH_Matrix is invalid, joint number has to be equal to columns')
            end

            DH_matrix = [DH_theta_Vector; DH_d_Vector;DH_a_Vector;DH_alpha_Vector];
            
            kin = DQ_SerialManipulator(DH_matrix,'standard');
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end

        
    end
end
