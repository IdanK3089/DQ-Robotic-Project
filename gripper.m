function gripper (clientID,closing)

sim=remApi('remoteApi');

 [res,j1]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active1',sim.simx_opmode_blocking);
 [res,j2]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active2',sim.simx_opmode_blocking);
   
    
[r,p1]=sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
[r,p2]=sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);

if (closing==1)
    r = sim.simxSetJointTargetPosition(clientID,j1,-0.04,sim.simx_opmode_blocking);
    r = sim.simxSetJointTargetPosition(clientID,j2,-0.045,sim.simx_opmode_blocking);
    
    if (p1<(p2-0.008))
        sim.simxSetJointTargetVelocity (clientID,j1,-0.01,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,-0.04,sim.simx_opmode_blocking);
        
    else 
        sim.simxSetJointTargetVelocity (clientID,j1,-0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,-0.04,sim.simx_opmode_blocking);
    end 
else 
    r = sim.simxSetJointTargetPosition(clientID,j1,0.01,sim.simx_opmode_blocking);
    r = sim.simxSetJointTargetPosition(clientID,j2,0.01,sim.simx_opmode_blocking);

    if (p1<p2)
        sim.simxSetJointTargetVelocity (clientID,j1,0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,0.02,sim.simx_opmode_blocking);
    else 
        sim.simxSetJointTargetVelocity (clientID,j1,0.02,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,0.04,sim.simx_opmode_blocking);
    end 
end 

end 