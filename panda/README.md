# Steps for BASEBOT: No simulation

1. Start Motive (Optitrack Software) at src-kvm-fieldbay.stanford.edu
    a. Ensure multiple markers on ball can be seen and Rigid Body 4 is checked.
    b. Ensure 4 markers for robot base can be seen and Rigid Body 5 is checked.
    b. Enable Nat Net streaming in Settings > Streaming
    c. Ensure IP address is serverAddress: 172.24.68.77
2. Start streaming Optitrack data
    a. Connect personal computer to SRC Wifi
    b. Manually set IP to clientAddress: 172.24.68.60
    c. Run StreamData.py
    d. You should see data printing to the screen. Can also run 'redis-cli' in a server
3. Put ball BEHIND start line. Run realball_realbot.py.
    a. Should start printing out reasonable ball values relative to robot arm base.
4. Go to website and prep arm. Run sh launch_driver.sh to launch the driver.
5. Run ./controller_catch. Arm should move to starting position.
6. Ready to throw! After crossing 0 position of arm in room, go back to start line to reset. Should see arm move back to starting position between throws. If not, expose ball to make sure position was registered. 


# Steps for BASEBOT: Real Ball in Optitrack + Catching in Simulation

0. Start redis client (if haven't already)
1. Start Motive (Optitrack Software) at src-kvm-fieldbay.stanford.edu
    a. Ensure multiple marker on ball can be seen and Rigid Body 4 is checked.
    b. Enable Nat Net streaming in Settings > Streaming
    c. Ensure IP address is serverAddress: 172.24.68.77
2. Start streaming Optitrack data
    a. Connect personal computer to SRC Wifi
    b. Manually set IP to clientAddress: 172.24.68.64
    c. Run StreamData.py
    d. You should see data printing to the screen. Can also run 'redis-cli' in a server
3. Run simviz_Motiv.cpp (by running ./simviz_Motiv in the bin folder)
    a. Need to hit enter to start simulation
4. Run controller_catcher.cpp (by running ./controller_catch in the bin folder)
5. Run realball_simbot.py
    a. Ball needs to start behind start line in code

NOTE: simviz may crash when running realball_simbot.py. Just restart scripts if so



# Panda with gripper example 

The panda is controlled at the wrist point (before the gripper) with a pose task, and the gripper (modeled as two prismatic joints) is controlled with a partial joint task. 