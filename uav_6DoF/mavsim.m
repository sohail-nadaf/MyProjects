%##########################################################################
% -6DoF simulation of a Fixed Wing UAV
%       - initial conditions are defined in msg_state.m
%       - simulation parameters in 'parameters/simulation parameters.m'
%       - the aerodynamic coefficents defined in 'parameters/aerosonde.m'
%       
%       - change the control deflections in range (+45, -45), 
%         throttle in range(0,1) before every simulation.
%##########################################################################
clear
run('parameters/simulation_parameters')  % load SIM: simulation parameters
run('parameters/aerosonde_parameters')  % load MAV: aircraft parameters
% initialize the mav viewer
addpath('model_viewer'); mav_view = spacecraft_viewer();  
addpath('data_viewer'); data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap4_video.avi', SIM.ts_video); end

%initialize the dryden parameters by passing msg_state Va

% initialize elements of the architecture
addpath('models'); mav = mav_dynamics(SIM.ts_simulation, MAV);
dryd = dryden(mav.true_state.Va);
%addpath('models'); 
wind = wind_simulation(SIM.ts_simulation,dryd);
%define steady wind vector in the body frame (m/s)
steady_wind = [-5; 5; 0.5];
% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------set control surfaces-------------
     delta_e = 6.3*pi/180;
    delta_t = 1;
    %delta_a = 0;  
    delta_r = 0*pi/180 ;
    % delta = [delta_a; delta_e; delta_t; delta_r];
    if(sim_time > SIM.end_time/2)
        delta_e = 6.3*pi/180;
    else
        delta_e = 6.3*pi/180;
    end
    if(sim_time > SIM.end_time/2)
        delta_a = -25*pi/180;
    else
        delta_a = 25*pi/180;
    end
    if(sim_time > SIM.end_time/2)
        delta_a = -25*pi/180.;end
    delta = [delta_a; delta_e; delta_t; delta_r];



    %-------physical system-------------
    % current_wind = wind.update();
    % set steady wind in m/s
    current_wind = wind.update(steady_wind,mav.true_state.Va);
    mav.update_state(delta, current_wind, MAV);
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);  % plot body of MAV
    data_view.update(mav.true_state,... % true states
                     mav.true_state,... % estimated states
                     mav.true_state,... % commmanded states
                     SIM.ts_simulation); 
    if VIDEO, video.update(sim_time);  end
    pause(0.01);
    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;

end

if VIDEO, video.close(); end

