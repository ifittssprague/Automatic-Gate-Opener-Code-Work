%IFS Honors Projet Script 10/14/19




motor_data=csvread('motor_raw_data_nums.csv');
motor_number=motor_data(:,1);
motor_weight=motor_data(:,2); %kg
motor_Kt=motor_data(:,3); %(N*m)/amp
motor_Rm=motor_data(:,4); %ohm
motor_Fi=motor_data(:,6); %(N*m)/rpm
motor_Tf=motor_data(:,7); %N*m

final_windings=[0,0,0,-1,-2,-2,-2,-3,-3,-3,-3,-2,-3,-3,-3,-3,-4,-4,-4,-4,-4,-4,-5,-5,-4,-5,-5,-5,-5,-5]';
%below: rpm
final_max_rpm=[18600.4378424282,12903.0925403496,11715.9392205307,11779.0367623729,9578.96880284871,7200.71404763738,9494.76922428189,7037.75242984509,6142.48611402144,5234.94892138605,4717.90286708767,5999.52098327507,4633.28307100862,4032.94028348067,3862.91478002100,3465.77390727235,3845.53213355088,5719.75911997725,3460.20699880131,3077.47790977945,2876.92979247667,2491.37840648554,3060.30010210065,2900.49539031162,4152.54936552615,3329.00415076468,2859.69573042210,2690.04569982452,2445.93121448740,2314.95797856346]';
%below: N*m
final_max_tor=[0.00416247807078957,0.00976405932344929,0.0118880002797926,0.0130268853375619,0.0234939029817834,0.0381160124954718,0.0395915305021464,0.0724857388979812,0.104106840199604,0.143446434116054,0.190716112560034,0.0773747837305107,0.161875884886142,0.229619155075114,0.292865468528573,0.383632174022100,0.438384656137271,0.121549664393288,0.300491781319957,0.409798320397174,0.545288085144451,0.723484253662563,0.667223456481609,0.829419188543173,0.321359744019648,0.622339437275959,0.965378009368471,1.26681333595053,1.60132252048911,1.89916631698364]';
%below: W
final_max_power=[8.10781337079570,13.1932818032995,14.5852720486072,16.0686349765068,23.5669047921184,28.7416464495203,39.3654591789215,53.4213886758261,66.9656465105604,78.6377036224438,94.2247510234016,48.6121290589444,78.5415752326990,96.9747175627439,118.470961239207,139.233540951452,176.538896347883,72.8048180746559,108.883806708319,132.066824582568,164.279695555991,188.754535876700,213.827687869594,251.927093133333,139.744554759559,216.955306493649,289.098505189148,356.862490597445,410.158454895213,460.399378501451]';
Volt=24; %power supply from two car batteries wired in series


deltaT=75; %deg C
Cp=420; %J/(kg*C) Cp of Iron (stator)

%LEAD SCREW INFO
screw_lead=0.00254; %meters = 0.1 in
screw_lead_starting_length = 1.38176; %meters = 54.4 inches
screw_lead_length=screw_lead_starting_length; %meters


%Winding work
motor_Rm=motor_Rm.*1.59.^final_windings;
motor_Kt=motor_Kt.*1.26.^final_windings;


%calculates the motor stall torque in N*m
for motor_iteration=1:length(motor_number)
   stall_torque(motor_iteration)=(motor_Kt(motor_iteration)/motor_Rm(motor_iteration))*Volt; %N*m
end


%calculates the required screw torque (N*m) and actuating force (N) to open
%the gate at each angle
for gate_angles=0:90
    [screw_torque(gate_angles+1),screw_force(gate_angles+1)]=screw_torque_requirement(gate_angles);
end

%Energy to heat up calculations for each motor
%assuming stator is the part of the motor which heats up and that its 60%
%of the motor mass
for motor_iteration=1:length(motor_number)
    stator_mass(motor_iteration)=motor_weight(motor_iteration)*0.6;
    energy_to_75C_J(motor_iteration)=Cp*75*stator_mass(motor_iteration);
    
end



%Calculate the gear ratio required if the motor is to experience the max
%torque at an efficiency given above
for motor_iteration=1:30
    required_GR(motor_iteration)=screw_torque(20)/final_max_tor(motor_iteration);
    
    time=0;
    time_gate_angle=0;
    counter=1;
    timestep=.001;
    screw_lead_length=screw_lead_starting_length;
    energy_dissipdated_J(motor_iteration)=0;
    
    while time<=20 && time_gate_angle<=90
        
        %required toque to move the gate 
        [screw_torque_time(counter),screw_force_time(counter)]=screw_torque_requirement(time_gate_angle);
        
        %motor torque output required after the required_efficiency_GR
        motor_GR_torque(counter)=screw_torque_time(counter)/required_GR(motor_iteration);
        
        %speed of motor at that required torque
        [motor_rad_s_time(counter)]=motor_speed(motor_GR_torque(counter),stall_torque(motor_iteration),motor_Rm(motor_iteration),motor_Kt(motor_iteration));
        
        %the speed of the screw in rad/s
        screw_rad_s(counter)=motor_rad_s_time(counter)/required_GR(motor_iteration);
        
        %distance the lead screw extends during the time step at the speed
        %calculated above
        rotations_traveled_per_timestep= screw_rad_s(counter) * 0.16 * timestep; %rotations,convert radians to rotations
        
        distance_traveled_per_timestep = rotations_traveled_per_timestep * screw_lead; %meters
        
        %new length of the lead screw at time time=time+timestep
        screw_lead_length = screw_lead_length+distance_traveled_per_timestep; %meters
        
        %new gate angle at time=time+timestep
        time_gate_angle=gate_angle_function(screw_lead_length);
        
        %motor power at timepoint
        motor_power_forward_int(motor_iteration,counter)=motor_GR_torque(counter)*motor_rad_s_time(counter);
        
        
        %POWER DISSIPATION CALCULATIONS
        %10/18/19
            %winding losses ; (Tmotor/Kt)^2*R ; Watts
            winding_losses_W=(motor_GR_torque(counter)/motor_Kt(motor_iteration))^2*motor_Rm(motor_iteration);
            
            %Hysteresis Losses ; Tf*w ; Watts
            hysteresis_losses_W=motor_Tf(motor_iteration)*motor_rad_s_time(counter);
            
            %Viscous losses ; Tv=Fi*w(rpm)  => Tv*w (rad/s) ; Watts
            Tv=motor_Fi(motor_iteration)*motor_rad_s_time(counter)*9.549;
            viscous_losses=Tv*motor_rad_s_time(counter); 
            
            %total power dissipation
            power_dissipation_total_W=winding_losses_W+hysteresis_losses_W+viscous_losses;
            
            %energy dissipated
            energy_dissipdated_J(motor_iteration)=energy_dissipdated_J(motor_iteration)+power_dissipation_total_W*timestep;

        
        
        
        time=time+timestep;
        counter=counter+1;
         
    end
    plot_number=16;
    if motor_iteration==plot_number
        figure
        plot(motor_rad_s_time(1:counter-1)*9.549,motor_GR_torque(1:counter-1))
        title('Motor 1204 torque speed characteristics while opening the gate')
        xlabel('rpm')
        ylabel('torque (N*m)')
        hold on
        plot(1:w_ten_min,TL_ten_min)
        hold on
        plot(max_power_10_min_speed(plot_number),max_power_10_min_torque(plot_number),'r*')
        legend('gate operation','10 min','max power')
        
        figure
        plot(1:counter-1,motor_rad_s_time(1:counter-1))
        title(['motor speed vs loop iterations; motor #', num2str(motor_iteration)])
        xlabel('loop iterations')
        ylabel('Motor Speed (Rad/s)')
    
        figure
        plot(1:counter-1,screw_rad_s(1:counter-1))
        title(['screw speed vs loop iterations; motor #', num2str(motor_iteration)])
        xlabel('loop iterations')
        ylabel('Screw Speed (Rad/s)')
    
        figure
        plot(1:counter-1,motor_GR_torque(1:counter-1))
        title(['motor torque vs counter; motor#', num2str(motor_iteration)])
        xlabel('loop iterations')
        ylabel('Motor Torque (N*m)')
    
        figure
        plot(1:counter-1,motor_power_forward_int(motor_iteration,1:counter-1))
        title(['motor power vs counter; motor #', num2str(motor_iteration)])
        xlabel('loop iterations')
        ylabel('Motor power (Watt)')
    
        
    end
    
    average_Q_in_W(motor_iteration)=energy_dissipdated_J(motor_iteration)/time;
    
    time_to_heat_75C_s(motor_iteration)=energy_to_75C_J(motor_iteration)/average_Q_in_W(motor_iteration); %seconds
    
    forward_integration_results(motor_iteration,1)=time_gate_angle;
    forward_integration_results(motor_iteration,2)=time;
    forward_integration_results(motor_iteration,3)=final_max_power(motor_iteration); %power of the motor times the effiiency of the lead screw and the gear ratio
    forward_integration_results(motor_iteration,4)=required_GR(motor_iteration); %GR required
    forward_integration_results(motor_iteration,5)=energy_dissipdated_J(motor_iteration); %joules
    forward_integration_results(motor_iteration,6)=time_to_heat_75C_s(motor_iteration)/60; %minutes
    
    
end



%calculates the torque speed curve and the efficiency for a motor
motor_efficiency_number=28
counter=1
for test_torques_NM=0:0.1:stall_torque(motor_efficiency_number)
    
    %calculates the motor speed in rad/s at a torque requirment in N*m
    speed(counter)=motor_speed(test_torques_NM,stall_torque(motor_efficiency_number),motor_Rm(motor_efficiency_number),motor_Kt(motor_efficiency_number));
    current(counter)=test_torques_NM/motor_Kt(motor_efficiency_number);
    power_in(counter)=current(counter)*Volt;
    power_out(counter)=test_torques_NM*speed(counter);
    efficiency(counter)=power_out(counter)/power_in(counter);
    
    counter=counter+1;
    
end

%function to calculate wall angle given lead screw length
%INPUT
%meters
%OUTPUT
%degrees
function [current_gate_angle]=gate_angle_function(lead_screw_length)
lead_screw_length_meters=lead_screw_length*39.3701;

%values defined in lead screw design analaysis
pie_angle=atand(6/12);
b=(6^2+12^2)^0.5;
c=(60^2+6^2)^0.5;
delta=atand(6/60);

alpha=acosd((lead_screw_length_meters^2-b^2-c^2)/(2*b*c));
current_gate_angle=180-alpha-delta-pie_angle-25.75; %degrees

end



%calculates the operating speed of the motor given an input toruq in N*m
    %INPUTS
    %T_motor_req = N*m
    %stall_toruqe = N*m
    %Rm in Ohms
    %Kt in (N*m)/amp
    %OUTPUTS
    %rad/s
function [motor_rad_s]=motor_speed(T_motor_req,stall_torque,Rm,Kt)
    %T_motor_req = N*m
    %stall_toruqe = N*m
    %Rm in Ohms
    %Kt in (N*m)/amp
    motor_rad_s=-(T_motor_req-stall_torque)*(Rm/(Kt^2)); %rad/s
    
end



%this function finds the required torque to move the screw at a given gate
%angle specified in degrees
%input is in degrees
% output force in N  and torque in N*m
function [screw_torque,screw_force]=screw_torque_requirement(gate_angle)
Gate_Torque=960; %lbf*in  originally 960

screw_dia=0.25; %in
screw_lead=0.1; %in
f=0.2; %coef of friction

%values defined in lead screw design analaysis
pie_angle=atand(6/12);
b=(6^2+12^2)^0.5;
c=(60^2+6^2)^0.5;
delta=atand(6/60);
alpha=180-gate_angle-delta-pie_angle;
a=(b^2+c^2-2*b*c*cosd(alpha)).^0.5;
beta=acosd((a.^2+c^2-b^2)./(2*a*c));

%axial force in the screw at a gate position
screw_force=Gate_Torque./(sind(beta).*a); %lbf

screw_force=screw_force*4.44822; %converts it to Newtons

%required torque to move the screw at a given gate position
screw_torque=((screw_force*screw_dia)/2)*((f*pi*screw_dia+screw_lead)/(pi*screw_dia-f*screw_lead)); %lbf*in

screw_torque=screw_torque*0.113; %conversts to Newtons*m
end




