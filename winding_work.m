%IFS Honors Projet Script 10/14/19
%% 
clear all
clc

motor_data=csvread('motor_raw_data_nums.csv');
motor_number=motor_data(:,1);
motor_weight=motor_data(:,2); %kg
motor_Kt=motor_data(:,3); %(N*m)/amp
motor_Rm=motor_data(:,4); %ohm
motor_Fi=motor_data(:,6); %(N*m)/rpm
motor_Tf=motor_data(:,7); %N*m

Volt=24; %power supply from two car batteries wired in series


deltaT=75; %deg C
Cp=420; %J/(kg*C) Cp of Iron (stator)

%LEAD SCREW INFO
screw_lead=0.00254; %meters = 0.1 in
screw_lead_starting_length = 1.38176; %meters = 54.4 inches
screw_lead_length=screw_lead_starting_length; %meters

gate_movement=linspace(0,90,91); %angles gate will move through as it opens

%Stall torque uneffected by windings
for motor_iteration=1:length(motor_number)
   stall_torque_no_windings(motor_iteration)=(motor_Kt(motor_iteration)/motor_Rm(motor_iteration))*Volt; %N*m
end


%Winding work
winding_change=-2;
motor_Rm=motor_Rm*1.59^winding_change;
motor_Kt=motor_Kt*1.26^winding_change;


%calculates the motor stall torque in N*m
for motor_iteration=1:length(motor_number)
   stall_torque(motor_iteration)=(motor_Kt(motor_iteration)/motor_Rm(motor_iteration))*Volt; %N*m
end

%calculates the required screw torque (N*m) and actuating force (N) to open
%the gate at each angle
for gate_angles=0:90
    [screw_torque(gate_angles+1),screw_force(gate_angles+1)]=screw_torque_requirement(gate_angles);
end


%calculates the power output of a motor at a specified efficiency
efficiency=0.7;
for motor_iteration=1:length(motor_number)
    
    efficiency_torque(motor_iteration)=stall_torque(motor_iteration)*(1-efficiency); %N*m
    efficiency_rad_s(motor_iteration)=motor_speed(efficiency_torque(motor_iteration),stall_torque(motor_iteration),motor_Rm(motor_iteration),motor_Kt(motor_iteration)); %rad/s
    efficiency_power_W(motor_iteration)=efficiency_torque(motor_iteration)*efficiency_rad_s(motor_iteration);
    
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
    required_efficiency_GR(motor_iteration)=screw_torque(1)/efficiency_torque(motor_iteration);
    
    time=0;
    time_gate_angle=0;
    counter=1;
    timestep=.1;
    screw_lead_length=screw_lead_starting_length;
    energy_dissipdated_J(motor_iteration)=0;
    
    while time<=20 && time_gate_angle<=90
        
        %required toque to move the gate 
        [screw_torque_time(counter),screw_force_time(counter)]=screw_torque_requirement(time_gate_angle);
        
        %motor torque output required after the required_efficiency_GR
        efficiency_motor_GR_torque(counter)=screw_torque_time(counter)/required_efficiency_GR(motor_iteration);
        
        %speed of motor at that required torque
        [motor_rad_s_time(counter)]=motor_speed(efficiency_motor_GR_torque(counter),stall_torque(motor_iteration),motor_Rm(motor_iteration),motor_Kt(motor_iteration));
        
        %the speed of the screw in rad/s
        screw_rad_s(counter)=motor_rad_s_time(counter)/required_efficiency_GR(motor_iteration);
        
        %distance the lead screw extends during the time step at the speed
        %calculated above
        rotations_traveled_per_timestep= screw_rad_s(counter) * 0.16 * timestep; %rotations,convert radians to rotations
        
        distance_traveled_per_timestep = rotations_traveled_per_timestep * screw_lead; %meters
        
        %new length of the lead screw at time time=time+timestep
        screw_lead_length = screw_lead_length+distance_traveled_per_timestep; %meters
        
        %new gate angle at time=time+timestep
        time_gate_angle=gate_angle_function(screw_lead_length);
        
        %motor power at timepoint
        motor_power_forward_int(motor_iteration,counter)=efficiency_motor_GR_torque(counter)*motor_rad_s_time(counter);
        
        
        %POWER DISSIPATION CALCULATIONS
        %10/18/19
            %winding losses ; (Tmotor/Kt)^2*R ; Watts
            winding_losses_W=(efficiency_motor_GR_torque(counter)/motor_Kt(motor_iteration))^2*motor_Rm(motor_iteration);
            
            %Hysteresis Losses ; Tf*w ; Watts
            hysteresis_losses_W=motor_Tf(motor_iteration)*motor_rad_s_time(counter);
            
            %Viscous losses ; Tv=Fi*w(rpm)  => Tv*w (rad/s) ; Watts
            Tv=motor_Fi(motor_iteration)*motor_rad_s_time(counter)*9.549;
            viscous_losses=Tv*motor_rad_s_time(counter); 
            
            %total power dissipation during opening time
            power_dissipation_total_W=winding_losses_W+hysteresis_losses_W+viscous_losses;
            
            %energy dissipated during opening time
            energy_dissipdated_J(motor_iteration)=energy_dissipdated_J(motor_iteration)+power_dissipation_total_W*timestep;

        
        
        
        time=time+timestep;
        counter=counter+1;
        
        
        
       
    end
    
    if motor_iteration==31
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
        plot(1:counter-1,efficiency_motor_GR_torque(1:counter-1))
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
    forward_integration_results(motor_iteration,3)=efficiency_power_W(motor_iteration)*0.3572; %power of the motor times the effiiency of the lead screw and the gear ratio
    forward_integration_results(motor_iteration,4)=required_efficiency_GR(motor_iteration); %GR required
    forward_integration_results(motor_iteration,5)=energy_dissipdated_J(motor_iteration); %joules of energy dissipated during opening time
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



figure
plot(speed,0:0.1:stall_torque(motor_efficiency_number))
xlabel('Motor speed (rad/s)')
ylabel('Torque (N*m)')
title('Torque vs speed curve for motor 1803')


figure
plot(efficiency,0:0.1:stall_torque(motor_efficiency_number))
xlabel('Motor Efficiency')
ylabel('Torque (N*m)')
title('Torque vs Efficiency for motor 1803')


%4th meeting plot generations 
%10/18/19

figure
plot(0:gate_angles,screw_torque)
title('Required Screw Torque VS Gate Angle')
xlabel('Gate angle (deg)')
ylabel('Screw Torque (N*m)')
%}


%% winding changes plot

%Winding work
for winding_changes=-1:1

speed_winding=[0]    
current_winding=[0]
power_in_winding=[0]
power_out_winding=[0]
efficiency_winding=[0]

motor_Rm_winding=motor_Rm*1.59^winding_changes;
motor_Kt_winding=motor_Kt*1.26^winding_changes;


%calculates the motor stall torque in N*m
for motor_iteration=1:length(motor_number)
   stall_torque_winding(motor_iteration)=(motor_Kt_winding(motor_iteration)/motor_Rm_winding(motor_iteration))*Volt; %N*m
end

%calculates the torque speed curve and the efficiency for a motor
motor_efficiency_number=28
counter=1
for test_torques_NM=0:0.1:stall_torque_winding(motor_efficiency_number)
    
    %calculates the motor speed in rad/s at a torque requirment in N*m
    speed_winding(counter)=motor_speed(test_torques_NM,stall_torque_winding(motor_efficiency_number),motor_Rm_winding(motor_efficiency_number),motor_Kt_winding(motor_efficiency_number));
    current_winding(counter)=test_torques_NM/motor_Kt_winding(motor_efficiency_number);
    power_in_winding(counter)=current_winding(counter)*Volt;
    power_out_winding(counter)=test_torques_NM*speed_winding(counter);
    efficiency_winding(counter)=power_out_winding(counter)/power_in_winding(counter);
    counter=counter+1;
end


plot(speed_winding,0:0.1:stall_torque_winding(motor_efficiency_number))
xlabel('Motor speed (rad/s)')
ylabel('Torque (N*m)')
title('Torque vs speed curve for motor 1803 under different windings')
hold on


end
legend('-1 (thicker guage)','0 (standard)','1 (finer guage)')


%%



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




