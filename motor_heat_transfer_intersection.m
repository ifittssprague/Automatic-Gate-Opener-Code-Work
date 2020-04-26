%Honors project heat generation differential equation setup
%continous and 10 minute duty curves

clc
clear all
close all


motor_data=csvread('motor_raw_data_nums.csv');
motor_number=motor_data(:,1);
motor_weight=motor_data(:,2); %kg
motor_Kt=motor_data(:,3); %(N*m)/amp
motor_Rm=motor_data(:,4); %ohm
motor_Fi=motor_data(:,6); %(N*m)/rpm
motor_Tf=motor_data(:,7); %N*m
motor_TPR=motor_data(:,8); %(C/W)

Q=70;
Cp=420; %J/(kg*K)
time=0:600;

Volt=24; %volts given to the motor by two car batteries wired in series 


%calculates the motor stall torque in N*m
for motor_iteration=1:length(motor_number)
   stall_torque(motor_iteration)=(motor_Kt(motor_iteration)/motor_Rm(motor_iteration))*Volt; %N*m
end


%iterated over each motor combo
for motor_number=1:30
m=motor_weight(motor_number)*0.6; %kg
TPR=motor_TPR(motor_number); %C/W

stop=0;
Q=0.5; %W
%this while loop will stop when DT is greater than 75C
while stop==0
    
    %iterates through the 10 minutes
    counter=1;
    DT=0;
    while counter<601
       d_dt=(Q-DT(counter)/TPR)/(m*Cp);
       DT(counter+1)=DT(counter)+d_dt;
       counter=counter+1;
    end
    
    %if the final DT is less than the allowable 75C then it runs
    %it again with Q being slightly higher
    if DT(end)<75
        stop=0;
        Q=Q+0.1;
    else
        %Q_final is the final amount of continous power that can be
        %dissipated over 10 min for each motor
        Q_final(motor_number)=Q; %W
        stop=1;
    end
        
end

    plot(1:601,DT)
    xlabel('time (s)')
    ylabel('DT (C)')

end

%% Continuous opperating curves for each motor
for motor_number=1:16

    Tf=motor_Tf(motor_number); %N*m
    Fi=motor_Fi(motor_number); %(N*m)/rpm
    Kt=motor_Kt(motor_number); %(N*m)/amp
    R=motor_Rm(motor_number);  %ohm
    TPR=motor_TPR(motor_number); %(C/W)
    Q_ten_min=Q_final(motor_number); %W
    
    zero_continuous=0;
    w_continuous=1; %rpm
    TL_continous=0; %N*m
    while zero_continuous==0
        %This is the equation for the continous operating toruqe speed curve of each motor
        %NOTES on the equation
        %Tf*w = N*m*RPM -> to convert to W you need to do Tf*W*0.1047
        %same above with Fi*w^2

        %input units of w are rpm
        %output toruqe is in N*m
        TL_continous(w_continuous)=((((Kt^2)/R)*(75/TPR-(Tf*w_continuous*0.1047)-(Fi*w_continuous^2*0.1047)))^0.5)-Fi*w_continuous-Tf; %N*m

        %if the toruqe is basically zero the while loop stops and moves onto
        %the next motor, if not it keeps going
        if TL_continous(w_continuous)<.0000001
            zero_continuous=1;
        else
            w_continuous=w_continuous+1;
        end
    end %end continuous motor while loop
    
    
%% 10 minute continous duty curve 
    zero_ten_min=0;
    w_ten_min=1; %rpm
    TL_ten_min=0; %N*m
    %10 minute continous duty curve 
    while zero_ten_min==0
        %This is the equation for the 10 min operating toruqe speed curve of each motor
        %NOTES on the equation
        %Tf*w = N*m*RPM -> to convert to W you need to do Tf*W*0.1047
        %same above with Fi*w^2
        %instead of it being 75/TPR its just Q_ten_min, both are unit W
        
        %input units of w are rpm
        %output toruqe is in N*m
        TL_ten_min(w_ten_min)=((((Kt^2)/R)*(Q_ten_min-(Tf*w_ten_min*0.1047)-(Fi*w_ten_min^2*0.1047)))^0.5)-Fi*w_ten_min-Tf; %N*m

        %if the toruqe is basically zero the while loop stops and moves onto
        %the next motor, if not it keeps going
        if TL_ten_min(w_ten_min)<.0000001
            zero_ten_min=1;
        else
            w_ten_min=w_ten_min+1;
        end
        
        
    end %end 10 min while loop
    
 
%% find the max power of each motor under the 10 min operating cycle

    max_power_10_min(motor_number)=0; %W
    %where i is the counter and also rpm
    for i=1:length(TL_ten_min)
        %this is the instantanious power at the given load torque and speed i
        power_at_i=TL_ten_min(i)*i*0.105; %W
        
        %if the current instantanious power is greater than any max power
        %for the motor before it, it gets saved
        if power_at_i>max_power_10_min(motor_number)
            max_power_10_min(motor_number)=power_at_i; %W
            max_power_10_min_torque(motor_number)=TL_ten_min(i); %N*m
            max_power_10_min_speed(motor_number)=i; %rpm
        end
    end
%%  plots the 10 min and continous operating curves for motor 1803
    if motor_number==28
        figure
        plot(1:w_continuous,TL_continous)
        xlabel('speed (rpm)')
        ylabel('torque (N*m)')
        hold on
        plot(1:w_ten_min,TL_ten_min)
        title('Motor 1803 continuous and 10min safe operating curves')
        hold on
        plot(max_power_10_min_speed(motor_number),max_power_10_min_torque(motor_number),'r*')
        legend('continuous','10 min','max power point')
    end
    
    
%% winding work
    
    winding_found=0;
    winding_change=0;
    winding_count=1;
    while winding_found==0

        
        %calculates the new motor windings
        R=motor_data(motor_number,4)*1.59^winding_change;
        Kt=motor_data(motor_number,3)*1.26^winding_change;
        
        stall_torque(motor_number)=(Kt/R)*Volt;
        
        %this calculates the motor torque speed curve
        counter=1;
        speed=0;
        speed_rpm=0;
        current=0;
        power_in=0;
        power_out=0;
        efficiency=0;
        for test_torques_NM=0:0.01:stall_torque(motor_number)
            %calculates the motor speed in rad/s at a torque requirment in N*m
            speed(counter)=motor_speed(test_torques_NM,stall_torque(motor_number),R,Kt);%rad/s
            speed_rpm(counter)=speed(counter)*9.549; %converts to rpm
            current(counter)=test_torques_NM/Kt; %amp
            power_in(counter)=current(counter)*Volt; %W
            power_out(counter)=test_torques_NM*speed(counter); %W
            efficiency(counter)=power_out(counter)/power_in(counter);
            counter=counter+1;
        end

        %this plots the 10-min operating curve and the motor torque speed
        %curve and the max power point
        plot(1:w_ten_min,TL_ten_min,'LineWidth',2.0)
        hold on
        plot(speed_rpm,0:0.01:stall_torque(motor_number),'LineWidth',2.0)
        hold on
        plot(max_power_10_min_speed(motor_number),max_power_10_min_torque(motor_number),'r*','MarkerSize',12)
        xlabel('Motor speed (rpm)')
        ylabel('Torque (N*m)')
        title(['Torque vs speed curve for motor ',num2str(motor_data(motor_number,1)),' at various windings'])
        
        
        %P finds the intersection of the 10-min curve and the winding curve
        P=InterX([1:w_ten_min;TL_ten_min],[speed_rpm;0:0.01:stall_torque(motor_number)]); 
        if length(P)<2
            winding_found=1;
        else
            %difference btw the max power point and the intersection of the two
            %lines
            difference(winding_count)=abs(max_power_10_min_torque(motor_number)/P(2)-1);

            %max_power_10_min_torque(motor_number)/P(2)-1 %negative when the line is to the left

            if (max_power_10_min_torque(motor_number)/P(2)-1)>0.1 %if its more than 10% to the right of the max power point
                winding_found=1;
                winding_count=0;

            elseif difference(winding_count)>0.1
                winding_change=winding_change-1;

            elseif difference(winding_count)<0.1
                winding_found=1;
                winding_count=0;
            end



        end

        if winding_found==1
            final_winding(motor_number)=winding_change;
            winding_count=0;
        end
        
        winding_count=winding_count+1;
    end %end of winding work 
        
 %% find the max power each motor can produce with its new winding 
 %also find the torque and rpm of this motor at the max power point. 

 %calculates the new motor windings
        R=motor_data(motor_number,4)*1.59^final_winding(motor_number);
        Kt=motor_data(motor_number,3)*1.26^final_winding(motor_number);
        
        stall_torque(motor_number)=(Kt/R)*Volt;
        
        %this calculates the motor torque speed curve
        counter=1;
        speed=0;
        speed_rpm=0;
        current=0;
        power_in=0;
        power_out=0;
        efficiency=0;
        for test_torques_NM=0:0.01:stall_torque(motor_number)
            %calculates the motor speed in rad/s at a torque requirment in N*m
            speed(counter)=motor_speed(test_torques_NM,stall_torque(motor_number),R,Kt);%rad/s
            speed_rpm(counter)=speed(counter)*9.549; %converts to rpm
            current(counter)=test_torques_NM/Kt; %amp
            power_in(counter)=current(counter)*Volt; %W
            power_out(counter)=test_torques_NM*speed(counter); %W
            efficiency(counter)=power_out(counter)/power_in(counter);
            counter=counter+1;
        end
        
        %finds the intersection of the torque speed curve of the motor with
        %its final winding and the same motors 10 min safe operating curve
        
        %P finds the intersection of the 10-min curve and the winding curve
        P=InterX([1:w_ten_min;TL_ten_min],[speed_rpm;0:0.01:stall_torque(motor_number)]); 
        final_max_rpm(motor_number)=P(1);
        final_max_Tor(motor_number)=P(2);
        gear_and_screw_efficiency=0.357;
        final_max_power(motor_number)=P(1)*P(2)*0.104719755*gear_and_screw_efficiency;

        plot(P(1),P(2),'bs','MarkerSize',12,'MarkerFaceColor','b')
end %end of motor iterations

%% Winding work



%P finds the intersection of the 10-min curve and the winding curve
P=InterX([1:w_ten_min;TL_ten_min],[speed_rpm;0:0.01:stall_torque(motor_number)]); 

%this plots the 10-min operating curve and the motor torque speed curve
figure
plot(1:w_ten_min,TL_ten_min)
hold on
plot(speed_rpm,0:0.01:stall_torque(motor_number))
hold on
plot(max_power_10_min_speed(motor_number),max_power_10_min_torque(motor_number),'r*')
xlabel('Motor speed (rpm)')
ylabel('Torque (N*m)')
title('Torque vs speed curve for motor 1803')
legend('10 min safe operating','motor with standard windings')




        
        
 
%% calculates the operating speed of the motor given an input toruq in N*m
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
