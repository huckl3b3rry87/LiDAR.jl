%% This code is written for LiDAR configuration
%  Notice the paramters come from http://velodynelidar.com/docs/datasheet/97-0038_Rev%20J_%20HDL-32E_Datasheet_Web.pdf 


%% Initialization
clc;
clear all;
close all;

%% Figure Output Setting
figure_show_long = 1;
figure_show_3d = 1;
figure_show_planar = 1;

%% Input Data
input_angle = input('Installation Angle set in degree (0 - 90 degrees):');
while (input_angle >= 90 || input_angle <= 0)
    fprintf('Warning: The input angle should between 0 and 90 degrees.\n');
    input_angle = input('Installation Angle set in degree (0 - 90 degrees):');
end    
clc;
fprintf('Your Installation Angle guess is %f degrees.\n',input_angle);
theta = input_angle / 180 * pi;

%% Parameter Setting
alpha = 15;  % upper bound of field of view (vertical): + 10.67 degrees 
beta = 15;   % lower bound of field of view (vertical): - 30.67 degrees
gamma = 2;   % angular resolution (vertical): 1.33 degrees
gamma_h = 0.4;  % angular resolution (horizontal): 0.1 - 0.4 degrees
d = 100;        % measurement range 
H = 2;          % vehicle height where LiDAR is mounted
W = 1.7;        % vehicle width where LiDAR is mounted
h = 1.5;        % object vehicle height for detection

%% Parameter Initializing
alpha = alpha / 180 * pi;
beta = beta / 180 * pi;
gamma = gamma / 180 * pi;
gamma_h = gamma_h / 180 * pi;
gamma_h = gamma_h * 2;
flag = 0;       % If there is any detection on th ground

%% Calculation Parts
% Lognitude Case
if theta <= alpha
    if abs(H/sin(theta+beta)) >= d
        flag = 0;
        Blind_spot_length = 0;
        maximum_detection_distance = d;
        maximum_longitudal_distance = 0; 
    else
        flag = 1;
        cr_angle = asin(H/d);
        maximum_detection_distance = d;
        maximum_longitudal_distance = d*cos(cr_angle);
        Blind_spot_length = H/tan(theta+beta);
    end   
elseif theta <= pi/2 - beta && theta > alpha
    if d > abs(H/sin(theta-alpha))
        flag = 1;
        Blind_spot_length = abs(H * tan(pi/2 - theta - beta));
        maximum_detection_distance = abs(H / tan(theta - alpha));
        maximum_longitudal_distance = maximum_detection_distance;
    elseif d <= abs(H/sin(theta+beta))
        flag = 0;
        Blind_spot_length = 0;
        maximum_detection_distance = d * cos(theta-alpha);
        maximum_longitudal_distance = 0; 
    else
        flag = 1;
        Blind_spot_length = abs(H * tan(pi/2 - theta - beta));
        maximum_detection_distance = d * cos(theta - alpha);
        cr_angle = asin(H/d);
        maximum_longitudal_distance = d*cos(cr_angle);
    end
elseif theta < pi/2 && theta > pi/2 - beta
    Blind_spot_length = 0;
    if d > abs(H/sin(theta-alpha))
        flag = 1;
        maximum_detection_distance = abs(H / tan(theta - alpha));
        maximum_longitudal_distance = maximum_detection_distance;
    elseif d < H
        flag = 0;
        maximum_detection_distance = d * cos(theta - alpha);
        maximum_longitudal_distance = 0;
    else
        flag = 1;
        maximum_detection_distance = d * cos(theta - alpha);
        cr_angle = asin(H/d);
        maximum_longitudal_distance = d * cos(cr_angle);
    end
end

% 3D Case
% we use rotation to find out points
upper_point = [d*cos(alpha);0;d*sin(alpha)];
lower_point = [d*cos(-beta);0;d*sin(-beta)];
% P_u0 = R_u0_d1 * P_u1
zeta = -pi/2:gamma_h:pi/2;
phi = 0:gamma:beta+alpha;
k = length(zeta);
kk = length(phi);
upper_point_set = zeros(3,k);
lower_point_set = zeros(3,k);
for i = 1:k
    upper_point_set(:,i) = Rotation_y(theta) * Rotation_z(zeta(i))*upper_point;
    lower_point_set(:,i) = Rotation_y(theta) * Rotation_z(zeta(i))*lower_point;
    if upper_point_set(3,i) < -H
        upper_point_set(:,i) = upper_point_set(:,i) * H / abs(upper_point_set(3,i));
    end
    if lower_point_set(3,i) < -H
        lower_point_set(:,i) = lower_point_set(:,i) * H / abs(lower_point_set(3,i));
    end
end
mest_pointset = cell(kk,k);
for i = 1:kk
    for j = 1:k
        mest_pointset{i}(:,j) = Rotation_y(theta) * Rotation_z(zeta(j)) * Rotation_y(-phi(i)) * lower_point;
        if mest_pointset{i}(3,j) < -H
            mest_pointset{i}(:,j) = mest_pointset{i}(:,j) * H / abs(mest_pointset{i}(3,j));
        end
    end
end
ch_upper_point = Rotation_y(theta) * upper_point;
ch_lower_point = Rotation_y(theta) * lower_point;

% Planar Case
% (none until now)

%% Output
if flag
fprintf('The length of blind spot is: %f m\n',Blind_spot_length);
fprintf('The length of detection length on the ground is: %f m\n',maximum_longitudal_distance);
else
fprintf('The ground can''t be detected!');
end

%% Visualization
if figure_show_long 
figure(1)
    plot([0,maximum_detection_distance],[0,0],'k-','lineWidth',0.5);
    hold on;
    plot([0,0],[0,H*2],'k-','lineWidth',0.5);
    hold on;
    plot([0,0],[0,-H],'k-','lineWidth',1.5);
    hold on;
    plot([0,H*2*sin(theta)],[0,H*2*cos(theta)],'b-.','lineWidth',0.5);
    hold on;
    if abs(H/tan(theta))<d
        plot([0,H/tan(theta)],[0,-H],'b-.','lineWidth',0.5);
        hold on;
    elseif abs(H/tan(theta))>=d
        plot([0,d*cos(-theta)],[0,d*sin(-theta)],'b-.','lineWidth',0.5);
        hold on;
    end

    if theta <= alpha
        if abs(H/sin(theta+beta)) >= d
            psi = -theta-beta:0.001:alpha-theta;
            plot([0,d*cos(-theta-beta)],[0,d*sin(-theta-beta)],'b--','lineWidth',1)
            hold on;
            plot([0,d*cos(alpha-theta)],[0,d*sin(alpha-theta)],'b--','lineWidth',1)
            hold on;
            plot(d*cos(psi),d*sin(psi),'b--','lineWidth',1);
        else
            cr_psi = asin(H/d);
            psi = -cr_psi:0.001:alpha-theta;
            plot([0,Blind_spot_length],[0,-H],'b--','lineWidth',1)
            hold on;
            plot([0,d*cos(alpha-theta)],[0,d*sin(alpha-theta)],'b--','lineWidth',1)
            hold on;
            plot([Blind_spot_length,maximum_longitudal_distance],[-H,-H],'b--','lineWidth',1)
            hold on;
            plot(d*cos(psi),d*sin(psi),'b--','lineWidth',1);
        end   
    elseif theta <= pi/2 - beta && theta > alpha
        if d > abs(H/sin(theta-alpha))
            plot([0,Blind_spot_length],[0,-H],'b--','lineWidth',1);
            hold on;
            plot([0,maximum_detection_distance],[0,-H],'b--','lineWidth',1);
            hold on;
            plot([Blind_spot_length,maximum_detection_distance],[-H,-H],'b--','lineWidth',1);
            hold on;
        elseif d <= abs(H/sin(theta+beta))
            plot([0,d*cos(-theta+alpha)],[0,d*sin(-theta+alpha)],'b--','lineWidth',1);
            hold on;
            plot([0,d*cos(-theta-beta)],[0,d*sin(-theta-beta)],'b--','lineWidth',1);
            hold on;
            psi = -theta-beta:0.001:-theta+alpha;
            plot(d*cos(psi),d*sin(psi),'b--','lineWidth',1);
            hold on;
        else
            cr_psi = asin(H/d);
            psi = -cr_psi:0.001:alpha-theta;
            plot([0,Blind_spot_length],[0,-H],'b--','lineWidth',1)
            hold on;
            plot([0,d*cos(alpha-theta)],[0,d*sin(alpha-theta)],'b--','lineWidth',1)
            hold on;
            plot([Blind_spot_length,maximum_longitudal_distance],[-H,-H],'b--','lineWidth',1)
            hold on;
            plot(d*cos(psi),d*sin(psi),'b--','lineWidth',1);
        end
    elseif theta < pi/2 && theta > pi/2 - beta
        if d > abs(H/sin(theta-alpha))
            plot([0,0],[0,-H],'b--','lineWidth',1);
            hold on;
            plot([0,maximum_longitudal_distance],[0,-H],'b--','lineWidth',1);
            hold on;
            plot([0,maximum_longitudal_distance],[-H,-H],'b--','lineWidth',1);
            hold on;
        elseif d < H
            plot([0,0],[0,-d],'b--','lineWidth',1);
            hold on;
            plot([0,d*cos(-theta+alpha)],[0,d*sin(-theta+alpha)],'b--','lineWidth',1);
            hold on;
        else
            cr_psi = asin(H/d);
            psi = -cr_psi:0.001:alpha-theta;
            plot([0,0],[0,-H],'b--','lineWidth',1)
            hold on;
            plot([0,d*cos(alpha-theta)],[0,d*sin(alpha-theta)],'b--','lineWidth',1)
            hold on;
            plot([0,maximum_longitudal_distance],[-H,-H],'b--','lineWidth',1)
            hold on;
            plot(d*cos(psi),d*sin(psi),'b--','lineWidth',1);
        end
    end
    xlabel('Longitudal Distance [m]');
    ylabel('Vertical Distance [m]');
    grid on;
    title('Longitual View');
    %axis equal
end

if figure_show_3d
    figure(2)
    % plotting set
    plot3([0,0],[0,0],[0,-H],'k--','lineWidth',1.5)
    hold on;
    plot3([0,0],[-W/2,W/2],[0,0],'k-','lineWidth',1.5)
    hold on;
    plot3([0,0],[W/2,W/2],[0,-H],'k-','lineWidth',1.5)
    hold on;
    plot3([0,0],[-W/2,-W/2],[0,-H],'k-','lineWidth',1.5)
    hold on;
    plot3([0,0],[-W/2,W/2],[-H,-H],'k-','lineWidth',1.5)
    hold on;
    plot3([-1,0],[-W/2,-W/2],[0,0],'k-','lineWidth',1.5)
    hold on;
    plot3([-1,0],[W/2,W/2],[0,0],'k-','lineWidth',1.5)
    hold on;
    plot3([-1,0],[-W/2,-W/2],[-H,-H],'k-','lineWidth',1.5)
    hold on;
    plot3([-1,0],[W/2,W/2],[-H,-H],'k-','lineWidth',1.5)
    hold on;
    plot3([0,d*sin(theta)],[0,0],[0,d*cos(theta)],'k--','lineWidth',0.5);
    hold on;
    plot3([0,d*cos(-theta)],[0,0],[0,d*sin(-theta)],'k--','lineWidth',0.5);
    hold on;
    plot3([0,ch_upper_point(1)],[0,ch_upper_point(2)],[0,ch_upper_point(3)],'b--','lineWidth',1);
    hold on;
    plot3([0,ch_lower_point(1)],[0,ch_lower_point(2)],[0,ch_lower_point(3)],'b--','lineWidth',1);
    hold on;
    for i = 1:kk
        for j = 1:k
            plot3(mest_pointset{i}(1,:),mest_pointset{i}(2,:),mest_pointset{i}(3,:),'g.');
            hold on;
        end
    end
    plot3(upper_point_set(1,:),upper_point_set(2,:),upper_point_set(3,:),'r-','lineWidth',1);
    hold on;
    plot3(lower_point_set(1,:),lower_point_set(2,:),lower_point_set(3,:),'r-','lineWidth',1);
    hold on;
    grid on;
    xlabel('x - Longitudal Distance [m]');
    ylabel('y - Latitude Distance [m]');
    zlabel('z - Vertical Distance [m]');
    axis equal
    title ('3D View');
end

if figure_show_planar
    figure(3)
    for i = 1:kk
        for j = 1:k
            plot(mest_pointset{i}(1,:),mest_pointset{i}(2,:),'g.');
            hold on;
        end
    end
    plot(upper_point_set(1,:),upper_point_set(2,:),'r-','lineWidth',1);
    hold on;
    plot(lower_point_set(1,:),lower_point_set(2,:),'r-','lineWidth',1);
    hold on;
    title('Planar View')
    xlabel('x - Longitudal Distance [m]');
    ylabel('y - Latitude Distance [m]');
    axis equal
    grid on;
end