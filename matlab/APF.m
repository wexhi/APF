clear;clc
    % 参数设置
    start = [-2.3, 1.4];   % 创建机器人起始位置
    goal = [17, -15];        % 目标位置
    k_att = 0.1;              % 引力增益参数
    k_rep = 50;             % 斥力增益参数
    d_safe = 4;             % 安全距离        
    
    
    % 创建障碍物
    %obstacles = [5,11.5;12.3,12;13,6.8];
    obstacles = [5,-5.15;5,-11;10.5,-4];
    
    % 创建机器人起始位置
    robot = start;
    
    % 迭代次数和步长
    maxIterations = 1000;
    stepSize = 0.2;
    
    % 轨迹记录器
    trajectory = robot;
    
    % 迭代更新机器人位置
    for i = 1:maxIterations
        % 计算机器人受到的引力
        attForce = k_att * (goal - robot);
        
        % 计算机器人受到的斥力
        repForce = [0, 0];
        for j = 1:size(obstacles, 1)
            obstacle = obstacles(j, :);
            dist = norm(robot - obstacle);
            if dist < d_safe
                repForce = repForce + k_rep * (1 / dist - 1 / d_safe) * (robot - obstacle) / dist^3;
            end
        end
        
        % 更新机器人位置
        robot = robot + stepSize * (attForce + repForce);
        
        % 记录轨迹
        trajectory = [trajectory; robot];
        
        % 绘制机器人和障碍物
        clf;
        hold on;
        plot(start(1),start(2), 'b*', 'MarkerSize', 10);
        plot(goal(1), goal(2), 'r*', 'MarkerSize', 10);
        plot(robot(1), robot(2), 'bo', 'MarkerSize', 5);
        plot(obstacles(:, 1), obstacles(:, 2), 'ks', 'MarkerSize', 8);
        plot(trajectory(:, 1), trajectory(:, 2), 'g--', 'LineWidth', 1);
        xlim([0, 20]);
        ylim([-20, 0]);
        drawnow;
        
        % 判断是否到达目标位置
        if norm(robot - goal) < 0.1
            disp('目标达到！');
            break;
        end
    end


