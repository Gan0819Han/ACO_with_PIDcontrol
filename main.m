% =========================================================================
%         使用蚁群算法(ACO)优化PID参数 (包含高级可视化功能)
% =========================================================================
%
% 功能:
% 本脚本实现了一个蚁群优化算法，用于自动整定PID控制器的三个核心参数：
% Kp, Ki, Kd。它不仅能找到最优参数，还能通过多种图表深入揭示算法的
% 内部工作机制。
%
% 新增可视化功能:
% 1. **最优路径演化图**: 动态展示构成最优PID参数的15位编码在迭代过程
%    中的变化轨迹，直观地看出算法的收敛过程。
% 2. **信息素热力图**: 展示最终的信息素矩阵分布，揭示蚁群的“集体智慧”，
%    即算法最终倾向于选择哪些数字组合。
% 3. **迭代过程文本输出**: 在设定的迭代节点，打印当前最优解的编码，
%    方便在控制台实时追踪。
%
% =========================================================================

clear;
close all;
clc;

fprintf('开始执行蚁群算法优化PID参数...\n');

%% 1. 初始化
% --- 蚁群算法 (ACO) 参数 ---
m = 20;            % 蚂蚁数量 (Number of ants)
NC_max = 50;       % 最大迭代次数 (Maximum number of iterations)
alpha = 1;       % 信息素权重 (Pheromone weight)
beta = 2;        % 启发式信息权重 (Heuristic weight)
gamma = 0.3;       % 信息素挥发系数 (Pheromone evaporation rate)
Q = 100;           % 信息素强度系数 (Pheromone intensity coefficient)
rho = 0.1;         % 控制量权重 (Weight for control effort in performance index)

% --- 被控对象与初始PID参数 ---
[plant, Kp_zn, Ki_zn, Kd_zn] = initialize_plant();

% --- PID参数搜索范围 ---
epsilon = 0.8;
param_ranges = struct(...
    'Kp', [(1-epsilon)*Kp_zn, (1+epsilon)*Kp_zn], ...
    'Ki', [(1-epsilon)*Ki_zn, (1+epsilon)*Ki_zn], ...
    'Kd', [(1-epsilon)*Kd_zn, (1+epsilon)*Kd_zn]);

% --- 信息素矩阵初始化 ---
num_positions = 15;
num_digits = 10;
tau = ones(num_positions, num_digits);

% --- 初始化用于记录和可视化的变量 ---
global_best_J = inf;
global_best_path = zeros(1, num_positions);
history_best_J = zeros(NC_max, 1); % 记录每代最优J值 (用于收敛曲线)
history_best_path = zeros(NC_max, num_positions); % 记录每代最优路径 (用于演化图)

% --- 可视化控制参数 ---
display_iterations = 10; % 每隔10次迭代，在控制台打印一次最优路径

fprintf('初始化完成。开始主循环...\n\n');

%% 2. 蚁群算法主循环
for NC = 1:NC_max
    ant_paths = zeros(m, num_positions);
    ant_J_values = zeros(m, 1);

    for k = 1:m
        path_k = select_path(tau, alpha, beta);
        ant_paths(k, :) = path_k;
        [Kp, Ki, Kd] = decode_params(path_k, param_ranges);
        J_k = calculate_performance(Kp, Ki, Kd, plant, rho);
        ant_J_values(k) = J_k;
    end
    
    [min_J_iter, min_idx_iter] = min(ant_J_values);
    if min_J_iter < global_best_J
        global_best_J = min_J_iter;
        global_best_path = ant_paths(min_idx_iter, :);
    end
    
    % --- 记录历史数据用于后续可视化 ---
    history_best_J(NC) = global_best_J;
    history_best_path(NC, :) = global_best_path;
    
    tau = update_pheromone(tau, gamma, Q, global_best_J, global_best_path);
    
    fprintf('迭代 %d / %d: 当前最优性能 J = %.4f\n', NC, NC_max, global_best_J);
    
    % --- [新增] 按设定的频率打印当前最优路径的编码 ---
    if mod(NC, display_iterations) == 0 || NC == 1 || NC == NC_max
        fprintf('--------------------------------------------------\n');
        fprintf('[迭代 %d 的最优路径编码]\n', NC);
        fprintf('  >> Kp 编码: %s\n', mat2str(global_best_path(1:5)));
        fprintf('  >> Ki 编码: %s\n', mat2str(global_best_path(6:10)));
        fprintf('  >> Kd 编码: %s\n', mat2str(global_best_path(11:15)));
        fprintf('--------------------------------------------------\n');
    end
end

fprintf('\n优化完成。\n');

%% 3. 结果分析与可视化
% --- 解码并显示最终的全局最优参数 ---
[best_Kp, best_Ki, best_Kd] = decode_params(global_best_path, param_ranges);
fprintf('\n--- 最优PID参数 ---\n');
fprintf('Kp: %.4f\n', best_Kp);
fprintf('Ki: %.4f\n', best_Ki);
fprintf('Kd: %.4f\n', best_Kd);
fprintf('对应的最优性能指标 J: %.4f\n', global_best_J);

% --- 绘图 1: 算法收敛曲线 ---
figure('Name', 'ACO Convergence Curve');
plot(1:NC_max, history_best_J, 'b-', 'LineWidth', 2);
title('蚁群算法收敛曲线');
xlabel('迭代次数');
ylabel('最优性能指标 (J)');
grid on;
box on;

% --- 绘图 2: 优化前后系统阶跃响应对比 ---
C_zn = pid(Kp_zn, Ki_zn, Kd_zn);
C_aco = pid(best_Kp, best_Ki, best_Kd, 0.01); % 注意加入滤波器
sys_closed_zn = feedback(C_zn * plant, 1);
sys_closed_aco = feedback(C_aco * plant, 1);
t = 0:0.01:4;
[y_zn, ~] = step(sys_closed_zn, t);
[y_aco, ~] = step(sys_closed_aco, t);
figure('Name', 'Step Response Comparison');
plot(t, y_zn, 'r--', 'LineWidth', 1.5);
hold on;
plot(t, y_aco, 'b-', 'LineWidth', 2);
plot(t, ones(size(t)), 'k:', 'LineWidth', 1);
hold off;
title('优化前后阶跃响应对比');
xlabel('时间 (s)');
ylabel('系统输出');
legend('初始参数 (Z-N)', '优化后参数 (ACO)', '设定值', 'Location', 'SouthEast');
grid on;
box on;
ylim([0, 1.8]);

% --- [新增] 绘图 3: 最优路径编码的演化轨迹图 ---
figure('Name', 'Evolution of Best Path Digits');
iterations = 1:NC_max;
% 子图1: Kp 编码演化
subplot(3, 1, 1);
plot(iterations, history_best_path(:, 1:5), '.-', 'MarkerSize', 8);
title('Kp 编码演化轨迹');
xlabel('迭代次数');
ylabel('数字值');
grid on;
legend('Kp_1', 'Kp_2', 'Kp_3', 'Kp_4', 'Kp_5', 'Location','EastOutside');
ylim([-1, 10]);

% 子图2: Ki 编码演化
subplot(3, 1, 2);
plot(iterations, history_best_path(:, 6:10), '.-', 'MarkerSize', 8);
title('Ki 编码演化轨迹');
xlabel('迭代次数');
ylabel('数字值');
grid on;
legend('Ki_1', 'Ki_2', 'Ki_3', 'Ki_4', 'Ki_5', 'Location','EastOutside');
ylim([-1, 10]);

% 子图3: Kd 编码演化
subplot(3, 1, 3);
plot(iterations, history_best_path(:, 11:15), '.-', 'MarkerSize', 8);
title('Kd 编码演化轨迹');
xlabel('迭代次数');
ylabel('数字值');
grid on;
legend('Kd_1', 'Kd_2', 'Kd_3', 'Kd_4', 'Kd_5', 'Location','EastOutside');
ylim([-1, 10]);

% --- [新增] 绘图 4: 最终信息素矩阵热力图 ---
figure('Name', 'Final Pheromone Matrix Heatmap');
y_labels = cell(1, 15);
for i=1:5, y_labels{i} = ['Kp_', num2str(i)]; end
for i=1:5, y_labels{i+5} = ['Ki_', num2str(i)]; end
for i=1:5, y_labels{i+10} = ['Kd_', num2str(i)]; end

% [已修正] 移除tau的转置，以匹配坐标轴维度
heatmap(0:9, y_labels, tau, ...
        'Title', '最终信息素矩阵热力图', ...
        'XLabel', '可选数字', ...
        'YLabel', '参数编码位置', ...
        'Colormap', jet);
