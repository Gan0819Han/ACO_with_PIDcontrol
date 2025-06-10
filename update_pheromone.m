function new_tau = update_pheromone(tau, gamma, Q, global_best_J, global_best_path)
% =========================================================================
%                              更新信息素矩阵
% =========================================================================
%
% 功能:
% 更新信息素矩阵tau。此过程包括两部分：信息素挥发和信息素增强。
%
% 修改说明:
% - 采用精英策略 (Elitist Ant System) 进行更新。
% - 只有迄今为止找到的全局最优蚂蚁才被允许释放信息素。
% - 信息素的增加量与全局最优解的质量 (global_best_J) 成反比，J值越小，
%   释放的信息素越多，从而引导后续的蚂蚁向更优的区域搜索。
%
% =========================================================================

    % --- 1. 信息素挥发 ---
    % 所有路径上的信息素都会随时间蒸发一部分
    new_tau = (1 - gamma) * tau;

    % --- 2. 信息素增强 (精英策略) ---
    % 计算全局最优蚂蚁应该释放的信息素增量
    delta_tau = Q / global_best_J;

    % 将信息素增量施加到全局最优路径上
    for i = 1:length(global_best_path)
        digit_value = global_best_path(i); % 路径上第i位选择的数字 (0-9)
        digit_idx = digit_value + 1;       % 转换为MATLAB的数组索引 (1-10)
        
        new_tau(i, digit_idx) = new_tau(i, digit_idx) + delta_tau;
    end
end
