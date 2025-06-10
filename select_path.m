function path = select_path(tau, alpha, beta)
% =========================================================================
%                              蚂蚁选择路径
% =========================================================================
%
% 功能:
% 单个蚂蚁根据信息素矩阵 `tau` 和启发式信息，通过轮盘赌的方式选择一条
% 完整的路径（一个15位的编码）。
%
% 修改说明:
% - 原代码中有一个任意的启发式函数，现已移除。
% - 当前版本使用均匀的启发式信息，意味着路径的选择主要由信息素浓度驱动，
%   这在没有明确先验知识的情况下是更通用的做法。
%
% =========================================================================

    path = zeros(1, 15);
    num_digits = size(tau, 2); % 编码中每个位置的可选数字数量 (0-9，共10个)

    % 为路径的每一位选择一个数字
    for i = 1:15
        
        % 启发式信息 (eta)。这里我们假设没有先验知识来偏好某个数字，
        % 因此使用均匀的启发式信息。
        eta = ones(1, num_digits); 

        % 计算状态转移概率 (式6)
        % 概率正比于 (信息素浓度^alpha) * (启发式信息^beta)
        pheromone_component = tau(i, :) .^ alpha;
        heuristic_component = eta .^ beta;
        
        probabilities = pheromone_component .* heuristic_component;
        
        % 归一化，得到概率分布
        sum_probs = sum(probabilities);
        if sum_probs == 0
            % 防止因所有概率为0而出错 (例如在算法初期)
            probabilities = ones(1, num_digits) / num_digits;
        else
            probabilities = probabilities / sum_probs;
        end
        
        % --- 轮盘赌选择 ---
        r = rand();
        cumulative_prob = cumsum(probabilities);
        selected_digit_idx = find(cumulative_prob >= r, 1, 'first');
        
        % 存储选择的数字 (0-9)
        path(i) = selected_digit_idx - 1;
    end
end
