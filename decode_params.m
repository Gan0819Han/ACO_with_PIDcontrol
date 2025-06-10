function [Kp, Ki, Kd] = decode_params(path, param_ranges)
% =========================================================================
%                         将路径编码解码为PID参数
% =========================================================================
%
% 功能:
% 将蚂蚁选择的15位数字路径，解码成实际的Kp, Ki, Kd参数值。
%
% 修改说明:
% - 采用线性映射方法替代了原有的简单钳位方法。
% - 路径的每5位被视为一个整数，然后这个整数被线性地映射到对应参数的
%   搜索范围 [min, max] 内。这种方法能更充分地利用搜索空间。
%
% =========================================================================

    % 5位数字能表示的最大整数值 (从0到99999)
    max_int_val = 10^5 - 1;

    % --- 解码 Kp ---
    kp_digits = path(1:5);
    % 将数字数组转换为整数 (e.g., [1 2 3 4 5] -> 12345)
    kp_int = dot(kp_digits, 10.^(4:-1:0)); 
    kp_range = param_ranges.Kp;
    % 线性映射到Kp的搜索范围
    Kp = kp_range(1) + (kp_int / max_int_val) * (kp_range(2) - kp_range(1));

    % --- 解码 Ki ---
    ki_digits = path(6:10);
    ki_int = dot(ki_digits, 10.^(4:-1:0));
    ki_range = param_ranges.Ki;
    % 线性映射到Ki的搜索范围
    Ki = ki_range(1) + (ki_int / max_int_val) * (ki_range(2) - ki_range(1));

    % --- 解码 Kd ---
    kd_digits = path(11:15);
    kd_int = dot(kd_digits, 10.^(4:-1:0));
    kd_range = param_ranges.Kd;
    % 线性映射到Kd的搜索范围
    Kd = kd_range(1) + (kd_int / max_int_val) * (kd_range(2) - kd_range(1));
end
