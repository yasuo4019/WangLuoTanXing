function Dijkstra_s_imp_resilient_cn_v2_pic()
% 基于 modified Dijkstra 的传感器网络 QoS 路由网络弹性扩展单文件程序
% 说明：
% 1) 本程序为 MATLAB 参数化计算实验代码；
% 2) 保持单标签 modified Dijkstra 风格，不实现多标签/Pareto 机制；
% 3) 重点体现：基础 QoS 路由 -> 异常/退化建模 -> 重路由 -> 恢复评价；
% 4) 在保留原命令行输出和工作区变量输出逻辑的前提下，新增图像可视化输出层；
% 5) 所有函数均置于同一个 .m 文件中，直接运行本主函数即可。
%
% 结果说明：
% - 程序会逐场景输出路径与 QoS 指标；
% - 程序会以 S0 为基准，对 S1~S4 计算恢复指标；
% - 程序会生成 5 个图窗，对应 S0~S4 五个场景；
% - 程序会将总结果写入工作区变量：
%     results_resilient_cn_v2
%     summary_table_resilient_cn_v2

clc;
close all;

%% ========================= 1. 构建基础网络与参数 =========================
base_net = build_base_network();
params   = build_params();

%% ========================= 2. 构建场景集 =========================
scenarios = build_scenarios(base_net);

%% ========================= 3. 逐场景运行 =========================
fprintf('\n');
fprintf('===============================================================\n');
fprintf(' 基于 modified Dijkstra 的传感器网络 QoS 路由网络弹性扩展实验\n');
fprintf(' 技术主线：modified Dijkstra -> 异常/退化建模 -> 重路由 -> 恢复评价\n');
fprintf('===============================================================\n');

num_scenarios = numel(scenarios);
results = repmat(init_empty_result_struct(), 1, num_scenarios);

for k = 1:num_scenarios
    scenario = scenarios(k);

    fprintf('\n');
    fprintf('---------------------------------------------------------------\n');
    fprintf('开始计算场景 %s\n', scenario.name);
    fprintf('---------------------------------------------------------------\n');

    result = run_resilient_qos_dijkstra(scenario.net, params);
    result.scenario_name  = scenario.name;
    result.scenario_desc  = scenario.description;
    result.scenario_notes = scenario.notes;

    % 先存储基础结果
    results(k) = result;

    % 逐场景打印
    print_single_result(result, scenario);
end

%% ========================= 4. 计算恢复指标 =========================
% 以 S0 正常场景为基准，对 S1~S4 计算恢复指标
idx_normal = find(strcmp({scenarios.name}, 'S0'), 1, 'first');

if ~isempty(idx_normal)
    result_normal = results(idx_normal);

    for k = 1:num_scenarios
        if scenarios(k).need_recovery
            metrics = compute_recovery_metrics(result_normal, results(k), params, params.timing);

            results(k).T_rec  = metrics.T_rec;
            results(k).Q_ret  = metrics.Q_ret;
            results(k).S_cont = metrics.S_cont;
        else
            results(k).T_rec  = NaN;
            results(k).Q_ret  = NaN;
            results(k).S_cont = NaN;
        end
    end
else
    warning('未找到 S0 正常场景，无法计算恢复指标。');
end

%% ========================= 4.5 T_calc 诊断输出（仅命令行） =========================
% 说明：
% - 该部分用于细化观察“旧路径探测 + 重路由计算”的时间组成；
% - 仅打印到命令行，不写入 results，不进入工作区，不进入图像面板。
if ~isempty(idx_normal)
    fprintf('\n');
    fprintf('===============================================================\n');
    fprintf(' T_calc 细化诊断输出（命令行观察用）\n');
    fprintf('===============================================================\n');

    old_path = results(idx_normal).path;
    for k = 1:num_scenarios
        if scenarios(k).need_recovery
            print_tcalc_diagnostics(scenarios(k), old_path, results(k), params);
        end
    end
end

%% ========================= 5. 二次打印（含恢复指标） =========================
fprintf('\n');
fprintf('===============================================================\n');
fprintf(' 含恢复指标的场景结果复核输出\n');
fprintf('===============================================================\n');

for k = 1:num_scenarios
    print_single_result(results(k), scenarios(k));
end

%% ========================= 6. 图像可视化输出层 =========================
% 注意：
% 图像绘制放在恢复指标计算之后执行，保证 S1~S4 的右侧参数面板能够完整显示
% T_rec、Q_ret、S_cont 等恢复性指标。
render_all_figures(base_net, scenarios, results, params);

%% ========================= 7. 汇总输出 =========================
summary_tbl = summarize_results(results);

%% ========================= 8. 写入工作区变量 =========================
assignin('base', 'results_resilient_cn_v2', results);
if ~isempty(summary_tbl)
    assignin('base', 'summary_table_resilient_cn_v2', summary_tbl);
end

fprintf('\n');
fprintf('===============================================================\n');
fprintf(' 实验结束：结果结构体已写入工作区变量 results_resilient_cn_v2\n');
if ~isempty(summary_tbl)
    fprintf(' 汇总表已写入工作区变量 summary_table_resilient_cn_v2\n');
end
fprintf('===============================================================\n');

end

%% ========================================================================
function net = build_base_network()
% 构建基础网络结构体
% 本函数负责建立论文第三章所使用的基础网络模型，包括：
% - 基础 QoS 参数矩阵：Y, D, J, Z
% - 节点与链路存活状态：node_alive, link_alive
% - 退化参数：degY, degD, degJ, degZ
% - 信任矩阵：trust

% 节点数
n = 14;

% 源点与终点
s = 1;
t = 14;

% ------------------------- 1) 带宽矩阵 Y -------------------------
Y = inf(n);

% 按示意图定义链路（按从左到右方向建模，避免双向标签堆叠）
Y(1, 2) = 34; Y(1, 3) = 31;
Y(2, 7) = 33;
Y(3, 4) = 29; Y(3, 5) = 30; Y(3, 6) = 28;
Y(4, 9) = 28;
Y(5, 8) = 29;
Y(6, 8) = 27; Y(6, 10) = 27;
Y(7, 11) = 28; Y(7, 9) = 31;
Y(8, 14) = 27;
Y(9, 13) = 29; Y(9, 14) = 30;
Y(10, 12) = 26;
Y(11, 13) = 28;
Y(12, 14) = 25;
Y(13, 14) = 27;

% ------------------------- 2) 时延矩阵 D -------------------------
D = inf(n);

D(1, 2) = 1.9; D(1, 3) = 2.1;
D(2, 7) = 2.0;
D(3, 4) = 2.3; D(3, 5) = 2.2; D(3, 6) = 2.4;
D(4, 9) = 2.2;
D(5, 8) = 2.3;
D(6, 8) = 2.3; D(6, 10) = 2.4;
D(7, 11) = 2.4; D(7, 9) = 2.0;
D(8, 14) = 2.8;
D(9, 13) = 2.3; D(9, 14) = 2.4;
D(10, 12) = 2.3;
D(11, 13) = 2.2;
D(12, 14) = 2.4;
D(13, 14) = 2.0;

% ------------------------- 3) 抖动矩阵 J -------------------------
J = inf(n);

J(1, 2) = 0.70; J(1, 3) = 0.80;
J(2, 7) = 0.75;
J(3, 4) = 0.95; J(3, 5) = 0.90; J(3, 6) = 1.00;
J(4, 9) = 0.90;
J(5, 8) = 0.85;
J(6, 8) = 0.95; J(6, 10) = 0.95;
J(7, 11) = 1.00; J(7, 9) = 0.85;
J(8, 14) = 1.05;
J(9, 13) = 0.95; J(9, 14) = 0.90;
J(10, 12) = 0.95;
J(11, 13) = 0.90;
J(12, 14) = 0.95;
J(13, 14) = 0.85;

% ------------------------- 4) 丢包率矩阵 Z -------------------------
Z = inf(n);

Z(1, 2) = 0.008; Z(1, 3) = 0.010;
Z(2, 7) = 0.009;
Z(3, 4) = 0.013; Z(3, 5) = 0.011; Z(3, 6) = 0.014;
Z(4, 9) = 0.012;
Z(5, 8) = 0.011;
Z(6, 8) = 0.013; Z(6, 10) = 0.014;
Z(7, 11) = 0.014; Z(7, 9) = 0.010;
Z(8, 14) = 0.015;
Z(9, 13) = 0.012; Z(9, 14) = 0.011;
Z(10, 12) = 0.013;
Z(11, 13) = 0.012;
Z(12, 14) = 0.014;
Z(13, 14) = 0.010;

% ------------------------- 5) 节点与链路状态 -------------------------
% 节点存活向量：1 表示节点正常，0 表示节点失效
node_alive = true(1, n);

% 只有当四类基础参数均存在时，才认为该有向链路实际存在
link_exist = isfinite(Y) & isfinite(D) & isfinite(J) & isfinite(Z);

% 所有存在的链路在基础场景下均可用
link_alive = link_exist;
link_alive(1:n+1:end) = false; % 对角线不参与路由

% ------------------------- 6) 退化参数 -------------------------
% degY: 带宽下降比例
% degD: 时延增加比例
% degJ: 抖动增加比例
% degZ: 丢包率附加量
degY = zeros(n);
degD = zeros(n);
degJ = zeros(n);
degZ = zeros(n);

% ------------------------- 7) 信任矩阵 -------------------------
% trust ∈ [0,1]，对不存在链路置 0
trust = ones(n);
trust(~link_exist) = 0;
trust(1:n+1:end) = 1;

% 正常场景下允许存在轻微背景风险，但不影响整体正常性
% 这里将备用路径上的若干链路设置为轻微风险背景
trust(3, 6) = 0.97;
trust(6, 10) = 0.96;
trust(9, 13) = 0.97;
trust(8, 14) = 0.96;

% 数值安全参数：防止 log(1 - Z_eff) 出现 log(0)
eps_z = 1e-6;

% 打包网络结构体
net = struct();
net.n = n;
net.s = s;
net.t = t;

net.Y = Y;
net.D = D;
net.J = J;
net.Z = Z;

net.node_alive = node_alive;
net.link_exist = link_exist;
net.link_alive = link_alive;

net.degY = degY;
net.degD = degD;
net.degJ = degJ;
net.degZ = degZ;

net.trust = trust;
net.eps_z = eps_z;

end

%% ========================================================================
function params = build_params()
% 构建参数结构体
% 本函数负责设置：
% - QoS 权重与阈值
% - 风险惩罚的数值安全参数
% - 恢复时间分解模型中的固定参数
% - Q_ret 的加权参数

params = struct();

% ------------------------- 1) QoS 权重 -------------------------
params.w_y = -0.7;
params.w_d = 0.1;
params.w_j = 0.1;
params.w_x = 0.1;
params.w_t = 0.15;

% ------------------------- 2) QoS 阈值 -------------------------
params.y_min = 20;
params.d_max = 16;
params.j_max = 8;
params.z_max = 0.2;
params.x_min = log(1 - params.z_max);

% ------------------------- 3) 数值安全参数 -------------------------
params.eps_z = 1e-6;
params.eps_t = 1e-12;

% ------------------------- 4) Q_ret 计算权重 -------------------------
% 默认等权，依次对应：
% [带宽保持率, 时延保持率, 抖动保持率, 成功率保持率]
params.qret_weights = [0.25, 0.25, 0.25, 0.25];

% ------------------------- 5) 恢复时间分解参数 -------------------------
% 说明：
% T_rec = T_det + T_upd + T_calc + T_sw
% 其中 T_calc 为算法求解实测时间；
% T_det / T_upd / T_sw 为 MATLAB 参数化计算实验中的近似设定，
% 用于表示检测、状态更新和切换过程的抽象时间，不是协议级真实恢复时间。
timing = struct();
timing.T_det = 0.002;
timing.T_upd = 0.001;
timing.T_sw  = 0.001;
params.timing = timing;

% ------------------------- 6) 其他开关 -------------------------
params.debug = false;

end

%% ========================================================================
function scenarios = build_scenarios(base_net)
% 构建五个标准场景
% 每个场景包含：
% - 名称
% - 描述
% - 修改后的网络结构体 net
% - 是否需要计算恢复指标
% - 场景说明 notes

scenarios = repmat(struct( ...
    'name', '', ...
    'description', '', ...
    'net', [], ...
    'need_recovery', false, ...
    'notes', ''), 1, 5);

%% ========================= S0 正常场景 =========================
net0 = base_net;
scenarios(1).name = 'S0';
scenarios(1).description = '正常场景：所有节点正常，所有存在链路可用，无退化，仅保留少量轻微风险背景。';
scenarios(1).net = net0;
scenarios(1).need_recovery = false;
scenarios(1).notes = '作为恢复评价的基准场景。';

%% ========================= S1 节点失效场景 =========================
net1 = base_net;

% 节点 7 失效，同时使与节点 7 相连的链路失效
% 该故障会切断上方主通道，通常触发中下方通道重路由
failed_node = 7;
net1.node_alive(failed_node) = false;
net1.link_alive(failed_node, :) = false;
net1.link_alive(:, failed_node) = false;

scenarios(2).name = 'S1';
scenarios(2).description = '节点失效场景：节点 7 失效，与节点 7 相连链路同步失效，不额外加入链路退化与风险惩罚。';
scenarios(2).net = net1;
scenarios(2).need_recovery = true;
scenarios(2).notes = '用于体现单点节点失效后，算法是否能够完成可行重路由。';

%% ========================= S2 链路退化场景 =========================
net2 = base_net;

% 对中路通道 3->5->8->14 施加退化，驱动切换到 3->6->10->12->14
net2.degY(3, 5) = 0.26; net2.degD(3, 5) = 0.30; net2.degJ(3, 5) = 0.35; net2.degZ(3, 5) = 0.020;
net2.degY(5, 8) = 0.24; net2.degD(5, 8) = 0.28; net2.degJ(5, 8) = 0.30; net2.degZ(5, 8) = 0.015;
net2.degY(8, 14) = 0.20; net2.degD(8, 14) = 0.25; net2.degJ(8, 14) = 0.28; net2.degZ(8, 14) = 0.012;

% 本场景信任保持不变
scenarios(3).name = 'S2';
scenarios(3).description = '链路退化场景：节点均正常，仅对中路通道 3->5->8->14 关键链路施加退化。';
scenarios(3).net = net2;
scenarios(3).need_recovery = true;
scenarios(3).notes = '用于体现路径候选集中的某些链路性能退化后，最终路径与 QoS 指标的变化。';

%% ========================= S3 风险链路场景 =========================
net3 = base_net;

% 节点和链路均可用，但设置风险链路
% 场景目标：体现“风险惩罚导致路径规避”
% 将 7->9->14 风险显著提高，促使路径转向 7->11->13->14
net3.trust(7, 9) = 0.35;
net3.trust(9, 14) = 0.32;
net3.trust(9, 13) = 0.55;

% 可选叠加轻微 degZ，体现风险链路并非完全失效，而是具有更高的不确定性
net3.degZ(7, 9) = 0.015;
net3.degZ(9, 14) = 0.018;
net3.degZ(9, 13) = 0.010;

scenarios(4).name = 'S3';
scenarios(4).description = '风险链路场景：节点与链路仍可用，但若干链路 trust 降低，并叠加轻微 degZ，以体现风险惩罚导致的路径规避。';
scenarios(4).net = net3;
scenarios(4).need_recovery = true;
scenarios(4).notes = '该场景的重点不是链路物理失效，而是路径搜索对低信任链路的主动规避。';

%% ========================= S4 组合异常场景 =========================
net4 = base_net;

% 节点 8 失效（中间汇聚节点失效）
net4.node_alive(8) = false;
net4.link_alive(8, :) = false;
net4.link_alive(:, 8) = false;

% 叠加上方关键链路退化，迫使路径走 1->3->4->9->13->14 等备选
net4.degY(2, 7) = 0.20; net4.degD(2, 7) = 0.28; net4.degJ(2, 7) = 0.30; net4.degZ(2, 7) = 0.016;
net4.degY(7, 9) = 0.22; net4.degD(7, 9) = 0.30; net4.degJ(7, 9) = 0.32; net4.degZ(7, 9) = 0.018;
net4.degY(9, 14) = 0.18; net4.degD(9, 14) = 0.24; net4.degJ(9, 14) = 0.26; net4.degZ(9, 14) = 0.015;

% 对上方路径继续施加风险惩罚
net4.trust(2, 7) = 0.78;
net4.trust(7, 9) = 0.74;
net4.trust(9, 14) = 0.76;

scenarios(5).name = 'S4';
scenarios(5).description = '组合异常场景：节点 8 失效，同时对上方主链路施加退化与风险惩罚。';
scenarios(5).net = net4;
scenarios(5).need_recovery = true;
scenarios(5).notes = '用于体现“异常后重路由 + 风险/退化共存”的复合网络状态。';

end

%% ========================================================================
function result = run_resilient_qos_dijkstra(net, params)
% 主算法函数
% 在给定网络状态下，执行带 QoS 约束、带风险惩罚的 modified Dijkstra 风格搜索。
%
% 路径状态聚合方式：
% - 带宽：沿路径取瓶颈最小值
% - 时延：沿路径累加
% - 抖动：沿路径累加
% - 成功率对数项：new_X = sum(log(1 - Z_eff))
% - 路径信任度：采用“路径最小信任度原则”
%
% 代价函数：
% alt = w_y * new_Y / y_min ...
%     + w_d * new_D / d_max ...
%     + w_j * new_J / j_max ...
%     + w_x * new_X / x_min ...
%     + w_t * risk_penalty
%
% 其中 risk_penalty 基于路径信任度构造。
% 本程序采用：
%   risk_penalty = -log(max(path_trust, eps_t))
% 这样可满足：
% - trust = 1 时，惩罚为 0；
% - trust 越低，惩罚越大；
% - 仍然保持单标签 modified Dijkstra 的实现风格。

n = net.n;
s = net.s;
t = net.t;

% 初始化结果结构体
result = init_empty_result_struct();
result.scenario_name = '';
result.scenario_desc = '';
result.scenario_notes = '';

% 若源点或终点不可用，则直接返回不可达
if ~net.node_alive(s) || ~net.node_alive(t)
    result.found = false;
    result.path = [];
    result.path_str = '[]';
    result.total_cost = inf;
    result.calc_time = 0;
    return;
end

% ------------------------- 1) 标签初始化 -------------------------
dist    = inf(1, n);      % 当前总代价
prev    = nan(1, n);      % 前驱节点
visited = false(1, n);    % 是否已确定

% 以下数组用于记录“当前最佳单标签路径”对应的累计 QoS 状态
accum_Y = -inf(1, n);     % 路径瓶颈带宽
accum_D = inf(1, n);      % 路径累计时延
accum_J = inf(1, n);      % 路径累计抖动
accum_X = -inf(1, n);     % 路径累计成功率对数项
accum_T = zeros(1, n);    % 路径最小信任度

% 源点初始化
dist(s)    = 0;
accum_Y(s) = inf;
accum_D(s) = 0;
accum_J(s) = 0;
accum_X(s) = 0;
accum_T(s) = 1;

Q = 1:n;

% ------------------------- 2) 主循环计时 -------------------------
tic_calc = tic;

while ~isempty(Q)
    % 从未处理节点中选择总代价最小者
    [~, idx_local] = min(dist(Q));
    u = Q(idx_local);

    % 若剩余节点均不可达，则结束
    if isinf(dist(u))
        break;
    end

    % 将当前节点移出候选集
    Q(idx_local) = [];

    if visited(u)
        continue;
    end

    % 若当前节点已经失效，则不再展开
    if ~net.node_alive(u)
        visited(u) = true;
        continue;
    end

    visited(u) = true;

    % 若已到达目标点，则可提前结束
    if u == t
        break;
    end

    % 遍历 u 的所有可存在后继节点（避免对不存在链路做无效扫描）
    neighbors = find(net.link_exist(u, :));
    for idx_nb = 1:numel(neighbors)
        v = neighbors(idx_nb);
        % ---------- 可达性与状态检查 ----------
        if u == v
            continue;
        end

        if ~net.link_alive(u, v)
            continue;
        end

        if ~net.node_alive(v)
            continue;
        end

        if visited(v)
            continue;
        end

        % ---------- 构造有效参数 ----------
        % 有效参数建模：
        % Y_eff = Y * (1 - degY)
        % D_eff = D * (1 + degD)
        % J_eff = J * (1 + degJ)
        % Z_eff = min(1 - eps_z, Z + degZ)
        [Y_eff, D_eff, J_eff, Z_eff] = compute_effective_link_values(net, u, v);

        % 将丢包率转为成功率对数项
        X_eff = log(1 - Z_eff);

        % ---------- 累计路径参数 ----------
        new_Y = min(accum_Y(u), Y_eff);
        new_D = accum_D(u) + D_eff;
        new_J = accum_J(u) + J_eff;
        new_X = accum_X(u) + X_eff;

        % 路径信任度采用“路径最小信任度原则”
        new_T = min(accum_T(u), net.trust(u, v));

        % ---------- QoS 约束检查 ----------
        if new_Y < params.y_min
            if params.debug
                fprintf('边 %d->%d 被剪枝：带宽不足。\n', u, v);
            end
            continue;
        end

        if new_D > params.d_max
            if params.debug
                fprintf('边 %d->%d 被剪枝：时延超限。\n', u, v);
            end
            continue;
        end

        if new_J > params.j_max
            if params.debug
                fprintf('边 %d->%d 被剪枝：抖动超限。\n', u, v);
            end
            continue;
        end

        if new_X < params.x_min
            if params.debug
                fprintf('边 %d->%d 被剪枝：成功率对数项不满足约束。\n', u, v);
            end
            continue;
        end

        % ---------- 风险惩罚 ----------
        risk_penalty = -log(max(new_T, params.eps_t));

        % ---------- 修正代价函数 ----------
        alt = params.w_y * (new_Y / params.y_min) + ...
              params.w_d * (new_D / params.d_max) + ...
              params.w_j * (new_J / params.j_max) + ...
              params.w_x * (new_X / params.x_min) + ...
              params.w_t * risk_penalty;

        if params.debug
            fprintf(['u=%d, v=%d | Y_eff=%.4f, D_eff=%.4f, J_eff=%.4f, ', ...
                     'Z_eff=%.6f, X_eff=%.6f, new_T=%.4f, alt=%.6f\n'], ...
                     u, v, Y_eff, D_eff, J_eff, Z_eff, X_eff, new_T, alt);
        end

        % ---------- 单标签更新 ----------
        if alt < dist(v)
            dist(v)    = alt;
            prev(v)    = u;
            accum_Y(v) = new_Y;
            accum_D(v) = new_D;
            accum_J(v) = new_J;
            accum_X(v) = new_X;
            accum_T(v) = new_T;
        end
    end
end

T_calc = toc(tic_calc);

% ------------------------- 3) 路径恢复与结果打包 -------------------------
path = reconstruct_path(prev, s, t);
found = ~isempty(path);

result.prev = prev;
result.dist = dist;
result.calc_time = T_calc;
result.path = path;
result.path_str = path_to_str(path);
result.found = found;

if found
    success_prob = exp(accum_X(t));
    loss_rate    = 1 - success_prob;
    risk_penalty = -log(max(accum_T(t), params.eps_t));

    result.total_cost   = dist(t);
    result.bandwidth    = accum_Y(t);
    result.delay        = accum_D(t);
    result.jitter       = accum_J(t);
    result.success_prob = success_prob;
    result.loss_rate    = loss_rate;
    result.trust        = accum_T(t);
    result.risk_penalty = risk_penalty;
else
    result.total_cost   = inf;
    result.bandwidth    = NaN;
    result.delay        = NaN;
    result.jitter       = NaN;
    result.success_prob = NaN;
    result.loss_rate    = NaN;
    result.trust        = NaN;
    result.risk_penalty = NaN;
end

end

%% ========================================================================
function metrics = compute_recovery_metrics(result_normal, result_scene, params, timing_params)
% 计算恢复指标
%
% 学术含义说明：
% 1) T_rec：恢复时间
%    本文采用参数化近似模型：
%       T_rec = T_det + T_upd + T_calc + T_sw
%    其中：
%       T_det  表示异常检测时间（参数设定）
%       T_upd  表示状态更新与拓扑重构时间（参数设定）
%       T_calc 表示 MATLAB 中实际路径重计算时间（实测）
%       T_sw   表示业务切换/路由切换时间（参数设定）
%    说明：该时间是算法级 MATLAB 参数化实验中的近似恢复时间，
%    不是协议级真实恢复时间。
%
% 2) Q_ret：QoS 保持率
%    以正常场景 S0 为基准，从带宽、时延、抖动、成功率四个维度衡量
%    异常场景下 QoS 维持能力。默认采用等权加权。
%
% 3) S_cont：服务连续性
%    若仍找到满足 QoS 约束的可行路径，则记为 1；否则记为 0。

metrics = struct();
metrics.T_rec  = NaN;
metrics.Q_ret  = NaN;
metrics.S_cont = 0;

% ------------------------- 1) 服务连续性 -------------------------
metrics.S_cont = double(result_scene.found);

% ------------------------- 2) 恢复时间 -------------------------
T_calc = result_scene.calc_time;
metrics.T_rec = timing_params.T_det + timing_params.T_upd + T_calc + timing_params.T_sw;

% ------------------------- 3) QoS 保持率 -------------------------
if ~result_normal.found
    metrics.Q_ret = NaN;
    return;
end

if ~result_scene.found
    metrics.Q_ret = 0;
    return;
end

% 四个维度的保持率
q_bw      = safe_ratio(result_scene.bandwidth,       result_normal.bandwidth,    'direct');
q_delay   = safe_ratio(result_normal.delay,          result_scene.delay,         'direct');
q_jitter  = safe_ratio(result_normal.jitter,         result_scene.jitter,        'direct');
q_success = safe_ratio(result_scene.success_prob,    result_normal.success_prob, 'direct');

% 截断到 [0,1]
q_bw      = clip01(q_bw);
q_delay   = clip01(q_delay);
q_jitter  = clip01(q_jitter);
q_success = clip01(q_success);

w = params.qret_weights(:).';
w = w ./ sum(w);

metrics.Q_ret = w(1) * q_bw + ...
                w(2) * q_delay + ...
                w(3) * q_jitter + ...
                w(4) * q_success;

end

%% ========================================================================
function path = reconstruct_path(prev, s, t)
% 根据前驱数组恢复路径
% 若目标不可达，则返回空数组

path = [];

if s == t
    path = s;
    return;
end

if isnan(prev(t))
    return;
end

u = t;
path_rev = nan(1, numel(prev));
cnt = 0;
path_rev(1) = u;
cnt = 1;

while u ~= s
    u = prev(u);

    if isnan(u)
        path = [];
        return;
    end

    cnt = cnt + 1;
    path_rev(cnt) = u;
end

path = fliplr(path_rev(1:cnt));

end

%% ========================================================================
function print_tcalc_diagnostics(scenario, old_path, result_scene, params)
% 打印 T_calc 细化诊断（仅命令行）
% 输出信息包括：
% 1) 旧路径逐跳探测在何处失败及耗时；
% 2) 重路由算法求解耗时（即 result_scene.calc_time）；
% 3) 若干可行候选路径代价对比与最终最优路径说明。

fprintf('\n');
fprintf('-------------------- T_calc 诊断：%s --------------------\n', scenario.name);
probe = struct('T_probe', 0, 'status_text', '未执行');

if isempty(old_path)
    fprintf('S0 基准旧路径为空，跳过旧路径探测。\n');
else
    probe = probe_old_path_until_break(old_path, scenario.net, params);
    fprintf('旧路径：%s\n', path_to_str(old_path));
    fprintf('旧路径探测耗时 T_probe：%.6f 秒\n', probe.T_probe);
    fprintf('旧路径探测结论：%s\n', probe.status_text);
end

fprintf('重路由计算耗时 T_reroute（算法实测）：%.6f 秒\n', result_scene.calc_time);
fprintf('细化对比：T_probe + T_reroute = %.6f 秒\n', probe.T_probe + result_scene.calc_time);

% 候选路径代价对比（仅用于说明“为什么选中当前最优路径”）
all_paths = enumerate_simple_paths(scenario.net, scenario.net.s, scenario.net.t, 8, 80);
cmp = build_candidate_cost_table(all_paths, scenario.net, params);

if isempty(cmp)
    fprintf('候选路径对比：当前场景无可行候选路径。\n');
else
    [~, idx_sort] = sort([cmp.total_cost], 'ascend');
    cmp = cmp(idx_sort);

    topN = min(3, numel(cmp));
    fprintf('候选路径代价对比（前 %d 条）：\n', topN);
    for i = 1:topN
        fprintf('  #%d 路径=%s | cost=%.6f | bw=%.3f | delay=%.3f | jitter=%.3f | succ=%.6f\n', ...
            i, cmp(i).path_str, cmp(i).total_cost, cmp(i).bandwidth, ...
            cmp(i).delay, cmp(i).jitter, cmp(i).success_prob);
    end

    selected_str = path_to_str(result_scene.path);
    fprintf('算法选中路径：%s\n', selected_str);
    if strcmp(selected_str, cmp(1).path_str)
        fprintf('结论：算法选中路径与候选集中最低代价路径一致。\n');
    else
        fprintf('结论：算法选中路径与候选最低代价路径不一致（需进一步排查参数/约束）。\n');
    end
end

fprintf('---------------------------------------------------------------\n');

end

%% ========================================================================
function probe = probe_old_path_until_break(path, net, params)
% 逐跳探测旧路径在当前场景中的可通行性
% 仅用于命令行诊断，不影响算法求解与结果结构体

probe = struct();
probe.T_probe = 0;
probe.status_text = '未执行';

if isempty(path) || numel(path) < 2
    probe.status_text = '旧路径长度不足，无需探测';
    return;
end

accum_Y = inf;
accum_D = 0;
accum_J = 0;
accum_X = 0;

t_probe = tic;

for k = 1:(numel(path) - 1)
    u = path(k);
    v = path(k + 1);
    t_hop = tic;

    if ~net.node_alive(u)
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在旧路径节点 %d 处发现节点失效；该跳检查耗时 %.6f 秒', u, toc(t_hop));
        return;
    end

    if ~net.node_alive(v)
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在旧路径下一跳节点 %d 处发现节点失效；该跳检查耗时 %.6f 秒', v, toc(t_hop));
        return;
    end

    if ~net.link_exist(u, v)
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 处发现链路不存在；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end

    if ~net.link_alive(u, v)
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 处发现链路失效；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end

    [Y_eff, D_eff, J_eff, Z_eff] = compute_effective_link_values(net, u, v);

    accum_Y = min(accum_Y, Y_eff);
    accum_D = accum_D + D_eff;
    accum_J = accum_J + J_eff;
    accum_X = accum_X + log(1 - Z_eff);

    if accum_Y < params.y_min
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 后触发带宽约束失败；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end
    if accum_D > params.d_max
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 后触发时延约束失败；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end
    if accum_J > params.j_max
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 后触发抖动约束失败；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end
    if accum_X < params.x_min
        probe.T_probe = toc(t_probe);
        probe.status_text = sprintf('在边 %d->%d 后触发成功率约束失败；该跳检查耗时 %.6f 秒', u, v, toc(t_hop));
        return;
    end
end

probe.T_probe = toc(t_probe);
probe.status_text = sprintf('旧路径在当前场景仍可行；完整探测耗时 %.6f 秒', probe.T_probe);

end

%% ========================================================================
function cmp = build_candidate_cost_table(paths, net, params)
% 评估候选路径代价，返回可行路径对比表（结构体数组）

cmp = struct('path', {}, 'path_str', {}, 'total_cost', {}, ...
             'bandwidth', {}, 'delay', {}, 'jitter', {}, 'success_prob', {});

for i = 1:numel(paths)
    eval_res = evaluate_path_cost(paths{i}, net, params);
    if ~eval_res.feasible
        continue;
    end

    item = struct();
    item.path = paths{i};
    item.path_str = path_to_str(paths{i});
    item.total_cost = eval_res.total_cost;
    item.bandwidth = eval_res.bandwidth;
    item.delay = eval_res.delay;
    item.jitter = eval_res.jitter;
    item.success_prob = eval_res.success_prob;

    cmp(end + 1) = item; %#ok<AGROW>
end

end

%% ========================================================================
function eval_res = evaluate_path_cost(path, net, params)
% 评估给定路径在当前网络下的可行性与总代价

eval_res = struct();
eval_res.feasible = false;
eval_res.total_cost = inf;
eval_res.bandwidth = NaN;
eval_res.delay = NaN;
eval_res.jitter = NaN;
eval_res.success_prob = NaN;

if isempty(path) || numel(path) < 2
    return;
end

accum_Y = inf;
accum_D = 0;
accum_J = 0;
accum_X = 0;
accum_T = 1;

for k = 1:(numel(path) - 1)
    u = path(k);
    v = path(k + 1);

    if ~net.node_alive(u) || ~net.node_alive(v)
        return;
    end
    if ~net.link_exist(u, v) || ~net.link_alive(u, v)
        return;
    end

    [Y_eff, D_eff, J_eff, Z_eff] = compute_effective_link_values(net, u, v);
    accum_Y = min(accum_Y, Y_eff);
    accum_D = accum_D + D_eff;
    accum_J = accum_J + J_eff;
    accum_X = accum_X + log(1 - Z_eff);
    accum_T = min(accum_T, net.trust(u, v));

    if accum_Y < params.y_min || accum_D > params.d_max || ...
       accum_J > params.j_max || accum_X < params.x_min
        return;
    end
end

risk_penalty = -log(max(accum_T, params.eps_t));
total_cost = params.w_y * (accum_Y / params.y_min) + ...
             params.w_d * (accum_D / params.d_max) + ...
             params.w_j * (accum_J / params.j_max) + ...
             params.w_x * (accum_X / params.x_min) + ...
             params.w_t * risk_penalty;

eval_res.feasible = true;
eval_res.total_cost = total_cost;
eval_res.bandwidth = accum_Y;
eval_res.delay = accum_D;
eval_res.jitter = accum_J;
eval_res.success_prob = exp(accum_X);

end

%% ========================================================================
function paths = enumerate_simple_paths(net, s, t, max_depth, max_paths)
% 枚举从 s 到 t 的简单路径（深度受限，数量受限）

if nargin < 4
    max_depth = 8;
end
if nargin < 5
    max_paths = 80;
end

paths = {};
if ~net.node_alive(s) || ~net.node_alive(t)
    return;
end

visited = false(1, net.n);
visited(s) = true;
[paths, ~] = dfs_collect_paths(net, s, t, visited, s, paths, max_depth, max_paths);

end

%% ========================================================================
function [paths, stop_flag] = dfs_collect_paths(net, u, t, visited, curr_path, paths, max_depth, max_paths)
% 深度优先收集简单路径

stop_flag = false;

if numel(paths) >= max_paths
    stop_flag = true;
    return;
end

if u == t
    paths{end + 1} = curr_path; %#ok<AGROW>
    if numel(paths) >= max_paths
        stop_flag = true;
    end
    return;
end

if numel(curr_path) >= max_depth
    return;
end

neighbors = find(net.link_exist(u, :) & net.link_alive(u, :) & net.node_alive);
for idx = 1:numel(neighbors)
    v = neighbors(idx);
    if visited(v)
        continue;
    end

    visited2 = visited;
    visited2(v) = true;
    [paths, stop_flag] = dfs_collect_paths(net, v, t, visited2, [curr_path, v], paths, max_depth, max_paths); %#ok<AGROW>
    if stop_flag
        return;
    end
end

end

%% ========================================================================
function print_single_result(result, scenario)
% 清晰打印单个场景结果

fprintf('\n');
fprintf('===============================================================\n');
fprintf('场景名称：%s\n', scenario.name);
fprintf('场景描述：%s\n', scenario.description);
if ~isempty(scenario.notes)
    fprintf('场景说明：%s\n', scenario.notes);
end
fprintf('---------------------------------------------------------------\n');

fprintf('是否找到可行路径：%s\n', bool_to_cn(result.found));
fprintf('路径序列：%s\n', result.path_str);

if result.found
    fprintf('总代价：%.6f\n', result.total_cost);
    fprintf('瓶颈带宽：%.4f\n', result.bandwidth);
    fprintf('累计时延：%.4f\n', result.delay);
    fprintf('累计抖动：%.4f\n', result.jitter);
    fprintf('路径成功率：%.6f\n', result.success_prob);
    fprintf('路径丢包率：%.6f\n', result.loss_rate);
    fprintf('路径信任度：%.4f\n', result.trust);
    fprintf('风险惩罚项：%.6f\n', result.risk_penalty);
    fprintf('T_calc（路径重计算时间）：%.6f 秒\n', result.calc_time);
else
    fprintf('总代价：Inf\n');
    fprintf('瓶颈带宽：NaN\n');
    fprintf('累计时延：NaN\n');
    fprintf('累计抖动：NaN\n');
    fprintf('路径成功率：NaN\n');
    fprintf('路径丢包率：NaN\n');
    fprintf('路径信任度：NaN\n');
    fprintf('风险惩罚项：NaN\n');
    fprintf('T_calc（路径重计算时间）：%.6f 秒\n', result.calc_time);
end

% 若不是 S0，则打印恢复指标
if isfield(result, 'T_rec') && ~isnan(result.T_rec)
    fprintf('---------------------------------------------------------------\n');
    fprintf('恢复指标：\n');
    fprintf('T_rec：%.6f 秒\n', result.T_rec);
    fprintf('Q_ret：%.4f\n', result.Q_ret);
    fprintf('S_cont：%.0f\n', result.S_cont);
end

fprintf('===============================================================\n');

end

%% ========================================================================
function summary_tbl = summarize_results(results)
% 汇总所有场景结果，并尽可能转为 MATLAB table

fprintf('\n');
fprintf('###############################################################\n');
fprintf(' 所有场景结果汇总表\n');
fprintf('###############################################################\n');

num_results = numel(results);

scenario_name = cell(num_results, 1);
found_col     = false(num_results, 1);
path_col      = cell(num_results, 1);
cost_col      = nan(num_results, 1);
bw_col        = nan(num_results, 1);
delay_col     = nan(num_results, 1);
jitter_col    = nan(num_results, 1);
succ_col      = nan(num_results, 1);
loss_col      = nan(num_results, 1);
trust_col     = nan(num_results, 1);
calc_col      = nan(num_results, 1);
trec_col      = nan(num_results, 1);
qret_col      = nan(num_results, 1);
scont_col     = nan(num_results, 1);

for k = 1:num_results
    scenario_name{k} = results(k).scenario_name;
    found_col(k)     = results(k).found;
    path_col{k}      = results(k).path_str;
    cost_col(k)      = results(k).total_cost;
    bw_col(k)        = results(k).bandwidth;
    delay_col(k)     = results(k).delay;
    jitter_col(k)    = results(k).jitter;
    succ_col(k)      = results(k).success_prob;
    loss_col(k)      = results(k).loss_rate;
    trust_col(k)     = results(k).trust;
    calc_col(k)      = results(k).calc_time;

    if isfield(results(k), 'T_rec')
        trec_col(k) = results(k).T_rec;
    end
    if isfield(results(k), 'Q_ret')
        qret_col(k) = results(k).Q_ret;
    end
    if isfield(results(k), 'S_cont')
        scont_col(k) = results(k).S_cont;
    end
end

% 先以格式化文本方式输出，保证各版本 MATLAB 均可见
fprintf('%-6s %-6s %-20s %-12s %-10s %-10s %-10s %-12s %-12s %-10s %-12s %-10s %-10s %-10s\n', ...
    '场景', '可行', '路径', '总代价', '带宽', '时延', '抖动', '成功率', '丢包率', '信任度', 'T_calc(s)', 'T_rec(s)', 'Q_ret', 'S_cont');

for k = 1:num_results
    fprintf('%-6s %-6s %-20s %-12s %-10s %-10s %-10s %-12s %-12s %-10s %-12s %-10s %-10s %-10s\n', ...
        scenario_name{k}, ...
        bool_to_cn(found_col(k)), ...
        path_col{k}, ...
        num2str_or_inf(cost_col(k), '%.6f'), ...
        num2str_or_nan(bw_col(k), '%.4f'), ...
        num2str_or_nan(delay_col(k), '%.4f'), ...
        num2str_or_nan(jitter_col(k), '%.4f'), ...
        num2str_or_nan(succ_col(k), '%.6f'), ...
        num2str_or_nan(loss_col(k), '%.6f'), ...
        num2str_or_nan(trust_col(k), '%.4f'), ...
        num2str_or_nan(calc_col(k), '%.6f'), ...
        num2str_or_nan(trec_col(k), '%.6f'), ...
        num2str_or_nan(qret_col(k), '%.4f'), ...
        num2str_or_nan(scont_col(k), '%.0f'));
end

% 再尽可能转为 table
summary_tbl = [];

try
    summary_tbl = table( ...
        scenario_name, ...
        found_col, ...
        path_col, ...
        cost_col, ...
        bw_col, ...
        delay_col, ...
        jitter_col, ...
        succ_col, ...
        loss_col, ...
        trust_col, ...
        calc_col, ...
        trec_col, ...
        qret_col, ...
        scont_col, ...
        'VariableNames', { ...
        'Scenario', ...
        'Found', ...
        'Path', ...
        'TotalCost', ...
        'Bandwidth', ...
        'Delay', ...
        'Jitter', ...
        'SuccessProb', ...
        'LossRate', ...
        'Trust', ...
        'T_calc', ...
        'T_rec', ...
        'Q_ret', ...
        'S_cont'});

    fprintf('\n');
    fprintf('MATLAB table 显示如下：\n');
    disp(summary_tbl);

catch
    fprintf('\n当前 MATLAB 版本未成功生成 table，已完成文本汇总输出。\n');
end

end

%% ========================================================================
function pos = build_node_positions()
% 手工固定节点坐标
% 采用论文拓扑示意图风格，避免自动布局导致的随机性和重复运行差异。

pos = zeros(14, 2);

% 按用户示意图重排节点位置
pos(1, :)  = [0.0, 4.0];
pos(2, :)  = [2.0, 5.5];
pos(3, :)  = [2.0, 2.5];
pos(4, :)  = [4.2, 4.8];
pos(5, :)  = [4.2, 3.0];
pos(6, :)  = [4.2, 1.2];
pos(7, :)  = [4.2, 7.0];
pos(8, :)  = [6.8, 3.0];
pos(9, :)  = [6.8, 4.8];
pos(10, :) = [6.8, 1.2];
pos(11, :) = [6.8, 7.0];
pos(12, :) = [8.8, 2.5];
pos(13, :) = [8.8, 5.5];
pos(14, :) = [10.8, 4.0];

end

%% ========================================================================
function viz = build_visual_config()
% 构建绘图配置
% 所有绘图风格集中在本函数中设置，便于后续论文配图统一调整。

viz = struct();

% ------------------------- 图窗布局 -------------------------
viz.figure_position      = [60, 60, 1450, 800];
viz.figure_dx            = 35;
viz.figure_dy            = 20;
viz.left_axes_position   = [0.04, 0.08, 0.60, 0.84];
viz.right_axes_position  = [0.68, 0.08, 0.29, 0.84];

% ------------------------- 拓扑绘图风格 -------------------------
viz.node_size            = 1300;
viz.node_radius          = 0.30;
viz.node_font_size       = 11;
viz.edge_font_size       = 6.8;
viz.panel_title_size     = 12;
viz.panel_font_size      = 10;

viz.base_line_width      = 1.2;
viz.path_line_width      = 2.8;
viz.failed_line_width    = 1.0;

viz.edge_color           = [0.00, 0.00, 0.00];
viz.path_edge_color      = [0.85, 0.00, 0.00];
viz.failed_edge_color    = [0.72, 0.72, 0.72];

viz.node_alive_color     = [1.00, 1.00, 1.00];
viz.node_dead_color      = [0.72, 0.72, 0.72];
viz.node_edge_color      = [0.00, 0.00, 0.00];

viz.label_box_color      = [1.00, 1.00, 1.00];
viz.edge_label_offset    = 0.30;

% ------------------------- 右侧面板风格 -------------------------
viz.panel_line_step      = 0.058;
viz.panel_left_x         = 0.02;
viz.panel_top_y          = 0.98;

end

%% ========================================================================
function render_all_figures(base_net, scenarios, results, params)
% 统一渲染所有场景图窗
% 每个图窗分为左右两部分：
% - 左侧：网络拓扑图
% - 右侧：结果参数面板

pos = build_node_positions();
viz = build_visual_config();

for k = 1:numel(scenarios)
    render_scenario_figure(base_net, scenarios(k), results(k), params, pos, viz, k);
end

end

%% ========================================================================
function render_scenario_figure(base_net, scenario, result, params, pos, viz, fig_idx)
% 渲染单个场景图窗
% 说明：
% 1) S0 显示原始参数 Y, D, J, Z；
% 2) S1~S4 显示有效参数 Yeff, Deff, Jeff, Zeff 和 trust；
% 3) 所有图窗均保留固定拓扑布局和一致的配色逻辑。

if strcmp(scenario.name, 'S0')
    net_to_draw    = base_net;
    show_effective = false;
else
    net_to_draw    = scenario.net;
    show_effective = true;
end

fig_title = get_scenario_title(scenario.name);

fig_position = viz.figure_position + [ ...
    (fig_idx - 1) * viz.figure_dx, ...
    -(fig_idx - 1) * viz.figure_dy, ...
    0, ...
    0];

fig = figure( ...
    'Color', 'w', ...
    'Name', fig_title, ...
    'NumberTitle', 'off', ...
    'Position', fig_position);

% ------------------------- 左侧网络拓扑 -------------------------
ax_left = axes('Parent', fig, 'Position', viz.left_axes_position);
hold(ax_left, 'on');
axis(ax_left, 'equal');
axis(ax_left, 'off');

xlim(ax_left, [min(pos(:, 1)) - 1.5, max(pos(:, 1)) + 1.5]);
ylim(ax_left, [min(pos(:, 2)) - 1.5, max(pos(:, 2)) + 1.5]);

draw_network_edges(ax_left, net_to_draw, result, pos, viz, show_effective);
draw_network_nodes(ax_left, net_to_draw, pos, viz);

title(ax_left, '网络拓扑图', 'FontWeight', 'bold');

if show_effective
    note_text = '边标签：当前场景有效参数 Yeff, Deff, Jeff, Zeff, trust';
else
    note_text = '边标签：原始参数 Y, D, J, Z';
end

text(ax_left, ...
    min(pos(:, 1)) - 1.2, ...
    min(pos(:, 2)) - 1.15, ...
    note_text, ...
    'FontSize', 9, ...
    'Interpreter', 'none');

% ------------------------- 右侧结果面板 -------------------------
ax_right = axes('Parent', fig, 'Position', viz.right_axes_position);
draw_result_panel(ax_right, scenario, result);

% ------------------------- 图窗总标题 -------------------------
annotation(fig, 'textbox', [0.02, 0.94, 0.96, 0.05], ...
    'String', fig_title, ...
    'LineStyle', 'none', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', ...
    'FontSize', 14, ...
    'Interpreter', 'none');

% 参数 params 保留接口，便于后续论文版本扩展可视化内容
if nargin < 4 %#ok<*INUSD>
end

end

%% ========================================================================
function draw_network_edges(ax, net, result, pos, viz, show_effective)
% 绘制网络链路
% 绘制规则：
% - 默认可用链路：黑色实线
% - 最终可行路径：红色加粗
% - 失效链路：浅灰色虚线
% - 边标签：白底文字框，S0 显示原始参数，S1~S4 显示有效参数与 trust
% - 双向边标签采用法向偏移，避免 i->j 与 j->i 重叠

n = net.n;
path_mask = get_path_edge_mask(result.path, n);

for i = 1:n
    for j = 1:n
        if i == j
            continue;
        end

        if ~net.link_exist(i, j)
            continue;
        end

        p1 = pos(i, :);
        p2 = pos(j, :);
        vec = p2 - p1;
        len = norm(vec);

        if len <= eps
            continue;
        end

        dir_vec = vec / len;
        perp_vec = [-dir_vec(2), dir_vec(1)];

        % 为避免线段穿过节点圆形区域，对线段两端做适当收缩
        p1s = p1 + dir_vec * viz.node_radius;
        p2s = p2 - dir_vec * viz.node_radius;

        % 当前边是否有效
        edge_alive = net.link_alive(i, j) && net.node_alive(i) && net.node_alive(j);

        % 当前边是否属于最终路径
        on_path = false;
        if ~isempty(path_mask)
            on_path = path_mask(i, j);
        end

        % ------------------------- 绘制链路本体 -------------------------
        if ~edge_alive
            line(ax, [p1s(1), p2s(1)], [p1s(2), p2s(2)], ...
                'LineStyle', '--', ...
                'LineWidth', viz.failed_line_width, ...
                'Color', viz.failed_edge_color);
            label_color = [0.45, 0.45, 0.45];

        elseif on_path
            line(ax, [p1s(1), p2s(1)], [p1s(2), p2s(2)], ...
                'LineStyle', '-', ...
                'LineWidth', viz.path_line_width, ...
                'Color', viz.path_edge_color);
            label_color = [0.60, 0.00, 0.00];

        else
            line(ax, [p1s(1), p2s(1)], [p1s(2), p2s(2)], ...
                'LineStyle', '-', ...
                'LineWidth', viz.base_line_width, ...
                'Color', viz.edge_color);
            label_color = [0.00, 0.00, 0.00];
        end

        % ------------------------- 绘制边标签 -------------------------
        mid = (p1 + p2) / 2;

        % 双向边采用相反法向偏移，减少标签重叠
        if i < j
            offset_sign = 1;
        else
            offset_sign = -1;
        end

        % 使用轻微的确定性偏移扰动，降低局部密集区域标签堆叠
        offset_mag = viz.edge_label_offset * (1 + 0.18 * mod(i + j, 3));
        x_label = mid(1) + offset_sign * offset_mag * perp_vec(1);
        y_label = mid(2) + offset_sign * offset_mag * perp_vec(2);

        label_str = format_edge_label(net, i, j, show_effective);

        text(ax, x_label, y_label, label_str, ...
            'FontSize', viz.edge_font_size, ...
            'Color', label_color, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'BackgroundColor', viz.label_box_color, ...
            'Interpreter', 'none');
    end
end

end

%% ========================================================================
function draw_network_nodes(ax, net, pos, viz)
% 绘制节点
% 绘制规则：
% - 正常节点：白色圆形
% - 失效节点：灰色圆形
% - 节点编号显示于圆内

n = net.n;

for i = 1:n
    if net.node_alive(i)
        face_color = viz.node_alive_color;
    else
        face_color = viz.node_dead_color;
    end

    scatter(ax, pos(i, 1), pos(i, 2), viz.node_size, ...
        'o', ...
        'MarkerFaceColor', face_color, ...
        'MarkerEdgeColor', viz.node_edge_color, ...
        'LineWidth', 1.5);

    text(ax, pos(i, 1), pos(i, 2), num2str(i), ...
        'FontSize', viz.node_font_size, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'Interpreter', 'none');
end

end

%% ========================================================================
function draw_result_panel(ax, scenario, result)
% 绘制右侧结果参数面板（自适应行距版本）
% 解决问题：
% - 某些场景描述较长时，固定 dy 会把最后的 S_cont 挤出面板可视区域
% - 现在根据总行数自动压缩行距，保证三个恢复指标都能显示出来

axis(ax, [0, 1, 0, 1]);
axis(ax, 'off');
hold(ax, 'on');

viz = build_visual_config();

x        = viz.panel_left_x;
top_y    = viz.panel_top_y;
bottom_y = 0.04;   % 给底部留一点边距

title_str  = get_scenario_title(scenario.name);
desc_lines = wrap_text_by_length(['场景描述：', scenario.description], 23);

if result.found
    path_show = result.path_str;
else
    path_show = '未找到可行路径';
end

info_lines = {
    ['是否找到可行路径：', bool_to_cn(result.found)]
    ['路径：', path_show]
    ['总代价：', num2str_or_inf(result.total_cost, '%.6f')]
    ['瓶颈带宽：', num2str_or_nan(result.bandwidth, '%.4f')]
    ['累计时延：', num2str_or_nan(result.delay, '%.4f')]
    ['累计抖动：', num2str_or_nan(result.jitter, '%.4f')]
    ['路径成功率：', num2str_or_nan(result.success_prob, '%.6f')]
    ['路径丢包率：', num2str_or_nan(result.loss_rate, '%.6f')]
    ['路径信任度：', num2str_or_nan(result.trust, '%.4f')]
    ['风险惩罚项：', num2str_or_nan(result.risk_penalty, '%.6f')]
    ['T_calc：', num2str_or_nan(result.calc_time, '%.6f'), ' s']
};

if strcmp(scenario.name, 'S0')
    trec_str  = '--';
    qret_str  = '--';
    scont_str = '--';
else
    trec_str  = panel_value_or_dash(result.T_rec,  '%.6f');
    qret_str  = panel_value_or_dash(result.Q_ret,  '%.4f');
    scont_str = panel_value_or_dash(result.S_cont, '%.0f');
end

recovery_lines = {
    ['T_rec：', trec_str]
    ['Q_ret：', qret_str]
    ['S_cont：', scont_str]
};

% ------------------------- 自适应行距 -------------------------
% 需要显示的总行数：
% 1 行“结果参数面板”
% 1 行“场景名称”
% N 行描述
% 1 个小间隔
% info_lines 各行
% 1 行“恢复指标”
% recovery_lines 各行
n_total_lines = 1 + 1 + numel(desc_lines) + 1 + numel(info_lines) + 1 + numel(recovery_lines);

% 使用更稳妥的动态行距，避免末尾文字被裁掉
dy_auto = (top_y - bottom_y) / max(n_total_lines, 1);

% 不超过原始设定，也不要太小
dy = min(viz.panel_line_step, dy_auto);
dy = max(dy, 0.038);

% ------------------------- 开始绘制 -------------------------
y = top_y;

text(ax, x, y, '结果参数面板', ...
    'Units', 'normalized', ...
    'FontWeight', 'bold', ...
    'FontSize', viz.panel_title_size, ...
    'Interpreter', 'none');
y = y - dy;

text(ax, x, y, ['场景名称：', title_str], ...
    'Units', 'normalized', ...
    'FontSize', viz.panel_font_size, ...
    'FontWeight', 'bold', ...
    'Interpreter', 'none');
y = y - dy;

for k = 1:numel(desc_lines)
    text(ax, x, y, desc_lines{k}, ...
        'Units', 'normalized', ...
        'FontSize', viz.panel_font_size, ...
        'Interpreter', 'none');
    y = y - dy;
end

% 小间隔
y = y - dy * 0.25;

for k = 1:numel(info_lines)
    text(ax, x, y, info_lines{k}, ...
        'Units', 'normalized', ...
        'FontSize', viz.panel_font_size, ...
        'Interpreter', 'none');
    y = y - dy;
end

text(ax, x, y, '恢复指标', ...
    'Units', 'normalized', ...
    'FontWeight', 'bold', ...
    'FontSize', viz.panel_font_size, ...
    'Interpreter', 'none');
y = y - dy;

for k = 1:numel(recovery_lines)
    text(ax, x, y, recovery_lines{k}, ...
        'Units', 'normalized', ...
        'FontSize', viz.panel_font_size, ...
        'Interpreter', 'none');
    y = y - dy;
end

end

%% ========================================================================
function link_info = get_effective_link_params(net, i, j)
% 获取指定链路的当前有效参数
% 输出字段包括：
% - is_exist：链路是否存在
% - is_alive：链路在当前场景是否可用
% - Yeff, Deff, Jeff, Zeff：有效参数
% - trust：当前链路信任度

link_info = struct();
link_info.is_exist = false;
link_info.is_alive = false;
link_info.Yeff     = NaN;
link_info.Deff     = NaN;
link_info.Jeff     = NaN;
link_info.Zeff     = NaN;
link_info.trust    = NaN;

if i == j
    return;
end

if ~net.link_exist(i, j)
    return;
end

link_info.is_exist = true;
link_info.trust    = net.trust(i, j);
link_info.is_alive = net.link_alive(i, j) && net.node_alive(i) && net.node_alive(j);

if ~link_info.is_alive
    return;
end

[link_info.Yeff, link_info.Deff, link_info.Jeff, link_info.Zeff] = ...
    compute_effective_link_values(net, i, j);

end

%% ========================================================================
function edge_mask = get_path_edge_mask(path, n)
% 根据路径序列生成路径边掩码矩阵
% edge_mask(i,j)=true 表示边 i->j 属于最终路径

edge_mask = false(n);

if isempty(path) || numel(path) < 2
    return;
end

for k = 1:(numel(path) - 1)
    i = path(k);
    j = path(k + 1);
    edge_mask(i, j) = true;
end

end

%% ========================================================================
function label_str = format_edge_label(net, i, j, show_effective)
% 生成边标签字符串
% - S0：显示原始参数 Y, D, J, Z
% - S1~S4：显示有效参数 Yeff, Deff, Jeff, Zeff 与 trust
% - 若链路失效，则直接标记“失效”

if ~net.link_exist(i, j)
    label_str = '';
    return;
end

if show_effective
    link_info = get_effective_link_params(net, i, j);

    if ~link_info.is_alive
        label_str = sprintf('%d->%d\n失效', i, j);
    else
        label_str = sprintf(['%d->%d\n', ...
                             'Yeff=%.1f\n', ...
                             'Deff=%.1f\n', ...
                             'Jeff=%.1f\n', ...
                             'Zeff=%.3f\n', ...
                             'trust=%.2f'], ...
                             i, j, ...
                             link_info.Yeff, ...
                             link_info.Deff, ...
                             link_info.Jeff, ...
                             link_info.Zeff, ...
                             link_info.trust);
    end

else
    label_str = sprintf(['%d->%d\n', ...
                         'Y=%.1f\n', ...
                         'D=%.1f\n', ...
                         'J=%.1f\n', ...
                         'Z=%.3f'], ...
                         i, j, ...
                         net.Y(i, j), ...
                         net.D(i, j), ...
                         net.J(i, j), ...
                         net.Z(i, j));
end

end

%% ========================================================================
function s = path_to_str(path)
% 将路径数组转为字符串，便于命令行打印与汇总表显示

if isempty(path)
    s = '[]';
    return;
end

parts = arrayfun(@num2str, path, 'UniformOutput', false);
s = strjoin(parts, ' -> ');

end

%% ========================================================================
function [Y_eff, D_eff, J_eff, Z_eff] = compute_effective_link_values(net, i, j)
% 统一计算链路有效参数，并进行数值安全裁剪

Y_eff = max(0, net.Y(i, j) * (1 - net.degY(i, j)));
D_eff = max(0, net.D(i, j) * (1 + net.degD(i, j)));
J_eff = max(0, net.J(i, j) * (1 + net.degJ(i, j)));

Z_raw = net.Z(i, j) + net.degZ(i, j);
Z_eff = max(0, min(1 - net.eps_z, Z_raw));

end

%% ========================================================================
function out = safe_ratio(a, b, mode)
% 安全比值计算
% mode 仅保留接口一致性，这里统一做 a / b

if nargin < 3
    mode = 'direct'; %#ok<NASGU>
end

if isnan(a) || isnan(b) || isinf(a) || isinf(b) || b == 0
    out = 0;
else
    out = a / b;
end

end

%% ========================================================================
function y = clip01(x)
% 截断到 [0,1]

if isnan(x)
    y = 0;
else
    y = min(1, max(0, x));
end

end

%% ========================================================================
function txt = bool_to_cn(flag)
% 布尔值转中文显示

if flag
    txt = '是';
else
    txt = '否';
end

end

%% ========================================================================
function txt = num2str_or_nan(x, fmt)
% 对 NaN / Inf 做安全字符串转换

if nargin < 2
    fmt = '%.6f';
end

if isnan(x)
    txt = 'NaN';
elseif isinf(x)
    if x > 0
        txt = 'Inf';
    else
        txt = '-Inf';
    end
else
    txt = sprintf(fmt, x);
end

end

%% ========================================================================
function txt = num2str_or_inf(x, fmt)
% 对 Inf / NaN 做安全字符串转换

if nargin < 2
    fmt = '%.6f';
end

if isnan(x)
    txt = 'NaN';
elseif isinf(x)
    txt = 'Inf';
else
    txt = sprintf(fmt, x);
end

end

%% ========================================================================
function result = init_empty_result_struct()
% 初始化结果结构体，保证各场景结构一致

result = struct();

result.scenario_name  = '';
result.scenario_desc  = '';
result.scenario_notes = '';

result.found    = false;
result.path     = [];
result.path_str = '[]';

result.prev = [];
result.dist = [];

result.total_cost   = inf;
result.bandwidth    = NaN;
result.delay        = NaN;
result.jitter       = NaN;
result.success_prob = NaN;
result.loss_rate    = NaN;
result.trust        = NaN;
result.risk_penalty = NaN;

result.calc_time = NaN;

result.T_rec  = NaN;
result.Q_ret  = NaN;
result.S_cont = NaN;

end

%% ========================================================================
function title_str = get_scenario_title(scenario_name)
% 将场景编号映射为图窗标题

switch upper(scenario_name)
    case 'S0'
        title_str = 'S0 正常场景';
    case 'S1'
        title_str = 'S1 节点失效场景';
    case 'S2'
        title_str = 'S2 链路退化场景';
    case 'S3'
        title_str = 'S3 风险链路场景';
    case 'S4'
        title_str = 'S4 组合异常场景';
    otherwise
        title_str = scenario_name;
end

end

%% ========================================================================
function txt = panel_value_or_dash(x, fmt)
% 面板数值转字符串
% 对 S0 的恢复指标或缺失值，统一显示为 --

if nargin < 2
    fmt = '%.6f';
end

if isnan(x)
    txt = '--';
elseif isinf(x)
    if x > 0
        txt = 'Inf';
    else
        txt = '-Inf';
    end
else
    txt = sprintf(fmt, x);
end

end

%% ========================================================================
function lines = wrap_text_by_length(txt, max_len)
% 简单文本分行函数
% 作用：将较长的中文描述按固定长度分成多行，以适配右侧参数面板

if nargin < 2
    max_len = 24;
end

if isempty(txt)
    lines = {''};
    return;
end

n = length(txt);
idx = 1;
lines = {};

while idx <= n
    idx2 = min(n, idx + max_len - 1);
    lines{end + 1} = txt(idx:idx2); %#ok<AGROW>
    idx = idx2 + 1;
end

end
