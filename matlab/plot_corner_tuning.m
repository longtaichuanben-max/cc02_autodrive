function plot_corner_tuning(wp_csv, varargin)
%PLOT_CORNER_TUNING コーナー減速チューニング比較グラフ
%
%   plot_corner_tuning(wp_csv, log_csv1, label1, log_csv2, label2, ...)
%
%   第1引数: ウェイポイントCSV（WP番号マッピング用）
%   以降:    (pure_pursuit_log CSVパス, 凡例ラベル) のペアを繰り返す
%
%   例:
%     plot_corner_tuning('wp_position_advance.csv', ...
%       'pure_pursuit_log_advance_base1p8.csv', 'base\_dist=1.8m', ...
%       'pure_pursuit_log_advance_base3p0.csv', 'base\_dist=3.0m')
%
%   グラフ1 - 速度プロファイル（コーナーWPごとにサブプロット）
%     X軸: WPまでの距離 [m]（右端=WP通過点, 左=接近中）
%     Y軸: 速度 [m/s]
%     実線: 実速度 / 破線: 指令速度
%     ▼マーカー: approaching_corner=1 になった地点（減速開始）
%     ●マーカー: WP通過時の実速度
%     縦点線: 減速開始距離（各パラメータ値に対応）
%
%   グラフ2 - コーナー通過速度まとめ（棒グラフ）
%     横軸: コーナーWP番号（写真のWP番号と一致）
%     縦軸: WP通過時の実速度
%     赤破線: 目標コーナー速度 (speed_max × corner_slowdown_ratio)

if mod(nargin - 1, 2) ~= 0 || nargin < 3
    error('引数は wp_csv の後に (logパス, ラベル) のペアで指定してください');
end

n_runs = (nargin - 1) / 2;
COL    = lines(n_runs);

PRE_S = 15.0;   % WP通過の何秒前まで表示するか

SPEED_MAX             = 3.0;
CORNER_SLOWDOWN_RATIO = 0.35;
TARGET_CORNER_SPEED   = SPEED_MAX * CORNER_SLOWDOWN_RATIO;

% ---- WP番号マッピング（配列インデックス → 写真のWP番号） ----
wp_table = readtable(wp_csv, 'VariableNamingRule', 'preserve');
wp_nums  = wp_table.WP;
% 最終行がループクローズ（最初と同じ座標）なら除く
if wp_nums(end) == wp_nums(1)
    wp_nums(end) = [];
end
% log の waypoint_index（0始まり配列インデックス）→ WP番号
idx_to_wp = @(idx) wp_nums(idx + 1);

% ---- データ収集 ----------------------------------------
all_corners = {};

for r = 1:n_runs
    csv_file = varargin{2*r - 1};
    fprintf('読み込み中: %s\n', csv_file);
    T = readtable(csv_file, 'VariableNamingRule', 'preserve');

    t      = T.time;
    wi     = T.waypoint_index;
    ac     = T.approaching_corner;
    dist   = T.dist_to_target;
    actual = T.actual_speed;
    cmd    = T.cmd_speed;
    N      = height(T);

    starts = find(diff([0; double(ac)]) > 0);

    for si = 1:length(starts)
        s      = starts(si);
        wi_log = wi(s);   % ログ上の配列インデックス

        % 有効範囲チェック
        if wi_log < 0 || wi_log >= length(wp_nums)
            continue;
        end
        wp_n = idx_to_wp(wi_log);   % 写真のWP番号に変換

        % WP通過行を探す
        e = s + 1;
        while e <= N && wi(e) == wi_log
            e = e + 1;
        end
        if e > N, continue; end

        t_pass = t(e);
        mask   = t >= (t_pass - PRE_S) & t < t_pass;
        if sum(mask) < 5, continue; end

        corner.wp          = wp_n;
        corner.run         = r;
        corner.dist        = dist(mask);
        corner.actual      = actual(mask);
        corner.cmd         = cmd(mask);
        corner.ac          = double(ac(mask));
        corner.pass_actual = actual(e - 1);
        corner.decel_dist  = dist(s);

        all_corners{end+1} = corner; %#ok<AGROW>
    end
end

if isempty(all_corners)
    warning('approaching_corner=1 のデータが見つかりませんでした。');
    return;
end

% ---- コーナーWP一覧（写真のWP番号） ----------------------------------------
all_wps = unique(cellfun(@(c) c.wp, all_corners));
n_wps   = length(all_wps);

% ---- 図1: 速度プロファイル ----------------------------------------
n_cols = min(n_wps, 3);
n_rows = ceil(n_wps / n_cols);
fig_w  = max(500, 420 * n_cols);
fig_h  = 360 * n_rows;

fig1 = figure('Name', 'コーナー減速プロファイル', 'NumberTitle', 'off', ...
    'Position', [50, 50, fig_w, fig_h]);

for wi_i = 1:n_wps
    wp_n = all_wps(wi_i);
    ax   = subplot(n_rows, n_cols, wi_i, 'Parent', fig1);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    x_max = 0;

    for r = 1:n_runs
        label = varargin{2*r};
        col   = COL(r, :);
        dark  = col * 0.55 + 0.15;

        matches = cellfun(@(c) c.wp == wp_n && c.run == r, all_corners);
        if ~any(matches), continue; end
        c = all_corners{find(matches, 1)};

        plot(ax, c.dist, c.cmd, '--', 'Color', [dark 0.7], 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
        plot(ax, c.dist, c.actual, '-', 'Color', col, 'LineWidth', 2.2, ...
            'DisplayName', label);

        xline(ax, c.decel_dist, ':', 'Color', col, 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');

        first_ac = find(c.ac > 0, 1);
        if ~isempty(first_ac)
            scatter(ax, c.dist(first_ac), c.actual(first_ac), 80, col, 'v', ...
                'filled', 'HandleVisibility', 'off');
        end

        scatter(ax, 0, c.pass_actual, 80, col, 'o', 'filled', 'HandleVisibility', 'off');
        text(ax, 0.4, c.pass_actual + 0.09, sprintf('%.2f m/s', c.pass_actual), ...
            'Color', col, 'FontSize', 8, 'FontWeight', 'bold', 'Interpreter', 'none');

        x_max = max(x_max, max(c.dist));
    end

    xline(ax, 0, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(ax, TARGET_CORNER_SPEED, '--', 'Color', [0.8 0 0], 'LineWidth', 1.2, ...
        'Label', sprintf('目標 %.2f m/s', TARGET_CORNER_SPEED), ...
        'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'left', ...
        'HandleVisibility', 'off');

    set(ax, 'XDir', 'reverse');
    xlim(ax, [-1.5, x_max + 2]);
    ylim(ax, [0, SPEED_MAX + 0.6]);
    xlabel(ax, 'WPまでの距離 [m]', 'FontSize', 9);
    ylabel(ax, '速度 [m/s]', 'FontSize', 9);
    title(ax, sprintf('コーナーWP%d', wp_n), 'FontSize', 10, 'Interpreter', 'none');
    legend(ax, 'Location', 'northwest', 'FontSize', 8);
end

sgtitle(fig1, ...
    '速度プロファイル比較 — 実線=実速度, 破線=指令速度, ▼=減速開始, ●=WP通過', ...
    'FontSize', 10);

% ---- 図2: コーナー通過速度まとめ ----------------------------------------
fig2 = figure('Name', 'コーナー通過速度まとめ', 'NumberTitle', 'off', ...
    'Position', [70 + fig_w, 50, 520, 400]);
ax2 = axes('Parent', fig2);
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');

bw = 0.72 / n_runs;

for r = 1:n_runs
    label = varargin{2*r};
    col   = COL(r, :);
    v_arr = nan(1, n_wps);

    for wi_i = 1:n_wps
        wp_n = all_wps(wi_i);
        matches = cellfun(@(c) c.wp == wp_n && c.run == r, all_corners);
        if any(matches)
            c = all_corners{find(matches, 1)};
            v_arr(wi_i) = c.pass_actual;
        end
    end

    offset = (r - (n_runs + 1) / 2) * bw;
    bar(ax2, (1:n_wps) + offset, v_arr, bw, 'FaceColor', col, 'DisplayName', label);
end

yline(ax2, TARGET_CORNER_SPEED, '--', 'Color', [0.8 0 0], 'LineWidth', 1.8, ...
    'Label', sprintf('目標 %.2f m/s', TARGET_CORNER_SPEED), ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');

xticks(ax2, 1:n_wps);
xticklabels(ax2, arrayfun(@(w) sprintf('WP%d', w), all_wps, 'UniformOutput', false));
xlabel(ax2, 'コーナーWP', 'FontSize', 10);
ylabel(ax2, 'WP通過時の実速度 [m/s]', 'FontSize', 10);
title(ax2, 'コーナー通過速度比較', 'FontSize', 11);
legend(ax2, 'Location', 'best', 'FontSize', 9);
ylim(ax2, [0, SPEED_MAX + 0.6]);

end
