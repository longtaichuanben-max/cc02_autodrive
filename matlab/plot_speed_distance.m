function plot_speed_distance(log_csv, wp_file)
%PLOT_SPEED_DISTANCE 走行距離を横軸に、速度を縦軸にプロットする。
%   実測速度・計算速度・理論速度（WP幾何配置ベースのランプ込み）を重ねて表示。
%
%   使用例:
%       plot_speed_distance('gnss_log_basic_latest.csv', 'wp_position_basic.csv')

%% ---- コントローラパラメータ（pure_pursuit_controller.py のデフォルト値）----------
SPEED_MIN              = 1.0;
SPEED_MAX              = 3.0;
SPEED_DIST_SHORT       = 5.0;
SPEED_DIST_LONG        = 10.0;
CORNER_ANGLE_THRESH    = 36.0;
CORNER_SLOWDOWN_RATIO  = 0.35;
CORNER_BASE_DIST       = 1.8;    % corner_slowdown_base_dist [m]
CORNER_ACCEL_RATIO     = 1.0;    % corner_accel_ratio：加速ランプ距離 = 減速ランプ距離 × この値
WP_RADIUS_MAX          = 2.0;    % wp_radius_max [m]
WP_RADIUS_MIN          = 0.5;    % wp_radius_min [m]
WP_RADIUS_SEG_RATIO    = 0.3;    % wp_radius_seg_ratio

if nargin < 2, wp_file = ''; end

%% ---- GNSSログ読み込み ----------------------------------------
T = readtable(log_csv, 'TextType', 'string');
valid = ~(T.latitude == 0 & T.longitude == 0);
T = T(valid, :);
if isempty(T)
    error('有効な緯度経度を持つ行が見つかりませんでした: %s', log_csv);
end

%% ---- 累積距離・計算速度 ----------------------------------------
origin_lat = T.latitude(1);
origin_lon = T.longitude(1);
[track_e, track_n] = latlon2enu(T.latitude, T.longitude, origin_lat, origin_lon);
de = diff(track_e);
dn = diff(track_n);
ds = sqrt(de.^2 + dn.^2);
dist_m = [0; cumsum(ds)];


isFix   = T.status_str == "FIX";
isFloat = T.status_str == "FLOAT";

%% ---- WP読み込み・幾何解析 ----------------------------------------
has_wp = ~isempty(wp_file);
seg_speed  = [];
is_corner  = [];
wp_e = []; wp_n = [];
n_wp = 0;

if has_wp
    WP     = readtable(wp_file, 'VariableNamingRule', 'preserve');
    wp_lat = WP.("Latitude(deg)");
    wp_lon = WP.("Longitude(deg)");
    n_wp   = numel(wp_lat);
    [wp_e, wp_n] = latlon2enu(wp_lat, wp_lon, origin_lat, origin_lon);

    % セグメント長 & 速度（コントローラと同じ計算）
    seg_len   = zeros(n_wp, 1);
    seg_speed = zeros(n_wp, 1);
    d_range   = SPEED_DIST_LONG - SPEED_DIST_SHORT;
    for i = 1:n_wp
        ni = mod(i, n_wp) + 1;
        seg_len(i) = hypot(wp_e(ni)-wp_e(i), wp_n(ni)-wp_n(i));
        t = max(0, min(1, (seg_len(i) - SPEED_DIST_SHORT) / max(d_range, 1e-9)));
        seg_speed(i) = SPEED_MIN + t * (SPEED_MAX - SPEED_MIN);
    end

    % コーナー判定（偏向角）+ WP到達半径（コントローラと同じ計算式）
    is_corner = false(n_wp, 1);
    wp_radius = WP_RADIUS_MAX * ones(n_wp, 1);
    for i = 1:n_wp
        pi_ = mod(i-2, n_wp) + 1;
        ni  = mod(i,   n_wp) + 1;
        v_in  = [wp_e(i)-wp_e(pi_), wp_n(i)-wp_n(pi_)];
        v_out = [wp_e(ni)-wp_e(i),  wp_n(ni)-wp_n(i)];
        len_in  = norm(v_in);
        len_out = norm(v_out);
        if len_in > 0.1 && len_out > 0.1
            ca  = max(-1, min(1, dot(v_in,v_out)/(len_in*len_out)));
            deg = rad2deg(acos(ca));
            if deg >= CORNER_ANGLE_THRESH
                is_corner(i) = true;
            end
            r_angle      = WP_RADIUS_MAX * (1 - deg/180);
            r_seg        = min(len_in, len_out) * WP_RADIUS_SEG_RATIO;
            wp_radius(i) = max(WP_RADIUS_MIN, min(r_angle, r_seg));
        end
    end
end

%% ---- WP通過を全ログで順番に検出 ----------------------------------------
pass_dist   = [];   % 通過時の累積距離
pass_wp_num = [];   % WP番号（0-indexed）
pass_is_lap = [];   % 周回境界かどうか

if has_wp
    next_wp  = 0;
    lap      = 1;
    cooldown = 0;
    for k = 1:length(dist_m)
        if cooldown > 0; cooldown = cooldown-1; continue; end
        wi1 = next_wp + 1;
        d = hypot(track_e(k)-wp_e(wi1), track_n(k)-wp_n(wi1));
        if d < wp_radius(wi1)
            pass_dist(end+1)   = dist_m(k);
            pass_wp_num(end+1) = next_wp;
            is_lap = (next_wp == 0) && numel(pass_dist) > 1;
            pass_is_lap(end+1) = is_lap;
            if is_lap
                fprintf('周回 %d 完了: %.1f m\n', lap, dist_m(k));
                lap = lap + 1;
            end
            next_wp  = mod(next_wp + 1, n_wp);
            cooldown = 5;
        end
    end
    fprintf('検出WP通過数: %d回\n', numel(pass_dist));
end

%% ---- 理論速度プロファイルを生成 ----------------------------------------
% theory_state: 0=定速, 1=減速, 2=加速
speed_theory = NaN(size(dist_m));
theory_state = zeros(size(dist_m));

if has_wp && numel(pass_dist) >= 2
    MAX_DELTA = max(SPEED_MAX - SPEED_MIN, 1e-3);
    n_pass = numel(pass_dist);

    %% ---- ランプイベントを事前計算（WP通過ごとに記録）----
    % 各行: [開始距離, v_from, v_to, ramp_dist]
    % ランプはセグメント境界をまたいで有効であるため、ここで一括管理する
    ramp_tbl = zeros(0, 4);
    for pi = 1:n_pass
        cur_wp_   = pass_wp_num(pi);   % 0-indexed
        prev_seg1 = mod(cur_wp_ - 1, n_wp) + 1;
        cur_seg1  = cur_wp_ + 1;
        v_prev    = seg_speed(prev_seg1);
        v_curr    = seg_speed(cur_seg1);

        if is_corner(cur_wp_ + 1)
            v_from = v_prev * CORNER_SLOWDOWN_RATIO;
            ramp_d = CORNER_BASE_DIST * (v_prev / SPEED_MIN) * CORNER_ACCEL_RATIO;
            ramp_d = min(ramp_d, seg_len(cur_seg1));
        else
            v_from = v_prev;
            ramp_d = CORNER_BASE_DIST * abs(v_curr - v_from) / MAX_DELTA;
            if v_curr > v_from
                ramp_d = ramp_d * CORNER_ACCEL_RATIO;
            end
            ramp_d = min(ramp_d, seg_len(cur_seg1));
        end

        if ramp_d > 0.01 && abs(v_curr - v_from) > 0.01
            ramp_tbl(end+1, :) = [pass_dist(pi), v_from, v_curr, ramp_d]; %#ok<AGROW>
        end
    end

    %% ---- 各ログ点の理論速度を計算 ----------------------------------------
    for k = 1:length(dist_m)
        d = dist_m(k);
        if d < pass_dist(1) || d > pass_dist(end), continue; end

        seg_k = find(pass_dist <= d, 1, 'last');
        if seg_k >= n_pass, continue; end

        d_to_next = pass_dist(seg_k+1) - d;
        next_wp_  = pass_wp_num(seg_k+1);  % 0-indexed
        cur_wp_   = pass_wp_num(seg_k);
        cur_seg1  = cur_wp_ + 1;
        v_seg     = seg_speed(cur_seg1);
        v_corner  = v_seg * CORNER_SLOWDOWN_RATIO;
        corner_d  = CORNER_BASE_DIST * (v_seg / SPEED_MIN);

        % ---- 優先度1: 次コーナーへの接近ランプ（減速）----
        if is_corner(next_wp_ + 1) && d_to_next <= corner_d
            ratio = d_to_next / corner_d;
            speed_theory(k) = v_corner + ratio * (v_seg - v_corner);
            theory_state(k) = 1;
            continue;
        end

        % ---- 優先度2: 最新の有効ランプイベントを探す（セグメントをまたいでOK）----
        active = [];
        for ri = size(ramp_tbl, 1):-1:1
            r_start = ramp_tbl(ri, 1);
            r_dist  = ramp_tbl(ri, 4);
            if d >= r_start && d <= r_start + r_dist
                active = ramp_tbl(ri, :);
                break;
            end
        end

        if ~isempty(active)
            r_start = active(1);
            v_from  = active(2);
            v_to    = active(3);
            r_dist  = active(4);
            remaining = r_dist - (d - r_start);
            ratio = remaining / r_dist;
            speed_theory(k) = v_to + ratio * (v_from - v_to);
            if v_from > v_to
                theory_state(k) = 1;  % 減速
            else
                theory_state(k) = 2;  % 加速
            end
            continue;
        end

        % ---- 定速 ----
        speed_theory(k) = v_seg;
        theory_state(k) = 0;
    end
    speed_theory = min(SPEED_MAX, max(0, speed_theory));
end

%% ---- Figure作成 ----------------------------------------
figure('Name', ['Speed vs Distance: ' log_csv], 'NumberTitle', 'off', ...
    'Position', [100 100 1100 480]);
ax = axes;
hold(ax, 'on');

%% ---- FIX区間を薄緑、FLOAT区間を薄黄色の帯 ----------------------------------------
v_vals = [T.speed_mps; speed_theory(~isnan(speed_theory))];
y_top  = max(v_vals) * 1.2 + 0.1;
in_fix = false;
for k = 1:length(dist_m)
    if isFix(k) && ~in_fix
        seg_s = dist_m(k); in_fix = true;
    elseif ~isFix(k) && in_fix
        fill(ax,[seg_s dist_m(k) dist_m(k) seg_s],[0 0 y_top y_top], ...
            [0.85 1 0.85],'EdgeColor','none','HandleVisibility','off');
        in_fix = false;
    end
end
if in_fix
    fill(ax,[seg_s dist_m(end) dist_m(end) seg_s],[0 0 y_top y_top], ...
        [0.85 1 0.85],'EdgeColor','none','HandleVisibility','off');
end
in_float = false;
for k = 1:length(dist_m)
    if isFloat(k) && ~in_float
        seg_s = dist_m(k); in_float = true;
    elseif ~isFloat(k) && in_float
        fill(ax,[seg_s dist_m(k) dist_m(k) seg_s],[0 0 y_top y_top], ...
            [1 1 0.80],'EdgeColor','none','HandleVisibility','off');
        in_float = false;
    end
end
if in_float
    fill(ax,[seg_s dist_m(end) dist_m(end) seg_s],[0 0 y_top y_top], ...
        [1 1 0.80],'EdgeColor','none','HandleVisibility','off');
end

%% ---- 速度プロット ----------------------------------------
plot(ax, dist_m, T.speed_mps, '-', 'Color', [0 0.45 0.74], ...
    'LineWidth', 1.5, 'DisplayName', '実測速度（GNSS Doppler）');
if has_wp
    % 理論速度を状態ごとに色分けして描画
    % 定速(0)=紫, 減速(1)=赤, 加速(2)=緑
    % 各シリーズのマスクを隣接1点ずつ拡張し、シリーズ間の途切れをなくす
    % 後から描画するシリーズが境界点を上書きするため色は正しく表示される
    COL = {[0.50 0.00 0.55], [0.85 0.10 0.10], [0.78 0.58 0.90]};
    LABELS = {'理論速度（定速）', '理論速度（減速）', '理論速度（加速）'};
    for state = 0:2
        is_st = (theory_state == state) & ~isnan(speed_theory);
        is_st = is_st | [false; is_st(1:end-1)] | [is_st(2:end); false];
        v_plot = speed_theory;
        v_plot(~is_st) = NaN;
        if all(isnan(v_plot)), continue; end
        plot(ax, dist_m, v_plot, '-', 'Color', COL{state+1}, ...
            'LineWidth', 2.2, 'DisplayName', LABELS{state+1});
    end
end

%% ---- WP通過マーカー（実測速度上の点）----------------------------------------
if has_wp && ~isempty(pass_dist)
    for i = 1:numel(pass_dist)
        % 対応するログ点の速度を取得
        [~, ki] = min(abs(dist_m - pass_dist(i)));
        v_at_wp = T.speed_mps(ki);

        if pass_is_lap(i)
            % 周回境界WP0 → 赤い大きいダイヤモンド
            scatter(ax, pass_dist(i), v_at_wp, 80, 'r', 'd', 'filled', ...
                'HandleVisibility', 'off');
            xline(ax, pass_dist(i), '-', 'Color', [0.8 0 0], ...
                'LineWidth', 1.5, 'HandleVisibility', 'off');
        end

        text(ax, pass_dist(i), y_top*0.97, ...
            sprintf('WP%d', pass_wp_num(i)), ...
            'FontSize', 7, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'Color', [0.2 0.2 0.2], 'HandleVisibility', 'off');
        xline(ax, pass_dist(i), ':', 'Color', [0.55 0.55 0.55], ...
            'LineWidth', 0.6, 'HandleVisibility', 'off');
    end
end

%% ---- 装飾 ----------------------------------------
fill(ax, NaN, NaN, [0.85 1 0.85], 'EdgeColor', 'none', 'DisplayName', 'FIX区間');
fill(ax, NaN, NaN, [1 1 0.80],   'EdgeColor', 'none', 'DisplayName', 'FLOAT区間');
scatter(ax, NaN, NaN, 80, 'r', 'd', 'filled', 'DisplayName', '周回完了（WP0）');

xlabel(ax, '走行距離 [m]');
ylabel(ax, '速度 [m/s]');
xlim(ax, [0, dist_m(end)]);
ylim(ax, [0, y_top]);
legend(ax, 'Location', 'northeast');
title(ax, sprintf('%s  （総距離: %.1f m）', log_csv, dist_m(end)), ...
    'Interpreter', 'none');
grid(ax, 'on');

%% ---- コンソール出力 ----------------------------------------
fprintf('総走行距離: %.1f m\n', dist_m(end));
fprintf('最高速度（実測）: %.3f m/s\n', max(T.speed_mps));
if any(isFix)
    fprintf('平均速度（FIX）: %.3f m/s\n', mean(T.speed_mps(isFix)));
end
fprintf('\nセグメント速度（理論）:\n');
for i = 1:n_wp
    ni = mod(i, n_wp) + 1;
    c = '';
    if is_corner(ni), c = ' ← コーナー'; end
    fprintf('  WP%d→WP%d: %.2fm/s  (%.1fm)%s\n', ...
        i-1, ni-1, seg_speed(i), seg_len(i), c);
end
end

%% ---- ローカル関数 --------------------------------------------------
function [e, n] = latlon2enu(lat, lon, lat0, lon0)
    n = (lat - lat0) * 111111;
    e = (lon - lon0) * 111111 * cosd(lat0);
end
