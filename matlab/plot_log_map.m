function plot_log_map(log_csv, wp_file)
%PLOT_LOG_MAP gnss_loggerが出力したログCSVを地図上にプロットする。
%
%   plot_log_map(LOG_CSV)
%   plot_log_map(LOG_CSV, WP_FILE)
%
%   使用例:
%       plot_log_map('gnss_log_latest.csv', 'wp_position_basic.csv')

%% ---- コントローラパラメータ（pure_pursuit_controller.py の declare_parameter と合わせること）----
% WPのコーナー・到達半径はコントローラと同じロジックで自動計算する（手動設定不要）
CORNER_ANGLE_THRESH_DEG  = 40.0;  % corner_angle_thresh_deg：これ以上の偏向角をコーナーと判定
LH_RAMP_ANGLE_THRESH_DEG = 85.0;  % lh_ramp_angle_thresh_deg
WP_RADIUS_MAX             = 2.0;   % wp_radius_max [m]
WP_RADIUS_MIN             = 0.5;   % wp_radius_min [m]
WP_RADIUS_SEG_RATIO       = 0.3;   % wp_radius_seg_ratio

if nargin < 2, wp_file = ''; end

%% ---- GNSSログ読み込み・表示 ----------------------------------------
T = readtable(log_csv, 'TextType', 'string');
n_total = height(T);
valid = ~(T.latitude == 0 & T.longitude == 0);
T = T(valid, :);

if isempty(T)
    error('plot_log_map:noValidPoints', ...
        '有効な緯度経度を持つ行が見つかりませんでした（全%d行）: %s', n_total, log_csv);
end

figure('Name', ['GNSS Log Map: ' log_csv], 'NumberTitle', 'off');
gx = geoaxes;
geobasemap(gx, 'satellite');
hold(gx, 'on');

geoplot(gx, T.latitude, T.longitude, '-', ...
    'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '走行軌跡');

isFix   = T.status_str == "FIX";
isFloat = T.status_str == "FLOAT";
isOther = ~isFix & ~isFloat;
if any(isFix)
    geoscatter(gx, T.latitude(isFix),   T.longitude(isFix),   20, ...
        [0 0.6 0], 'filled', 'DisplayName', 'FIX');
end
if any(isFloat)
    geoscatter(gx, T.latitude(isFloat), T.longitude(isFloat), 20, ...
        [1 0.55 0], 'filled', 'DisplayName', 'FLOAT');
end
if any(isOther)
    geoscatter(gx, T.latitude(isOther), T.longitude(isOther), 20, ...
        [0.8 0 0], 'filled', 'DisplayName', 'その他（NONE等）');
end
geoplot(gx, T.latitude(1),   T.longitude(1),   '^', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'DisplayName', '走行開始');
geoplot(gx, T.latitude(end), T.longitude(end), 's', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'DisplayName', '走行終了');

fprintf('読み込み: 全%d行 → 有効%d点 (FIX=%d, FLOAT=%d, その他=%d)\n', ...
    n_total, height(T), sum(isFix), sum(isFloat), sum(isOther));

if isempty(wp_file)
    legend(gx, 'Location', 'best');
    title(gx, log_csv, 'Interpreter', 'none');
    return
end

%% ---- WaypointCSV読み込み -------------------------------------------
WP     = readtable(wp_file, 'VariableNamingRule', 'preserve');
wp_lat = WP.("Latitude(deg)");
wp_lon = WP.("Longitude(deg)");
n_wp   = numel(wp_lat);

% ENU原点 = WP[0]のlat/lon
origin_lat = wp_lat(1);
origin_lon = wp_lon(1);

[wp_e, wp_n] = latlon2enu(wp_lat, wp_lon, origin_lat, origin_lon);

%% ---- コーナー・到達半径をコントローラと同じロジックで自動計算 --------
% pure_pursuit_controller.py の _detect_corner_wps() と同じ計算
corner_new  = [];   % コーナーWPの0-basedインデックス
lh_ramp_new = [];   % lh_ramp WPの0-basedインデックス
rm_new = containers.Map('KeyType','int32','ValueType','double');

fprintf('\n--- WP自動解析 (corner≥%.0f°, lh_ramp≥%.0f°, r_min=%.1fm r_max=%.1fm) ---\n', ...
    CORNER_ANGLE_THRESH_DEG, LH_RAMP_ANGLE_THRESH_DEG, WP_RADIUS_MIN, WP_RADIUS_MAX);

for i = 0:n_wp-1
    i1     = i + 1;
    prev1  = mod(i - 1, n_wp) + 1;
    next1  = mod(i + 1, n_wp) + 1;
    v_in   = [wp_e(i1) - wp_e(prev1), wp_n(i1) - wp_n(prev1)];
    v_out  = [wp_e(next1) - wp_e(i1), wp_n(next1) - wp_n(i1)];
    len_in  = norm(v_in);
    len_out = norm(v_out);
    if len_in < 0.1 || len_out < 0.1
        fprintf('  WP[%d] スキップ（ほぼ同位置）\n', i);
        continue
    end
    cos_a = dot(v_in, v_out) / (len_in * len_out);
    cos_a = max(-1.0, min(1.0, cos_a));
    deg   = rad2deg(acos(cos_a));

    tag = '';
    if deg >= CORNER_ANGLE_THRESH_DEG
        corner_new(end+1)  = i; %#ok<AGROW>
        tag = [tag ' ← コーナー'];
    end
    if deg >= LH_RAMP_ANGLE_THRESH_DEG
        lh_ramp_new(end+1) = i; %#ok<AGROW>
        tag = [tag ' [lh_ramp]'];
    end
    r_angle = WP_RADIUS_MAX * (1.0 - deg / 180.0);
    r_seg   = min(len_in, len_out) * WP_RADIUS_SEG_RATIO;
    r       = max(WP_RADIUS_MIN, min(r_angle, r_seg));
    rm_new(int32(i)) = r;
    fprintf('  WP[%d] 偏向角=%.1f°  r=%.2fm%s\n', i, deg, r, tag);
end

% スキップWPなし（コントローラと同じ）
wps_e   = wp_e;
wps_n   = wp_n;
wps_lat = wp_lat;
wps_lon = wp_lon;
n_wps   = n_wp;

%% ---- WP直線経路 -----------------------------------------------------
geoplot(gx, wps_lat, wps_lon, '-', 'Color', [0 0.3 1], 'LineWidth', 2.2, ...
    'DisplayName', 'コントローラ経路（skip後）');

%% ---- 各WPマーカーと番号ラベル---------------------------------------
th = linspace(0, 2*pi, 72)';
for ni = 0:n_wps-1
    lat_i = wps_lat(ni+1);
    lon_i = wps_lon(ni+1);
    e_i   = wps_e(ni+1);
    n_i   = wps_n(ni+1);

    is_corner = ismember(ni, corner_new);

    % マーカー形状・色
    if is_corner
        geoscatter(gx, lat_i, lon_i, 70, 'r', 'd', 'filled', ...
            'HandleVisibility', 'off');
    else
        geoscatter(gx, lat_i, lon_i, 50, 'm', 'v', 'filled', ...
            'HandleVisibility', 'off');
    end

    % 到達判定半径
    if isKey(rm_new, int32(ni))
        r = rm_new(int32(ni));
    else
        r = WP_RADIUS_MIN;
    end
    cx = e_i + r*cos(th);
    cy = n_i + r*sin(th);
    [clat, clon] = enu2latlon(cx, cy, origin_lat, origin_lon);
    if is_corner
        col = [1 0.1 0.1]; lw = 1.2;
    else
        col = [0.15 0.4 1]; lw = 1.0;
    end
    geoplot(gx, clat, clon, '-', 'Color', col, 'LineWidth', lw, ...
        'HandleVisibility', 'off');

    % WP番号テキスト（0-based）
    text(gx, lat_i, lon_i, sprintf(' %d', ni), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'w', ...
        'HorizontalAlignment', 'left', 'HandleVisibility', 'off');
end

% 凡例用ダミーマーカー
geoscatter(gx, NaN, NaN, 50, 'm', 'v', 'filled', 'DisplayName', 'Waypoint（skip後）');
geoscatter(gx, NaN, NaN, 70, 'r', 'd', 'filled', 'DisplayName', 'コーナーWP');

%% ---- コンソール出力（WP一覧）---------------------------------------
fprintf('\n--- コントローラ経路 WP一覧（%d点）---\n', n_wps);
for ni = 0:n_wps-1
    tag = '';
    if ismember(ni, corner_new),  tag = [tag '  ← コーナー']; end
    if ismember(ni, lh_ramp_new), tag = [tag ' [lh_ramp]'];   end
    if isKey(rm_new, int32(ni))
        r = rm_new(int32(ni));
    else
        r = WP_RADIUS_MIN;
    end
    fprintf('  [%2d] E=%+7.2fm N=%+8.2fm  r=%.2fm%s\n', ...
        ni, wps_e(ni+1), wps_n(ni+1), r, tag);
end

legend(gx, 'Location', 'best');
title(gx, log_csv, 'Interpreter', 'none');
end

%% ---- ローカル関数 --------------------------------------------------
function [e, n] = latlon2enu(lat, lon, lat0, lon0)
    n = (lat - lat0) * 111111;
    e = (lon - lon0) * 111111 * cosd(lat0);
end

function [lat, lon] = enu2latlon(e, n, lat0, lon0)
    lat = lat0 + n / 111111;
    lon = lon0 + e / (111111 * cosd(lat0));
end
