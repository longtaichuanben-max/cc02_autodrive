function plot_log_map(log_csv, wp_file)
%PLOT_LOG_MAP gnss_loggerが出力したログCSVを地図上にプロットする。
%
%   plot_log_map(LOG_CSV)
%   plot_log_map(LOG_CSV, WP_FILE)
%
%   使用例:
%       plot_log_map('gnss_log_latest.csv', 'wp_position_basic.csv')

%% ---- コントローラパラメータ（pure_pursuit_controller.py の declare_parameter と合わせること）----
CORNER_ANGLE_THRESH_DEG  = 36.0;
LH_RAMP_ANGLE_THRESH_DEG = 85.0;
WP_RADIUS_MAX             = 2.0;
WP_RADIUS_MIN             = 0.5;
WP_RADIUS_SEG_RATIO       = 0.3;
SPEED_MIN                 = 1.0;
SPEED_MAX                 = 3.0;
SPEED_DIST_SHORT          = 5.0;
SPEED_DIST_LONG           = 10.0;
CORNER_SLOWDOWN_RATIO     = 0.35;
CORNER_BASE_DIST          = 1.8;
CORNER_ACCEL_RATIO        = 1.0;    % corner_accel_ratio：加速ランプ距離 = 減速ランプ距離 × この値

if nargin < 2, wp_file = ''; end

%% ---- GNSSログ読み込み ----------------------------------------
T = readtable(log_csv, 'TextType', 'string');
n_total = height(T);
valid = ~(T.latitude == 0 & T.longitude == 0);
T = T(valid, :);

if isempty(T)
    error('plot_log_map:noValidPoints', ...
        '有効な緯度経度を持つ行が見つかりませんでした（全%d行）: %s', n_total, log_csv);
end

%% ---- Figure・axes作成 ----------------------------------------
figure('Name', ['GNSS Log Map: ' log_csv], 'NumberTitle', 'off');
gx = axes;
hold(gx, 'on');
set(gx, 'Color', 'white');

%% ---- GNSSログ表示 ----------------------------------------
plot(gx, T.longitude, T.latitude, '-', ...
    'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '走行軌跡');

isFix   = T.status_str == "FIX";
isFloat = T.status_str == "FLOAT";
isOther = ~isFix & ~isFloat;
if any(isFix)
    scatter(gx, T.longitude(isFix),   T.latitude(isFix),   20, ...
        [0 0.6 0], 'filled', 'DisplayName', 'FIX');
end
if any(isFloat)
    scatter(gx, T.longitude(isFloat), T.latitude(isFloat), 20, ...
        [1 0.55 0], 'filled', 'DisplayName', 'FLOAT');
end
if any(isOther)
    scatter(gx, T.longitude(isOther), T.latitude(isOther), 20, ...
        [0.8 0 0], 'filled', 'DisplayName', 'その他（NONE等）');
end

xlabel(gx, '経度 [°]');
ylabel(gx, '緯度 [°]');
axis(gx, 'equal');

% データ範囲にズームイン（余白 0.0002°≒20m）
pad = 0.0002;
xlim(gx, [min(T.longitude)-pad, max(T.longitude)+pad]);
ylim(gx, [(min(T.latitude)-pad), (max(T.latitude)+pad)]);

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

origin_lat = wp_lat(1);
origin_lon = wp_lon(1);
[wp_e, wp_n] = latlon2enu(wp_lat, wp_lon, origin_lat, origin_lon);

%% ---- コーナー・到達半径 自動計算 ----------------------------------------
corner_new  = [];
lh_ramp_new = [];
rm_new = containers.Map('KeyType','int32','ValueType','double');

fprintf('\n--- WP自動解析 (corner≥%.0f°, lh_ramp≥%.0f°, r_min=%.1fm r_max=%.1fm) ---\n', ...
    CORNER_ANGLE_THRESH_DEG, LH_RAMP_ANGLE_THRESH_DEG, WP_RADIUS_MIN, WP_RADIUS_MAX);

for i = 0:n_wp-1
    i1    = i + 1;
    prev1 = mod(i - 1, n_wp) + 1;
    next1 = mod(i + 1, n_wp) + 1;
    v_in  = [wp_e(i1) - wp_e(prev1), wp_n(i1) - wp_n(prev1)];
    v_out = [wp_e(next1) - wp_e(i1), wp_n(next1) - wp_n(i1)];
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

wps_lat = wp_lat;
wps_lon = wp_lon;
wps_e   = wp_e;
wps_n   = wp_n;
n_wps   = n_wp;

%% ---- WP経路・マーカー表示 ----------------------------------------
plot(gx, wps_lon, wps_lat, '-', 'Color', [0 0.3 1], 'LineWidth', 2.2, ...
    'DisplayName', 'コントローラ経路');

th = linspace(0, 2*pi, 72)';
for ni = 0:n_wps-1
    lat_i = wps_lat(ni+1);
    lon_i = wps_lon(ni+1);
    e_i   = wps_e(ni+1);
    n_i   = wps_n(ni+1);
    is_corner = ismember(ni, corner_new);

    if is_corner
        scatter(gx, lon_i, lat_i, 70, 'r', 'd', 'filled', 'HandleVisibility', 'off');
    else
        scatter(gx, lon_i, lat_i, 50, 'm', 'v', 'filled', 'HandleVisibility', 'off');
    end

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
    plot(gx, clon, clat, '-', 'Color', col, 'LineWidth', lw, 'HandleVisibility', 'off');

    text(gx, lon_i, lat_i, sprintf(' %d', ni), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'w', ...
        'HorizontalAlignment', 'left', 'HandleVisibility', 'off');
end

scatter(gx, NaN, NaN, 50, 'm', 'v', 'filled', 'DisplayName', 'Waypoint');
scatter(gx, NaN, NaN, 70, 'r', 'd', 'filled', 'DisplayName', 'コーナーWP');


%% ---- 速度ランプゾーンをWP経路上に色付き表示 ----------------------------------------
MAX_DELTA = max(SPEED_MAX - SPEED_MIN, 1e-3);

% セグメント長・速度
seg_len_map   = zeros(n_wps, 1);
seg_speed_map = zeros(n_wps, 1);
d_range_map   = SPEED_DIST_LONG - SPEED_DIST_SHORT;
for si = 1:n_wps
    ni = mod(si, n_wps) + 1;
    seg_len_map(si) = hypot(wps_e(ni)-wps_e(si), wps_n(ni)-wps_n(si));
    t = max(0, min(1, (seg_len_map(si) - SPEED_DIST_SHORT) / max(d_range_map, 1e-9)));
    seg_speed_map(si) = SPEED_MIN + t * (SPEED_MAX - SPEED_MIN);
end

% WP経路の累積距離
cum_dist_map = [0; cumsum(seg_len_map)];  % n_wps+1 要素（最後は1周分）

% 経路上の任意距離を lat/lon に変換するローカル関数を準備
% path_pos(d): 累積距離 d での [e, n] を返す（1周 = cum_dist_map(end)）
total_path = cum_dist_map(end);

% ランプイベントを累積距離ベースで列挙
% 各行: [start_d, end_d, type(1=減速/2=加速)]
ramp_zones = zeros(0, 3);
for wi = 1:n_wps
    wp0 = wi - 1;  % 0-indexed WP番号
    prev_seg1 = mod(wp0 - 1, n_wps) + 1;
    cur_seg1  = wi;
    v_prev = seg_speed_map(prev_seg1);
    v_curr = seg_speed_map(cur_seg1);
    wp_cum = cum_dist_map(wi);  % このWP通過時の累積距離

    % WP通過後ランプ（コーナー出口 or 非コーナー遷移）
    next_seg_len_map = seg_len_map(cur_seg1);
    if ismember(wp0, corner_new)
        v_from = v_prev * CORNER_SLOWDOWN_RATIO;
        ramp_d = CORNER_BASE_DIST * (v_prev / SPEED_MIN) * CORNER_ACCEL_RATIO;
        ramp_d = min(ramp_d, next_seg_len_map);
    else
        v_from = v_prev;
        ramp_d = CORNER_BASE_DIST * abs(v_curr - v_from) / MAX_DELTA;
        if v_curr > v_from
            ramp_d = ramp_d * CORNER_ACCEL_RATIO;
        end
        ramp_d = min(ramp_d, next_seg_len_map);
    end
    if ramp_d > 0.01 && abs(v_curr - v_from) > 0.01
        ramp_type = 1 + (v_from < v_curr);  % 1=減速, 2=加速
        ramp_zones(end+1, :) = [wp_cum, wp_cum + ramp_d, ramp_type]; %#ok<AGROW>
    end

    % コーナーWP への接近ランプ（コーナーWP が次のWPである場合は前のセグメントで処理）
    if ismember(wp0, corner_new)
        % このWP自体がコーナー → 前セグメントの減速ゾーンを追加
        corner_d = CORNER_BASE_DIST * (v_prev / SPEED_MIN);
        decel_start = wp_cum - corner_d;
        ramp_zones(end+1, :) = [decel_start, wp_cum, 1]; %#ok<AGROW>
    end
end

% 描画: サンプリング間隔 0.2m で経路上の色を決定してセグメント描画
sample_d  = 0 : 0.2 : total_path;
n_samples = numel(sample_d);
state_map = zeros(n_samples, 1);  % 0=定速, 1=減速, 2=加速

for ki = 1:n_samples
    d = sample_d(ki);
    % コーナー接近（減速）ゾーン優先
    for ri = 1:size(ramp_zones, 1)
        if ramp_zones(ri, 3) == 1 && d >= ramp_zones(ri, 1) && d <= ramp_zones(ri, 2)
            state_map(ki) = 1;
            break;
        end
    end
    if state_map(ki) ~= 0, continue; end
    % 加速ゾーン
    for ri = size(ramp_zones, 1):-1:1
        if ramp_zones(ri, 3) == 2 && d >= ramp_zones(ri, 1) && d <= ramp_zones(ri, 2)
            state_map(ki) = 2;
            break;
        end
    end
end

% サンプル点の lat/lon を計算
e_map = zeros(n_samples, 1);
n_map = zeros(n_samples, 1);
for ki = 1:n_samples
    d = mod(sample_d(ki), total_path);
    si = find(cum_dist_map <= d, 1, 'last');
    if si > n_wps, si = n_wps; end
    ni = mod(si, n_wps) + 1;
    frac = (d - cum_dist_map(si)) / max(seg_len_map(si), 1e-9);
    frac = min(1, frac);
    e_map(ki) = wps_e(si) + frac * (wps_e(ni) - wps_e(si));
    n_map(ki) = wps_n(si) + frac * (wps_n(ni) - wps_n(si));
end
[lat_map, lon_map] = enu2latlon(e_map, n_map, origin_lat, origin_lon);

% 各状態をNaNで区切りながら描画
COL_RAMP = {[0.50 0.00 0.55], [0.85 0.10 0.10], [0.78 0.58 0.90]};
LBL_RAMP = {'定速', '減速ランプ', '加速ランプ'};
LW_RAMP  = [2.2, 3.5, 3.5];
for st = 0:2
    lon_p = lon_map;  lat_p = lat_map;
    lon_p(state_map ~= st) = NaN;
    lat_p(state_map ~= st) = NaN;
    if all(isnan(lon_p)), continue; end
    if st == 0
        hv = 'off';  % 定速は経路線と重複するので凡例非表示
    else
        hv = 'on';
    end
    plot(gx, lon_p, lat_p, '-', 'Color', COL_RAMP{st+1}, ...
        'LineWidth', LW_RAMP(st+1), 'DisplayName', LBL_RAMP{st+1}, ...
        'HandleVisibility', hv);
end

fprintf('\n--- 速度ランプゾーン（1周分）---\n');
fprintf('  %-6s  %-10s  %-10s  %s\n', '種別', '開始[m]', '終了[m]', '距離[m]');
for ri = 1:size(ramp_zones, 1)
    types = {'減速', '加速'};
    fprintf('  %-6s  %8.1f  %8.1f  %6.1f\n', ...
        types{ramp_zones(ri,3)}, ramp_zones(ri,1), ramp_zones(ri,2), ...
        ramp_zones(ri,2)-ramp_zones(ri,1));
end

%% ---- コンソール出力 ----------------------------------------
fprintf('\n--- コントローラ経路 WP一覧（%d点）---\n', n_wps);
for ni = 0:n_wps-1
    tag = '';
    if ismember(ni, corner_new),  tag = [tag '  ← コーナー']; end
    if ismember(ni, lh_ramp_new), tag = [tag ' [lh_ramp]'];   end
    if isKey(rm_new, int32(ni)),  r = rm_new(int32(ni));
    else,                          r = WP_RADIUS_MIN; end
    fprintf('  [%2d] E=%+7.2fm N=%+8.2fm  r=%.2fm%s\n', ...
        ni, wps_e(ni+1), wps_n(ni+1), r, tag);
end

legend(gx, 'Location', 'best');
title(gx, log_csv, 'Interpreter', 'none');
end

%% ---- ローカル関数 --------------------------------------------------

function [img, lat_lim, lon_lim] = fetch_gsi_tiles(lat_lim, lon_lim, zoom)
% 国土地理院 seamlessphoto タイルを webread でダウンロードして結合する。
% 戻り値: img=uint8 RGB画像, lat_lim=[south north], lon_lim=[west east]
% 失敗時: img=[]
    img = [];
    n    = 2^zoom;
    base = 'http://cyberjapandata.gsi.go.jp/xyz/ort';

    x1 = floor((lon_lim(1) + 180) / 360 * n);
    x2 = floor((lon_lim(2) + 180) / 360 * n);
    y1 = floor((1 - log(tan(lat_lim(2)*pi/180) + sec(lat_lim(2)*pi/180)) / pi) / 2 * n);
    y2 = floor((1 - log(tan(lat_lim(1)*pi/180) + sec(lat_lim(1)*pi/180)) / pi) / 2 * n);

    nx = x2 - x1 + 1;
    ny = y2 - y1 + 1;
    canvas = zeros(ny*256, nx*256, 3, 'uint8');
    ok = false;

    fprintf('タイルダウンロード中 (%d×%d枚)...\n', nx, ny);
    opts = weboptions('Timeout', 10);
    for yi = 0:ny-1
        for xi = 0:nx-1
            url = sprintf('%s/%d/%d/%d.jpg', base, zoom, x1+xi, y1+yi);
            try
                tile = webread(url, opts);
                if ndims(tile) == 2
                    tile = repmat(tile, [1 1 3]);
                end
                if size(tile,1)==256 && size(tile,2)==256
                    r0 = yi*256+1; c0 = xi*256+1;
                    canvas(r0:r0+255, c0:c0+255, :) = tile;
                    ok = true;
                end
            catch
            end
        end
    end

    if ~ok, return; end

    img = canvas;
    lon_lim = [x1/n*360-180,  (x2+1)/n*360-180];
    north   = atan(sinh(pi*(1 - 2*y1    /n))) * 180/pi;
    south   = atan(sinh(pi*(1 - 2*(y2+1)/n))) * 180/pi;
    lat_lim = [south, north];
end

function [e, n] = latlon2enu(lat, lon, lat0, lon0)
    n = (lat - lat0) * 111111;
    e = (lon - lon0) * 111111 * cosd(lat0);
end

function [lat, lon] = enu2latlon(e, n, lat0, lon0)
    lat = lat0 + n / 111111;
    lon = lon0 + e / (111111 * cosd(lat0));
end
