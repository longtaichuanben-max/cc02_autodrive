function plot_log_map(log_csv, wp_file)
%PLOT_LOG_MAP gnss_loggerが出力したログCSVを地図上にプロットする。
%
%   plot_log_map(LOG_CSV)
%   plot_log_map(LOG_CSV, WP_FILE)
%
%   使用例:
%       plot_log_map('gnss_log_latest.csv', 'wp_position_basic.csv')

%% ---- コントローラパラメータ（control_bringup の設定と合わせること）----
WP_RADIUS    = 0.7;            % m: デフォルトWP到達半径 (wp_radius)
WP_RADII_STR = '5:1.2,7:2.0'; % 個別半径 (0-based・skip前) ← wp_radii と合わせること
CORNER_IDX   = [0,2,6,8];     % corner_wp_indices (0-based)
SKIP_IDX     = [3];            % wp_skip_indices (0-based) ← wp_skip_indices と合わせること

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

% WP_RADII_STR をパース → radius_map (0-based raw index → radius)
radius_map = containers.Map('KeyType','int32','ValueType','double');
if ~isempty(WP_RADII_STR)
    for tok = strsplit(WP_RADII_STR, ',')
        parts = strsplit(strtrim(tok{1}), ':');
        if numel(parts) == 2
            try
                radius_map(int32(str2double(parts{1}))) = str2double(parts{2});
            catch; end
        end
    end
end

%% ---- WPスキップ適用 -----------------------------------------------
valid_skip = SKIP_IDX(SKIP_IDX > 0 & SKIP_IDX < n_wp - 1);  % 0-based
keep_0  = setdiff(0:n_wp-1, valid_skip);  % 0-based
keep_1  = keep_0 + 1;                      % 1-based
wps_e   = wp_e(keep_1);
wps_n   = wp_n(keep_1);
wps_lat = wp_lat(keep_1);
wps_lon = wp_lon(keep_1);
n_wps   = numel(wps_e);

% old(0-based) → new(0-based) マッピング
old2new = -ones(1, n_wp);
for ni = 0:numel(keep_0)-1
    old2new(keep_0(ni+1)+1) = ni;
end

% radius_map をスキップ後の新インデックスにリマップ
rm_new = containers.Map('KeyType','int32','ValueType','double');
for k = keys(radius_map)
    oi = double(k{1});
    if oi+1 >= 1 && oi+1 <= n_wp && old2new(oi+1) >= 0
        rm_new(int32(old2new(oi+1))) = radius_map(int32(oi));
    end
end

% corner_idx をスキップ後の新インデックスにリマップ
corner_new = [];
for c = CORNER_IDX
    if c+1 >= 1 && c+1 <= n_wp && old2new(c+1) >= 0
        corner_new(end+1) = old2new(c+1); %#ok<AGROW>
    end
end

%% ---- 元のWP直線経路（薄い破線）--------------------------------------
geoplot(gx, wp_lat, wp_lon, '--', 'Color', [0.5 0.5 1], 'LineWidth', 1.0, ...
    'DisplayName', '元のWP経路（skip前）');

% スキップされたWP
if ~isempty(valid_skip)
    sk1 = valid_skip + 1;
    geoscatter(gx, wp_lat(sk1), wp_lon(sk1), 80, [0.5 0.5 0.5], 'x', ...
        'LineWidth', 2, 'DisplayName', 'スキップWP');
end

%% ---- コントローラが走行するWP経路（skip適用後）---------------------
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
        r = WP_RADIUS;
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
fprintf('\n--- コントローラ経路 WP一覧（skip後、%d点）---\n', n_wps);
for ni = 0:n_wps-1
    tag = '';
    if ismember(ni, corner_new), tag = '  ← コーナー'; end
    if isKey(rm_new, int32(ni))
        r = rm_new(int32(ni));
    else
        r = WP_RADIUS;
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
