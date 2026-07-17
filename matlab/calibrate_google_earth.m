function calibrate_google_earth(img_file, wp_csv)
%CALIBRATE_GOOGLE_EARTH Google Earthスクリーンショットの地理座標を逆算する。
%
%   calibrate_google_earth('course_satellite.png')
%       → 橋端2点をクリックするだけで完了（デフォルト：2クリックモード）
%
%   calibrate_google_earth('course_satellite.png', 'wp_position_advance.csv')
%       → CSV内の全WPをクリックして高精度キャリブレーション
%
%   操作:
%       スクロールホイール : ズームイン/アウト（カーソル中心）
%       R キー             : 全体表示にリセット
%       左クリック         : 点を確定
%
%   完了すると plot_log_map で使う west/east/south/north が表示される。
%   前提: 画像は北上（North-up）であること。

%% ---- 橋端参照座標（Google Earth 2026-07-15 実測）
REF_LAT = [35 + 41/60 + 22.30/3600,  35 + 41/60 + 24.62/3600];
REF_LON = [140 + 1/60 + 14.57/3600, 140 + 1/60 + 19.55/3600];
REF_LBL = {'左橋端 (hidarihasi)', '右橋端 (migihasi)'};

%% ---- 画像表示
img = imread(img_file);
[h, w, ~] = size(img);
fig = figure('Name', 'Google Earth 座標キャリブレーション', 'NumberTitle', 'off');
ax = axes(fig);
imshow(img, 'Parent', ax);
hold(ax, 'on');

xl0 = [0.5, w+0.5];
yl0 = [0.5, h+0.5];

% スクロールズーム・R キーリセットを登録（uiwait中も有効）
set(fig, 'WindowScrollWheelFcn', @(~,e) scroll_zoom(ax, e));
set(fig, 'KeyPressFcn',          @(~,e) key_press(ax, xl0, yl0, e));

%% ---- モード分岐
if nargin < 2
    %% ==== 2クリックモード（橋端参照点） ====
    fprintf('\n--- キャリブレーション開始（2クリックモード）---\n');
    fprintf('操作: スクロールでズーム / R でリセット / 左クリックで確定\n\n');

    px = nan(1, 2);
    py = nan(1, 2);
    for ri = 1:2
        set_title(ax, sprintf('%s を左クリックで確定  [スクロール:ズーム / R:リセット]', REF_LBL{ri}));
        [xi, yi] = wait_for_click(fig, ax);
        if isnan(xi), error('図が閉じられました。'); end
        px(ri) = xi;
        py(ri) = yi;
        scatter(ax, xi, yi, 200, 'p', ...
            'MarkerFaceColor', [0.95 0.75 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        text(ax, xi+10, yi, sprintf('  %s', REF_LBL{ri}), ...
            'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold');
        fprintf('  %s: pixel=(%.0f, %.0f)\n', REF_LBL{ri}, xi, yi);
    end
    set_title(ax, '完了');

    if abs(px(2) - px(1)) < 1 || abs(py(2) - py(1)) < 1
        error('2点が近すぎます。');
    end
    a = (REF_LON(2) - REF_LON(1)) / (px(2) - px(1));
    b = (REF_LAT(2) - REF_LAT(1)) / (py(2) - py(1));
    c = REF_LON(1) - a * px(1);
    d = REF_LAT(1) - b * py(1);

    west  = a * 0 + c;
    east  = a * w + c;
    south = min(b * 0 + d, b * h + d);
    north = max(b * 0 + d, b * h + d);

    m_per_px_x = abs(a) * 111111 * cosd(mean(REF_LAT));
    m_per_px_y = abs(b) * 111111;
    ratio = m_per_px_x / m_per_px_y;
    fprintf('\n  x解像度: %.4fm/px  y解像度: %.4fm/px\n', m_per_px_x, m_per_px_y);
    if ratio < 0.8 || ratio > 1.25
        fprintf('  警告: x/y スケール比 = %.2f（北上でない可能性）\n', ratio);
    end

else
    %% ==== 多WPクリックモード（高精度）====
    T = readtable(wp_csv, 'VariableNamingRule', 'preserve');
    wp_num = T.WP;
    wp_lat = T.("Latitude(deg)")';
    wp_lon = T.("Longitude(deg)")';
    if numel(wp_lat) > 1 && wp_lat(end) == wp_lat(1) && wp_lon(end) == wp_lon(1)
        wp_lat(end) = [];
        wp_lon(end) = [];
        wp_num(end) = [];
    end
    n_wp = numel(wp_lat);

    fprintf('\n--- キャリブレーション開始（多WPモード: %s）---\n', wp_csv);
    fprintf('操作: スクロールでズーム / R でリセット / 左クリックで確定\n');
    wp_list = sprintf('WP%d', wp_num(1));
    for k = 2:n_wp
        wp_list = [wp_list sprintf(' → WP%d', wp_num(k))]; %#ok<AGROW>
    end
    fprintf('順番: %s\n\n', wp_list);

    pxw = zeros(n_wp, 1);
    pyw = zeros(n_wp, 1);
    for i = 1:n_wp
        set_title(ax, sprintf('[%d/%d]  WP%d  (lat=%.6f, lon=%.6f)  — 左クリックで確定', ...
            i, n_wp, wp_num(i), wp_lat(i), wp_lon(i)));
        [xi, yi] = wait_for_click(fig, ax);
        if isnan(xi), error('図が閉じられました。'); end
        pxw(i) = xi;
        pyw(i) = yi;
        plot(ax, xi, yi, 'g+', 'MarkerSize', 15, 'LineWidth', 2);
        text(ax, xi+8, yi, sprintf('WP%d', wp_num(i)), ...
            'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
        fprintf('  WP%d [%d/%d] pixel=(%.0f, %.0f)\n', wp_num(i), i, n_wp, xi, yi);
    end
    set_title(ax, '完了');

    A = [pxw, pyw, ones(n_wp, 1)];
    coef_lon = A \ wp_lon(:);
    coef_lat = A \ wp_lat(:);

    lon_err_m = (wp_lon(:) - A*coef_lon) * 111111 * cosd(mean(wp_lat));
    lat_err_m = (wp_lat(:) - A*coef_lat) * 111111;
    fprintf('\n--- フィット残差 ---\n');
    for i = 1:n_wp
        fprintf('  WP%d: Δlon=%.2fm  Δlat=%.2fm\n', wp_num(i), lon_err_m(i), lat_err_m(i));
    end
    fprintf('  RMS: %.2fm\n', sqrt(mean(lon_err_m.^2 + lat_err_m.^2)));

    corners_px = [1, 1;  w, 1;  1, h;  w, h];
    corners_lon = corners_px * coef_lon(1:2) + coef_lon(3);
    corners_lat = corners_px * coef_lat(1:2) + coef_lat(3);
    west  = min(corners_lon);
    east  = max(corners_lon);
    south = min(corners_lat);
    north = max(corners_lat);
end

%% ---- 結果表示
fprintf('\n========================================\n');
fprintf('plot_log_map.m に貼り付ける値:\n');
fprintf('========================================\n');
fprintf('west  = %.8f;\n', west);
fprintf('east  = %.8f;\n', east);
fprintf('south = %.8f;\n', south);
fprintf('north = %.8f;\n', north);
fprintf('img_file = ''%s'';\n', img_file);
fprintf('========================================\n\n');
end

%% ---- ローカル関数 --------------------------------------------------

function [xi, yi] = wait_for_click(fig, ax)
% ginput の代替: uiwait/uiresume を使うためスクロールズームが有効なまま左クリックを待つ
    set(fig, 'UserData', [NaN NaN]);
    set(fig, 'WindowButtonDownFcn', @on_click);
    uiwait(fig);
    pos = get(fig, 'UserData');
    xi  = pos(1);
    yi  = pos(2);
    set(fig, 'WindowButtonDownFcn', '');

    function on_click(src, ~)
        if strcmp(get(src, 'SelectionType'), 'normal')  % 左クリックのみ
            cp = get(ax, 'CurrentPoint');
            set(src, 'UserData', [cp(1,1), cp(1,2)]);
            uiresume(src);
        end
    end
end

function scroll_zoom(ax, evt)
    xl = xlim(ax);
    yl = ylim(ax);
    cp = get(ax, 'CurrentPoint');
    cx = cp(1,1);
    cy = cp(1,2);
    factor = 1.15 ^ abs(evt.VerticalScrollCount);
    if evt.VerticalScrollCount > 0
        xl = cx + (xl - cx) * factor;
        yl = cy + (yl - cy) * factor;
    else
        xl = cx + (xl - cx) / factor;
        yl = cy + (yl - cy) / factor;
    end
    xlim(ax, xl);
    ylim(ax, yl);
end

function key_press(ax, xl0, yl0, evt)
    if strcmpi(evt.Key, 'r')
        xlim(ax, xl0);
        ylim(ax, yl0);
    end
end

function set_title(ax, str)
    title(ax, str, 'FontSize', 11, 'Color', 'r', 'Interpreter', 'none');
    drawnow;
end
