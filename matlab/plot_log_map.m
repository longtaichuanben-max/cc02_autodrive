function plot_log_map(log_csv, wp_file)
%PLOT_LOG_MAP gnss_loggerが出力したログCSVを地図上にプロットする。
%
%   plot_log_map(LOG_CSV) は LOG_CSV（cc02_autodriveのgnss_logger_nodeが
%   書き出すCSV）を衛星写真ベースマップ上に表示する。
%
%   plot_log_map(LOG_CSV, WP_FILE) は元のWaypoint CSV
%   （WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）も重ねて表示
%   する。Waypoint間は直線で接続される（pid_node/stanley_node/
%   pure_pursuit_nodeが実際に走行する経路と同じ。スプライン補間はしない）。
%
%   要件: MATLAB Mapping Toolbox（geoplot/geoscatter/geobasemapを使用）。
%   ラズパイ（ARM Linux）ではMATLAB Desktopが動作しないため、ログCSVを
%   Windows/Mac等のMATLABインストール済みPCに転送して実行すること。
%
%   使用例:
%       plot_log_map('gnss_log_20260624_132350.csv', 'wp_position_basic.csv')

    if nargin < 2
        wp_file = '';
    end

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

    % 走行経路全体（連続線、グレー）
    geoplot(gx, T.latitude, T.longitude, '-', ...
        'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, ...
        'DisplayName', '走行経路（全体）');

    % ステータス別の点（FIX=緑、FLOAT=オレンジ、その他=赤）
    isFix   = T.status_str == "FIX";
    isFloat = T.status_str == "FLOAT";
    isOther = ~isFix & ~isFloat;

    if any(isFix)
        geoscatter(gx, T.latitude(isFix), T.longitude(isFix), 25, ...
            [0 0.6 0], 'filled', 'DisplayName', 'FIX');
    end
    if any(isFloat)
        geoscatter(gx, T.latitude(isFloat), T.longitude(isFloat), 25, ...
            [1 0.55 0], 'filled', 'DisplayName', 'FLOAT');
    end
    if any(isOther)
        geoscatter(gx, T.latitude(isOther), T.longitude(isOther), 25, ...
            [0.8 0 0], 'filled', 'DisplayName', 'その他（NONE等）');
    end

    % 開始・終了マーカー
    geoplot(gx, T.latitude(1), T.longitude(1), '^', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', ...
        'DisplayName', '走行開始');
    geoplot(gx, T.latitude(end), T.longitude(end), 's', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
        'DisplayName', '走行終了');

    % Waypoint（直線で接続。スプラインは使わない — 実際の走行経路と同じ）
    if ~isempty(wp_file)
        WP = readtable(wp_file);
        wp_lat = WP.("Latitude(deg)");
        wp_lon = WP.("Longitude(deg)");

        geoplot(gx, wp_lat, wp_lon, '-', ...
            'Color', [0 0.2 1], 'LineWidth', 2, ...
            'DisplayName', '経路（Waypoint間を直線で接続）');
        geoplot(gx, wp_lat, wp_lon, 'v', ...
            'MarkerSize', 9, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm', ...
            'DisplayName', 'Waypoint（元のCSV）');
    end

    legend(gx, 'Location', 'best');
    title(gx, log_csv, 'Interpreter', 'none');

    n_fix = sum(isFix);
    n_float = sum(isFloat);
    n_other = sum(isOther);
    fprintf('読み込み: 全%d行 → 有効%d点 (FIX=%d, FLOAT=%d, その他=%d)\n', ...
        n_total, height(T), n_fix, n_float, n_other);
end
