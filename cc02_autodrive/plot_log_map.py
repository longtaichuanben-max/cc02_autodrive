import argparse
import csv
import os

import folium

_STATUS_COLORS = {
    'FIX':   'green',
    'FLOAT': 'orange',
}
_DEFAULT_COLOR = 'red'  # NONE / SINGLE / SBAS / DGPS / PPP / EKF / unknown


def _load_log(log_csv: str):
    points = []
    n_total = 0
    with open(log_csv, 'r', newline='') as f:
        for row in csv.DictReader(f):
            n_total += 1
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            if lat == 0.0 and lon == 0.0:
                continue  # 無効値（プレースホルダ）はスキップ
            points.append({
                'lat': lat,
                'lon': lon,
                'status_str': row['status_str'],
                'num_sats': row['num_sats'],
                'ratio': row['ratio'],
                'speed_mps': row['speed_mps'],
                'wall_time': row['wall_time'],
            })
    return points, n_total


def _load_waypoints(wp_file: str):
    wps = []
    with open(wp_file, 'r', newline='') as f:
        for row in csv.DictReader(f):
            wps.append((float(row['Latitude(deg)']), float(row['Longitude(deg)'])))
    return wps


def build_map(points: list, waypoints: list = None) -> folium.Map:
    center_lat = sum(p['lat'] for p in points) / len(points)
    center_lon = sum(p['lon'] for p in points) / len(points)

    fmap = folium.Map(location=[center_lat, center_lon], zoom_start=19, max_zoom=22, tiles='OpenStreetMap')

    # 走行経路全体（連続線）
    track_group = folium.FeatureGroup(name='走行経路（全体）', show=True)
    folium.PolyLine(
        locations=[(p['lat'], p['lon']) for p in points],
        color='gray', weight=2, opacity=0.6,
    ).add_to(track_group)
    track_group.add_to(fmap)

    # ステータス別の点（FIX/FLOAT/その他）をレイヤーごとに分けてトグル可能にする
    status_groups = {}
    for p in points:
        status = p['status_str']
        color = _STATUS_COLORS.get(status, _DEFAULT_COLOR)
        label = status if status in _STATUS_COLORS else 'その他（NONE等）'
        if label not in status_groups:
            status_groups[label] = folium.FeatureGroup(name=f'測位ステータス: {label}', show=True)

        folium.CircleMarker(
            location=(p['lat'], p['lon']),
            radius=3,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.8,
            popup=folium.Popup(
                f"時刻: {p['wall_time']}<br>"
                f"Status: {p['status_str']}<br>"
                f"Sats: {p['num_sats']}  Ratio: {p['ratio']}<br>"
                f"速度: {p['speed_mps']} m/s",
                max_width=250,
            ),
        ).add_to(status_groups[label])

    for group in status_groups.values():
        group.add_to(fmap)

    # 開始点・終了点
    folium.Marker(
        location=(points[0]['lat'], points[0]['lon']),
        popup='走行開始',
        icon=folium.Icon(color='blue', icon='play'),
    ).add_to(fmap)
    folium.Marker(
        location=(points[-1]['lat'], points[-1]['lon']),
        popup='走行終了',
        icon=folium.Icon(color='black', icon='stop'),
    ).add_to(fmap)

    # 元のWaypoint（教授のCSV）を重ねて表示
    if waypoints:
        wp_group = folium.FeatureGroup(name='Waypoint（元のCSV）', show=True)
        for i, (lat, lon) in enumerate(waypoints):
            folium.Marker(
                location=(lat, lon),
                popup=f'WP[{i}]',
                icon=folium.Icon(color='purple', icon='flag'),
            ).add_to(wp_group)
        wp_group.add_to(fmap)

    folium.LayerControl(collapsed=False).add_to(fmap)
    return fmap


def main():
    parser = argparse.ArgumentParser(description='gnss_logger.pyが出力したログCSVを地図上にプロットするHTMLを生成する')
    parser.add_argument('log_csv', help='gnss_loggerが出力したCSVファイルのパス')
    parser.add_argument('-o', '--output', default=None,
                         help='出力HTMLファイルのパス（省略時はlog_csvと同じ場所・同じ名前で拡張子を.htmlにする）')
    parser.add_argument('--wp-file', default=None,
                         help='元のWaypoint CSV（WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)）を重ねて表示する場合に指定')
    args = parser.parse_args()

    points, n_total = _load_log(args.log_csv)
    if not points:
        print(f'有効な緯度経度を持つ行が見つかりませんでした（全{n_total}行）: {args.log_csv}')
        return

    waypoints = _load_waypoints(args.wp_file) if args.wp_file else None

    fmap = build_map(points, waypoints)

    output = args.output or os.path.splitext(args.log_csv)[0] + '.html'
    fmap.save(output)

    n_fix = sum(1 for p in points if p['status_str'] == 'FIX')
    n_float = sum(1 for p in points if p['status_str'] == 'FLOAT')
    n_other = len(points) - n_fix - n_float
    print(f'読み込み: 全{n_total}行 → 有効{len(points)}点 (FIX={n_fix}, FLOAT={n_float}, その他={n_other})')
    print(f'地図HTMLを出力しました: {os.path.abspath(output)}')


if __name__ == '__main__':
    main()
