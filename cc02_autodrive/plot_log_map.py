import argparse
import csv
import os

import folium
import numpy as np
import pymap3d as pm
from folium.plugins import MeasureControl
from scipy.interpolate import splprep, splev

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


def _build_spline_latlon(waypoints: list, spacing: float = 0.1) -> list:
    """疎なWaypoint(lat,lon)を、stanley_controller.pyと同じ方式でスプライン化する。

    先頭WPを原点としたENU平面上でスプラインを組んでから緯度経度に戻す
    （度のまま当てはめると緯度経度のスケール差で形が歪むため）。
    """
    origin_lat, origin_lon = waypoints[0]
    pts = []
    for lat, lon in waypoints:
        e, n, _u = pm.geodetic2enu(lat, lon, 0.0, origin_lat, origin_lon, 0.0)
        pts.append((float(e), float(n)))
    pts = np.array(pts)

    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.any(np.diff(pts, axis=0) != 0.0, axis=1)
    pts = pts[keep]
    if len(pts) < 2:
        return []

    k = min(3, len(pts) - 1)
    tck, _u = splprep([pts[:, 0], pts[:, 1]], s=0, k=k)

    uu = np.linspace(0.0, 1.0, 4000)
    sx, sy = splev(uu, tck)
    seg = np.hypot(np.diff(sx), np.diff(sy))
    cumlen = np.concatenate([[0.0], np.cumsum(seg)])
    total_len = float(cumlen[-1])

    n_samples = max(2, int(round(total_len / spacing)) + 1)
    target = np.linspace(0.0, total_len, n_samples)
    u_arc = np.interp(target, cumlen, uu)
    px, py = splev(u_arc, tck)

    latlon = []
    for e, n in zip(px, py):
        lat, lon, _alt = pm.enu2geodetic(float(e), float(n), 0.0, origin_lat, origin_lon, 0.0)
        latlon.append((lat, lon))
    return latlon


def build_map(points: list, waypoints: list = None, spline_latlon: list = None) -> folium.Map:
    center_lat = sum(p['lat'] for p in points) / len(points)
    center_lon = sum(p['lon'] for p in points) / len(points)

    # max_zoomをタイルの実解像度（max_native_zoom）より大きくしておくと、
    # Leafletがその先は最後のタイルを引き伸ばして表示するため、cm単位の点の
    # 位置関係を見るためにさらにズームインできる（画像はぼやけるが点・線は鮮明）。
    # maxZoom（camelCase）はLeafletのMapオブジェクト自体に渡る生オプション。
    # folium.Mapのmax_zoom（snake_case）はtiles=None時は実質無視されるため、
    # 両方を明示的に渡してMap・TileLayer双方の上限を揃える。
    _MAX_ZOOM = 25
    fmap = folium.Map(
        location=[center_lat, center_lon], zoom_start=19, tiles=None,
        maxZoom=_MAX_ZOOM, zoomSnap=0.25, zoomDelta=0.5,
    )

    # 背景地図（航空写真2種 + 通常地図）。LayerControlでラジオボタン切り替え。
    folium.TileLayer(
        tiles='https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg',
        attr='地図・写真:国土地理院',
        name='航空写真（国土地理院）',
        max_zoom=_MAX_ZOOM, max_native_zoom=18,
        overlay=False, control=True, show=True,
    ).add_to(fmap)
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri, Maxar, Earthstar Geographics, and the GIS User Community',
        name='航空写真（Esri World Imagery）',
        max_zoom=_MAX_ZOOM, max_native_zoom=19,
        overlay=False, control=True, show=False,
    ).add_to(fmap)
    # 'OpenStreetMap'という名前文字列で渡すとfoliumがxyzservicesのプリセット
    # （max_zoom=19固定）で上書きしてしまうため、生のタイルURLで明示的に渡す。
    folium.TileLayer(
        tiles='https://tile.openstreetmap.org/{z}/{x}/{y}.png',
        attr='&copy; OpenStreetMap contributors',
        name='通常地図（OpenStreetMap）',
        max_zoom=_MAX_ZOOM, max_native_zoom=19,
        overlay=False, control=True, show=False,
    ).add_to(fmap)

    # 距離測定ツール（2点間をクリックしてm/cm単位で距離を測れる）
    MeasureControl(primary_length_unit='meters', secondary_length_unit='feet').add_to(fmap)

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

    # スプライン経路（Stanleyが実際に追従する滑らかな曲線）を青色で表示
    if spline_latlon:
        spline_group = folium.FeatureGroup(name='スプライン経路（Stanley追従曲線）', show=True)
        folium.PolyLine(
            locations=spline_latlon,
            color='blue', weight=3, opacity=0.8,
        ).add_to(spline_group)
        spline_group.add_to(fmap)

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
    parser.add_argument('--no-spline', action='store_true',
                         help='--wp-file指定時にスプライン経路（青線）を表示しない')
    args = parser.parse_args()

    points, n_total = _load_log(args.log_csv)
    if not points:
        print(f'有効な緯度経度を持つ行が見つかりませんでした（全{n_total}行）: {args.log_csv}')
        return

    waypoints = _load_waypoints(args.wp_file) if args.wp_file else None
    spline_latlon = None
    if waypoints and not args.no_spline and len(waypoints) >= 2:
        spline_latlon = _build_spline_latlon(waypoints)

    fmap = build_map(points, waypoints, spline_latlon)

    output = args.output or os.path.splitext(args.log_csv)[0] + '.html'
    fmap.save(output)

    n_fix = sum(1 for p in points if p['status_str'] == 'FIX')
    n_float = sum(1 for p in points if p['status_str'] == 'FLOAT')
    n_other = len(points) - n_fix - n_float
    print(f'読み込み: 全{n_total}行 → 有効{len(points)}点 (FIX={n_fix}, FLOAT={n_float}, その他={n_other})')
    print(f'地図HTMLを出力しました: {os.path.abspath(output)}')


if __name__ == '__main__':
    main()
