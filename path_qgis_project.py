#!/usr/bin/env python3
"""경로 GeoPackage 를 위성지도 배경과 함께 열리는 QGIS 프로젝트로 묶는다.

path_to_qgis.py 가 만든 .gpkg 를 받아 같은 이름의 .qgz 를 만든다. 그 파일을
열면 배경지도, 경로 선, 곡률로 색칠한 점이 한 번에 올라온다. 지도를 따로
추가하거나 좌표계를 맞출 필요가 없다.

배경 타일은 인터넷에서 받아오므로 LTE 나 WiFi 가 연결돼 있어야 보인다.

쓰는 법:
    python3 path_qgis_project.py gps_recordings/path5.gpkg
    python3 path_qgis_project.py gps_recordings/path5.gpkg --open
"""

import argparse
import os
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from qgis.core import (
    QgsApplication, QgsCategorizedSymbolRenderer, QgsCoordinateReferenceSystem,
    QgsCoordinateTransform, QgsLineSymbol, QgsMarkerSymbol, QgsProject,
    QgsPalLayerSettings, QgsRasterLayer, QgsReferencedRectangle,
    QgsRendererCategory, QgsTextBufferSettings, QgsTextFormat, QgsVectorLayer,
    QgsVectorLayerSimpleLabeling,
)
from qgis.PyQt.QtGui import QColor

# 곡률 판정별 색. path_to_qgis.py 의 verdict 값과 짝을 이룬다.
VERDICT_STYLE = [
    ('못 돔', '#d62728', 3.4),
    ('한계 근접', '#ff7f0e', 3.0),
    ('LAD보다 급함', '#fdd835', 2.6),
    ('여유', '#2ca02c', 2.0),
    ('직선', '#9e9e9e', 1.6),
]

# 배경지도 후보. 첫 줄이 위성, 둘째가 일반 지도다. 위성이 막히면 OSM 이 뜬다.
BASEMAPS = [
    ('위성지도',
     'type=xyz&url=https://mt1.google.com/vt/lyrs%3Ds%26x%3D%7Bx%7D%26y%3D%7By%7D'
     '%26z%3D%7Bz%7D&zmax=20&zmin=0'),
    ('OpenStreetMap',
     'type=xyz&url=https://tile.openstreetmap.org/%7Bz%7D/%7Bx%7D/%7By%7D.png'
     '&zmax=19&zmin=0'),
]


def styled_points(layer):
    """verdict 값마다 다른 색을 준다. 문제 지점이 지도에서 바로 튄다."""
    categories = []
    for value, colour, size in VERDICT_STYLE:
        symbol = QgsMarkerSymbol.createSimple({
            'name': 'circle', 'color': colour, 'size': str(size),
            'outline_color': '#ffffff', 'outline_width': '0.3',
        })
        categories.append(QgsRendererCategory(value, symbol, value))
    layer.setRenderer(QgsCategorizedSymbolRenderer('verdict', categories))

    labels = QgsPalLayerSettings()
    labels.fieldName = 'idx'
    labels.placement = QgsPalLayerSettings.AroundPoint
    text = QgsTextFormat()
    text.setSize(10)
    text.setColor(QColor('#111111'))
    buffer = QgsTextBufferSettings()
    buffer.setEnabled(True)
    buffer.setSize(1.2)
    buffer.setColor(QColor('#ffffff'))
    text.setBuffer(buffer)
    labels.setFormat(text)
    layer.setLabeling(QgsVectorLayerSimpleLabeling(labels))
    layer.setLabelsEnabled(True)


def build(gpkg_path, open_after):
    if not os.path.isfile(gpkg_path):
        raise SystemExit('파일이 없다: %s' % gpkg_path)

    QgsApplication.setPrefixPath('/usr', True)
    app = QgsApplication([], False)
    app.initQgis()

    project = QgsProject.instance()
    project.setCrs(QgsCoordinateReferenceSystem('EPSG:3857'))

    added = []
    for name, uri in BASEMAPS:
        basemap = QgsRasterLayer(uri, name, 'wms')
        if basemap.isValid():
            project.addMapLayer(basemap)
            added.append(name)

    track = QgsVectorLayer('%s|layername=track' % gpkg_path, '경로', 'ogr')
    if track.isValid():
        track.setRenderer(track.renderer().clone())
        track.renderer().setSymbol(QgsLineSymbol.createSimple(
            {'color': '#1f77b4', 'width': '0.9'}))
        project.addMapLayer(track)

    points = QgsVectorLayer('%s|layername=points' % gpkg_path, '점 (곡률)', 'ogr')
    if not points.isValid():
        raise SystemExit('points 레이어를 못 읽었다: %s' % gpkg_path)
    styled_points(points)
    project.addMapLayer(points)

    # 경로가 화면에 꽉 차도록 저장 범위를 잡아 둔다.
    extent = points.extent()
    extent.grow(max(extent.width(), extent.height()) * 0.15 or 0.0005)
    transform = QgsCoordinateTransform(
        points.crs(), project.crs(), project.transformContext())
    view = project.viewSettings()
    view.setDefaultViewExtent(QgsReferencedRectangle(
        transform.transformBoundingBox(extent), project.crs()))

    out_path = os.path.splitext(gpkg_path)[0] + '.qgz'
    project.write(out_path)
    app.exitQgis()

    print('%s  ->  %s' % (gpkg_path, out_path))
    print('  배경지도: %s' % (', '.join(added) if added else '실패 (인터넷 확인)'))
    print('  레이어  : 점 (곡률), 경로, 배경')
    print()
    if open_after:
        os.execvp('qgis', ['qgis', out_path])
    print('열기:  qgis %s' % out_path)
    print('또는 파일 관리자에서 더블클릭하면 된다.')
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('gpkg_file', help='path_to_qgis.py 가 만든 .gpkg')
    parser.add_argument('--open', action='store_true', help='만든 뒤 QGIS 로 연다')
    arguments = parser.parse_args()
    build(arguments.gpkg_file, arguments.open)


if __name__ == '__main__':
    main()
