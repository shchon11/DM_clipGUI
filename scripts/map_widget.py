#!/usr/bin/env python3
# map_widget.py — 순수 PyQt5 슬리피 맵 (OpenStreetMap 타일).
#
# QtWebEngine 없이 QGraphicsView로 구현한 인터랙티브 지도:
#   - 휠 확대/축소 (커서 기준), 드래그 이동, +/- 버튼, 더블클릭 확대, 줌 3~19 (전국 → 골목)
#   - 타일은 스레드풀에서 받아 ~/.cache/dm_clip_gui/tiles 에 캐시 (gnss_tools와 공유)
#   - 오버레이: 폴리라인(궤적), 마커(현재 위치 등). 줌이 바뀌면 자동 재투영.
#   - 바탕 지도는 기본으로 흑백 · 흐리게 — OSM 도로가 주황/노랑/빨강이라 컬러 그대로면 같은 색 계열의
#     궤적이 묻힌다. 궤적은 흰 테두리를 둘러 어떤 바탕에서도 보이게 한다. [◐] 버튼으로 원래 색.
#
# 사용:
#   m = MapWidget(); m.set_view(37.56, 126.98, 15)
#   m.add_track("clipA", [(lat, lon), ...], "#1565c0")
#   m.set_marker("me", lat, lon, "#2962ff", label="현재")
#   m.fit_bounds([(lat, lon), ...])

import math
import urllib.request
from pathlib import Path

from PyQt5.QtCore import QObject, QPointF, QRectF, QRunnable, Qt, QThreadPool, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsPathItem, QGraphicsPixmapItem, QGraphicsScene,
    QGraphicsSimpleTextItem, QGraphicsView, QLabel, QToolButton)

TILE = 256
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_CACHE = Path.home() / ".cache" / "dm_clip_gui" / "tiles"
USER_AGENT = "dm_clip_gui/0.1 (ROS2 bag GNSS viewer)"
MIN_Z, MAX_Z = 3, 19
KOREA = (36.3, 127.8, 7)          # 기본 뷰: 대한민국 전역
MUTED_OPACITY = 0.6               # 흑백 바탕의 불투명도 (밝은 배경 위라 흐려 보인다)


def world_px(lat, lon, z):
    """위경도 → 줌 z에서의 월드 픽셀 좌표 (Web Mercator)."""
    n = TILE * (2 ** z)
    lat = max(-85.05, min(85.05, lat))
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(math.radians(lat)) +
                        1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def px_geo(x, y, z):
    n = TILE * (2 ** z)
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lon


class _TileSignal(QObject):
    done = pyqtSignal(int, int, int, str)


class _TileJob(QRunnable):
    def __init__(self, z, x, y, sig):
        super().__init__()
        self.z, self.x, self.y, self.sig = z, x, y, sig
        self.setAutoDelete(True)

    def run(self):
        p = TILE_CACHE / str(self.z) / str(self.x) / f"{self.y}.png"
        try:
            if not p.exists():
                p.parent.mkdir(parents=True, exist_ok=True)
                req = urllib.request.Request(
                    TILE_URL.format(z=self.z, x=self.x, y=self.y),
                    headers={"User-Agent": USER_AGENT})
                with urllib.request.urlopen(req, timeout=8) as r:
                    data = r.read()
                p.write_bytes(data)
            self.sig.done.emit(self.z, self.x, self.y, str(p))
        except Exception:
            self.sig.done.emit(self.z, self.x, self.y, "")


class MapWidget(QGraphicsView):
    zoomChanged = pyqtSignal(int)
    clicked = pyqtSignal(float, float)          # (lat, lon) 좌클릭

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setRenderHints(self.renderHints() | 0x02 | 0x04)   # Antialiasing | SmoothPixmap
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(QColor("#f4f5f7")))
        self._muted = True          # 바탕 지도 흑백 · 흐리게 (궤적이 잘 보이게)
        self.setMinimumSize(320, 240)

        self.z = KOREA[2]
        self._tiles = {}            # (z,x,y) -> QGraphicsPixmapItem
        self._pending = set()
        self._overlays = {}         # id -> dict
        self._pool = QThreadPool(self)
        self._pool.setMaxThreadCount(6)
        self._sig = _TileSignal()
        self._sig.done.connect(self._tile_ready)
        # 아직 안 온 타일 자리는 투명 → 아래 깔린 이전 줌 타일(스케일)이 비쳐 보임
        self._placeholder = QPixmap(TILE, TILE)
        self._placeholder.fill(Qt.transparent)
        self._placeholder_key = self._placeholder.cacheKey()
        self._stale = []            # 이전 줌의 타일을 스케일해 임시 유지 (Leaflet 방식)
        self._stale_timer = QTimer(self, singleShot=True, interval=5000,
                                   timeout=self._clear_stale)
        self._tile_timer = QTimer(self, singleShot=True, interval=40,
                                  timeout=self._update_tiles)

        # 줌 버튼 + 저작권 표기 (뷰 위에 고정)
        self._btn_in = QToolButton(self)
        self._btn_in.setText("+")
        self._btn_out = QToolButton(self)
        self._btn_out.setText("−")
        for b in (self._btn_in, self._btn_out):
            b.setFixedSize(28, 28)
            b.setStyleSheet("QToolButton{background:white;border:1px solid #999;"
                            "font-weight:bold;font-size:14px;}")
        self._btn_in.clicked.connect(lambda: self.set_zoom(self.z + 1))
        self._btn_out.clicked.connect(lambda: self.set_zoom(self.z - 1))
        self._btn_mute = QToolButton(self)
        self._btn_mute.setText("◐")
        self._btn_mute.setFixedSize(28, 28)
        self._btn_mute.setStyleSheet("QToolButton{background:white;border:1px solid #999;font-size:13px;}")
        self._btn_mute.setToolTip("바탕 지도: 흑백(궤적이 잘 보임) ↔ 원래 색")
        self._btn_mute.clicked.connect(lambda: self.set_muted(not self._muted))
        self._attr = QLabel("© OpenStreetMap contributors", self)
        self._attr.setStyleSheet("background:rgba(255,255,255,190);color:#333;"
                                 "font-size:9px;padding:1px 4px;")
        self._zoom_lbl = QLabel("", self)
        self._zoom_lbl.setStyleSheet("background:rgba(255,255,255,190);color:#333;"
                                     "font-size:9px;padding:1px 4px;")
        self.set_view(*KOREA)

    # ---------- 뷰 ----------
    def set_view(self, lat, lon, z=None):
        if z is not None:
            self._set_zoom_level(z)
        self.centerOn(*world_px(lat, lon, self.z))
        self._tile_timer.start()

    def set_zoom(self, z, anchor_pos=None):
        """줌 변경. anchor_pos(뷰포트 좌표)가 있으면 그 아래 지점을 고정."""
        z = max(MIN_Z, min(MAX_Z, int(z)))
        if z == self.z:
            return
        if anchor_pos is None:
            anchor_pos = self.viewport().rect().center()
        sp = self.mapToScene(anchor_pos)
        geo = px_geo(sp.x(), sp.y(), self.z)
        self._set_zoom_level(z)
        nx, ny = world_px(geo[0], geo[1], self.z)
        vc = self.viewport().rect().center()
        self.centerOn(nx - (anchor_pos.x() - vc.x()), ny - (anchor_pos.y() - vc.y()))
        self._tile_timer.start()

    def _set_zoom_level(self, z):
        z = max(MIN_Z, min(MAX_Z, int(z)))
        dz = z - self.z
        self._clear_stale()
        if 0 < abs(dz) <= 3:
            # 로드된 이전 줌 타일은 새 줌 좌표계로 스케일해서 아래에 깔아둔다.
            # 새 타일이 도착하면 그 위에 그려지므로 확대/축소 중 회색 화면이 없다.
            f = 2.0 ** dz
            for (_, x, y), it in self._tiles.items():
                if it.pixmap().cacheKey() == self._placeholder_key:
                    self._scene.removeItem(it)
                    continue
                it.setPos(x * TILE * f, y * TILE * f)
                it.setScale(f)
                it.setZValue(-1)
                self._stale.append(it)
            self._stale_timer.start()
        else:
            for it in self._tiles.values():
                self._scene.removeItem(it)
        self._tiles.clear()
        self._pending.clear()
        self.z = z
        n = TILE * (2 ** self.z)
        self._scene.setSceneRect(QRectF(0, 0, n, n))
        for oid in list(self._overlays):
            self._draw(oid)
        self._zoom_lbl.setText(f"z{self.z}")
        self._zoom_lbl.adjustSize()
        self.zoomChanged.emit(self.z)

    def _clear_stale(self):
        for it in self._stale:
            self._scene.removeItem(it)
        self._stale = []

    def center_geo(self):
        c = self.mapToScene(self.viewport().rect().center())
        return px_geo(c.x(), c.y(), self.z)

    def fit_bounds(self, latlons, padding=0.12):
        pts = [(la, lo) for la, lo in latlons if math.isfinite(la) and math.isfinite(lo)]
        if not pts:
            return
        lat0, lat1 = min(p[0] for p in pts), max(p[0] for p in pts)
        lon0, lon1 = min(p[1] for p in pts), max(p[1] for p in pts)
        vw = max(1, self.viewport().width()) * (1 - padding)
        vh = max(1, self.viewport().height()) * (1 - padding)
        z = MAX_Z
        while z > MIN_Z:
            x0, y1 = world_px(lat0, lon0, z)
            x1, y0 = world_px(lat1, lon1, z)
            if (x1 - x0) <= vw and (y1 - y0) <= vh:
                break
            z -= 1
        self.set_view((lat0 + lat1) / 2, (lon0 + lon1) / 2, min(z, 18))

    # ---------- 이벤트 ----------
    def wheelEvent(self, ev):
        self.set_zoom(self.z + (1 if ev.angleDelta().y() > 0 else -1), ev.pos())

    def mouseDoubleClickEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.set_zoom(self.z + 1, ev.pos())

    def mouseReleaseEvent(self, ev):
        super().mouseReleaseEvent(ev)
        if ev.button() == Qt.LeftButton:
            sp = self.mapToScene(ev.pos())
            self.clicked.emit(*px_geo(sp.x(), sp.y(), self.z))

    def keyPressEvent(self, ev):
        if ev.key() in (Qt.Key_Plus, Qt.Key_Equal):
            self.set_zoom(self.z + 1)
        elif ev.key() == Qt.Key_Minus:
            self.set_zoom(self.z - 1)
        else:
            super().keyPressEvent(ev)

    def scrollContentsBy(self, dx, dy):
        super().scrollContentsBy(dx, dy)
        self._tile_timer.start()

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        w, h = self.viewport().width(), self.viewport().height()
        self._btn_in.move(w - 36, 8)
        self._btn_out.move(w - 36, 38)
        self._btn_mute.move(w - 36, 72)
        self._attr.adjustSize()
        self._attr.move(4, h - self._attr.height() - 4)
        self._zoom_lbl.move(w - self._zoom_lbl.width() - 8, h - 18)
        self._tile_timer.start()

    # ---------- 타일 ----------
    def _update_tiles(self):
        n_tiles = 2 ** self.z
        r = self.mapToScene(self.viewport().rect()).boundingRect()
        x0 = max(0, int(r.left() // TILE) - 1)
        x1 = min(n_tiles - 1, int(r.right() // TILE) + 1)
        y0 = max(0, int(r.top() // TILE) - 1)
        y1 = min(n_tiles - 1, int(r.bottom() // TILE) + 1)
        want = {(self.z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)}
        # 화면에서 멀어진 타일 정리
        for key in [k for k in self._tiles if k not in want and
                    (abs(k[1] - (x0 + x1) / 2) > (x1 - x0 + 6) or
                     abs(k[2] - (y0 + y1) / 2) > (y1 - y0 + 6))]:
            self._scene.removeItem(self._tiles.pop(key))
        for key in want:
            if key in self._tiles:
                continue
            z, x, y = key
            item = QGraphicsPixmapItem(self._placeholder)
            item.setTransformationMode(Qt.SmoothTransformation)
            item.setOpacity(MUTED_OPACITY if self._muted else 1.0)
            item.setPos(x * TILE, y * TILE)
            item.setZValue(0)
            self._scene.addItem(item)
            self._tiles[key] = item
            cached = TILE_CACHE / str(z) / str(x) / f"{y}.png"
            if cached.exists():
                self._set_tile(item, str(cached))
            elif key not in self._pending:
                self._pending.add(key)
                self._pool.start(_TileJob(z, x, y, self._sig))
        if not self._pending:
            self._clear_stale()

    def _tile_ready(self, z, x, y, path):
        self._pending.discard((z, x, y))
        item = self._tiles.get((z, x, y))
        if item is not None and path and z == self.z:
            self._set_tile(item, path)
        if not self._pending:
            self._clear_stale()

    def _set_tile(self, item, path):
        item.setData(0, path)
        if self._muted:
            gray = QImage(path).convertToFormat(QImage.Format_Grayscale8)
            item.setPixmap(QPixmap.fromImage(gray))
        else:
            item.setPixmap(QPixmap(path))
        item.setOpacity(MUTED_OPACITY if self._muted else 1.0)

    def set_muted(self, muted):
        """바탕 지도를 흑백 · 흐리게(True) / 원래 색(False). 이미 받은 타일도 바로 바꾼다."""
        self._muted = muted
        for item in list(self._tiles.values()) + list(self._stale):
            path = item.data(0) if hasattr(item, "data") else None
            if path:
                self._set_tile(item, path)
            elif hasattr(item, "setOpacity"):
                item.setOpacity(MUTED_OPACITY if muted else 1.0)

    # ---------- 오버레이 ----------
    def add_track(self, oid, latlons, color="#1565c0", width=4.0, z=10):
        self._overlays[oid] = {"kind": "track", "pts": list(latlons), "color": color,
                               "width": width, "z": z, "items": []}
        self._draw(oid)

    def set_marker(self, oid, lat, lon, color="#2962ff", radius=6, label=None, z=20):
        self._overlays[oid] = {"kind": "marker", "lat": lat, "lon": lon, "color": color,
                               "radius": radius, "label": label, "z": z, "items": []}
        self._draw(oid)

    def remove(self, oid):
        ov = self._overlays.pop(oid, None)
        if ov:
            for it in ov["items"]:
                self._scene.removeItem(it)

    def clear_overlays(self):
        for oid in list(self._overlays):
            self.remove(oid)

    def has(self, oid):
        return oid in self._overlays

    def _draw(self, oid):
        ov = self._overlays[oid]
        for it in ov["items"]:
            self._scene.removeItem(it)
        ov["items"] = []
        if ov["kind"] == "track":
            pts = [world_px(la, lo, self.z) for la, lo in ov["pts"]
                   if math.isfinite(la) and math.isfinite(lo)]
            if len(pts) < 2:
                return
            path = QPainterPath(QPointF(*pts[0]))
            for x, y in pts[1:]:
                path.lineTo(x, y)
            # 흰 테두리(아래) + 색 선(위) — 어떤 바탕에서도 선이 떠 보인다
            for color, width, dz in ((QColor(255, 255, 255, 235), ov["width"] + 3.5, -0.5),
                                     (QColor(ov["color"]), ov["width"], 0.0)):
                item = QGraphicsPathItem(path)
                pen = QPen(color, width)
                pen.setCosmetic(True)
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
                item.setPen(pen)
                item.setZValue(ov["z"] + dz)
                self._scene.addItem(item)
                ov["items"].append(item)
        else:
            x, y = world_px(ov["lat"], ov["lon"], self.z)
            r = ov["radius"]
            dot = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
            dot.setBrush(QBrush(QColor(ov["color"])))
            dot.setPen(QPen(QColor("white"), 2))
            dot.setPos(x, y)
            dot.setZValue(ov["z"])
            dot.setFlag(QGraphicsEllipseItem.ItemIgnoresTransformations, True)
            self._scene.addItem(dot)
            ov["items"].append(dot)
            if ov["label"]:
                txt = QGraphicsSimpleTextItem(ov["label"])
                txt.setFont(QFont("Sans", 9, QFont.Bold))
                txt.setBrush(QBrush(QColor(ov["color"])))
                txt.setPos(x, y)
                txt.setZValue(ov["z"])
                txt.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations, True)
                # 점 오른쪽 위로 살짝 띄움 (뷰 좌표 기준이라 transform 무시 아이템에 오프셋 적용)
                txt.setPos(x, y)
                txt.moveBy(0, 0)
                self._scene.addItem(txt)
                ov["items"].append(txt)
