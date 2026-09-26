"""Bounded OpenGL scenes. All coordinates are metres in explicitly named frames."""
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl


CYAN = (0.24, 0.88, 0.84, 1.0)
AMBER = (1.0, 0.70, 0.32, 1.0)
VIOLET = (0.63, 0.60, 1.0, 1.0)


def state_color(camera):
    if camera.get('state') == 'failed' or camera.get('gate_pass') is False:
        return (1.0, 0.40, 0.43, 1.0)
    if camera.get('gate_pass') is not True:
        return (0.52, 0.59, 0.68, 1.0)
    return CYAN


def line(view, points, color, width=1, mode='lines'):
    item = gl.GLLinePlotItem(pos=np.asarray(points, dtype=np.float32), color=color,
                             width=width, mode=mode, antialias=True)
    item.setGLOptions('translucent')
    view.addItem(item)
    return item


def box(view, low, high, color):
    vertices = np.array([[x, y, z] for x in [low[0], high[0]]
                         for y in [low[1], high[1]] for z in [low[2], high[2]]])
    edges = [(i, j) for i in range(8) for j in range(i + 1, 8)
             if bin(i ^ j).count('1') == 1]
    line(view, vertices[np.array(edges).ravel()], color)
    faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                      [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                      [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    mesh = gl.GLMeshItem(vertexes=vertices, faces=faces, smooth=False,
                         color=(*color[:3], 0.10), shader='shaded')
    mesh.setGLOptions('translucent')
    view.addItem(mesh)


class RigScene(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundColor('#0c1524')
        self.setMinimumSize(350, 250)
        self.reset_view()
        grid = gl.GLGridItem()
        grid.setSize(12, 12)
        grid.setSpacing(0.5, 0.5)
        grid.setColor((37, 57, 77, 125))
        grid.translate(0, 0, -1.85)
        self.addItem(grid)
        # A subdued scale reference, not measured vehicle geometry.
        box(self, (-1.85, -0.85, -1.60), (2.0, 0.85, -1.1), (0.32, 0.45, 0.59, 0.4))
        box(self, (-1.03, -0.77, -1.1), (0.95, 0.77, -0.63), (0.40, 0.56, 0.68, 0.4))
        for x in [-1.30, 1.35]:
            for y in [-0.89, 0.89]:
                box(self, (x - .28, y - .10, -1.8),
                    (x + .28, y + .10, -1.35), (0.23, 0.34, 0.45, 0.45))
        for z, radius in [(-.11, .10), (-.055, .115), (.015, .10)]:
            mesh = gl.GLMeshItem(meshdata=gl.MeshData.cylinder(2, 32, [radius, radius], .05),
                                 color=(.24, .82, .80, .85), shader='shaded', smooth=True)
            mesh.translate(0, 0, z)
            self.addItem(mesh)
        ring = np.linspace(0, 2 * np.pi, 100)
        line(self, np.column_stack((.22 * np.cos(ring), .22 * np.sin(ring), ring * 0)),
             (*CYAN[:3], .4), mode='line_strip')
        line(self, [[1.2, 0, -.60], [1.75, 0, -.60], [1.60, .09, -.60],
                    [1.75, 0, -.60], [1.60, -.09, -.60]], CYAN, 2, 'line_strip')
        axes = [([.65, 0, .03], (1, .40, .43, 1)),
                ([0, .65, .03], (.40, .85, .65, 1)),
                ([0, 0, .65], (.4, .6, 1, 1))]
        for end, color in axes:
            line(self, [[0, 0, .03], end], color, 2)
        self.frustums = line(self, np.zeros((2, 3)), CYAN, 1.6)
        self.axes = line(self, np.zeros((2, 3)), CYAN, 1.5)
        self.ghosts = line(self, np.zeros((2, 3)), (*CYAN[:3], .2), 1)
        self.centers = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=8, color=CYAN, pxMode=True)
        self.addItem(self.centers)
        self.label_position = np.zeros(3)
        self.label_text = ''
        self.overlay = RigLabels(self)
        self.history = {}
        self.last_seq = None
        self.show_ghosts = True

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'overlay'):
            self.overlay.setGeometry(self.rect())

    def reset_view(self):
        self.setCameraPosition(pos=pg.Vector(0, 0, -.62), distance=6.7, elevation=28, azimuth=-53)

    def update_poses(self, poses, cameras, selected, seq):
        vertices, colors, centers, center_colors, axes, axis_colors = [], [], [], [], [], []
        ghosts, ghost_colors = [], []
        for name, pose in poses.items():
            origin, rotation = pose[:3, 3], pose[:3, :3]
            chosen = name == selected
            camera = cameras.get(name, {})
            failed = camera.get('gate_pass') is False or camera.get('state') == 'failed'
            color = VIOLET if chosen and not failed else state_color(camera)
            length = .36 if chosen else .23
            corners = np.array([[-.63, -.43, 1], [.63, -.43, 1],
                                [.63, .43, 1], [-.63, .43, 1]]) * length
            world = corners @ rotation.T + origin
            for i in range(4):
                vertices.extend([origin, world[i], world[i], world[(i + 1) % 4]])
                colors.extend([color] * 4)
            centers.append(origin)
            center_colors.append(color)
            for i, c in enumerate([(1, .35, .4, .8), (.4, .9, .6, .8), (.35, .6, 1, .8)]):
                axes.extend([origin, origin + rotation[:, i] * .10])
                axis_colors.extend([c, c])
            if seq != self.last_seq:
                history = self.history.setdefault(name, deque(maxlen=14))
                history.append((origin.copy(), world.copy()))
            if self.show_ghosts:
                hist = self.history.get(name, [])
                for j, (old_origin, old_world) in enumerate(hist):
                    alpha = .03 + .14 * (j + 1) / max(len(hist), 1)
                    for k in range(4):
                        ghosts.extend([old_origin, old_world[k]])
                        ghost_colors.extend([(*color[:3], alpha)] * 2)
            if chosen:
                self.label_position = origin + [0, 0, .27]
                self.label_text = name
        if vertices:
            self.frustums.setData(pos=np.asarray(vertices, np.float32), color=np.asarray(colors, np.float32))
            self.centers.setData(pos=np.asarray(centers, np.float32), color=np.asarray(center_colors, np.float32))
            self.axes.setData(pos=np.asarray(axes, np.float32), color=np.asarray(axis_colors, np.float32))
        self.ghosts.setData(pos=np.asarray(ghosts, np.float32).reshape(-1, 3),
                            color=np.asarray(ghost_colors, np.float32).reshape(-1, 4))
        self.last_seq = seq
        self.overlay.update()


class RigLabels(QtWidgets.QWidget):
    """Composite labels above GL, including when QWidget.grab() is used."""
    def __init__(self, scene):
        super().__init__(scene)
        self.scene = scene
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground)

    def paintEvent(self, event):
        scene = self.scene
        transform = scene.projectionMatrix() * scene.viewMatrix()
        p = QtGui.QPainter(self)
        p.setRenderHint(p.Antialiasing)
        p.setFont(QtGui.QFont('Noto Sans CJK KR', 9))
        labels = [(np.array([1.85, 0, -.57]), '전방 +X', '#49dfd6'),
                  (np.array([0, .8, .03]), '좌 +Y', '#71dfa7'),
                  (np.array([0, 0, .76]), '상 +Z', '#77a5ff'),
                  (np.array([0, 0, .10]), 'Ouster', '#49dfd6')]
        if scene.label_text:
            labels.append((scene.label_position, scene.label_text, '#c6bfff'))
        for position, text, color in labels:
            clip = transform * QtGui.QVector4D(*position, 1.)
            if clip.w() <= 0:
                continue
            x = (clip.x()/clip.w() + 1) * self.width()/2 + 6
            y = (1 - clip.y()/clip.w()) * self.height()/2
            width = p.fontMetrics().horizontalAdvance(text) + 12
            x = float(np.clip(x, 3, max(3, self.width()-width-3)))
            y = float(np.clip(y, 19, max(19, self.height()-4)))
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor(10, 20, 35, 220))
            p.drawRoundedRect(QtCore.QRectF(x-4, y-15, width, 21), 4, 4)
            p.setPen(QtGui.QColor(color))
            p.drawText(QtCore.QPointF(x+2, y), text)


class MapScene(gl.GLViewWidget):
    def __init__(self, parent=None, max_points=70000):
        super().__init__(parent)
        self.setBackgroundColor('#0c1524')
        self.setMinimumSize(280, 180)
        self.max_points = max_points
        self.setCameraPosition(distance=45, elevation=52, azimuth=-65)
        self.cloud = gl.GLScatterPlotItem(pos=np.zeros((0, 3)), size=1.6, color=CYAN, pxMode=True)
        self.cloud.setGLOptions('translucent')
        self.addItem(self.cloud)
        self.trajectory = line(self, np.zeros((0, 3)), (1, .75, .35, 1), 2.5, 'line_strip')
        self.head = gl.GLScatterPlotItem(pos=np.zeros((0, 3)), size=9, color=(1, .85, .5, 1))
        self.addItem(self.head)
        self.frame_id = None
        self.point_count = 0
        self.auto_fit = True

    def mousePressEvent(self, event):
        self.auto_fit = False
        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        self.auto_fit = True
        super().mouseDoubleClickEvent(event)

    def set_map(self, data, frame_id):
        points = np.asarray(data.get('points', np.empty((0, 3))), dtype=np.float32)
        points = points[::max(1, int(np.ceil(len(points) / self.max_points)))][:self.max_points]
        points = points[np.isfinite(points).all(axis=1)]
        colors = np.ones((len(points), 4), np.float32)
        if len(points):
            height = np.clip((points[:, 2] + 2) / 7, 0, 1)
            colors[:, 0] = .15 + .44 * height
            colors[:, 1] = .55 + .24 * (1 - height)
            colors[:, 2] = .64 + .32 * height
            colors[:, 3] = .64
        self.cloud.setData(pos=points, color=colors)
        trajectory = np.asarray(data.get('trajectory', np.empty((0, 3))), dtype=np.float32)
        self.trajectory.setData(pos=trajectory)
        self.head.setData(pos=trajectory[-1:] if len(trajectory) else np.empty((0, 3)))
        if (self.auto_fit or self.frame_id != frame_id) and len(points):
            center = (np.percentile(points, 3, axis=0) + np.percentile(points, 97, axis=0)) / 2
            aspect = self.width() / max(self.height(), 1)
            distance = float(np.clip(np.percentile(np.linalg.norm(points-center, axis=1), 94)
                                     * max(2.4, aspect * 2.0), 15, 500))
            self.setCameraPosition(pos=pg.Vector(*center), distance=distance)
        self.frame_id = frame_id
        self.point_count = len(points)
