#!/usr/bin/env python3
# ui_theme.py — DM Clip GUI 공통 테마 (라이트).
#
# Fusion 스타일 + 명시적 라이트 팔레트 + 앱 전역 QSS. 팔레트를 명시하는 이유: 데스크톱이
# 다크 테마면 Qt 기본 팔레트가 어둡게 잡혀 QSS로 칠한 곳과 안 칠한 곳이 섞여 보인다.
#
# 버튼 변형은 동적 프로퍼티로 고른다 (set_variant):
#   "primary"  파란 채움 — 화면의 주 동작 하나
#   "success"  초록 채움 — 끝났으니 다음 단계로
#   "danger"   빨간 테두리 — 중지/삭제

from pathlib import Path

from PyQt5.QtGui import QColor, QGuiApplication, QPalette

ACCENT = "#2563eb"
OK = "#16a34a"
WARN = "#d97706"
ERR = "#dc2626"
MUTED = "#6b7280"
TEXT = "#111827"
BORDER = "#e3e5e8"
BG = "#f5f6f8"

# 실행 상태 알약: (글자, 배경)
PILL = {
    "stopped": ("#4b5563", "#eef0f3"),
    "starting": ("#92400e", "#fef3c7"),
    "running": ("#166534", "#dcfce7"),
    "failed": ("#991b1b", "#fee2e2"),
}

# 콤보/스핀 화살표. QSS 로 테두리를 바꾸면 Fusion 기본 화살표가 깨져 그리므로 직접 준다.
_ARROW_SVG = ('<svg xmlns="http://www.w3.org/2000/svg" width="10" height="10" viewBox="0 0 10 10">'
              '<path d="{d}" fill="none" stroke="#6b7280" stroke-width="1.6" '
              'stroke-linecap="round" stroke-linejoin="round"/></svg>')
_ARROW_DIR = Path.home() / ".cache" / "dm_clip_gui" / "theme"


def _arrow_files():
    _ARROW_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, d in (("down", "M2 3.5 L5 6.5 L8 3.5"), ("up", "M2 6.5 L5 3.5 L8 6.5")):
        path = _ARROW_DIR / f"{name}.svg"
        path.write_text(_ARROW_SVG.format(d=d), encoding="utf-8")
        out[name] = path.as_posix()
    return out


# 칸 경계: 가운데 2px 선만 보이게 (그라디언트 트릭). 올리면 파랗게 — 끌 수 있다는 게 보여야 한다.
def _handle(axis, color):
    x2, y2 = ("1", "0") if axis == "h" else ("0", "1")
    return (f"qlineargradient(x1:0, y1:0, x2:{x2}, y2:{y2}, stop:0 transparent, "
            f"stop:0.36 transparent, stop:0.37 {color}, stop:0.63 {color}, "
            f"stop:0.64 transparent, stop:1 transparent)")


def _qss(arrow_down, arrow_up):
    return f"""
QMainWindow, QDialog {{ background: {BG}; }}
/* 글자색은 팔레트(apply)가 정한다. 여기서 QWidget {{ color }} 를 주면 Qt 5.15 스타일시트가
   입력칸의 안내문(placeholder) 색까지 본문색으로 덮어서, 비어 있는 칸이 값이 든 것처럼 보인다. */

QTabWidget::pane {{ border: none; background: {BG}; }}
QTabBar::tab {{
    background: transparent; border: none; border-bottom: 2px solid transparent;
    padding: 8px 18px; margin-right: 2px; color: {MUTED}; font-weight: 600;
}}
QTabBar::tab:selected {{ color: {TEXT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color: {TEXT}; }}

QGroupBox {{
    background: #ffffff; border: 1px solid {BORDER}; border-radius: 8px;
    margin-top: 16px; padding: 12px 8px 8px 8px; font-weight: 600;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #374151; }}

QPushButton {{
    background: #ffffff; border: 1px solid #d1d5db; border-radius: 6px;
    padding: 5px 12px; min-height: 18px;
}}
QPushButton:hover {{ background: #f3f4f6; }}
QPushButton:pressed {{ background: #e5e7eb; }}
QPushButton:disabled {{ color: #9ca3af; background: #f9fafb; border-color: {BORDER}; }}
QPushButton[variant="primary"] {{
    background: {ACCENT}; border-color: {ACCENT}; color: #ffffff; font-weight: 600;
}}
QPushButton[variant="primary"]:hover {{ background: #1d4ed8; }}
QPushButton[variant="primary"]:disabled {{ background: #93c5fd; border-color: #93c5fd; color: #eff6ff; }}
QPushButton[variant="success"] {{
    background: {OK}; border-color: {OK}; color: #ffffff; font-weight: 600;
}}
QPushButton[variant="success"]:hover {{ background: #15803d; }}
QPushButton[variant="danger"] {{ color: #b91c1c; border-color: #fca5a5; }}
QPushButton[variant="danger"]:hover {{ background: #fef2f2; }}
QPushButton[variant="danger"]:disabled {{ color: #d1d5db; border-color: {BORDER}; }}
QToolButton {{ border: none; border-radius: 4px; padding: 2px 4px; color: {MUTED}; }}
QToolButton:hover {{ background: #eef0f3; color: {TEXT}; }}

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background: #ffffff; border: 1px solid #d1d5db; border-radius: 5px;
    padding: 3px 6px; min-height: 20px;
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {ACCENT}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
    background: #f3f4f6; color: #9ca3af;
}}

QTableWidget, QListWidget {{
    background: #ffffff; border: 1px solid {BORDER}; border-radius: 6px;
    gridline-color: #f0f1f3; alternate-background-color: #fafbfc;
    selection-background-color: #dbeafe; selection-color: {TEXT};
}}
QHeaderView::section {{
    background: #f9fafb; border: none; border-bottom: 1px solid {BORDER};
    padding: 5px 6px; color: {MUTED}; font-weight: 600;
}}
QTextEdit, QPlainTextEdit {{ background: #ffffff; border: 1px solid {BORDER}; border-radius: 6px; }}

QProgressBar {{
    border: 1px solid {BORDER}; border-radius: 5px; background: #f3f4f6;
    text-align: center; min-height: 16px;
}}
QProgressBar::chunk {{ background: #3b82f6; border-radius: 4px; }}

QComboBox {{ padding-right: 24px; }}
QComboBox::drop-down {{
    subcontrol-origin: padding; subcontrol-position: center right; width: 22px; border: none;
}}
QComboBox::down-arrow {{ image: url({arrow_down}); width: 10px; height: 10px; }}
QComboBox QAbstractItemView {{
    background: #ffffff; border: 1px solid {BORDER}; outline: 0;
    selection-background-color: #dbeafe; selection-color: {TEXT};
}}
QAbstractSpinBox {{ padding-right: 20px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{
    subcontrol-origin: border; width: 18px; border: none; background: transparent;
}}
QAbstractSpinBox::up-button {{ subcontrol-position: top right; }}
QAbstractSpinBox::down-button {{ subcontrol-position: bottom right; }}
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: #eef0f3; }}
QAbstractSpinBox::up-arrow {{ image: url({arrow_up}); width: 8px; height: 8px; }}
QAbstractSpinBox::down-arrow {{ image: url({arrow_down}); width: 8px; height: 8px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle {{ background: #cfd4da; border-radius: 3px; }}
QScrollBar::handle:vertical {{ min-height: 28px; }}
QScrollBar::handle:horizontal {{ min-width: 28px; }}
QScrollBar::handle:hover {{ background: #9ca3af; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; border: none; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QSplitter::handle:horizontal {{ width: 9px; background: {_handle("h", "#d6dade")}; }}
QSplitter::handle:vertical {{ height: 9px; background: {_handle("v", "#d6dade")}; }}
QSplitter::handle:horizontal:hover {{ background: {_handle("h", "#60a5fa")}; }}
QSplitter::handle:vertical:hover {{ background: {_handle("v", "#60a5fa")}; }}

QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QToolTip {{ background: #111827; color: #f9fafb; border: none; padding: 5px 7px; }}

/* ---- 센서 기동 탭 ---- */
QFrame#Toolbar, QFrame#Panel {{
    background: #ffffff; border: 1px solid {BORDER}; border-radius: 10px;
}}
QFrame#SensorCard {{
    background: #ffffff; border: 1px solid {BORDER}; border-radius: 10px;
}}
QFrame#SensorCard:hover {{ border-color: #93c5fd; }}
QFrame#SensorCard[selected="true"] {{ border: 2px solid {ACCENT}; background: #f5f9ff; }}
QFrame#SensorCard[disabled="true"] {{ background: #f9fafb; }}
/* 실행 상태 띠 — [selected] 뒤에 와야 선택된 카드에서도 왼쪽 띠가 보인다 */
QFrame#SensorCard[run="running"] {{ border-left: 6px solid {OK}; }}
QFrame#SensorCard[run="starting"] {{ border-left: 6px solid {WARN}; }}
QFrame#SensorCard[run="failed"] {{ border-left: 6px solid {ERR}; }}
QLabel#RunDot {{ color: #d1d5db; font-size: 12pt; }}
QLabel#RunDot[run="running"] {{ color: {OK}; }}
QLabel#RunDot[run="starting"] {{ color: {WARN}; }}
QLabel#RunDot[run="starting"][dim="true"] {{ color: #fde68a; }}
QLabel#RunDot[run="failed"] {{ color: {ERR}; }}
QLabel#CardTitle {{ font-size: 11pt; font-weight: 700; }}
QLabel#CardCount {{ font-size: 22pt; font-weight: 700; }}
QLabel#CardCount[zero="true"] {{ color: #b6bbc3; }}
QLabel#CardSub, QLabel#PageSub, QLabel#Hint {{ color: {MUTED}; }}
QLabel#CardSub[warn="true"] {{ color: {WARN}; }}
QLabel#PageTitle {{ font-size: 15pt; font-weight: 700; }}
QLabel#Section {{ font-weight: 700; color: #374151; }}
QLabel#Pill {{ border-radius: 9px; padding: 2px 10px; font-weight: 600; }}
QLabel#Changed {{ color: {ACCENT}; font-weight: 700; }}
QTabWidget#Inner::pane {{
    background: #ffffff; border: 1px solid {BORDER}; border-radius: 10px; top: -1px;
}}
QTabWidget#Inner > QTabBar::tab {{ padding: 7px 14px; }}
/* ---- 센서 미리보기 ---- */
QFrame#Tile {{ background: #ffffff; border: 1px solid {BORDER}; border-radius: 8px; }}
QFrame#Tile[dropTarget="true"] {{ border: 2px dashed {ACCENT}; background: #f5f9ff; }}
QLabel#TileName {{ font-weight: 700; }}
QLabel#TileImage {{ background: #0f172a; border-radius: 5px; color: #64748b; }}
QLabel#PillSmall {{ border-radius: 7px; padding: 0px 6px; font-size: 8pt; font-weight: 600; }}
QFrame#Banner {{ background: #ecfdf5; border: 1px solid #a7f3d0; border-radius: 10px; }}
QFrame#Warn {{ background: #fffbeb; border: 1px solid #fde68a; border-radius: 8px; }}
QPlainTextEdit#Log, QTextEdit#Log {{
    background: #0f172a; color: #e2e8f0; border: none; border-radius: 6px;
    selection-background-color: #334155;
}}

/* ---- 녹화 탭: 현재 상태 줄 (kind 마다 바탕색) ---- */
QLabel#StateBar {{
    border-radius: 8px; padding: 6px 12px; font-size: 10.5pt; font-weight: 700;
    background: #f1f5f9; color: #334155; border: 1px solid #cbd5e1;
}}
QLabel#StateBar[kind="clip"] {{ background: #fef3c7; color: #92400e; border-color: #f59e0b; }}
QLabel#StateBar[kind="write"] {{ background: #dbeafe; color: #1e3a8a; border-color: #60a5fa; }}
QLabel#StateBar[kind="diag"] {{ background: #ede9fe; color: #4c1d95; border-color: #a78bfa; }}
QLabel#StateBar[kind="rec"] {{ background: #fee2e2; color: #991b1b; border-color: #f87171; }}

/* ---- 설정 탭 ---- */
QFrame#SettingsBox {{ background: #ffffff; border: 1px solid {BORDER}; border-radius: 8px; }}
QFrame#SettingsBox[primary="true"] {{ border-color: #bfdbfe; }}
QPushButton#SectionHead {{
    font-weight: 700; color: #374151; padding: 4px 4px; text-align: left; border: none;
    border-radius: 5px; background: transparent;
}}
QPushButton#SectionHead:hover {{ background: #f3f4f6; color: {TEXT}; }}
QPushButton#SectionHead:pressed {{ background: #e5e7eb; }}
QLabel#SubHead {{
    color: {MUTED}; font-weight: 600; font-size: 8pt; padding-top: 6px;
    border-bottom: 1px solid #f0f1f3;
}}
QLabel#KeyLabel {{ font-family: "DejaVu Sans Mono", "Monospace"; font-size: 8.5pt; }}
QLabel#KeyLabel[unset="true"], QLabel#FieldLabel[unset="true"] {{ color: #9ca3af; }}
QLabel#FieldLabel[caution="true"] {{ color: #b45309; }}
QLineEdit[invalid="true"], QComboBox[invalid="true"] {{ border-color: {ERR}; background: #fef2f2; }}
QLabel#BigTitle {{ font-size: 10.5pt; font-weight: 700; color: {TEXT}; }}
"""


def apply(app):
    """QApplication 에 테마를 건다. main() 에서 창을 만들기 전에 한 번."""
    app.setStyle("Fusion")
    palette = QPalette()
    for role, color in (
            (QPalette.Window, BG), (QPalette.WindowText, TEXT),
            (QPalette.Base, "#ffffff"), (QPalette.AlternateBase, "#fafbfc"),
            (QPalette.Text, TEXT), (QPalette.Button, "#ffffff"),
            (QPalette.ButtonText, TEXT), (QPalette.Highlight, "#dbeafe"),
            (QPalette.HighlightedText, TEXT), (QPalette.ToolTipBase, "#111827"),
            (QPalette.ToolTipText, "#f9fafb"), (QPalette.PlaceholderText, "#9ca3af")):
        palette.setColor(role, QColor(color))
    palette.setColor(QPalette.Disabled, QPalette.Text, QColor("#9ca3af"))
    palette.setColor(QPalette.Disabled, QPalette.WindowText, QColor("#9ca3af"))
    palette.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#9ca3af"))
    app.setPalette(palette)
    arrows = _arrow_files()
    app.setStyleSheet(_qss(arrows["down"], arrows["up"]))


def repolish(widget):
    """동적 프로퍼티를 바꾼 뒤 QSS를 다시 먹인다 (안 하면 [selected="true"] 가 안 바뀐다)."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def set_variant(button, variant):
    button.setProperty("variant", variant)
    repolish(button)


def pill_css(state):
    fg, bg = PILL.get(state, PILL["stopped"])
    return f"color:{fg}; background:{bg};"


def fit_to_screen(window, width, height, margin=40):
    """원하는 크기로 열되 화면보다 크면 줄인다.

    GPU 드라이버가 빠져 800x600 프레임버퍼로 떠 있던 적이 있다 — 그때 1000x900 창은
    아래쪽 버튼이 화면 밖으로 나간다.
    """
    screen = QGuiApplication.primaryScreen()
    if screen is None:
        window.resize(width, height)
        return
    avail = screen.availableGeometry()
    window.resize(min(width, avail.width() - margin), min(height, avail.height() - margin))
