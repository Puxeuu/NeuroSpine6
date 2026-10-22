C_BG_WINDOW   = "#0b0c10"
C_BG_HEADER   = "#141416"
C_BG_SIDEBAR  = "#141416"

C_PANEL       = "#1c1c1e"
C_PANEL_ALT   = "#242426"
C_BORDER      = "#2c2c2e"
C_BORDER_STRONG = "#3a3a3c"

C_TEXT        = "#ffffff"
C_TEXT_DIM    = "#8e8e93"
C_TEXT_MUTED  = "#636366"

C_ACCENT      = "#0a84ff"
C_ACCENT_SOFT = "rgba(10, 132, 255, 0.15)"
C_ACCENT_LINE = "rgba(10, 132, 255, 0.35)"

C_SAFE        = "#5cb85c"
C_WARN        = "#f0ad4e"
C_DANGER      = "#d9534f"

C_AXIS_X      = "#ff453a"
C_AXIS_Y      = "#30d158"
C_AXIS_Z      = "#0a84ff"

CARGOL_COLORS = [
    "#0a84ff",
    "#ff9f0a",
    "#30d158",
    "#bf5af2",
    "#64d2ff",
    "#ff375f",
]


FONT_FAMILY  = "-apple-system, 'Segoe UI', 'Helvetica Neue', Arial, sans-serif"
FONT_MONO    = "'SF Mono', 'JetBrains Mono', Consolas, monospace"

SIZE_XS   = 10
SIZE_SM   = 11
SIZE_BASE = 13
SIZE_LG   = 15
SIZE_XL   = 18


def global_stylesheet():
    return f"""
        /* ============ BASE ============ */
        QMainWindow {{ background-color: {C_BG_WINDOW}; }}
        QWidget {{
            color: {C_TEXT};
            font-family: {FONT_FAMILY};
            font-size: {SIZE_BASE}px;
            background-color: transparent;
        }}

        /* ============ PANELLS ============ */
        QFrame#Panel {{
            background-color: {C_PANEL};
            border: 1px solid {C_BORDER};
            border-radius: 12px;
        }}
        QFrame#PanelAlt {{
            background-color: {C_PANEL_ALT};
            border: 1px solid {C_BORDER};
            border-radius: 12px;
        }}

        /* ============ TITOLS ============ */
        QLabel#SectionTitle {{
            color: {C_TEXT};
            font-size: {SIZE_SM}px;
            font-weight: 700;
            text-transform: uppercase;
            letter-spacing: 1.2px;
            padding-bottom: 6px;
            border: none;
            border-bottom: 1px solid {C_BORDER};
            margin-bottom: 4px;
            background: transparent;
        }}
        QLabel#Caption {{
            color: {C_TEXT_DIM};
            font-size: {SIZE_XS}px;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.8px;
        }}
        QLabel#Value {{
            color: {C_TEXT};
            font-size: {SIZE_LG}px;
            font-weight: 700;
            font-family: {FONT_MONO};
        }}

        /* ============ BOTONS ============ */
        QPushButton {{
            background-color: {C_PANEL_ALT};
            color: {C_TEXT};
            border: 1px solid {C_BORDER_STRONG};
            border-radius: 8px;
            padding: 8px 14px;
            font-weight: 600;
            font-size: {SIZE_BASE}px;
        }}
        QPushButton:hover {{
            background-color: #3a3a3c;
            border: 1px solid #48484a;
        }}
        QPushButton:pressed  {{ background-color: #48484a; }}
        QPushButton:disabled {{ color: {C_TEXT_MUTED}; background-color: #1a1a1c; }}
        QPushButton:checked  {{
            background-color: {C_ACCENT_SOFT};
            color: {C_ACCENT};
            border: 1px solid {C_ACCENT};
        }}

        /* Botons secundaris petits (dins de vistes CT, etc.) */
        QPushButton#MiniTool {{
            background-color: transparent;
            color: {C_TEXT_DIM};
            border: 1px solid {C_BORDER};
            border-radius: 4px;
            padding: 3px 8px;
            font-size: {SIZE_XS}px;
            font-weight: 600;
        }}
        QPushButton#MiniTool:hover {{
            background-color: {C_ACCENT_SOFT};
            color: {C_ACCENT};
            border: 1px solid {C_ACCENT_LINE};
        }}

        /* ============ COMBOBOX ============ */
        QComboBox {{
            background-color: {C_PANEL_ALT};
            color: {C_TEXT};
            border: 1px solid {C_BORDER_STRONG};
            border-radius: 6px;
            padding: 6px 10px;
            font-size: {SIZE_BASE}px;
        }}
        QComboBox:hover     {{ border: 1px solid {C_ACCENT}; }}
        QComboBox::drop-down {{ border: none; }}
        QComboBox QAbstractItemView {{
            background-color: {C_PANEL_ALT};
            border: 1px solid {C_BORDER_STRONG};
            selection-background-color: {C_ACCENT};
            color: {C_TEXT};
            padding: 4px;
        }}

        /* ============ LLISTES ============ */
        QListWidget {{
            background-color: {C_BG_WINDOW};
            border: 1px solid {C_BORDER};
            border-radius: 8px;
            padding: 4px;
            font-size: {SIZE_BASE}px;
        }}
        QListWidget::item {{
            padding: 8px 10px;
            border-bottom: 1px solid {C_BORDER};
            border-radius: 4px;
        }}
        QListWidget::item:selected {{
            background-color: {C_ACCENT_SOFT};
            color: {C_ACCENT};
            border-left: 3px solid {C_ACCENT};
        }}

        /* ============ CHECKBOXES ============ */
        QCheckBox {{ spacing: 8px; }}
        QCheckBox::indicator {{
            width: 18px; height: 18px;
            border-radius: 4px;
            border: 1px solid #8e8e93;
            background-color: {C_PANEL_ALT};
        }}
        QCheckBox::indicator:checked {{
            background-color: {C_ACCENT};
            border: 1px solid {C_ACCENT};
        }}

        /* ============ SLIDERS ============ */
        QSlider::groove:horizontal {{
            background: {C_BORDER_STRONG};
            height: 6px;
            border-radius: 3px;
        }}
        QSlider::sub-page:horizontal {{
            background: #6e6e73;
            border-radius: 3px;
        }}
        QSlider::handle:horizontal {{
            background: #ffffff;
            border: 1px solid #8e8e93;
            width: 10px; height: 10px;
            margin: -2px 0;
            border-radius: 5px;
        }}

        /* ============ TEXT AREA ============ */
        QTextEdit {{
            background-color: #121214;
            border: 1px solid {C_BORDER};
            border-radius: 10px;
            font-family: {FONT_MONO};
            font-size: {SIZE_SM}px;
            padding: 10px;
            color: #e5e5ea;
        }}
    """


def style_pill(color_hex, bg_alpha=0.12, border_alpha=0.3):
    r, g, b = _hex_to_rgb(color_hex)
    return (
        f"color: {color_hex}; "
        f"background: rgba({r},{g},{b},{bg_alpha}); "
        f"border: 1px solid rgba({r},{g},{b},{border_alpha}); "
        f"border-radius: 4px; padding: 3px 10px; "
        f"font-size: {SIZE_XS}px; font-weight: 700; letter-spacing: 0.6px;"
    )


def _hex_to_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))

