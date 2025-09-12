import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(BASE_DIR, 'resources', 'icons')
PROFILE_PATH = os.path.join(BASE_DIR, 'resources', 'profiles')
DB_FILE = os.path.join(BASE_DIR, 'data', 'acss_stats.db')

COLORS = {
    'bg_primary': '#FFFFFF',
    'bg_secondary': '#FFFFFF',
    'btn_active': '#4CAF50',
    'btn_inactive': '#F44336',
    'text_primary': '#374151',
    'text_secondary': '#000000',
    'accent': '#1d4ed8',
    'highlight': '#D3D3D3',
}
FONTS = {
    'heading': ('Arial', 24, 'bold'),
    'subheading': ('Arial', 18, 'bold'),
    'paragraph': ('Arial', 14),
    'paragraph_bold': ('Arial', 14, 'bold'),
    'table': ('Arial', 16),
    'log': ('Arial', 12),
    'button': ('Arial', 14),
    'confirm': ('Arial', 16, 'bold'),
}
SIZES = {
    'sidebar_width': 300,
    'padding': 10,
    'icon_size': 64,
    'start_stop_size': 128,
    'small_button_size': 64,
    'photo_size': 100,
    'camera_height': 400,
}