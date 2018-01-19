import rospkg
import os.path

BASE_DIR = rospkg.RosPack().get_path('villa_sound_localization')
DATA_DIR = os.path.join(BASE_DIR, 'data')
NETWORK_FILES_DIR = os.path.join(BASE_DIR, 'share/network')
