import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'       #ignores everything except errors
logging.getLogger('tensorflow').setLevel
warnings.filterwarnings('ignore', category=UserWarning, module = 'google.protobuf')

import tensorflow as tf 
import numpy as np
from PIL import Image

model_path = 'copra_classifier/models/copra_model.keras'
model = tf.keras.models.load_model(model_path)
print(f'\nModel loaded from: {model_path}')

class_names = ['overcooked', 'raw', 'standard']

image_path = input('\nEnter image file path to predict: ').strip()

if not os.path.exists(image_path):
    print('\nImage not found, input correct path')
    exit()
    
img = Image.open(image_path).resize((180, 180))
img_array = tf.keras.utils.img_to_array(img)
img_array = tf.expand_dims(img_array, 0)

predictions = model.predict(img_array)
predicted_class = class_names[np.argmax(predictions)]
confidence = 100 * np.max(predictions)

print(f"""\n Prediction Result:
    \n Class: {predicted_class}
    \n Confidence: {confidence:.2f}%
    """)