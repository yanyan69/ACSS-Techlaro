import sys
import numpy as np
import tensorflow as tf
from PIL import Image

model_path = 'copra_classifier\models\copra_model.keras'
image_size = (180, 180)
class_names = ['overcooked', 'raw', 'undercooked']

print('\nLoading Model...')
model = tf.keras.models.load_model(model_path)
print('\nModel Loaded.')

if len(sys.argv) < 2:
    image_path = input('\nEnter image path to classify: ')
else:
    image_path = sys.argv[1]

try:
    img = Image.open(image_path).convert('RGB')
    img = img.resize(image_size)
    img_array = np.array(img) / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    
    prediction = model.predict(img_array)[0]
    predicted_index = np.argmax(prediction)
    confidence = prediction[predicted_index] * 100
    predicted_label = class_names[predicted_index]
    
    print(f'\nPrediction: {predicted_label.upper()} ({confidence: .2f}%)')
    
except Exception as e:
    print(f'\nError processing image: {e}')