# convert_model.py
import tensorflow as tf

model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # or skip for full precision
tflite_model = converter.convert()

with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model converted to TFLite!")