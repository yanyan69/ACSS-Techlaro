# convert_model.py
import tensorflow as tf

# Load your normalized Keras model (trained with images scaled to 0~1)
model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # optional: quantization (optional)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]  # use only built-in ops

# Convert the model
tflite_model = converter.convert()

# Save it
with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model successfully converted to TFLite.")