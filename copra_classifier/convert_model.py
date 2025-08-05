import tensorflow as tf
import os

model_path = 'copra_classifier/models/copra_model.keras'
output_path = 'copra_classifier/models/copra_model.tflite'

print(f"🔍 Loading model from: {model_path}")
if not os.path.exists(model_path):
    raise FileNotFoundError(f"❌ Model not found at {model_path}")

# Load Keras model
model = tf.keras.models.load_model(model_path)

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # Optional: dynamic range quantization
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]

print("⚙️ Converting to TFLite...")
tflite_model = converter.convert()

# Save TFLite model
with open(output_path, 'wb') as f:
    f.write(tflite_model)

print(f"✅ Model successfully converted to: {output_path}")
