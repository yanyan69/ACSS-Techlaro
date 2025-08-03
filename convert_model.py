# convert_model.py
import tensorflow as tf

# Load the Keras model
model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')

# Convert to TFLite with compatible ops (v1 for FULLY_CONNECTED, etc.)
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]  # No Select TensorFlow ops
tflite_model = converter.convert()

# Save the .tflite model
with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)


print("✅ Model converted to TFLite!")


