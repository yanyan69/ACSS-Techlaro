import tensorflow as tf

# Load the normalized model
model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
converter.inference_input_type = tf.float32
converter.inference_output_type = tf.float32

# Convert
tflite_model = converter.convert()

# Save model
with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model successfully converted to TFLite.")
