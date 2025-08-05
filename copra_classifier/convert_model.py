import tensorflow as tf

# Load your normalized Keras model
model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# 👇 This ensures you only use basic built-in ops compatible with older Pi runtimes
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]

# Force float32 input/output
converter.inference_input_type = tf.float32
converter.inference_output_type = tf.float32

# Convert the model
tflite_model = converter.convert()

# Save the model
with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model successfully converted to TFLite.")
