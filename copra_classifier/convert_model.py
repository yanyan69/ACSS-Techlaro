import tensorflow as tf

# Load your Keras model
model = tf.keras.models.load_model('models/copra_model.keras')

# Set up converter with legacy ops
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS,
    tf.lite.OpsSet.SELECT_TF_OPS  # Optional if your model uses TF ops
]
converter.experimental_new_converter = False  # 🔧 Force legacy converter

# Convert and save
tflite_model = converter.convert()

with open('models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ TFLite model converted using legacy ops for compatibility.")
