import tensorflow as tf

model = tf.keras.models.load_model('./copra_classifier/models/copra_model.keras', compile=False)

# Enable the modern TFLite converter
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.experimental_new_converter = True  # ✅ Add this line
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]

tflite_model = converter.convert()

with open('./copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model successfully converted to TFLite.")
