import tensorflow as tf
import os

def convert_model(model, keras_path="copra_classifier/models/copra_model.keras", tflite_path="copra_classifier/models/copra_model.tflite"):
    print(f"\n💾 Saving Keras model to: {keras_path}")
    os.makedirs(os.path.dirname(keras_path), exist_ok=True)
    model.save(keras_path)

    print("🔁 Converting to TFLite...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
    tflite_model = converter.convert()

    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    print(f"✅ TFLite model saved to: {tflite_path}")
