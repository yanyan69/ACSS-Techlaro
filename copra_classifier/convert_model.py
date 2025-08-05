import os, warnings, logging
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'       #ignores everything except errors
logging.getLogger('tensorflow').setLevel
warnings.filterwarnings('ignore', category=UserWarning, module = 'google.protobuf')

import tensorflow as tf

# Load your normalized Keras model
model = tf.keras.models.load_model('copra_classifier/models/copra_model.keras')

# Convert to TFLite (using only built-in ops, compatible with Pi)
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]

# Force float32 input/output (so you can keep using img / 255.0)
converter.inference_input_type = tf.float32
converter.inference_output_type = tf.float32

# Convert the model
tflite_model = converter.convert()

# Save the converted model
with open('copra_classifier/models/copra_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("✅ Model successfully converted to TFLite using TFLITE_BUILTINS.")
