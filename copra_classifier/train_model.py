import os, warnings, logging, tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'       #ignores everything except errors
logging.getLogger('tensorflow').setLevel
warnings.filterwarnings('ignore', category=UserWarning, module = 'google.protobuf')

from build_model import model
from load_dataset import load_dataset

print('\nLoading training and validation data...')
train_ds, val_ds, _ = load_dataset()

EPOCHS = 10
print(f"\nStarting training for {EPOCHS} epochs...")

history = model.fit(train_ds, validation_data=val_ds, epochs=EPOCHS)

# Evaluate
loss, acc = model.evaluate(val_ds)
print(f"\nFinal Validation Accuracy: {acc:.2%}")

# Save model
model.save('./copra_classifier/models/copra_model.keras')
print("\nModel saved to: copra_classifier/models/copra_model.keras")

model = tf.keras.models.load_model("copra_classifier/models/copra_model.keras")

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

with open("copra_classifier/models/copra_model.tflite", "wb") as f:
    f.write(tflite_model)

print("✅ TFLite conversion successful!")