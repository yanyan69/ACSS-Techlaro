import os, warnings, logging, tensorflow as tf

# === Suppress TensorFlow Warnings and Logs ===
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
warnings.filterwarnings('ignore', category=UserWarning, module='google.protobuf')
logging.getLogger('tensorflow').setLevel(logging.ERROR)

# === Load model + data ===
from build_model import model
from load_dataset import load_dataset
from convert_model import convert_model  # Import the converter function

print('\n📂 Loading training and validation data...')
train_ds, val_ds, _ = load_dataset()

# === Train ===
EPOCHS = 10
print(f"\n🚀 Starting training for {EPOCHS} epochs...")
history = model.fit(train_ds, validation_data=val_ds, epochs=EPOCHS)

# === Evaluate ===
loss, acc = model.evaluate(val_ds)
print(f"\n✅ Final Validation Accuracy: {acc:.2%}")

# === Convert to TFLite + Save .keras
convert_model(model)
