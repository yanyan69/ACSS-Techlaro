import os, warnings, logging
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
print(f"\n✅ Final Validation Accuracy: {acc:.2%}")

# Save model
model.save('models/copra_model.h5')
print("✅ Model saved to: copra_classifier/models/copra_model.h5")
