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
model.save('copra_classifier/models/copra_model.keras')
print("✅ Model saved to: copra_classifier/models/copra_model.keras")
