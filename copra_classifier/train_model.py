from load_dataset import load_dataset
from build_model import model
import tensorflow as tf

train_ds, class_names = load_dataset()

EPOCHS = 10

print(f'\nStarting training for {EPOCHS} epochs...')

history = model.fit(train_ds, epochs = EPOCHS)

print('\nSaving model...')
model.save('copra_classifier/models/copra_model.keras')
print('\nSave complete!')