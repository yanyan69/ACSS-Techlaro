import os
import tensorflow as tf
from tensorflow import keras
from keras import layers
from load_dataset import load_dataset

# Check if a saved model exists
model_path = "copra_classifier/models/copra_model.keras"
if os.path.exists(model_path):
    print("✅ Loading existing model...")
    model = tf.keras.models.load_model(model_path)
else:
    print("🛠️ Building new model...")
    train_ds, val_ds, class_names = load_dataset()

    model = keras.Sequential([
        layers.Rescaling(1./255, input_shape=(180, 180, 3)),
        layers.Conv2D(16, 3, activation='relu'),
        layers.MaxPooling2D(),
        layers.Conv2D(32, 3, activation='relu'),
        layers.MaxPooling2D(),
        layers.Conv2D(64, 3, activation='relu'),
        layers.MaxPooling2D(),
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dense(len(class_names), activation='softmax')
    ])

    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
