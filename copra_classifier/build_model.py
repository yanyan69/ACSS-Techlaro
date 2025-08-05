import tensorflow as tf
from tensorflow import keras
from keras import layers
from load_dataset import load_dataset

# === Load datasets
train_ds, val_ds, class_names = load_dataset()
print('\n✅ Preprocessing dataset...')

AUTOTUNE = tf.data.AUTOTUNE

# === Normalize + Augment
normalization_layer = layers.Rescaling(1./255)

data_augmentation = keras.Sequential([
    layers.RandomFlip('horizontal'),
    layers.RandomRotation(0.1),
    layers.RandomZoom(0.1),
])

train_ds = train_ds.map(lambda x, y: (data_augmentation(normalization_layer(x)), y))
val_ds = val_ds.map(lambda x, y: (normalization_layer(x), y))

train_ds = train_ds.cache().shuffle(100).prefetch(buffer_size=AUTOTUNE)
val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)

# === Build model (batch dimension compatible)
print('\n🛠️ Building CNN model...')

model = keras.Sequential([
    layers.Input(shape=(180, 180, 3)),  # explicitly defines input layer
    layers.Rescaling(1./255),
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

# === Compile
print('\n⚙️ Compiling model...')
model.compile(
    optimizer='adam',
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

# === Save class names
with open('./copra_classifier/models/class_names.txt', 'w') as f:
    for name in class_names:
        f.write(name + '\n')

# === Show summary
print('\n📊 Model summary:')
model.summary()
