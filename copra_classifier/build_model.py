import tensorflow as tf
from tensorflow import keras
from keras import layers
from load_dataset import load_dataset

train_ds, class_names = load_dataset()
print('\nPreprocessing dataset...')

AUTOTUNE = tf.data.AUTOTUNE

train_ds = train_ds.cache().shuffle(100).prefetch(buffer_size=AUTOTUNE)

print('\nBuilding cnn model...')

model = keras.Sequential([
    layers.Rescaling(1./255, input_shape = (180, 180, 3)),
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

print('\nCompiling model...')

model.compile(
    optimizer = 'adam',
    loss = 'sparse_categorical_crossentropy',
    metrics = ['accuracy']
)

print('\nModel Summary: ')
model.summary()