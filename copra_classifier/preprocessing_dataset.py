import tensorflow as tf
from tensorflow import keras
from keras import layers
from load_dataset import load_dataset

train_ds, class_names = load_dataset()

normalization_layer = layers.Rescaling(1./255)
data_augmentation = keras.Sequential([
    
    layers.RandomFlip('horizontal'),
    layers.RandomRotation(0.1),
    layers.RandomZoom(0.1),
])
AUTOTUNE = tf.data.AUTOTUNE
train_ds = train_ds.map(lambda x,y: (normalization_layer(x), y))
train_ds = train_ds.cache().shuffle(100).prefetch(buffer_size=AUTOTUNE)

print('\nDataset normalized and augmented.')