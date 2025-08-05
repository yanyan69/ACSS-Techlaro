import tensorflow as tf
import os

def load_dataset(dataset_dir='copra_classifier/dataset', img_size=(180, 180), batch_size=8):
    print('\nLoading datasets...')

    train_ds = tf.keras.preprocessing.image_dataset_from_directory(
        dataset_dir,
        validation_split=0.2,
        subset='training',
        seed=123,
        image_size=img_size,
        batch_size=batch_size
    )

    val_ds = tf.keras.preprocessing.image_dataset_from_directory(
        dataset_dir,
        validation_split=0.2,
        subset='validation',
        seed=123,
        image_size=img_size,
        batch_size=batch_size
    )

    print(f"\nClasses found: {train_ds.class_names}")
    print(f"Training batches: {len(train_ds)}")
    print(f"Validation batches: {len(val_ds)}")

    return train_ds, val_ds, train_ds.class_names
