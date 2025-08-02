import tensorflow as tf
import os

def load_dataset(dataset_dir = 'copra_classifier\dataset', img_size = (180,180), batch_size=8):
    print('\nLoading data sets...')

    print(f'\nLoading from: {os.path.abspath(dataset_dir)}')
    print(f'\nSubfolders: {os.listdir(dataset_dir)}')
    
    #loads train set
    train_ds = tf.keras.preprocessing.image_dataset_from_directory(
        dataset_dir, 
        image_size=img_size,
        batch_size=batch_size
        )

    #load validation set
    val_ds = tf.keras.preprocessing.image_dataset_from_directory(
        dataset_dir,
        image_size=img_size,
        batch_size=batch_size
    )

    print(f"""\nDataset loaded... 
        \nclass found: {train_ds.class_names}... 
        \ntraining batches: {len(train_ds)}... 
        \nvalidation batches: {len(val_ds)}...""")
    
    return train_ds, train_ds.class_names