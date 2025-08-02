import tensorflow as tf

def save_model(model, path='copra_classifier/models/copra_model.keras'):
    model.save(path)
    print(f'\nModel saved to: {path}')

def load_model(path='copra_classifier/models/copra_model.keras'):
    model = tf.keras.models.load_model(path)
    print(f'\nModel loaded from: {path}')
    return model