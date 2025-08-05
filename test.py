import tensorflow as tf

interpreter = tf.lite.Interpreter(model_path='copra_classifier/models/copra_model.tflite')
interpreter.allocate_tensors()

input_type = interpreter.get_input_details()[0]['dtype']
print("✅ Input type:", input_type)
