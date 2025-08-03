try:
    from tflite_runtime.interpreter import Interpreter
    print("✅ tflite-runtime is working!")
except Exception as e:
    print("❌ Failed to import tflite-runtime")
    print(e)
