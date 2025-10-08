from ultralytics import YOLO
model = YOLO("my_model/my_model.pt")
print(model.names)  # Should output: {0: 'Overcooked', 1: 'Standard', 2: 'Raw'}