from copra_classifier.train_model import model
from copra_classifier.model_utils import save_model, load_model
import numpy as np

save_model(model)
loaded_model = load_model()

print('testing model prediction on 1 sample: ')
sample = np.random.rand(1,180,180,3)
prediction = loaded_model.predict(sample)
print(f'Prediction output: {prediction}')
print('akosi jerald')