import tensorflow as tf, matplotlib.pyplot as plt
from load_dataset import load_dataset

train_ds, class_names = load_dataset()

#load dataset
print('\nLoading dataset...')

class_names = train_ds.class_names
print(f'\nClass names found: {class_names}')

#display imgs
plt.figure(figsize = (10, 10))
for class_index, class_names in enumerate(class_names):
    for images, labels in train_ds:
        for i in range(len(labels)):
            if labels[i].numpy() == class_index:
                ax = plt.subplot(1, 3, class_index + 1)
        
                plt.imshow(images[i].numpy().astype('uint8'))

                plt.title(class_names)
                plt.axis('off')
                break
            else:
                continue
        break
plt.suptitle('\nSample copra images from dataset', fontsize=16)
plt.tight_layout()
plt.show()