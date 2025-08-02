import os
from rembg import remove
from PIL import Image
from pathlib import Path

input_root = Path('copra_classifier/dataset-temp')
output_root = Path('copra_classifier/dataset')

output_root.mkdir(parents=True, exist_ok=True)

valid_exts = ('.jpg', '.jpeg', '.png')
for class_folder in input_root.iterdir():
    if not class_folder.is_dir():
        continue
    
    class_name = class_folder.name
    output_class_folder = output_root / class_name
    output_class_folder.mkdir(parents=True, exist_ok=True)
    
    print(f'\nProcessing: {class_name}')
    count = 0
    
    for img_path in sorted(class_folder.glob('*')):
        if img_path.suffix.lower() not in valid_exts:
            continue
        try:
            with Image.open(img_path) as img:
                img = img.convert('RGBA')
                result = remove(img)
                
                #a choice for final image to png
                filename = f'{class_name}_{count:09d}.png'
                output_path = output_class_folder / filename
                result.save(output_path)
                
                #changes background to white
                #background = Image.new('RGBA', result.size, (255, 255, 255, 255))
                #blended = Image.alpha_composite(background, result).convert('RGB')
                    
                #jpg final image
                #filename = f'{class_name}_{count:09d}.jpg'
                #output_path = output_class_folder / filename
                #rgb_result = result.convert('RGB')
                #rgb_result.save(output_path, format='JPEG', quality=95)
                
                print(f'\nSaved: {output_path.name}')
                count += 1
        except Exception as e:
            print(f'\nError with {img_path.name}: {e}')
print('\nImage cleaning successfully!')