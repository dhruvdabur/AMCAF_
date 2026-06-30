import os

root_dir = '/home/dhruv/amcaf/gazebo_sim'

# Replacement rules:
# 1.  -> '' (relative to gazebo_worlds/)
# 2. ../ -> '../' (relative to gazebo_worlds/)
# 3.  -> ''
# 4. ../ -> '../'

rules = [
    ('', ''),
    ('../', '../'),
    ('', ''),
    ('../', '../')
]

updated_files = []

for r, d, files in os.walk(root_dir):
    d[:] = [dirname for dirname in d if not dirname.startswith('.')]
    for f in files:
        if f.endswith(('.sdf', '.config', '.py', '.txt', '.json', '.xml')):
            filepath = os.path.join(r, f)
            try:
                with open(filepath, 'r', encoding='utf-8', errors='ignore') as file_obj:
                    content = file_obj.read()
                
                original_content = content
                for old_path, new_path in rules:
                    content = content.replace(old_path, new_path)
                
                if content != original_content:
                    with open(filepath, 'w', encoding='utf-8') as file_obj:
                        file_obj.write(content)
                    updated_files.append(filepath)
            except Exception as e:
                print(f'Error processing {filepath}: {e}')

if updated_files:
    print('Updated files:')
    for f in updated_files:
        print(f'  - {f}')
else:
    print('No absolute paths found to convert.')
