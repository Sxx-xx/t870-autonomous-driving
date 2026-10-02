import os
import re

def to_snake_case(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def fix_msg_files(directory, changes_file):
    changes = {}
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".msg"):
                filepath = os.path.join(root, file)
                with open(filepath, 'r') as f:
                    lines = f.readlines()
                
                new_lines = []
                changed_in_file = False
                for line in lines:
                    if '=' in line:
                        new_lines.append(line)
                        continue
                    
                    match = re.match(r'^\s*(\w+(\[\])?)\s+([a-z]+[A-Z]+\w*)\s*.*$', line)
                    if match:
                        field_type = match.group(1)
                        field_name = match.group(3)
                        new_field_name = to_snake_case(field_name)
                        
                        if field_name != new_field_name:
                            changes[field_name] = new_field_name
                            new_line = line.replace(field_name, new_field_name)
                            new_lines.append(new_line)
                            changed_in_file = True
                        else:
                            new_lines.append(line)
                    else:
                        new_lines.append(line)
                
                if changed_in_file:
                    with open(filepath, 'w') as f:
                        f.writelines(new_lines)
    
    with open(changes_file, 'w') as f:
        for old, new in changes.items():
            f.write(f"{old},{new}\n")

if __name__ == "__main__":
    fix_msg_files("src/ublox_msgs/msg", "changes.txt")