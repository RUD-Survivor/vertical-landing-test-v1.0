import os
import re

path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_input_system.h'
with open(path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

out = []
skip = False
for line in lines:
    st = line.strip()
    # If the line is an auto& injection OUTSIDE a block
    if st.startswith('auto& guid =') or st.startswith('auto& mnv =') or st.startswith('auto& tele =') or st.startswith('auto& trans ='):
        # Only skip if it's not indented like inside a lambda (e.g. less than 12 spaces)
        spaces = len(line) - len(line.lstrip())
        if spaces < 12:
            continue
    out.append(line)

content = "".join(out)
with open(path, 'w', encoding='utf-8') as f:
    f.write(content)
print("Done")
