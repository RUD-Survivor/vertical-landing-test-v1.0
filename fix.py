import codecs
import re

content = codecs.open('src/render/renderer3d.h', 'r', 'utf-8').read()

if 'inline void sendMat4' not in content:
    content = content.replace('#include "vegetation_system.h"', '#include "vegetation_system.h"\n\ninline void sendMat4(GLint loc, const Mat4& m) {\n    float arr[16];\n    m.toFloatArray(arr);\n    glUniformMatrix4fv(loc, 1, GL_FALSE, arr);\n}')

old_lake = '''        // Soft-flatten ground to lake surface level
        float originalH = hStr;
        hStr = mix(hStr, filledH, lakeMask * 0.95); // 0.95 to keep some floor shape
        vWaterDepth = max(0.0, filledH - originalH);'''
new_lake = '''        // Soft-flatten ground to lake surface level
        // We ensure that only the local valleys that are BELOW the water line get submerged.
        // If the local high-frequency noise created a hill above the water level, we leave it alone.
        float originalH = hStr;
        if (originalH < filledH && lakeMask > 0.01) {
             float fillMask = smoothstep(0.0001, 0.005, filledH - originalH);
             hStr = mix(originalH, filledH, lakeMask * fillMask * 0.95); 
        }
        vWaterDepth = max(0.0, filledH - originalH);'''
if old_lake in content:
    content = content.replace(old_lake, new_lake)

old_hydro = '''        // 3. Hydrology: Geometric lake/river flattening
        vec4 hydro = texture(uHydroMap, geoUV);
        float filledH = hydro.r;'''
new_hydro = '''        // 3. Hydrology: Geometric lake/river flattening
        vec4 hydro = texture(uHydroMap, geoUV);
        float filledH = hydro.r; 
        float acc = hydro.g;
        float strahler = hydro.b;'''
if old_hydro in content:
    content = content.replace(old_hydro, new_hydro)

content = re.sub(r'glUniformMatrix4fv\s*\(\s*([a-zA-Z0-9_>]+)\s*,\s*1\s*,\s*GL_FALSE\s*,\s*([a-zA-Z0-9_]+)\.m\s*\)', r'sendMat4(\1, \2)', content)

codecs.open('src/render/renderer3d.h', 'w', 'utf-8').write(content)
print("done")
