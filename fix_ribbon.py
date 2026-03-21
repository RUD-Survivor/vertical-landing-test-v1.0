import codecs

content = codecs.open('src/render/renderer3d.h', 'r', 'utf-8').read()

old_push = '''      // Zigzag for triangle strip: first left, then right
      stripVerts.push_back({p + right * halfW, colors[i], 1.0f});
      stripVerts.push_back({p - right * halfW, colors[i], -1.0f});'''

new_push = '''      // Zigzag for triangle strip: first left, then right
      Vec3 pL = p + right * halfW;
      stripVerts.push_back({(float)pL.x, (float)pL.y, (float)pL.z, (float)colors[i].x, (float)colors[i].y, (float)colors[i].z, (float)colors[i].w, 1.0f});
      Vec3 pR = p - right * halfW;
      stripVerts.push_back({(float)pR.x, (float)pR.y, (float)pR.z, (float)colors[i].x, (float)colors[i].y, (float)colors[i].z, (float)colors[i].w, -1.0f});'''

# Normalize newlines for matching
content_normalized = content.replace('\r\n', '\n')
old_push_normalized = old_push.replace('\r\n', '\n')

if old_push_normalized in content_normalized:
    content_normalized = content_normalized.replace(old_push_normalized, new_push.replace('\r\n', '\n'))
    content = content_normalized

old_ptrs = '''    // Re-establish vertex layout
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3) + sizeof(Vec4)));
    glEnableVertexAttribArray(2);'''

new_ptrs = '''    // Re-establish vertex layout
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(7 * sizeof(float)));
    glEnableVertexAttribArray(2);'''

old_ptrs_normalized = old_ptrs.replace('\r\n', '\n')

if old_ptrs_normalized in content_normalized:
    content_normalized = content_normalized.replace(old_ptrs_normalized, new_ptrs.replace('\r\n', '\n'))
    content = content_normalized

codecs.open('src/render/renderer3d.h', 'w', 'utf-8').write(content)
print("Ribbon updated successfully.")
