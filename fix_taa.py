import codecs

content = codecs.open('src/render/renderer3d.h', 'r', 'utf-8').read()
content = content.replace('glUniformMatrix4fv(glGetUniformLocation(taaProg, "uInvViewProj"), 1, GL_FALSE, invViewProj.m);', 'sendMat4(glGetUniformLocation(taaProg, "uInvViewProj"), invViewProj);')
content = content.replace('glUniformMatrix4fv(glGetUniformLocation(taaProg, "uPrevViewProj"), 1, GL_FALSE, prevViewProj.m);', 'sendMat4(glGetUniformLocation(taaProg, "uPrevViewProj"), prevViewProj);')

# Wait, the third one was: glUniformMatrix4fv(us_invViewProj, 1, GL_FALSE, inv);
# But 'inv' is a float array! From math3d.h `inv[16]`, so it MUST use glUniformMatrix4fv!
# Wait, let me check `renderer3d.h` around line 4200 inside `view_file`

codecs.open('src/render/renderer3d.h', 'w', 'utf-8').write(content)
print("TAA fixed")
