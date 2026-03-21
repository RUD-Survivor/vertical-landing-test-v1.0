import codecs

content = codecs.open('src/render/renderer3d.h', 'r', 'utf-8').read()
content = content.replace("float* m = vp.m;\n    float det;", "const double* m = vp.m;\n    double det;")
content = content.replace("float* m = vp.m;\r\n    float det;", "const double* m = vp.m;\r\n    double det;")

codecs.open('src/render/renderer3d.h', 'w', 'utf-8').write(content)
print("Finished fixing renderer3d double ptr bug")
