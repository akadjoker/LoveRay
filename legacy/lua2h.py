import os, sys, shutil, re, textwrap

def make_c_include(name, data):
    name = re.sub("[^a-z0-9]", "_", name.lower())
    res = "static const char " + name + "[] = {"
    for c in data:
        res += str(ord(c)) + ", "
    res = res.rstrip(", ") + "};"
    return name, textwrap.fill(res, width=79)


embedded_files=[]
embedded_files.append("boot.lua")
embedded_files.append("nogame.lua")

for filename in embedded_files:
    name = os.path.basename(filename)
    name, text = make_c_include(name, open(filename).read())
    open(name+".h", "w").write(text)
    #open("%s.h" % (name), "wb").write(text)


