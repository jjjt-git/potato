#!/usr/bin/python

code = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

base  = len(code)
len_n = len(str(base))

out = "(\n"

for ii in range(base):
    out += f"\t{ii:0>{len_n}} => x\"{hex(ord(code[ii]))[2:]}\",\n"

out = out[:-2]
out += "\n);"

print(out)
