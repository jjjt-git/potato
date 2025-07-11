import sys
import ctypes as ct

code = "NMLDFR"

rev_code = dict()

ii = 0
for cc in code:
    rev_code[cc] = ii
    ii += 1

if len(sys.argv) != 2:
    print("first and only argument should be input file\n")
    exit(1)

benchmark_cnt = 0
in_trace = False

time = 0

print("benchmark,time,sample,policy")

with open(sys.argv[1]) as trace:
    while True:
        cc = trace.read(1)
        if not cc:
            exit(0)
        if cc == '$' and not in_trace:
            time = 0
            in_trace = True
            benchmark_cnt += 1
            continue;
        if cc == '$' and in_trace:
            in_trace = False
        if in_trace:
            print(f"{benchmark_cnt},{time * 16},{time},{rev_code[cc]}")
            time += 1
