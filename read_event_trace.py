import sys
import ctypes as ct

code = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

rev_code = dict()

ii = 0
for cc in code:
    rev_code[cc] = ii
    ii += 1

if len(sys.argv) != 2:
    print("first and only argument should be input file\n")
    exit(1)

class Record_t(ct.LittleEndianStructure):
    _fields_ = (
            ('pc'  , ct.c_uint64,14),
            ('time', ct.c_uint64,10),
            ('rand', ct.c_uint64,6),
            ('fifo', ct.c_uint64,6),
            ('lru' , ct.c_uint64,6),
            ('mru' , ct.c_uint64,6),
            ('dlfu', ct.c_uint64,6)
        )
    _basetype_ = ct.c_uint64

class Record_u(ct.Union):
    _fields_ = [
            ("s", Record_t),
            ("i", ct.c_uint64)
        ]

benchmark_cnt = 0
in_trace = False
trace_cnt = 0
buf = 0

time_cnt = 0
time_l   = 0

print("benchmark,pc,time,random,fifo,lru,mru,dlfu")

with open(sys.argv[1]) as trace:
    while True:
        cc = trace.read(1)
        if not cc:
            exit(0)
        if cc == '%' and not in_trace:
            trace_cnt = 0
            in_trace = True
            benchmark_cnt += 1
            continue;
        if cc == '%' and in_trace:
            in_trace = False
        if in_trace:
            trace_cnt += 1
            buf = buf << 6
            buf = buf | rev_code[cc]

            if trace_cnt == 9:
                pkt = Record_u()
                pkt.i = buf

                if pkt.s.time < time_l:
                    time_cnt += 1
                time_l = pkt.s.time

                trace_cnt = 0
                print(f"{benchmark_cnt},{hex(pkt.s.pc << 2)},{time_cnt * 1024 + pkt.s.time},{pkt.s.rand},{pkt.s.fifo},{pkt.s.lru},{pkt.s.mru},{pkt.s.dlfu}")
                buf = 0
