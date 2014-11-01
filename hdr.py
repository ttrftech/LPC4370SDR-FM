#!/usr/bin/env python
import struct
import sys

fin = open(sys.argv[1], 'rb')
fout = open(sys.argv[2], 'wb')

bin = fin.read()
fin.close()

n = (len(bin) + 511) / 512
fout.write('\32\77%s\0\0\0\0\0\0\0\0\377\377\377\377' % struct.pack('<H', n))
fout.write(bin)
fout.close()
