import sys

import pickle as pk

seed_filename = sys.argv[1]

wfd = open(seed_filename.replace("_", "_trj.pickle"), "wb")

afd = open(seed_filename.replace("_", "_pre.pickle"), "rb")
while True:
    try:
        pk.dump(pk.load(afd), wfd)
    except EOFError:
        break
afd.close()


bfd = open(seed_filename.replace("_", "_dat.pickle"), "rb")
while True:
    try:
        pk.dump(pk.load(bfd), wfd)
    except EOFError:
        break
bfd.close()

wfd.close()

