import time

import pickle

import numpy as np


fname = f"{str(int((time.time())))}_data.pickle"
wfd = open(fname, "wb")


for i in range(10):
    pickle.dump(i, wfd)


pickle.dump({'a': 2}, wfd)

pickle.dump(np.ones((10,10)), wfd)


wfd.close()




