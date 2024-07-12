#For this to work, you need to install the ffmpeg backend:
#pip install imageio[ffmpeg]

import imageio.v3 as iio
import numpy as np

for idx, frame in enumerate(iio.imiter("<video0>")): #change from 0 if multiple devices
    print(f"Frame {idx}: avg. color {np.sum(frame, axis=-1)}")