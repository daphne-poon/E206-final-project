import os
import time
import cv2
import sys
import csv
import time
import numpy as np
import glob

def clear_dir(dir):
    for f in os.listdir(dir):
        os.remove(os.path.join(dir, f))


def make_vide():
    img_array = []
    for filename in sorted(glob.glob('./images_1/*.png')):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)

    out = cv2.VideoWriter('video.mp4',cv2.VideoWriter_fourcc(*'MP4V'), 5.0, size)
    
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()


if __name__ == '__main__':
    #clear_dir('./images')
    make_vide()