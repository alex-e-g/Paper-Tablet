# -*- coding: utf-8 -*-
"""
Created on Fri Jan 14 14:37:16 2022

@author: sofia
"""
import sys
import numpy as np
import cv2 as cv
from scipy.io import savemat
import argparse

def run(image_path,template_path):

    image = cv.imread(image_path)
    template = cv.imread(template_path)
    
    # Initiate SIFT detector
    sift = cv.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1,des1 = sift.detectAndCompute(template,None)
    kp2,des2 = sift.detectAndCompute(image,None)

    # use brute force matching
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)
        
    good = []
    if len(matches) >  100:
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)
        if len(good) < 100:
            good = []
            for m,n in matches:
                good.append(m)
            

    dst_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,2)
    src_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,2)
    
    dict = {"src": src_pts, "dst": dst_pts}
    savemat("matched_points.mat",dict)
    
    return