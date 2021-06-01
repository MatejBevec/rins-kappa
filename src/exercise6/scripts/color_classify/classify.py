import sys
import dlib
import cv2
import numpy as np
import pandas as pd
import math
import random
import os
import matplotlib.pyplot as plt
#from os import path, isfile, listdir, rename
from sklearn import svm
from sklearn import ensemble
from sklearn import neighbors

import pickle
from scipy import stats

colors = {
    "red": [255,0,0],
    "green": [0,255,0],
    "blue": [0,0,255],
    "yellow": [255,255,0],
    "white": [255,255,255],
    "black": [0,0,0]
}

class HardcodedModel():
    # for 10 bins per channel
    def __init__(self):

        self.sat_border = 15 # nad tem binom so "barve"
        self.sat_thr = 5000

        self.black_border = 23 # ta bin in nizje je crna
        self.black_thr = 5000

    def classify(self, h):
        hist = np.array(h)
        color = ""

        sat_pixels = np.sum(hist[self.sat_border:21])

        if(sat_pixels > self.sat_thr):
            #print("colored object")
            # 30 bin border: red 1-3, yellow 3-5, green 6-9, blue 9-14
            # 10 bin: red 1, yellow 1-2, green 2-3, blue 3-5

            colors = ["red", "yellow", "green", "blue"]
            sums = [np.sum(hist[0:1]), np.sum(hist[1:2]), np.sum(hist[2:4]), np.sum(hist[3:6])]
            #print(sums)
            index = np.argmax(sums)
            color = colors[index]

        else:
            #print("black or white obj")
            black_pixels = np.sum(hist[20:self.black_border+1])
            if(black_pixels > self.black_thr):
                #print("black obj")
                color = "black"
            else:
                #print("white obj")
                color = "white"
        #print(color)
        return(color)

    def predict(self, hist_mat):
        results = []
        for row in hist_mat:
            pred = self.classify(row)
            # print(pred)
            results.append(pred)
        return results

def hsv_hist(img, bins):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h,bins = np.histogram(img_hsv[:,:,0].ravel(), bins, [0,256])
    s,bins = np.histogram(img_hsv[:,:,1].ravel(), bins, [0,256])
    v,bins = np.histogram(img_hsv[:,:,2].ravel(), bins, [0,256])

    return np.concatenate([h,s,v], axis=0)

def plot_img_hist(img, hist, label):
    fig = plt.figure(figsize=(12,8))
    plt.subplot(121), plt.imshow(img)
    fig.suptitle(label)
    bins = len(hist)/3
    plt.subplot(122), plt.plot(hist, color="r")
    plt.axvline(x=bins, color="b", linestyle='--')
    plt.axvline(x=bins*2, color="b", linestyle='--')
    plt.show()

def get_hist_matrix(img_array, hist_func, bins):
    hist_mat = []
    for img in img_array:
        #img = cv2.imread(os.path.join(data_path, f), cv2.IMREAD_COLOR)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #img = img[50:250, 100:300]
        hist = hist_func(img, bins)
        hist_mat.append(hist)

    return hist_mat

def load_model(filename):
    file = open(filename, "rb")
    model = pickle.load(file)
    file.close()
    return model


def classify_all(models, hist_mat):
    all_predicts = np.array([])
    for model in models:
        pred = model.predict(hist_mat)
        all_predicts = np.concatenate([all_predicts, pred], axis=0)
    # stats.mode(all_predicts) for majority prediction
    return all_predicts


def classify_wrapper(imgs):
    model2 = load_model("trained_models/combined_hsv_rf_10bin.model")
    model4 = HardcodedModel()
    bins = 10
    for img in imgs:
        img = cv2.resize(img, (400, 300))
    hist_mat = get_hist_matrix(imgs, hsv_hist, bins)
    pred = classify_all([model4], hist_mat)
    final_choice = stats.mode(pred)[0][0]

    return final_choice

if __name__ == "__main__":

    # natrenirani sk-learn modeli
    model1 = load_model("trained_models/cg_hsv_rf_10bin_nocrop.model")
    model2 = load_model("trained_models/combined_hsv_rf_10bin.model")
    model3 = load_model("trained_models/combined_hsv_rf_10bin_nocrop.model")
    model4 = HardcodedModel() # 3x10 bin!

    data_path = "./examples"
    bins = 10 #se mora ujemat z modeli

    """
    f = sys.argv[1] if len(sys.argv) > 1 else "ex_2.png"
    img = cv2.imread(os.path.join(data_path, f), cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (400, 300))
    """

    filenames = ["ex_0.png", "ex_1.png", "ex_2.png"]
    imgs = []
    for f in filenames:
        img = cv2.imread(os.path.join(data_path, f), cv2.IMREAD_COLOR)
        # input mora bit openCV rgb slika
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # MISLM DA TREBA KER MODELI NISO NORMALIZIRANI
        img = cv2.resize(img, (400, 300))
        imgs.append(img)

    # imgs = array uporabljenih slik
    # hist_mat = matrika histogramov teh slik
    hist_mat = get_hist_matrix(imgs, hsv_hist, bins)
    models = [model1, model2, model3, model4] # kere modele hoces uporbit

    # classify_all klasificira vse slike iz historgrama z vsemi podanimi modeli
    # in sestavi rezultate
    pred = classify_all(models, hist_mat)
    final_choice = stats.mode(pred)[0][0]

    #HITREJE ZA EN SAM MODEL, npr hardcoded
    #pred = model4.predict(hist_mat)
    #final_choice = stats.mode(pred)[0][0]

    #hist = hsv_hist(img, bins)
    #pred = model.predict([hist])
    #pred = classify_many_models([model, model2, model3, model4], hist)
    #hm = HardcodedModel()
    #pred = hm.classify(hist)
    #pred = model.predict([hist])


    #print(hist)
    print(pred)
    print(final_choice)

    #plot_img_hist(img, hist, pred[0])
