#!/usr/bin/env python  
import math
import numpy as np
import scipy
import scipy.linalg as la
import os
from sklearn import linear_model


class LinearPredictor:
    def __init__(self):
        self.m_w = 0
        self.m_mean_x = 0
        self.m_mean_y = 0
        self.m_std_x = 1

        # ----------------------------------------------------------------------

    #load predictor from file
    #----------------------------------------------------------------------
    def load(self, filename):
        A = np.load(filename)  #load numpy array stored in filename
        self.m_w = A[0, :]
        self.m_mean_x = A[1, :]
        self.m_std_x = A[2, :]
        self.m_mean_y = A[3, 0]

    #----------------------------------------------------------------------
    #compute ouput prediction given input features
    #----------------------------------------------------------------------
    def predict(self, feat_array):
        #renormalize features
        xtmp = (feat_array - self.m_mean_x) / self.m_std_x
        #compute dot product between features and predictor
        return np.dot(xtmp, self.m_w) + self.m_mean_y

    def to_string(self):
        print self.m_w


def load(filename):
    pred = LinearPredictor()
    pred.load(filename)
    return pred


def train(X, y, filename, options, feature_weight=np.array([1.0]), sample_weight_type="None", print_flag=0):
    mean_x = X.mean(0)
    std_x = X.std(0)
    mean_y = y.mean(0)
    n = mean_x.size
    # hack to keep bias feature when removing mean and renormalizing with std
    #mean_x[n - 1] = 0;
    #std_x[n - 1] = 1;

    for index, x in enumerate(std_x):
        if x == 0.0:
            std_x[index] = 1.0
            print "WARNING: Failing index with zero stddev = %d" % index

    #renormalize features
    X = (X - mean_x) / std_x
    (r, c) = X.shape
    #solve ridge regression
    if options.size == 0:
        options = np.array([1])

    # compute sample weights
    y = y
    m = y.size
    sample_weights = np.ones(m)
    X_sub = np.array([])
    y_sub = np.array([])
    nonzero_val = 0.01
    if sample_weight_type == "weighted":
        nb_nonzero = np.sum(abs(y) > nonzero_val)
        weight_nonzero = m / (2.0 * nb_nonzero)
        weight_zero = m / (2.0 * (m - nb_nonzero))
        sample_weights[abs(y) > nonzero_val] = weight_nonzero
        sample_weights[abs(y) <= nonzero_val] = weight_zero
    elif sample_weight_type == "subsample":
        nb_nonzero = np.sum(abs(y) > nonzero_val)
        if nb_nonzero < m - nb_nonzero:
            X_sub = X[abs(y) > nonzero_val, :]
            y_sub = y[abs(y) > nonzero_val]
            Xtmp = X[abs(y) <= nonzero_val, :]
            ytmp = y[abs(y) <= nonzero_val]
            X_sub = np.vstack((X_sub, Xtmp[range(0, m - nb_nonzero, int((m - nb_nonzero) / nb_nonzero)), :]))
            y_sub = np.append(y_sub, ytmp[range(0, m - nb_nonzero, int((m - nb_nonzero) / nb_nonzero))])
            y_sub = y_sub - mean_y
        else:
            X_sub = X[abs(y) <= nonzero_val, :]
            y_sub = y[abs(y) <= nonzero_val]
            Xtmp = X[abs(y) > nonzero_val, :]
            ytmp = y[abs(y) > nonzero_val]
            X_sub = np.vstack((X_sub, Xtmp[range(0, nb_nonzero, int(nb_nonzero / (m - nb_nonzero))), :]))
            y_sub = np.append(y_sub, ytmp[range(0, nb_nonzero, int(nb_nonzero / (m - nb_nonzero)))])
            y_sub = y_sub - mean_y
    y = y - mean_y
    A = np.zeros((4, n))
    A[1, :] = mean_x
    A[2, :] = std_x
    A[3,:] = mean_y

    for i in range(options.size):
        #print "[DAgger] Training with Regularizer %f" % (options[i])

        reg = math.sqrt(r) * options[i]
        outname, outext = os.path.splitext(filename)
        fname = "%s-%f%s" % (outname, options[i], outext)
        reg_algo = linear_model.Ridge(alpha=reg, fit_intercept=False)
        #reg_algo = linear_model.Lasso(alpha=reg/math.sqrt(r), fit_intercept=False)
        if sample_weight_type == "None":
            reg_algo.fit(X,y)
            w = reg_algo.coef_
        elif sample_weight_type == "subsample":
            reg_algo.fit(X_sub,y_sub)
            w = reg_algo.coef_
        if print_flag ==1:
            print "[DAgger] learned weights for reg ", options[i], ": "
            print w
        A[0, :] = w
        np.save(fname, A)





