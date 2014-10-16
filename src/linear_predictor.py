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
		self.m_std_x = 1

	#----------------------------------------------------------------------	
	#load predictor from file
	#----------------------------------------------------------------------
	def load(self, filename):
		A = np.load(filename) #load numpy array stored in filename
		self.m_w = A[0, :]
		self.m_mean_x = A[1, :]
		self.m_std_x = A[2, :]

	#----------------------------------------------------------------------
	#compute ouput prediction given input features
	#----------------------------------------------------------------------
	def predict(self, feat_array):
		#renormalize features
		xtmp = (feat_array - self.m_mean_x) / self.m_std_x
		#compute dot product between features and predictor
		return np.dot(xtmp, self.m_w)
	
	def to_string(self):
		print self.m_w


def load(filename):
	pred = LinearPredictor()
	pred.load(filename)
	return pred

def train(X, y, filename, options, feature_weight=np.array([1.0]), sample_weight_type="None"):
	mean_x = X.mean(0)
	std_x = X.std(0)
	n = mean_x.size
	#hack to keep bias feature when removing mean and renormalizing with std
	mean_x[n - 1] = 0; 
	std_x[n - 1] = 1;	
	
	for index, x in enumerate(std_x):
		if x == 0.0:
			std_x[index] = 1.0
			print "WARNING: Failing index with zero stddev = %d" % index

	#renormalize features
	X = (X - mean_x) / std_x
	(r, c) = X.shape
	#solve ridge regression
	if options.size == 0:
		options = np.array(1)
		
	# compute sample weights
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
		else:
			X_sub = X[abs(y) <= nonzero_val, :]
			y_sub = y[abs(y) <= nonzero_val]
			Xtmp = X[abs(y) > nonzero_val, :]
			ytmp = y[abs(y) > nonzero_val]
			X_sub = np.vstack((X_sub, Xtmp[range(0, nb_nonzero, int(nb_nonzero / (m - nb_nonzero))), :]))
			y_sub = np.append(y_sub, ytmp[range(0, nb_nonzero, int(nb_nonzero / (m - nb_nonzero)))])
			
	A = np.zeros((3, n))
	A[1, :] = mean_x
	A[2, :] = std_x
	for i in range(0, options.size):
		print "[DAgger] Training with Regularizer %f" % (options[i])
		reg = math.sqrt(r) * options[i]
		#ridge = linear_model.Ridge(alpha=reg, fit_intercept=False)
		outname, outext = os.path.splitext(filename)
		fname = "%s-%f%s" % (outname, options[i], outext)
		#ridge.fit(X,y)
		#w = ridge.coef_
		#print feature_weight
		if sample_weight_type == "weighted":
				w = own_ridge(X, y, options[i], feature_weight, sample_weights)
		elif sample_weight_type == "subsample":
				w = own_ridge(X_sub, y_sub, options[i], feature_weight, np.array([]))
		else:
			w = own_ridge(X, y, options[i], feature_weight, np.array([]))
		#w = basispursuit(X,y,m_reg1,m_reg2)
		A[0, :] = w
		np.save(fname, A)

def own_ridge(X, y, reg, feature_weight, sample_weight):
    (r, c) = X.shape
    reg_mat = np.eye(c)
    if feature_weight.size > 1:
        reg_mat = np.diag(feature_weight)
    
    # Sample weights
    if sample_weight.size > 1:
     	sum_weights = np.sum(sample_weight)
      	Ws = np.diag(sample_weight)
       	XT_W = np.dot(X.T, Ws)
    	#return np.dot(np.dot(la.inv(np.dot(XT_W, X) + reg * math.sqrt(sum_weights) * reg_mat), XT_W), y)
    	return np.linalg.solve(np.dot(XT_W, X) + reg * math.sqrt(sum_weights) * reg_mat, np.dot(XT_W, y))
    else:
    	print X.shape
     	#return np.dot(np.dot(la.inv(np.dot(X.T, X) + reg * math.sqrt(r) * reg_mat), X.T), y)
     	return np.linalg.solve(np.dot(X.T, X) + reg * math.sqrt(r) * reg_mat, np.dot(X.T, y))

def basispursuit(X, y, reg1, reg2):
	(r, c) = X.shape
	res = y
	u = np.zeros(c)
	B = np.zeros(c)
	I = np.array([], dtype=np.int32)
	#find basis
	foundBasis = False
	while not foundBasis:
		for i in range(c):
			if B[i] == 0:
				u[i] = abs(np.dot(X[:, i], res)) / r
		imax = u.argmax()
		if u[imax] > reg1:
			I = np.append(I, imax)
			print "[DAgger] New Basis of %d dimensions, Last Index: %d Utility: %f" % (I.size, imax, u[imax])
			B[imax] = 1
			u[imax] = 0
			wtmp = ridge(X[:, I], y, 0)
			res = np.dot(X[:, I], wtmp) - y
		else:
			foundBasis = True
	print "[DAgger] Found Basis of %d dimensions" % I.size
	wtmp = ridge(X[:, I], y, reg2)
	w = np.zeros(c)
	w[I] = wtmp
	return w
