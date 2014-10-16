#!/usr/bin/env python  
import roslib
roslib.load_manifest('ardrone_dagger_im')

import rospy
import rosbag
import numpy as np
import random
import pickle

import os
from subprocess import call
import shlex

#this code can use different type of predictors
#the predictor .py file needs to implement 
# - a global method train, which takes numpy array X (input features), a numpy array y (target outputs), a filename as input, and an numpy array for optional parameters
#see linear_predictor.py for an example
#to abstract the type of predictor, simply import the predictor .py file as predictor
#e.g. import linear_predictor as predictor
import linear_predictor as predictor

import default_feature_constructor as feature 

class Dataset:
	def __init__(self):
		self.X = np.array([]) #r x c matrix of r datapoints, each with c input features
		self.y = np.array([]) #target output (r x 1 vector)
		self.r = 0 #number of datapoints in X
		self.c = 0 #number of features

	def load(self,ds,path_bag,pub_record):
		rospy.loginfo("[DAgger] Opening dataset %s", ds)
		last_nb = 0
		f = open(ds,'r')
		#load all bags in f
		for line in f:
			#open the current bag file
			line2 = line.rstrip(' \t\n')
			rospy.loginfo("[DAgger] Opening bag file %s", path_bag + line2)
			try:
				bag = rosbag.Bag(path_bag + line2)
			except rosbag.bag.ROSBagUnindexedException:
				rospy.loginfo("[DAgger] Unindexed Bag file %s. Attempting to reindex", path_bag + line2)
				call(shlex.split("rosbag reindex %s"%(path_bag + line2)))
				try:
					bag = rosbag.Bag(path_bag + line2)
					rospy.loginfo("[DAgger] Reindexing Succesful")
				except rosbag.bag.ROSBagUnindexedException:
					rospy.loginfo("[DAgger] Reindexing failed, skipping file %s", path_bag + line2)
					continue
			#look at msg in dagger_record topic
			for topic, msg, t in bag.read_messages(topics=[pub_record]):
				#convert msg.data to a numpy array
				ar = np.array(msg.data, dtype=np.float32)
				n = ar.size
				if self.r == 0:
					self.X = ar[0:n-1] #this takes up all elems of ar except last one
				else:			
					self.X = np.vstack((self.X,ar[0:n-1])) #this takes all elems of ar except last one
				self.y = np.append(self.y,ar[n-1]) #this takes last elem of ar
				self.r+= 1
				self.c = n-1
			rospy.loginfo("[DAgger] Loaded %d datapoints from bag file", self.r-last_nb)
			last_nb = self.r
			bag.close()
		f.close()

	def random_permute(self):
		p = range(0,self.r)
		for i in range(0,self.r):
			r = random.randint(i,self.r-1)
			x = p[i]
			p[i] = p[r]
			p[r] = x
		self.X = self.X[p,:]
		self.y = self.y[p]
		
	def split(self,nbSplit):
		dsSplit1 = Dataset()
		dsSplit2 = Dataset()
		dsSplit1.X = self.X[0:nbSplit,:]
		dsSplit1.y = self.y[0:nbSplit]
		dsSplit1.r = nbSplit
		dsSplit1.c = self.c
		dsSplit2.X = self.X[nbSplit:self.r,:]
		dsSplit2.y = self.y[nbSplit:self.r]
		dsSplit2.r = self.r - nbSplit
		dsSplit2.c = self.c
		return (dsSplit1,dsSplit2)

	def nfold_split(self,nfold):
		nbEach = int(self.r/float(nfold))
		dsList = list()
		idx = 0
		for i in range(nfold-1):
			ds = Dataset()
			ds.X = self.X[idx:idx+nbEach,:]
			ds.y = self.y[idx:idx+nbEach]
			ds.r = nbEach
			ds.c = self.c
			idx+=nbEach
			dsList.append(ds)
		ds = Dataset()
		ds.X = self.X[idx:self.r,:]
		ds.y = self.y[idx:self.r]
		ds.r = nbEach
		ds.c = self.c
		return dsList

	def merge(self,ds):
		self.X = np.append(self.X,ds.X)
		self.y = np.append(self.y,ds.y)
		self.r += ds.r


def cv_idx_train(n_data,n_fold,fold_id):
	idx = range(0,n_data)
	ratio = 1.0/n_fold
	last_id = int(n_data * fold_id * ratio)
	first_id = int(n_data * (fold_id-1) * ratio)
	idx = np.delete(idx,range(first_id,last_id))
	return idx

def cv_idx_test(n_data,n_fold,fold_id):
	ratio = 1.0/n_fold
	last_id = int(n_data * fold_id * ratio)
	first_id = int(n_data * (fold_id-1) * ratio)
	return np.array(range(first_id,last_id))

def rand_perm(n):
	p = range(0,n)
	for i in range(0,n):
		r = random.randint(i,n-1)
		x = p[i]
		p[i] = p[r]
		p[r] = x
	return p

if __name__ == '__main__':
	rospy.init_node('train')

	# Read in the full path to dataset containing bags to train on
	dataset_train = rospy.get_param('~dataset_train')
	dataset_test = rospy.get_param('~dataset_test')
	ratio_test = rospy.get_param('~ratio_test')
	cv_fold = rospy.get_param('~cv_fold')
	path_bag = rospy.get_param('~path_bag')
	outfile = rospy.get_param('~pred_file')
	pub_record = rospy.get_param('~pub_record')
	feature_weighted = rospy.get_param('~feature_weighted')
	sample_weight_type = rospy.get_param('~sample_weight_type')
	nb_ctrl_filters = rospy.get_param('~nb_ctrl_filters')
	quickload = rospy.get_param('~quickload')

	#regs = np.array([0.001,0.01,0.1,1.0,10.0,100.0,1000.0,10000.0])
	#regs = np.array([1.0,10.0,100.0])
	regs = np.array([1.0])

	ds_test = Dataset()
	ds_valid = Dataset()
	ds_train = Dataset()
    
	err_tol = 0.05 #for support vector regression loss, and ''classification'' loss
	feature_weights = np.array([1.0])
    
	#train
	if dataset_train != "":
		ds_train_orig = Dataset()
		if quickload:
			pickle_file_id = open("quickload_data.dat", "r")
			ds_train_orig = pickle.load(pickle_file_id)
			pickle_file_id.close()
		else:
			ds_train_orig.load(dataset_train,path_bag,pub_record)
			ds_train_orig.random_permute()
			pickle_file_id = open("quickload_data.dat", "w")
			pickle.dump(ds_train_orig, pickle_file_id)
			pickle_file_id.close()

		if ratio_test > 0:
			(ds_valid,ds_train) = ds_train_orig.split(int(ratio_test*ds_train_orig.r))
		else:
			ds_train = ds_train_orig

        # get weights
        if(feature_weighted == True):
            (feat_array_dummy, feature_weights) = feature.construct_weighted(np.zeros(ds_train_orig.X.shape[1]-nb_ctrl_filters-3), np.zeros(nb_ctrl_filters), [0], [0])
        
        options = np.array([1.0])
        outname,outext = os.path.splitext(outfile)
        if cv_fold <= 1:
			rospy.loginfo("[DAgger] Training on a total of %d datapoints with %d features", ds_train.r, ds_train.c)
			rospy.loginfo("[DAgger] Feature Values: Mean: %f Min: %f Max %f", ds_train.X.mean(), ds_train.X.min(), ds_train.X.max())
			rospy.loginfo("[DAgger] Output Values: Mean: %f Min: %f Max %f Abs Mean %f, Squared Mean %f, %f-SVR %f, %f-Cls %f", ds_train.y.mean(), ds_train.y.min(), ds_train.y.max(), abs(ds_train.y).mean(), np.dot(ds_train.y,ds_train.y)/ds_train.r,err_tol,sum((abs(ds_train.y)-err_tol)[abs(ds_train.y) > err_tol]),err_tol,sum(abs(ds_train.y)>=err_tol))
			
			#for reg in regs:
				#options[0] = reg
				#rospy.loginfo("[DAgger] Training with Regularizer %f",options[0])
				#fname = "%s-%f%s"%(outname,options[0],outext)
				#predictor.train(ds_train.X,ds_train.y,fname,options,feature_weights)
			predictor.train(ds_train.X,ds_train.y,outfile,regs,feature_weights,sample_weight_type)
			rospy.loginfo("[DAgger] Training complete")
        else:
			rospy.loginfo("[DAgger] Cross-Validation on %d fold of %d datapoints with %d features", cv_fold, ds_train.r, ds_train.c)
			rospy.loginfo("[DAgger] Training on all %d data",ds_train_orig.r)			
			predictor.train(ds_train.X,ds_train.y,outfile,regs,feature_weights,sample_weight_type)
			for i in range(cv_fold):
				idx_train = cv_idx_train(ds_train.r,cv_fold,i+1)
				fname = "%s-%d%s"%(outname,i,outext)
				rospy.loginfo("[DAgger] Training without fold %d",i+1)			
				predictor.train(ds_train.X[idx_train,:],ds_train.y[idx_train],fname,regs,feature_weights,sample_weight_type)
			
			best_err1 = 0
			best_err2 = 0
			best_errh = 0 #with err_tol tolerance
			best_errcls = 0 #with err_tol tolerance
			best_reg1 = 0
			best_reg2 = 0
			best_regh = 0
			best_regcls = 0
			is_first = True
			for reg in regs:
				err1 = 0
				err2 = 0
				errh = 0
				errcls = 0
				for i in range(cv_fold):
					rospy.loginfo("[DAgger] Testing Reg %f on heldout folds", reg)			
					fname = "%s-%d-%f%s"%(outname,i,reg,outext)
					lp = predictor.load(fname)
					idx_test = cv_idx_test(ds_train.r,cv_fold,i+1)
					for j in idx_test:
						err = lp.predict(ds_train.X[j,:])-ds_train.y[j]
						err1 += abs(err)
						err2 += err*err
						if abs(err) >= err_tol:
							errh += abs(err)-err_tol
							errcls += 1
				rospy.loginfo("[DAgger] Test Folds Abs. Error: %f Squared Error: %f %f-SVR Loss: %f %f-Cls Error %f",err1,err2,err_tol,errh,err_tol,errcls)
				if is_first or err1 < best_err1:
					best_err1 = err1
					best_reg1 = reg
				if is_first or err2 < best_err2:
					best_err2 = err2
					best_reg2 = reg
				if is_first or errh < best_errh:
					best_errh = errh
					best_regh = reg
				if is_first or errcls < best_errcls:
					best_errcls = errcls
					best_regcls = reg
				is_first = False
			rospy.loginfo("[DAgger] Best Regularizer Abs. error: %f",best_reg1)
			rospy.loginfo("[DAgger] Best regularizer Squared error: %f",best_reg2)
			rospy.loginfo("[DAgger] Best Regularizer SVR: %f",best_regh)
			rospy.loginfo("[DAgger] Best regularizer Cls: %f",best_regcls)
			fname = "%s-%f%s"%(outname,best_reg1,outext)
			fnamecp = "%s-%f-bestl1%s"%(outname,best_reg1,outext)
			call(shlex.split("cp %s %s"%(fname,fnamecp)))
			fname = "%s-%f%s"%(outname,best_reg2,outext)
			fnamecp = "%s-%f-bestl2%s"%(outname,best_reg2,outext)
			call(shlex.split("cp %s %s"%(fname,fnamecp)))
			fname = "%s-%f%s"%(outname,best_regh,outext)
			fnamecp = "%s-%f-bestsvr%s"%(outname,best_regh,outext)
			call(shlex.split("cp %s %s"%(fname,fnamecp)))
			fname = "%s-%f%s"%(outname,best_reg2,outext)
			fnamecp = "%s-%f-bestcls%s"%(outname,best_regcls,outext)
			call(shlex.split("cp %s %s"%(fname,fnamecp)))

	#validation to optimize regularizer
	if ratio_test > 0:
		rospy.loginfo("[DAgger] Validating on a total of %d datapoints with %d features.", ds_valid.r, ds_valid.c)
		rospy.loginfo("[DAgger] Feature Values: Mean: %f Min: %f Max %f", ds_valid.X.mean(), ds_valid.X.min(), ds_valid.X.max())
		rospy.loginfo("[DAgger] Output Values: Mean: %f Min: %f Max %f Abs Mean %f, Squared Mean %f, %f-SVR %f, %f-Cls %f", ds_valid.y.mean(), ds_valid.y.min(), ds_valid.y.max(), abs(ds_valid.y).mean(), np.dot(ds_valid.y,ds_valid.y)/ds_valid.r,err_tol,sum((abs(ds_valid.y)-err_tol)[abs(ds_valid.y) > err_tol]),err_tol,sum(abs(ds_valid.y)>=err_tol))
		best_reg1 = 0
		best_err1 = 0
		best_reg2 = 0
		best_err2 = 0
		best_regh = 0
		best_errh = 0
		best_regcls = 0
		best_errcls = 0
		is_first = True
		for reg in regs:
			fname = "%s-%f%s"%(outname,reg,outext)
			lp = predictor.load(fname)
			rospy.loginfo("[DAgger] Validating Predictor with Regularizer %f",reg)
			err1 = np.zeros(ds_valid.r)
			err2 = np.zeros(ds_valid.r)
			yhat = np.zeros(ds_valid.r)
			errh = 0
			errcls = 0
			for i in range(ds_valid.r):
				yhat[i] = lp.predict(ds_valid.X[i,:])
				err = ds_valid.y[i] - yhat[i]
				err1[i] = abs(err)
				err2[i] = err*err
				if abs(err) >= err_tol:
					errcls += 1
					errh += abs(err)-err_tol 
			mean_err1 = err1.mean()
			mean_err2 = err2.mean()
			if is_first or mean_err1 < best_err1:
				best_err1 = mean_err1
				best_reg1 = reg
			if is_first or mean_err2 < best_err2:
				best_err2 = mean_err2
				best_reg2 = reg
			if is_first or errh < best_errh:
				best_errh = errh
				best_regh = reg
			if is_first or errcls < best_errcls:
				best_errcls = errcls
				best_regcls = reg
			is_first = False			
			rospy.loginfo("[DAgger] Mean Abs. error: %f Mean Squared error: %f %f-SVR Error: %f %f-cls Error: %f",mean_err1,mean_err2,err_tol,errh,err_tol,errcls)
			I = np.argsort(err1)
			rospy.loginfo("[DAgger] Largest 30 Abs. errors:")
			for i in range(30):
				rospy.loginfo("[DAgger] Index: %d Correct: %f Pred: %f",I[ds_valid.r-i-1], ds_valid.y[I[ds_valid.r-i-1]],yhat[I[ds_valid.r-i-1]])

		rospy.loginfo("[DAgger] Best Regularizer Valid Abs. error: %f",best_reg1)
		rospy.loginfo("[DAgger] Best regularizer Valid Squared error: %f",best_reg2)
		rospy.loginfo("[DAgger] Best regularizer Valid SVR: %f",best_regh)
		rospy.loginfo("[DAgger] Best regularizer Valid Cls: %f",best_regcls)
		options = np.array([best_reg1])
		rospy.loginfo("[DAgger] Training on all %d data with Regularizer %f",ds_train_orig.r,options[0])
		fname = "%s-bestl1%s"%(outname,outext)
		predictor.train(ds_train_orig.X,ds_train_orig.y,fname,options,feature_weights,sample_weight_type)
		options = np.array([best_reg2])
		rospy.loginfo("[DAgger] Training on all %d data with Regularizer %f",ds_train_orig.r,options[0])
		fname = "%s-bestl2%s"%(outname,outext)
		predictor.train(ds_train_orig.X,ds_train_orig.y,fname,options,feature_weights,sample_weight_type)
		options = np.array([best_regh])
		rospy.loginfo("[DAgger] Training on all %d data with Regularizer %f",ds_train_orig.r,options[0])
		fname = "%s-bestsvr%s"%(outname,outext)
		predictor.train(ds_train_orig.X,ds_train_orig.y,fname,options,feature_weights,sample_weight_type)
		options = np.array([best_regcls])
		rospy.loginfo("[DAgger] Training on all %d data with Regularizer %f",ds_train_orig.r,options[0])
		fname = "%s-bestcls%s"%(outname,outext)
		predictor.train(ds_train_orig.X,ds_train_orig.y,fname,options,feature_weights,sample_weight_type)

	#test
	if dataset_test != "":
		ds_test.load(dataset_test,path_bag,pub_record)

		rospy.loginfo("[DAgger] Testing on a total of %d datapoints with %d features. Avg. absolute output %f", ds_test.r, ds_test.c, abs(ds_test.y).mean())
		rospy.loginfo("[DAgger] Feature Values: Mean: %f Min: %f Max %f", ds_test.X.mean(), ds_test.X.min(), ds_test.X.max())
		rospy.loginfo("[DAgger] Output Values: Mean: %f Min: %f Max %f Abs Mean %f, Squared Mean %f, %f-SVR %f, %f-Cls %f", ds_test.y.mean(), ds_test.y.min(), ds_test.y.max(), abs(ds_test.y).mean(), np.dot(ds_test.y,ds_test.y)/ds_test.r,err_tol,sum((abs(ds_test.y)-err_tol)[abs(ds_test.y) > err_tol]),err_tol,sum(abs(ds_test.y)>=err_tol))
		outname,outext = os.path.splitext(outfile)
		
		best_reg1 = 0
		best_err1 = 0
		best_reg2 = 0
		best_err2 = 0
		best_regh = 0
		best_errh = 0
		best_regcls = 0
		best_errcls = 0
		is_first = True
		for reg in regs:
			fname = "%s-%f%s"%(outname,reg,outext)
			lp = predictor.load(fname)
			rospy.loginfo("[DAgger] Testing Predictor with Regularizer %f",reg)
			err1 = np.zeros(ds_test.r)
			err2 = np.zeros(ds_test.r)
			yhat = np.zeros(ds_test.r)
			errh = 0
			errcls = 0
			for i in range(ds_test.r):
				yhat[i] = lp.predict(ds_test.X[i,:])
				err = ds_test.y[i] - yhat[i]
				err1[i] = abs(err)
				err2[i] = err*err
				if abs(err) >= err_tol:
					errcls += 1
					errh += abs(err)-err_tol 
			mean_err1 = err1.mean()
			mean_err2 = err2.mean()
			if is_first or mean_err1 < best_err1:
				best_err1 = mean_err1
				best_reg1 = reg
			if is_first or mean_err2 < best_err2:
				best_err2 = mean_err2
				best_reg2 = reg
			if is_first or errh < best_errh:
				best_errh = errh
				best_regh = reg
			if is_first or errcls < best_errcls:
				best_errcls = errcls
				best_regcls = reg
			is_first = False			
			rospy.loginfo("[DAgger] Mean Abs. error: %f Mean Squared error: %f %f-SVR Error: %f %f-cls Error: %f",mean_err1,mean_err2,err_tol,errh,err_tol,errcls)
			I = np.argsort(err1)
			rospy.loginfo("[DAgger] Largest 30 Abs. errors:")
			for i in range(30):
				rospy.loginfo("[DAgger] Index: %d Correct: %f Pred: %f",I[ds_test.r-i-1], ds_test.y[I[ds_test.r-i-1]],yhat[I[ds_test.r-i-1]])
		rospy.loginfo("[DAgger] Best Regularizer Valid Abs. error: %f",best_reg1)
		rospy.loginfo("[DAgger] Best regularizer Valid Squared error: %f",best_reg2)
		rospy.loginfo("[DAgger] Best regularizer Valid SVR: %f",best_regh)
		rospy.loginfo("[DAgger] Best regularizer Valid Cls: %f",best_regcls)
	
