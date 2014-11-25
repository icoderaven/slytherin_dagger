#!/usr/bin/env python
import roslib

roslib.load_manifest('slytherin_dagger')

import rospy
import rosbag
import numpy as np
import random
import pickle

import os
from subprocess import call
import shlex

import sys
# import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import src.linear_predictor as predictor

import src.visual_features as feature
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Dataset:
    def __init__(self):
        self.X = np.array([])  # r x c matrix of r datapoints, each with c input features
        self.yaw = np.array([])  # target output (r x 1 vector)
        self.pitch = np.array([])  # target output (r x 1 vector)
        self.r = 0  # number of datapoints in X
        self.c = 0  # number of features
        self.bridge = CvBridge()

    def load(self, ds, path_bag, pub_record):
        rospy.loginfo("[DAgger] Opening dataset %s", ds)
        last_nb = 0
        f = open(ds, 'r')
        # load all bags in f
        for line in f:
            # open the current bag file
            line2 = line.rstrip(' \t\n')
            rospy.loginfo("[DAgger] Opening bag file %s", path_bag + line2)
            try:
                bag = rosbag.Bag(path_bag + line2)
            except rosbag.bag.ROSBagUnindexedException:
                rospy.loginfo("[DAgger] Unindexed Bag file %s. Attempting to reindex", path_bag + line2)
                call(shlex.split("rosbag reindex %s" % (path_bag + line2)))
                try:
                    bag = rosbag.Bag(path_bag + line2)
                    rospy.loginfo("[DAgger] Reindexing Succesful")
                except rosbag.bag.ROSBagUnindexedException:
                    rospy.loginfo("[DAgger] Reindexing failed, skipping file %s", path_bag + line2)
                    continue
            # look at msg in dagger_record topic
            for topic, msg, t in bag.read_messages(topics=[pub_record]):
                #convert msg.data to a numpy array
                ar = np.array(msg.data, dtype=np.float32)
                n = ar.size
                if self.r == 0:
                    self.X = ar[0:n - 3]  #this takes up all elems of ar except last two
                else:
                    self.X = np.vstack((self.X, ar[0:n - 3]))  #this takes all elems of ar except last two
                self.yaw = np.append(self.yaw, ar[n - 2])  #this takes second to last elem of ar
                self.pitch = np.append(self.pitch, ar[n - 1])  #this takes last elem of ar
                self.r += 1
                self.c = n - 2
            rospy.loginfo("[DAgger] Loaded %d datapoints from bag file", self.r - last_nb)
            last_nb = self.r
            bag.close()
        f.close()

    def load2(self, ds, path_bag, pub_record,act_record):
        rospy.loginfo("[DAgger] Opening dataset %s", ds)
        last_nb = 0
        f = open(ds, 'r')
        # load all bags in f
        for line in f:
            # open the current bag file
            line2 = line.rstrip(' \t\n')
            rospy.loginfo("[DAgger] Opening bag file %s", path_bag + line2)
            try:
                bag = rosbag.Bag(path_bag + line2)
            except rosbag.bag.ROSBagUnindexedException:
                rospy.loginfo("[DAgger] Unindexed Bag file %s. Attempting to reindex", path_bag + line2)
                call(shlex.split("rosbag reindex %s" % (path_bag + line2)))
                try:
                    bag = rosbag.Bag(path_bag + line2)
                    rospy.loginfo("[DAgger] Reindexing Succesful")
                except rosbag.bag.ROSBagUnindexedException:
                    rospy.loginfo("[DAgger] Reindexing failed, skipping file %s", path_bag + line2)
                    continue
            # look at msg in dagger_record topic
            for topic, msg,t in bag.read_messages(topics=[pub_record]):
                #convert msg.data to a numpy array
                ar = np.array(self.bridge.imgmsg_to_cv2(msg,desired_encoding= 'passthrough'), dtype=np.uint8)
                n = ar.size
                if self.r == 0:
                    self.X = ar[:,:,:,np.newaxis]
                else:
                    self.X = np.concatenate((self.X, ar[:,:,:,np.newaxis]),axis=3)
                self.r += 1
                self.c = n
            for topic, msg, t in bag.read_messages(topics=[act_record]):
                #rospy.loginfo("msg %s",msg)
                yaw = np.array(msg.linear.x,dtype=np.float32)
                pitch = np.array(msg.linear.y,dtype=np.float32)
                self.yaw = np.append(self.yaw, yaw)
                self.pitch = np.append(self.pitch, pitch)
            rospy.loginfo("[DAgger] Loaded %d datapoints from bag file", self.r - last_nb)
            last_nb = self.r
            bag.close()
        f.close()

    def random_permute(self):
        p = range(0, self.r)
        for i in range(0, self.r):
            r = random.randint(i, self.r - 1)
            x = p[i]
            p[i] = p[r]
            p[r] = x
        self.X = self.X[p, :]
        self.yaw = self.yaw[p]
        self.pitch = self.pitch[p]

    def split(self, nbSplit):
        dsSplit1 = Dataset()
        dsSplit2 = Dataset()
        dsSplit1.X = self.X[0:nbSplit, :]
        dsSplit1.yaw = self.yaw[0:nbSplit]
        dsSplit1.pitch = self.pitch[0:nbSplit]
        dsSplit1.r = nbSplit
        dsSplit1.c = self.c
        dsSplit2.X = self.X[nbSplit:self.r, :]
        dsSplit2.yaw = self.yaw[nbSplit:self.r]
        dsSplit2.pitch = self.pitch[nbSplit:self.r]
        dsSplit2.r = self.r - nbSplit
        dsSplit2.c = self.c
        return (dsSplit1, dsSplit2)

    def nfold_split(self, nfold):
        nbEach = int(self.r / float(nfold))
        dsList = list()
        idx = 0
        for i in range(nfold - 1):
            ds = Dataset()
            ds.X = self.X[idx:idx + nbEach, :]
            ds.yaw = self.yaw[idx:idx + nbEach]
            ds.pitch = self.pitch[idx:idx + nbEach]
            ds.r = nbEach
            ds.c = self.c
            idx += nbEach
            dsList.append(ds)
        ds = Dataset()
        ds.X = self.X[idx:self.r, :]
        ds.yaw = self.yaw[idx:self.r]
        ds.pitch = self.pitch[idx:self.r]
        ds.r = nbEach
        ds.c = self.c
        return dsList

    def merge(self, ds):
        self.X = np.append(self.X, ds.X)
        self.yaw = np.append(self.yaw, ds.yaw)
        self.pitch = np.append(self.pitch, ds.pitch)
        self.r += ds.r


def cv_idx_train(n_data, n_fold, fold_id):
    idx = range(0, n_data)
    ratio = 1.0 / n_fold
    last_id = int(n_data * fold_id * ratio)
    first_id = int(n_data * (fold_id - 1) * ratio)
    idx = np.delete(idx, range(first_id, last_id))
    return idx


def cv_idx_test(n_data, n_fold, fold_id):
    ratio = 1.0 / n_fold
    last_id = int(n_data * fold_id * ratio)
    first_id = int(n_data * (fold_id - 1) * ratio)
    return np.array(range(first_id, last_id))


def rand_perm(n):
    p = range(0, n)
    for i in range(0, n):
        r = random.randint(i, n - 1)
        x = p[i]
        p[i] = p[r]
        p[r] = x
    return p


class Trainer:
    def __init__(self):
        self.load_params()

    def load_params(self):
        self.dataset_train = rospy.get_param('~dataset_train')
        self.dataset_test = rospy.get_param('~dataset_test')
        self.cv_fold = rospy.get_param('~cv_fold')
        self.path_bag = rospy.get_param('~path_bag')
        self.outfile_yaw = rospy.get_param('~predy_file')  # stores the result of training on the whole dataset
        self.outfile_pitch = rospy.get_param('~predp_file')  # stores the result of training on the whole dataset
        self.pub_record = rospy.get_param('~pub_record')
        self.feature_weighted = rospy.get_param('~feature_weighted')
        self.sample_weight_type = rospy.get_param('~sample_weight_type')
        self.quickload = rospy.get_param('~quickload')


def rospyloginfo(ds, err_tol):
    rospy.loginfo("[DAgger] Feature Values: Mean: %f Min: %f Max %f", ds.X.mean(), ds.X.min(),
                  ds.X.max())
    rospy.loginfo(
        "[DAgger] Yaw Values: Mean: %f Min: %f Max %f Abs Mean %f, Squared Mean %f, %f-SVR %f, %f-Cls %f",
        ds.yaw.mean(), ds.yaw.min(), ds.yaw.max(), abs(ds.yaw).mean(),
        np.dot(ds.yaw, ds.yaw) / ds.r, err_tol,
        sum((abs(ds.yaw) - err_tol)[abs(ds.yaw) > err_tol]), err_tol, sum(abs(ds.yaw) >= err_tol))
    rospy.loginfo(
        "[DAgger] Pitch Values: Mean: %f Min: %f Max %f Abs Mean %f, Squared Mean %f, %f-SVR %f, %f-Cls %f",
        ds.pitch.mean(), ds.pitch.min(), ds.pitch.max(), abs(ds.pitch).mean(),
        np.dot(ds.pitch, ds.pitch) / ds.r, err_tol,
        sum((abs(ds.pitch) - err_tol)[abs(ds.pitch) > err_tol]), err_tol, sum(abs(ds.pitch) >= err_tol))


def cross_validate(ds, regs, err_tol, cv_fold, outname, outext,mvt):
    best_err1 = np.zeros([2, 1])
    best_err2 = np.zeros([2, 1])
    best_errh = np.zeros([2, 1])
    best_errcls = np.zeros([2, 1])
    best_reg1 = np.zeros([2, 1])
    best_reg2 = np.zeros([2, 1])
    best_regh = np.zeros([2, 1])
    best_regcls = np.zeros([2, 1])
    is_first = True  # is first regularizer to be considered
    # for each of the predictors
    for l in range(2):
        rospy.loginfo("[DAgger] Predictor %d", l)
        # for each of the regularizers
        for reg in regs:
            err1 = 0
            err2 = 0
            errh = 0
            errcls = 0
            # for each cross-validation fold
            for i in range(cv_fold):
                rospy.loginfo("[DAgger] Testing Reg %f on heldout folds", reg)
                fname = "%s-%d%s%s" % (outname[l], i, mvt[l],outext[l])
                lp = predictor.load(fname)
                idx_test = cv_idx_test(ds.r, learner.cv_fold, i + 1)

                for j in idx_test:
                    if l == 0:
                        err = lp.predict(ds.X[j, :]) - ds.yaw[j]
                    else:
                        err = lp.predict(ds.X[j, :]) - ds.pitch[j]
                    err1 += abs(err)
                    err2 += err * err
                    if abs(err) >= err_tol:
                        errh += abs(err) - err_tol
                        errcls += 1

            rospy.loginfo("[DAgger] Test Folds Abs. Error: %f Squared Error: %f %f-SVR Loss: %f %f-Cls Error %f", err1,
                          err2, err_tol, errh, err_tol, errcls)
        if is_first or err1 < best_err1:
            best_err1[l] = err1
            best_reg1[l] = reg
        if is_first or err2 < best_err2:
            best_err2[l] = err2
            best_reg2[l] = reg
        if is_first or errh < best_errh:
            best_errh[l] = errh
            best_regh[l] = reg
        if is_first or errcls < best_errcls:
            best_errcls[l] = errcls
            best_regcls[l] = reg
        is_first = False

        rospy.loginfo("[DAgger] Best Regularizer Abs. error: %f", best_reg1[l])
        rospy.loginfo("[DAgger] Best regularizer Squared error: %f", best_reg2[l])
        rospy.loginfo("[DAgger] Best Regularizer SVR: %f", best_regh[l])
        rospy.loginfo("[DAgger] Best regularizer Cls: %f", best_regcls[l])

    return best_reg1, best_reg2, best_regh, best_regcls


def batch_test(ds, regs, err_tol, outname, outext,mvt):
    best_err1 = np.zeros([2, 1])
    best_err2 = np.zeros([2, 1])
    best_errh = np.zeros([2, 1])
    best_errcls = np.zeros([2, 1])
    best_reg1 = np.zeros([2, 1])
    best_reg2 = np.zeros([2, 1])
    best_regh = np.zeros([2, 1])
    best_regcls = np.zeros([2, 1])
    is_first = True
    for l in range(2):
        for reg in regs:
            fname = "%s-%f%s%s" % (outname[l], reg, mvt[l],outext[l])
            lp = predictor.load(fname)
            rospy.loginfo("[DAgger] Testing Predictor with Regularizer %f", reg)
            err1 = 0.0
            err2 = 0.0
            errh = 0
            errcls = 0
            for i in range(ds.r):
                if l==0:
                    err = lp.predict(ds.X[i, :]) - ds.yaw[i]
                else:
                    err = lp.predict(ds.X[i, :]) - ds.pitch[i]
                err1[l] += abs(err)
                err2[l] += err * err
                if abs(err) >= err_tol:
                    errcls += 1
                    errh += abs(err) - err_tol
            mean_err1 = err1/float(ds.r)
            mean_err2 = err2/float(ds.r)
            if is_first or mean_err1 < best_err1:
                best_err1[l] = mean_err1
                best_reg1[l] = reg
            if is_first or mean_err2 < best_err2:
                best_err2[l] = mean_err2
                best_reg2[l] = reg
            if is_first or errh < best_errh:
                best_errh[l] = errh
                best_regh[l] = reg
            if is_first or errcls < best_errcls:
                best_errcls[l] = errcls
                best_regcls[l] = reg
            is_first = False
            rospy.loginfo("[DAgger] Mean Abs. error: %f \n Mean Squared error: %f \n SVR Error: %f \n cls Error: %f \n",
                      mean_err1, mean_err2, err_tol, errh, err_tol, errcls)
    rospy.loginfo("[DAgger] Best Regularizer Valid Abs. error: %f %f", best_reg1[0],best_reg1[1])
    rospy.loginfo("[DAgger] Best regularizer Valid Squared error: %f %f", best_reg2[0],best_reg2[1])
    rospy.loginfo("[DAgger] Best regularizer Valid SVR: %f %f", best_regh[0],best_regh[1])
    rospy.loginfo("[DAgger] Best regularizer Valid Cls: %f %f", best_regcls[0],best_regcls[1])

    return best_reg1, best_reg2, best_regh, best_regcls

def save_bestweights(best_reg1, best_reg2, best_regh, best_regcls,outname,outext,mvt):

    for l in range(2):
        fname = "%s-%f%s%s" % (outname[l], best_reg1, mvt[l], outext[l])
        fnamecp = "%s-%f-bestl1%s%s" % (outname[l], best_reg1, mvt[l], outext[l])
        call(shlex.split("cp %s %s" % (fname, fnamecp)))
        fname = "%s-%f%s%s" % (outname[l], best_reg2, mvt[l], outext[l])
        fnamecp = "%s-%f-bestl2%s%s" % (outname[l], best_reg2, mvt[l], outext[l])
        call(shlex.split("cp %s %s" % (fname, fnamecp)))
        fname = "%s-%f%s%s" % (outname[l], best_regh, mvt[l], outext[l])
        fnamecp = "%s-%f-bestsvr%s%s" % (outname[l], best_regh, mvt[l],outext[l])
        call(shlex.split("cp %s %s" % (fname, fnamecp)))
        fname = "%s-%f%s%s" % (outname[l], best_regcls, mvt[l], outext[l])
        fnamecp = "%s-%f-bestcls%s%s" % (outname[l], best_regcls, mvt[l], outext[l])
        call(shlex.split("cp %s %s" % (fname, fnamecp)))


if __name__ == '__main__':
    rospy.init_node('snake_trainer_node')

    learner = Trainer()

    regs = np.array([1.0])

    ds_test = Dataset()
    ds_train = Dataset()

    err_tol = 0.05  # use to compute support vector regression loss, and ''classification'' loss
    feature_weights = np.array([1.0])

    outname = []
    outext = []
    outnamey, outexty = os.path.splitext(learner.outfile_yaw)
    outnamep, outextp = os.path.splitext(learner.outfile_pitch)
    outname.append(outnamey)
    outname.append(outnamep)
    outext.append(outexty)
    outext.append(outextp)

    mvt = []
    mvt.append("yaw")
    mvt.append("pit")

    # train
    if learner.dataset_train != "":
        ds_train_orig = Dataset()
        if learner.quickload:
            pickle_file_id = open("quickload_data.dat", "r")
            ds_train_orig = pickle.load(pickle_file_id)
            pickle_file_id.close()
        else:
            ds_train_orig.load(learner.dataset_train, learner.path_bag, learner.pub_record)
            ds_train_orig.random_permute()
            pickle_file_id = open("quickload_data.dat", "w")
            pickle.dump(ds_train_orig, pickle_file_id)
            pickle_file_id.close()


        ds_train = ds_train_orig


    if learner.cv_fold <= 1:
        rospy.loginfo("[DAgger] No Cross-Validation: Training on a total of %d datapoints with %d features", ds_train.r,
                      ds_train.c)
        rospyloginfo(ds_train, err_tol)
        predictor.train(ds_train.X, ds_train.yaw, learner.outfile_yaw, regs, feature_weights,
                        learner.sample_weight_type)
        predictor.train(ds_train.X, ds_train.pitch, learner.outfile_pitch, regs, feature_weights,
                        learner.sample_weight_type)
        rospy.loginfo("[DAgger] Training complete")
    else:
        rospy.loginfo("[DAgger] Cross-Validation on %d fold of %d datapoints with %d features", learner.cv_fold,
                      ds_train.r,
                      ds_train.c)
        rospy.loginfo("[DAgger] Training on all %d data", ds_train_orig.r)
        predictor.train(ds_train.X, ds_train.yaw, learner.outfile_yaw, regs, feature_weights,
                        learner.sample_weight_type)
        predictor.train(ds_train.X, ds_train.pitch, learner.outfile_pitch, regs, feature_weights,
                        learner.sample_weight_type)
        for i in range(learner.cv_fold):
            idx_train = cv_idx_train(ds_train.r, learner.cv_fold, i + 1)
            fnamey = "%s-%d%s%s" % (outnamey, i, mvt[0],outexty)
            rospy.loginfo("[DAgger] Training without fold %d", i + 1)
            predictor.train(ds_train.X[idx_train, :], ds_train.yaw[idx_train], fnamey, regs, feature_weights,
                            learner.sample_weight_type)
            fnamep = "%s-%d%s%s" % (outnamep, i, mvt[1],outextp)
            predictor.train(ds_train.X[idx_train, :], ds_train.pitch[idx_train], fnamep, regs, feature_weights,
                            learner.sample_weight_type)


        (best_reg1, best_reg2, best_regh, best_regcls) = cross_validate(ds_train, regs, err_tol, learner.cv_fold,
                                                                        outname, outext,mvt)

        save_bestweights(best_reg1, best_reg2, best_regh, best_regcls,outname,outext,mvt)

    #test
    if learner.dataset_test != "":
        ds_test.load(learner.dataset_test, learner.path_bag, learner.pub_record)

        rospy.loginfo("[DAgger] Testing on a total of %d datapoints with %d features. Avg. absolute yaw %f , Avg. absolute pitch %f",
                      ds_test.r, ds_test.c, abs(ds_test.yaw).mean(), abs(ds_test.pitch).mean())
        rospyloginfo(ds_test, err_tol)
        (best_reg1, best_reg2, best_regh, best_regcls) = batch_test(ds_test, regs, err_tol,outname, outext,mvt)
