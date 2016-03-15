######################
## Version 0.1 #######
######################
## import pickle, os
## ###############################
import numpy as np
import scipy.io
## import networkx as nx
## import numpy.linalg as np_lin
## import pylab as lab
## import mpl_toolkits.mplot3d.axes3d as p3
## import mpl_toolkits.mplot3d.art3d as art3d
## ################################
import mmr_kernel_explicit
import mmr_initial_params
## ################################
class cls_data_load:

  def __init__(self):
    """
    """
    self.basedir='data/'
    ## first feature contains the labels
    self.list_features=["annot","DenseHue","DenseHueV3H1", \
                "DenseSift","DenseSiftV3H1","Gist", \
                "HarrisHue","HarrisHueV3H1","HarrisSift", \
                "HarrisSiftV3H1","Hsv","HsvV3H1","Lab", \
                "LabV3H1","Rgb","RgbV3H1"]

    ## data files in the corresponding directories
    self.datadirs=['corel5k','espgame','iaprtc12','mirflickr','pascal07']

    ##
    self.ncut=200    ## maximum size of randomly chosen subset 
    return

  ## --------------------------------------------------------------

  def load_data(self,cData,idata,lfeatures):
    """
    cData       refers to an mmr class object, created by mmr_mmr_cls
    idata       index of the chosen database from self.datadirs
    lfeatures   list of indexes of the selected features,
                the 'annot' the labels always implicitly selected

                the data files assumed to be matlab files
    """

    ## read the outputs, list_feature[0]=annot'
    dmat=scipy.io.loadmat(self.basedir+self.datadirs[idata]+'/' \
                          +self.list_features[0]+'.mat')
    ytrain=dmat['xtrain']
    ytest=dmat['xtest']
    print('ytrain',ytrain.shape)
    print('ytest',ytest.shape)

    ytrain=1*(ytrain>0)   ## label indicators are 1 and 0
    ytest=1*(ytest>0)
    mtrain=ytrain.shape[0]
    mtest=ytest.shape[0]

    ## select random subsample of training
    if self.ncut<mtrain:
      itrainsample=np.random.permutation(mtrain)
      itrainsample=itrainsample[:self.ncut]
    else:
      itrainsample=np.arange(mtrain)

    ## select random subsample of test
    if self.ncut<mtest:
      itestsample=np.random.permutation(mtest)
      itestsample=itestsample[:self.ncut]
    else:
      itestsample=np.arange(mtest)

    ## store the indexes of the subtraining and subtest in the MMR object
    cData.ifixtrain=np.arange(len(itrainsample))
    cData.ifixtest=np.arange(len(itestsample))

    ## we can concatenate now the training and test since ifixtrain
    ## and ifixtest will select them
    ydata=np.vstack((ytrain[itrainsample],ytest[itestsample]))
    ## subspace output kernel
    cData.YKernel=mmr_kernel_explicit.cls_feature(ifeature=0)
    cData.YKernel.load_data(ydata,ifeature=0)
    cData.YKernel.ifeature=0
    cData.YKernel.title='output'
    ## setting output parameters
    cparams=mmr_initial_params.cls_initial_params()
    ## kernel parameters
    cData.YKernel.kernel_params.set(cparams.get_yparams('kernel',0))
    ## cross validation ranges
    cData.YKernel.crossval.set(cparams.get_yparams('cross',0))
    ## normalization, localization and scaling type
    cData.YKernel.norm.set(cparams.get_yparams('norm',0))
    
    ## process the input features 
    iview=0
    for ifeature in lfeatures:
      if ifeature==0:   ## if annot(labels) included drop it!
        continue        
      dmat=scipy.io.loadmat(self.basedir+self.datadirs[idata]+'/' \
                          +self.list_features[ifeature]+'.mat')
      xtrain=dmat['xtrain']
      xtest=dmat['xtest']

      print('xtrain',xtrain.shape)
      print('xtest',xtest.shape)

      xdata=np.vstack((xtrain[itrainsample], \
                                    xtest[itestsample])).astype(np.double)
      print('>>>>>>>',self.datadirs[idata],'ifeature:', \
            self.list_features[ifeature], \
            len(itrainsample),len(itestsample))
      cData.XKernel[iview]=mmr_kernel_explicit.cls_feature(ifeature=0)
      cData.XKernel[iview].load_data(xdata,ifeature=0)
      cData.XKernel[iview].title='input_'+str(ifeature)

      ## setting input kernel parameters
      cData.XKernel[iview].kernel_params.set(cparams. \
                                             get_xparams('kernel',iview))
      ## cross validation ranges
      cData.XKernel[iview].crossval.set(cparams.get_xparams('cross',iview))
      ## normalization, localization and scaling type
      cData.XKernel[iview].norm.set(cparams.get_xparams('norm',iview))
      iview+=1

    cData.ninputview=iview  ## set the number of active views
    cData.mdata=cData.YKernel.dataraw.shape[0]


    return
## ##################################################################3

