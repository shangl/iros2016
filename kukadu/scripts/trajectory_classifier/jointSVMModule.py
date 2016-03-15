######################
## Version 0.1 #######
######################

import sys, time
## import pickle
import numpy as np
## import scipy.io 
## import pylab as plt

## ##########################################
import mmr_base_classes
import mmr_setparams
import mmr_mmr_cls
import mmr_solver_cls
## from mmr_load_data_state import load_data_state
import armar_load_data
## import mmr_validation_cls
from mmr_kernel import mmr_kernel
## from mmr_train import mmr_train
from mmr_test import inverse_knn
## from mmr_report import mmr_report
from mmr_eval import mmr_eval_binvector
from mmr_kernel_eval import kernel_eval_kernel
## from mmr_tools import mmr_cross_kernel
## from mmr_normalization_new import mmr_normalization
## ---------------------------------
## #################################
##

cMMR=None

def loadData(trainOutput,trainInput,dualParams,kernelParams):
  global cMMR
  
  params=mmr_setparams.cls_params()
  nview=1
  params.ninputview=nview
  cMMR=mmr_mmr_cls.cls_mmr(params.ninputview)
  nfold=cMMR.nfold
  nrepeat=cMMR.nrepeat

  ## @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  cdata_store=armar_load_data.cls_label_files(trainOutput,trainInput, \
                                              dualParams,kernelParams)  
  cdata_store.load_mmr(cMMR)
  mdata=cMMR.mdata
  ## ############################################################
  xselector=np.ones(mdata)
  xselector[-1]=0
  ifold=0
  cMMR.split_train_test(xselector,ifold)
  cMMR.compute_kernels()
  ## cMMR.Y0=cMMR.YKernel.get_train(cMMR.itrain)   ## candidates
  cMMR.csolver=mmr_solver_cls.cls_mmr_solver()


def prediction(testInput):
  global cMMR

  testInput=np.array(testInput)

  if len(testInput.shape)==1:
    testInput=testInput.reshape(1,len(testInput))
  (mtest,ntest)=testInput.shape
  testInput-=np.outer(np.ones(mtest),cMMR.training_center)
  xnorm=np.sqrt(np.sum(testInput**2))
  xnorm+=(xnorm==0)
  testInput=testInput/np.outer(xnorm,np.ones(ntest))
  xdata=[cMMR.XKernel[0].XTrainNorm,testInput]
  (cMMR.XKernel[0].Kcross,cMMR.XKernel[0].d1c,cMMR.XKernel[0].d2c)= \
             kernel_eval_kernel(xdata,None,None,cMMR.XKernel[0].kernel_params)
  
  ## ######################################     
  ## check the test accuracy
  t0=time.clock()
  cPredictTes= cMMR.mmr_test(cMMR.dual,itraindata=1)
  ## counts the proportion the ones predicted correctly
  if cMMR.itestmode==2:
    ypred=inverse_knn(cMMR.YKernel.get_Y0(cMMR.itrain), \
                      cPredictTes)
    yconf=cPredictTes.zPredconf
    
  else:
    ypred=cPredictTes.zPred
  ## cEvaluationTes=mmr_eval_binvector(cMMR.YKernel.get_test(cMMR.itest),ypred)

  return(list(ypred),list(yconf))
  
## ################################################################
if __name__ == "__main__":

  import scipy.io

  sdir='data/'

  ## rawY=scipy.io.loadmat(sdir+'annot.mat')
  ## ytrain=rawY['xtrain'][:500]
  ## ytest=rawY['xtest'][:100]
  ## ## rawY=scipy.io.loadmat(sdir+'DenseHue.mat')
  ## rawX=scipy.io.loadmat(sdir+'DenseSiftV3H1.mat')
  ## xtrain=rawX['xtrain'][:500]
  ## xtest=rawX['xtest'][:100]

  ## mtrain=xtrain.shape[0]

  ftrainOutput='data/trainOutput.txt'
  ftrainInput='data/trainInput.txt'
  ftestInput='data/testInput.txt'
  ## kernelParams=(0.5,0)

  ## np.savetxt(ftrainInput,xtrain,fmt='%9.4f')
  ## np.savetxt(ftrainOutput,ytrain,fmt='%9.4f')
  ## np.savetxt(ftestInput,xtest,fmt='%9.4f')

  ## dualalpha=np.ones(mtrain)
  fdualParams='data/dualParams.txt'
  fkernelParams='data/kernelParams.txt'

  
  ## np.savetxt(fdualParams,dualalpha,fmt='%6.4f')


  loadData(ftrainOutput,ftrainInput,fdualParams,fkernelParams)

  xtestInput=np.loadtxt(ftestInput)

  (ypredict,yconf)=prediction(xtestInput)

  print('Bye')
