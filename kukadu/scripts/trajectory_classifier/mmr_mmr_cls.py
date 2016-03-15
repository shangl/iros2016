######################
## Version 0.1 #######
## /**********************************************************************
##   Copyright 2015, Sandor Szedmak  
##   email: sandor.szedmak@uibk.ac.at
##          szedmak777@gmail.com
##
##   This file is part of Maximum Margin Multi-valued Regression code(MMMVR).
##
##   MMMVR is free software: you can redistribute it and/or modify
##   it under the terms of the GNU General Public License as published by
##   the Free Software Foundation, either version 3 of the License, or
##   (at your option) any later version. 
##
##   MMMVR is distributed in the hope that it will be useful,
##   but WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##   GNU General Public License for more details.
##
##   You should have received a copy of the GNU General Public License
##   along with MMMVR.  If not, see <http://www.gnu.org/licenses/>.
##
## ***********************************************************************/
######################

## import sys
import numpy as np
## ###########################################################
## from mmr_normalization_new import mmr_normalization
import mmr_base_classes as base
from mmr_kernel import mmr_kernel 
## from mmr_kernel_eval import kernel_operator_valued
import mmr_solver_cls
from mmr_tools import mmr_perceptron_primal, mmr_perceptron_dual
## from mmr_kernel_explicit import cls_feature

## class definitions
## ##################################################
class cls_mmr(base.cls_data):

  def __init__(self,ninputview):
    base.cls_data.__init__(self,ninputview)

    self.XKernel=[None]*ninputview
    self.YKernel=None
    self.KX=None
    self.KXCross=None
    self.KY=None
    self.d1x=None
    self.d2x=None
    self.d1x=None

    self.penalty=base.cls_penalty() ## setting C,D penelty term paramters
    self.penalty.set_crossval()
    ## perceptron
    self.iperceptron=0
    self.perceptron=base.cls_perceptron_param()
    ## other classes
    self.dual=None

    ## self.xbias=-0.6
    self.xbias=0.0

    self.kmode=1   ## =0 additive (feature concatenation)
                    ## =1 multiplicative (feature tensor product)

    self.ifixtrain=None
    self.ifixtest=None

    self.crossval_mode=0   ## =0 random cross folds =1 fixtraining
    ## itestmode can be 2 if YKernel is linear !!!
    self.itestmode=0        ## 2 against the training with knn, 10 vectorwise
                            ## 20 Y0 is available
    ## ##################################################
    ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    self.nrepeat=5000
    self.nfold=5
    ## ##################################################
    self.testknn=5

    self.ieval_type=0   
    self.mdata=0
    
## ---------------------------------------------------------
  def set_validation(self):

    self.dkernels={}
    self.dkernels[self.YKernel.title]=self.YKernel
    nkernel=len(self.XKernel)
    for ikernel in range(nkernel):
      self.dkernels[self.XKernel[ikernel].title]=self.XKernel[ikernel]  

## ---------------------------------------------------------
  def split_train_test(self,xmask,ifold):

    itrain=np.where(np.logical_and((xmask!=ifold),(xmask!=-1)))[0]
    itest=np.where(xmask==ifold)[0]

    self.itrain=itrain
    self.itest=itest
    self.mtrain=len(itrain)
    self.mtest=len(itest)

    for ikernel in range(len(self.XKernel)):
      self.XKernel[ikernel].set_train_test(itrain,itest)

    self.YKernel.set_train_test(itrain,itest)
## ---------------------------------------------------------
  def compute_kernels(self):

    ## print('Kernel computation')
    self.YKernel.compute_kernel(self.itrain,self.itest)
    nkernel=len(self.XKernel)
    for ikernel in range(nkernel):
      self.XKernel[ikernel].compute_kernel(self.itrain,self.itest)

## ---------------------------------------------------------
  def mmr_train(self):

    if self.iperceptron==0:
      mtra=self.mtrain

      KX=mmr_kernel(self,self.itrain,self.itrain,ioutput=0, \
                            itraintest=0,itraindata=0,itensor=self.kmode)[0]
      KY=mmr_kernel(self,self.itrain,self.itrain,ioutput=1, \
                            itraintest=0,itraindata=0)[0]

      cOptDual=base.cls_dual(None,None)
      self.dual=cOptDual
      self.csolver=mmr_solver_cls.cls_mmr_solver()
      self.dual.alpha=self.csolver.mmr_solver(KX,KY,self.penalty.c, \
                                              self.penalty.d)
    ## estimate the bias for linear output kernel

      if self.YKernel.ifeature==0:
        YTrain=self.YKernel.get_train_norm(self.itrain)
        ZW=np.dot(YTrain.T, \
                  (np.outer(self.dual.alpha,np.ones(mtra))*KX))
        xbias=np.median(YTrain-ZW.T,axis=0)
      else:
        KY=self.YKernel.Kcross
        ZW=np.dot(KY.T,(np.outer(self.dual.alpha,np.ones(mtra))*KX))
        xbias=np.zeros(mtra)

      self.dual.bias=xbias  
    elif self.iperceptron==1:
      cOptDual=base.cls_dual(None,None)
      XTrainNorm0=self.XKernel[0].get_train_norm()
      YTrainNorm=self.YKernel.get_train_norm(self.itrain)
      cOptDual.W=mmr_perceptron_primal(XTrainNorm0,YTrainNorm, \
                                       self.perceptron.margin, \
                                       self.perceptron.stepsize, \
                                       self.perceptron.nrepeat)      
    elif self.iperceptron==2:
      cOptDual=base.cls_dual(None,None)
      XTrainNorm0=self.XKernel[0].get_train_norm()
      YTrainNorm=self.YKernel.get_train_norm(self.itrain)
      cOptDual.alpha=mmr_perceptron_dual(XTrainNorm0,YTrainNorm, \
                                       self.XKernel[0].kernel_params.ipar1, \
                                       self.XKernel[0].kernel_params.ipar2, \
                                       self.XKernel[0].kernel_type, \
                                       self.perceptron.margin, \
                                       self.perceptron.stepsize, \
                                       self.perceptron.nrepeat)      

    return(cOptDual)

## ------------------------------------------------------------------
  def mmr_test(self,cOptDual,itraindata=1):

    if self.YKernel.kernel_params.kernel_type>0:
      self.itestmode=0
      
    if itraindata==0:
      itest=self.itrain
    else:
      itest=self.itest
    if self.iperceptron in (0,2):
      KXcross=mmr_kernel(self,self.itrain,itest,ioutput=0, \
                                 itraintest=1, itraindata=itraindata, \
                                 itensor=self.kmode)[0]
      KYcross=mmr_kernel(self,self.itrain,self.itrain,ioutput=1, \
                                 itraintest=1,itraindata=itraindata)[0]
      mtest=KXcross.shape[1]
      
      cPredict=base.cls_predict()

      if self.YKernel.ifeature==0:
        if self.YKernel.kernel_params.kernel_type==0:
          YTrain=self.YKernel.get_train_norm(self.itrain)
          Zw0=np.dot(YTrain.T,(np.outer(cOptDual.alpha, \
                                      np.ones(mtest))*KXcross))
          if self.csolver.ibias_estim==1:
            Zw0=Zw0+np.outer(cOptDual.bias,np.ones(mtest))

          xnorm=np.sqrt(np.sum(Zw0**2,axis=0))
          xnorm=xnorm+(xnorm==0)
          Zw=Zw0/np.outer(np.ones(Zw0.shape[0]),xnorm)
          Y0=self.YKernel.get_Y0_norm(self.itrain) 
          cPredict.ZTest=np.dot(Y0,Zw)
        else:
          Zw0=np.dot(KYcross.T, \
                     (np.outer(cOptDual.alpha,np.ones(mtest))*KXcross))
          cPredict.ZTest=Zw0
          if self.csolver.ibias_estim==1:
            cPredict.bias=cOptDual.bias
          else:
            cPredict.bias=0
          Zw=Zw0
        cPredict.Zw0=Zw0.T
        cPredict.Zw=Zw.T

        if self.itestmode in (0,2,3,4):
          Y0=self.YKernel.get_Y0(self.itrain)
          cPredict.zPremax=cPredict.ZTest.max(0)
          cPredict.iPredCat=cPredict.ZTest.argmax(0)
          cPredict.zPred=Y0[cPredict.iPredCat]
          if self.itestmode==2 and self.YKernel.kernel_params.kernel_type==0:
            cPredict.knnPredCat=np.argsort \
                              (-cPredict.ZTest,axis=0)[:self.testknn,:]
          elif self.itestmode==3:
            n=YTrain.shape[1]
            nsort=5
            for i in range(mtest):
              cPredict.zPred[i,:]=np.zeros(n)
              iones=np.argsort(-Zw[:,i])[:nsort]
              for j in range(nsort-1):
                if Zw[iones[j],i]<0.04 or \
                     Zw[iones[j],i]>10.0*Zw[iones[j+1],i]:
                  iones=iones[:(j+1)]
                  break
              cPredict.zPred[i,iones]=1
            
        elif self.itestmode==1:
          YTrain=self.YKernel.get_train(self.itrain)
          ndim=YTrain.shape[1]
          i2boro=np.argsort(-Zw0,axis=0)[0:self.testknn,:].T
          cPredict.zPred=np.zeros((mtest,ndim))
          for i in range(mtest):
            zsum=np.zeros(ndim)
            for j in range(self.testknn):
              zsum+=YTrain[i2boro[i,j]]
            cPredict.zPred[i,:]=1*(zsum>=self.testknn/2)

      elif self.YKernel.ifeature==1:
        Y0=self.YKernel.get_Y0(self.itrain)
        cPredict.ZTest=np.dot(KYcross.T,(np.outer(cOptDual.alpha, \
                                      np.ones(mtest))*KXcross))
        cPredict.zPremax=cPredict.ZTest.max(0)
        cPredict.iPredCat=cPredict.ZTest.argmax(0)
        cPredict.zPred=Y0[cPredict.iPredCat]
        if self.itestmode==2:
          cPredict.knnPredCat=np.argsort \
                               (-cPredict.ZTest,axis=0)[:self.testknn,:]
    elif self.iperceptron==1:
      if itraindata==0:
        X=self.XKernel[0].get_train_norm(self.itrain)
        for i in range(1,len(self.XKernel)):
          X=np.hstack((X,self.XKernel[i].get_train_norm(self.itrain)))
      else:
        X=self.XKernel[0].get_test_norm(itest)
        for i in range(1,len(self.XKernel)):
          X=np.hstack((X,self.XKernel[i].get_test_norm(itest)))
      cPredict=base.cls_predict()
      xmean=np.mean(X)
      mtest=X.shape[0]
      Zw=np.dot(cOptDual.W, \
                 np.hstack((X,np.ones((mtest,1))*xmean)).T)
      cPredict.ZTest=np.dot(YTrain,Zw)
      cPredict.zPremax=cPredict.ZTest.max(0)
      cPredict.iPredCat=cPredict.ZTest.argmax(0)
      cPredict.zPred=YTrain[cPredict.iPredCat]
      if self.itestmode==2:
        cPredict.knnPredCat=np.argsort(-cPredict.ZTest, \
                                       axis=0)[:self.testknn,:]
    
    return(cPredict)
## ------------------------------------------------------------------
  def prepare_repetition_folding(self):

    return
## ------------------------------------------------------------------
  def prepare_repetition_training(self,nfold0):

    ## split data into training and test
    if self.crossval_mode==0:  ## random selection
      self.xselector=np.zeros(self.mdata)
      ifold=0
      for i in range(self.mdata):
        self.xselector[i]=ifold
        ifold+=1
        if ifold>=nfold0:
          ifold=0
      np.random.shuffle(self.xselector)
      ## xselector=np.floor(np.random.random(self.mdata)*nfold0)
      ## xselector=xselector-(xselector==nfold0)
    elif self.crossval_mode==1: ## preddefined training and test
      self.xselector=np.zeros(self.mdata)
      self.xselector[self.ifixtrain]=1

    return
## ------------------------------------------------------------------
  def prepare_fold_training(self,ifold):

    self.split_train_test(self.xselector,ifold)
  
## ------------------------------------------------------------------
  def glm_norm_in(self,X):

    (m,n)=X.shape
    self.colmean=np.mean(X,axis=0)
    self.rowmean=np.mean(X,axis=1)
    self.totalmean=np.mean(self.colmean)
    
    Xres=X-np.outer(np.ones(m),self.colmean) \
          -np.outer(self.rowmean,np.ones(n))+self.totalmean

    return(Xres)
## ------------------------------------------------------------------
  def glm_norm_out(self,Xres):

    (m,n)=Xres.shape
    X=Xres+np.outer(np.ones(m),self.colmean) \
          +np.outer(self.rowmean,np.ones(n))-self.totalmean

    return(X)
## ------------------------------------------------------------------
  def copy(self,new_obj,itrain):

    nkernel=len(self.XKernel)
    for ikernel in range(nkernel):
      xdata=self.XKernel[ikernel].get_train(self.itrain)
      new_obj.XKernel[ikernel]=self.XKernel[ikernel].copy(xdata) 
    xdata=self.YKernel.get_train(self.itrain)
    new_obj.YKernel=self.YKernel.copy(xdata)
    new_obj.set_validation()

    new_obj.penalty=base.cls_penalty()
    new_obj.penalty.c=self.penalty.c
    new_obj.penalty.d=self.penalty.d
    new_obj.penalty.crossval=self.penalty.crossval

    new_obj.mdata=len(itrain)
    new_obj.itestmode=self.itestmode
    new_obj.testknn=self.testknn
    new_obj.kmode=self.kmode

    new_obj.xbias=self.xbias
    
## #######################################################################
  

    
