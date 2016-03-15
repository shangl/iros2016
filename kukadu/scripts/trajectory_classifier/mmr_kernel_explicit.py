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

import numpy as np
## ###########################################################
from mmr_base_classes import cls_crossval, cls_kernel_params, cls_norm
## from mmr_multic_label import mmr_multic_label
from mmr_normalization_new import mmr_normalization
from mmr_kernel_eval import kernel_eval_kernel, \
     kernel_localscale, gaussian_process_type_kernel

## class definitions

## ##################################################
class cls_feature:

  def __init__(self,ifeature=0):

    self.ifeature=ifeature       ## =0 explicit feature, =1 kernel 
    self.icategory=0      ## =0 vector =number of categories
    self.ncategory=0      ## idf icategory=1 it is the number of categories
    self.cat2vector=0     ## =0 indicator =1 mean 2 =median 3 =tetrahedron
    self.mdata=0
    self.itrain=None
    self.itest=None
    self.dataraw=None
    self.data=None       ## raw input
    self.XTrain=None      ## training features 
    self.XTrainNorm=None  ## normalized features
    self.XTest=None       ## test features 
    self.XTestNorm=None   ## normalized features
    self.Y0=None          ## set of distinc feature vectors     
    self.Y0Norm=None      ## set of distinc normalizedfeature vectors     

    self.K=None           ## external training kernel
    self.Kcross=None      ## externel test kernel
    self.Kraw=None
    self.Krawcross=None
    self.d1=None       ## norm of left factor of the kernel
    self.d2=None      ## norm of right factor of the kernel

    ## self.ilocal=2
    ## self.iscale=0

    self.norm=cls_norm()
    self.crossval=cls_crossval()
    self.kernel_params=cls_kernel_params()
    self.prekernel_params=cls_kernel_params()

    self.title=None

## -------------------------------------------------------------
  def load_data(self,dataraw,ifeature=0):

    self.dataraw=dataraw
    self.mdata=len(self.dataraw)
    self.data=self.dataraw
    self.ifeature=ifeature

## -------------------------------------------------------------
  def set_train_test(self,itrain,itest):

    self.itrain=itrain
    self.itest=itest
    self.Y0Norm=None

## -------------------------------------------------------------
  def get_train(self,itrain):

    return(self.data[itrain,:])

## -------------------------------------------------------------
  def get_test(self,itest):

    return(self.data[itest,:])
      
## ## -------------------------------------------------------------
  def get_Y0(self,itrain):

    if self.Y0 is None:
      return(self.data[itrain,:])
    else:
      return(self.Y0)

## ## -------------------------------------------------------------
  def get_Y0_norm(self,itrain):

    if self.Y0Norm is None:
      if self.Y0 is None:
        if self.XTrainNorm is None:
          self.XTrainNorm= \
                mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                  self.data[itrain],None,0)[0]
        Y0Norm=self.XTrainNorm
      else:
        Y0Norm=mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                  self.Y0,None,0)[0]
    ## if self.Y0Norm.shape[0]!=itrain.shape[0]:
    ##   print('!!!!',self.Y0Norm.shape[0],itrain.shape[0], \
    ##         self.itrain.shape[0],self.itest.shape[0])

    return(Y0Norm)

## --------------------------------------------------------------
  def get_train_norm(self,itrain):

    if self.XTrainNorm is None:
      (self.XTrainNorm,self.XTestNorm)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[self.itrain], \
                                self.data[self.itest],0)[:2]
    return(self.XTrainNorm)

## --------------------------------------------------------------
  def get_test_norm(self,itrain,itest):

    if self.XTestNorm is None:
      (self.XTrainNorm,self.XTestNorm)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[itrain], \
                                self.data[itest],0)[:2]

    return(self.XTestNorm)
## ---------------------------------------------------------------
  def compute_kernel(self,itrain,itest):

    if self.ifeature==0:  
      if self.icategory==0:
        (self.XTrainNorm,self.XTestNorm)= \
                      mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                        self.data[itrain], \
                                        self.data[itest],0)[:2]

        if self.Y0 is not None:
          self.Y0Norm=mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                        self.Y0,None,0)[0]
          
        if self.prekernel_params.kernel_type in (0,1,2,3):
          xdata=[self.XTrainNorm,None]
          (self.K,self.d1,self.d2)=kernel_eval_kernel(xdata,None,None, \
                                          self.kernel_params)

          if self.Y0 is  None:
            if len(itest)>0:
              xdata=[self.XTrainNorm,self.XTestNorm]
              (self.Kcross,self.d1c,self.d2c)=kernel_eval_kernel(xdata,None, \
                                                               None, \
                                          self.kernel_params)
          else:
            xdata=[self.XTrainNorm,self.Y0Norm]
            (self.Kcross,self.d1c,self.d2c)=kernel_eval_kernel(xdata,None, \
                                                               None, \
                                          self.kernel_params)
        elif self.prekernel_params.kernel_type in (5,):
          (m,n)=self.data.shape
          datanorm=np.zeros((m,n))
          datanorm[itrain]=self.XTrainNorm
          datanorm[itest]=self.XTestNorm
          K=gaussian_process_type_kernel(datanorm,self.prekernel_params.ipar1)
          (self.K,self.Kcross,self.d1,self.d2,self.d1c,self.d2c)= \
              kernel_localscale(K,None,itrain,itest,self.kernel_params, \
                             ilocal=self.norm.ilocal,iscale=self.norm.iscale)
        
          
    else: ## implicit feature given by kernel
      if self.Kraw is not None:  ## Tanimoto
        kdata=self.Kraw
        kdataY0=self.Krawcross
      else:
        kdata=self.data
        kdataY0=None
      (self.K,self.Kcross,self.d1,self.d2,self.d1c,self.d2c)= \
              kernel_localscale(kdata,kdataY0,itrain,itest, \
                                self.kernel_params, \
                             ilocal=self.norm.ilocal,iscale=self.norm.iscale)
      
    return
## ---------------------------------------------------------------
  def get_kernel(self,itrain,itest,ioutput=0,itraintest=0,itraindata=1):

    if ioutput==0:
      if itraintest==1:
        if itraindata==0:     ## test on the training
          return(self.K,self.d1,self.d2)
        else:
          return(self.Kcross,self.d1c,self.d2c)
      else:
        return(self.K,self.d1,self.d2)
    elif ioutput==1:
      if itraintest==1:
        if itraindata==0:
          return(self.K,self.d1,self.d2)
        else:  
          return(self.Kcross,self.d1,self.d2)
      else:  
        return(self.K,self.d1,self.d2)
        

## ---------------------------------------------------------------
  def copy(self,data=None):
    
    new_obj=cls_feature(self.ifeature)
    new_obj.icategory=self.icategory
    new_obj.ncategory=self.ncategory
    if data is not None:
      new_obj.load_data(data,ifeature=self.ifeature)
    else:
      new_obj.load_data(self.dataraw,ifeature=self.ifeature)
    if self.Kraw is not None:
      ntrain=len(self.itrain)
      new_obj.Kraw=self.Kraw[self.itrain.reshape((ntrain,1)),self.itrain]
    if self.Y0 is not None:
      new_obj.Y0=self.Y0
    if self.Krawcross is not None:
      ntrain=len(self.itrain)
      new_obj.Krawcross=self.Krawcross[self.itrain,:]
      
    new_obj.title=self.title
    new_obj.kernel_params=self.kernel_params
    if self.prekernel_params is not None:
      new_obj.prekernel_params=self.prekernel_params
    new_obj.crossval=self.crossval
    new_obj.norm=self.norm
    
    return(new_obj)
  ## #####################################################3 
