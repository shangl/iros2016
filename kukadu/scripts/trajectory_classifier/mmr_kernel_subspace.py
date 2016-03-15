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
## import pickle
## ###############################
import numpy as np
import scipy.linalg as sp_lin
## ###############################
from mmr_base_classes import cls_crossval, cls_kernel_params, cls_norm
from mmr_multic_label import mmr_multic_label
from mmr_kernel_eval import kernel_eval_kernel, kernel_eval_nl, \
     kernel_center, tanimoto
from mmr_normalization_new import mmr_normalization, mmr_geometricmedian_ker
## ###############################
class cls_feature:

  def __init__(self,ifeature=1):

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

    self.K=None           ## external training kernel
    self.Kcross=None      ## externel test kernel
    self.d1=None       ## norm of left factor of the kernel
    self.d2=None      ## norm of right factor of the kernel

    self.norm=cls_norm()
    self.crossval=cls_crossval()
    self.kernel_params=cls_kernel_params()
    self.prekernel_params=cls_kernel_params()

    self.ioperator_valued=0
    self.title=None
    self.kernel_computed=0

## -------------------------------------------------------------
  def load_data(self,dataraw,ifeature=1):

    self.dataraw=dataraw
    self.mdata=len(self.dataraw)
    if self.icategory==0:
      self.data=self.dataraw
    else:
      self.data=mmr_multic_label(self.cat2vector,self.xrawdata,None, \
                                    self.ncategory,None)
    self.ifeature=ifeature
## -------------------------------------------------------------
  def set_train_test(self,itrain,itest):

    self.itrain=itrain
    self.itest=itest

## -------------------------------------------------------------
  def get_train(self,itrain):

    return(self.data[itrain,:])

## -------------------------------------------------------------
  def get_test(self,itest):

    return(self.data[itest,:])
## --------------------------------------------------------------
  def get_train_norm(self,itrain):

    if self.XTrainNorm is None:
      (self.XTrainNorm,self.XTestNorm)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[self.itrain], \
                                self.data[self.itest],0)[:2]
    return(self.XTrainNorm)

## --------------------------------------------------------------
  def get_test_norm(self,itest):

    if self.XTestNorm is None:
      (self.XTrainNorm,self.XTestNorm)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[self.itrain], \
                                self.data[self.itest],0)[:2]

    return(self.XTestNorm)
## ---------------------------------------------------------------
  def compute_kernel(self,itrain,itest):

    (self.XTrainNorm,self.XTestNorm)= \
                    mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                      self.data[self.itrain], \
                                      self.data[self.itest],0)[:2]

    csubspace_kernel=cls_kernel_subspace(self.get_train(itrain), \
                                         None, \
                                         self.prekernel_params, \
                                         self.norm, \
                                         self.kernel_params)
    csubspace_kernel.compute_base_kernel_vectors()
    csubspace_kernel.compute_kernel_kernel_row(itrates=0)
    self.K=csubspace_kernel.kernel_row
    self.d1=np.diag(self.K)
    self.d2=self.d1
    ## csubspace_kernel.compute_kernel_kernel_row(itrates=1)
    ## self.Kcross=csubspace_kernel.kernel_row
    ## self.d1c=np.diag(self.Kcross)
    ## self.d2c=self.d1c

    ## print(self.kernel_params.ipar1,self.kernel_params.ipar2)
## ---------------------------------------------------------------
  def get_kernel(self,itrain,itest,itraintest=0):

    if itraintest==0:
      return(self.K,self.d1,self.d2)
    elif itraintest==1:
      return(self.Kcross,self.d1c,self.d2c)

## ---------------------------------------------------------------
  def copy(self,data=None):

    new_obj=cls_feature(self.ifeature)
    new_obj.icategory=self.icategory
    new_obj.ncategory=self.ncategory
    if data is not None:
      new_obj.load_data(data,ifeature=self.ifeature)
    else:
      new_obj.load_data(self.dataraw,ifeature=self.ifeature)
    new_obj.title=self.title
    new_obj.kernel_params=self.kernel_params
    if self.prekernel_params is not None:
      new_obj.prekernel_params=self.prekernel_params
    new_obj.crossval=self.crossval
    new_obj.norm=self.norm
    
    return(new_obj)
## #####################################################3 
## #######################################################################
class cls_kernel_subspace:
  """
  Computes the subspace kernel, see methods in this module as well

  xdata         2D array of \mathbb{B}^{m,n}
  param_col     column kernel parameters 
  param_row     kernel parameters
  """

  def __init__(self,xdata,xdata_test,param_col,norm_col,param_row):

    self.xdata=xdata                    ## data matrix
    if xdata_test is not None:
      self.xdata_test=xdata_test          ## data matrix
    else:
      self.xdata_test=xdata
    self.nrow=self.xdata.shape[0]            ## number of rows
    self.nrow_test=self.xdata_test.shape[0]  ## number of rows
    self.ncol=self.xdata.shape[1]            ## number of columns
    self.param_col=param_col            ## column kernel parameters
    self.norm_col=norm_col              ## column normalization params
    self.param_row=param_row            ## row kernel parameters
    self.kernel_col=None                ## column kernel
    self.kernel_row=None                ## row kernel training
    self.kernel_row_test=None                ## row kernel test
    self.base_vectors=None              ## column representing vectors

    return
  ## --------------------------------------

  def compute_base_vectors(self):

    ## column kernel
    xdata=self.xdata.T

    xdata=mmr_normalization( \
      self.norm_col.ilocal,self.norm_col.iscale, \
      xdata,np.array([]),0)[0]
    K=np.dot(xdata,xdata.T)
    
    Knl=kernel_eval_kernel((K,),None,None,self.param_col)[0]
    self.kernel_col=Knl
    ## normalize kernel items
    dK=np.sqrt(np.diag(self.kernel_col))
    idK=np.where(dK==0)[0]
    for i in idK:
      self.kernel_col[i,i]=1.0
      dK[i]=1.0
    self.kernel_col=self.kernel_col/np.outer(dK,dK)
    
    (v,w)=sp_lin.svd(self.kernel_col)[:2]

    self.base_vectors=v*np.outer(np.ones(self.ncol),np.sqrt(w))
    
    return

  ## --------------------------------------

  def compute_base_kernel_vectors(self):

    ## column kernel
    xdata0=np.copy(self.xdata)

    xdata=mmr_normalization( \
      self.norm_col.ilocal,-1, \
      xdata0,np.array([]),0)[0]
    xdata=xdata.T
    m=xdata.shape[0]
    xdata=mmr_normalization( \
      -1,self.norm_col.iscale, \
      xdata,np.array([]),0)[0]

    ikern_type=1          ## =0 linear, =1 cond prob
    if ikern_type==0:
      K=np.dot(xdata,xdata.T)
    elif ikern_type==1:
      K0=np.dot(xdata0.T,xdata0)+1
      K0=K0/m
      ## K=np.dot(K0,K0)/np.outer(np.diag(K0),np.diag(K0))
      K=np.dot(K0,K0)*np.outer(np.diag(K0),np.diag(K0))
      dK=np.sqrt(np.diag(K))
      K=K/np.outer(dK,dK)
    elif ikern_type==2:   # Tanimoto
      K=tanimoto(np.dot(xdata0.T,xdata0))

    Knl=kernel_eval_kernel((K,None),None,None,self.param_col)[0]

    icenter=0
    if icenter>=0:
      (Ka,aKa)=mmr_geometricmedian_ker(Knl)
      Ka1=np.outer(Ka,np.ones(m))
      Knl_center=Knl+aKa-Ka1-Ka1.T
      ineg=np.where(np.diag(Knl_center)<0)[0]
      for i in ineg:
        Knl_center[i,i]=0.0
      self.kernel_col=Knl_center
    else:
      self.kernel_col=Knl
      
    iscale=-1
    ## normalize kernel items
    if iscale>=0:
      dK=np.sqrt(np.diag(self.kernel_col))
      idK=np.where(dK==0)[0]
      for i in idK:
        self.kernel_col[i,i]=1.0
        dK[i]=1.0
      self.kernel_col=self.kernel_col/np.outer(dK,dK)

    if np.isnan(self.kernel_col).any()==True and \
       np.isfinite(self.kernel_col).all()!=True:
      print('!!!!!')
    
    return

  ## --------------------------------------

  def compute_kernel_row(self):

    K=np.zeros((self.nrow,self.nrow))

    drow={}  

    for i in range(self.nrow):
      iA=np.where(self.xdata[i]>0)[0]
      nsub=len(iA)
      if nsub>0:
        A=self.base_vectors[iA]
        AAT=self.kernel_col[iA.reshape((nsub,1)),iA]
        AATI=sp_lin.pinvh(AAT)
        drow[i]=(np.dot(AATI,A),iA)
      else:
        drow[i]=(None,np.array([]))

    for i in range(self.nrow):
      (BBTI,iB)=drow[i]
      if len(iB)>0:
        for j in range(i+1):
          (AATI,iA)=drow[j]
          ## = trace(Vhi.T,Vhj)=\braket{Vhi,Vhj}_{Frobenius}
          ## (BB^T)^{-1}BA^T(AA^T)^{-1}
          if len(iA)>0:
            BAT=self.kernel_col[iB.reshape((len(iB),1)),iA]
            ## xP=np.diag(np.dot(BBTI.T,np.dot(BAT,AATI)))
            xP=np.dot(BBTI,AATI.T)*BAT
            K[i,j]=np.sum(xP)  
            K[j,i]=K[i,j]
      ## if i%1000==0:
        ## print(i)
        
    d1=np.diag(K)
    d2=d1
    self.kernel_row=kernel_eval_nl(K,d1,d2,self.param_row)
    print('Modular kernel done')
    
    return

  ## --------------------------------------

  def compute_kernel_kernel_row(self,itrates=0):

    K=np.zeros((self.nrow,self.nrow_test))

    drow={}  

    for i in range(self.nrow):
      iA=np.where(self.xdata[i]>0)[0]
      nsub=len(iA)
      if nsub>0:
        AAT=self.kernel_col[iA.reshape((nsub,1)),iA]
        AATI=sp_lin.pinvh(AAT)
        drow[i]=(AATI,iA)
      else:
        drow[i]=(None,None)

    drow_test={}  
    if itrates==1:
      for i in range(self.nrow_test):
        iA=np.where(self.xdata_test[i]>0)[0]
        nsub=len(iA)
        if nsub>0:
          AAT=self.kernel_col[iA.reshape((nsub,1)),iA]
          AATI=sp_lin.pinvh(AAT)
          drow_test[i]=(AATI,iA)
        else:
          drow_test[i]=(None,None)
    else:
      drow_test=drow

    for i in range(self.nrow):
      (BBTI,iB)=drow[i]
      if iB is not None:
        if itrates==0:  ## to avoid double computation of symmetric kernel
          for j in range(i+1):
            (AATI,iA)=drow[j]
            if iA is not None:
              KBA=self.kernel_col[iB.reshape((len(iB),1)),iA]
              xP=np.dot(KBA.T,np.dot(BBTI,np.dot(KBA,AATI)))
              K[i,j]=np.sum(xP)  
              K[j,i]=K[i,j]
        elif itrates==1:
          for j in range(self.nrow_test):
            (AATI,iA)=drow_test[j]
            if iA is not None:
              KBA=self.kernel_col[iB.reshape((len(iB),1)),iA]
              xP=np.dot(KBA.T,np.dot(BBTI,np.dot(KBA,AATI)))
              K[i,j]=np.sum(xP)  
      ## if i%1000==0:
        ## print(i)

    dK=np.sqrt(np.diag(K))
    idK=np.where(dK==0)[0]
    for i in idK:
      K[i,i]=1.0
      dK[i]=1.0
    K=K/np.outer(dK,dK)
    
    d1=np.diag(K)
    if itrates==0:
      d2=d1
    elif itrates==1:
      d2=np.zeros(self.nrow_test)
      for i in range(self.nrow_test):
        (AATI,iA)=drow_test[i]
        if iA is not None:
          KBA=self.kernel_col[iA.reshape((len(iA),1)),iA]
          xP=np.dot(KBA.T,np.dot(AATI,np.dot(KBA,AATI)))
          d2[i]=np.sum(xP)  
      
    if itrates==0:
      self.kernel_row=kernel_eval_nl(K,d1,d2,self.param_row)
    elif itrates==1:
      self.kernel_row_test=kernel_eval_nl(K,d1,d2, \
                                                          self.param_row)
      
    self.kernel_row=kernel_center(self.kernel_row)
    dK=np.diag(self.kernel_row)
    idK=np.where(dK<0)[0]
    for i in idK:
      self.kernel_row[i,i]=0
      
    dK=np.sqrt(np.diag(self.kernel_row))
    idK=np.where(dK==0)[0]
    for i in idK:
      self.kernel_row[i,i]=1.0
      dK[i]=1.0
    self.kernel_row=self.kernel_row/np.outer(dK,dK)

    ## print('Modular kernel done')
    
    return

  ## --------------------------------------
        
