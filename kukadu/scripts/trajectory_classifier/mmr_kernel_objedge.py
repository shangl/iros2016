######################
## Version 0.1 #######
######################
## import pickle
## ###############################
import numpy as np
import scipy.linalg as sp_lin
## ###############################
from mmr_base_classes import cls_crossval, cls_kernel_params, cls_norm
from mmr_multic_label import mmr_multic_label
from mmr_kernel_eval import kernel_eval_kernel, kernel_eval_nl, kernel_center
from mmr_normalization_new import mmr_normalization, mmr_geometricmedian_ker
import mmr_solver_cls
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
  def load_data(self,dataraw):

    self.dataraw=dataraw
    self.mdata=len(self.dataraw)
    if self.icategory==0:
      self.data=self.dataraw
    else:
      self.data=mmr_multic_label(self.cat2vector,self.xrawdata,None, \
                                    self.ncategory,None)
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
      (self.XTrainNorm,self.XTestNorm,opar)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[self.itrain], \
                                self.data[self.itest],0)
    return(self.XTrainNorm)

## --------------------------------------------------------------
  def get_test_norm(self,itest):

    if self.XTestNorm is None:
      (self.XTrainNorm,self.XTestNorm,opar)= \
              mmr_normalization(self.norm.ilocal,self.norm.iscale, \
                                self.data[self.itrain], \
                                self.data[self.itest],0)

    return(self.XTestNorm)
## ---------------------------------------------------------------
  def compute_kernel(self,itrain,itest):

    ## (self.XTrainNorm,self.XTestNorm,opar)= \
    ##                 mmr_normalization(self.ilocal,self.iscale, \
    ##                                   self.data[self.itrain], \
    ##                                   self.data[self.itest],0)

    cobjectedge_kernel=cls_kernel_objectedge(self.norm)
    cobjectedge_kernel.edge_prekernel(self.dataraw,self.prekernel_params)
    (K,d1,d2)=cobjectedge_kernel.prekernel(itrain,itrain,self.kernel_params)
    self.K=K
    self.d1=d1
    self.d2=d2
    (Kcross,d1c,d2c)=cobjectedge_kernel.prekernel(itrain,itest, \
                                                  self.kernel_params)
    self.Kcross=Kcross
    self.d1c=d1c
    self.d2c=d2c

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
    if data is not None:
      new_obj.load_data(data)
    else:
      new_obj.load_data(self.dataraw)
    new_obj.title=self.title
    new_obj.kernel_params=self.kernel_params
    if self.prekernel_params is not None:
      new_obj.prekernel_params=self.prekernel_params
    new_obj.crossval=self.crossval
    new_obj.norm=self.norm
    
    return(new_obj)
## ###################################################### 
## ###################################################### 
class cls_kernel_objectedge:

  def __init__(self,norm):

    self.dalpha={}
    self.dxy={}
    self.ddiag=None
    self.norm=norm

    
## ----------------------------------------
  def edge_prekernel(self,X,params_spec):

    (m,n)=X.shape   ## xdataraw[0]
    dalpha={}
    dxy={}
    ialpha=1
    ny=2
    nitem=3

    self.ddiag=np.zeros(m)
    csolver=mmr_solver_cls.cls_mmr_solver()

    ## read the edges
    for iview in range(m):
      xy=X[iview]
      nxy=len(xy)//nitem
      xy=xy[:nxy*nitem]
      xy=xy.reshape((nxy,nitem))
      ixy=np.where(xy[:,0]>0)[0]
      xy=xy[ixy]
      x=xy[:,:2]  ## edge position
      ya=xy[:,2]   ## edge angle

      y=np.vstack((np.cos(ya),np.sin(ya))).T
      ## normalize the locations
      ilocal=self.norm.ilocal
      iscale=self.norm.iscale
      (xnorm,xdummy,opar)=mmr_normalization(ilocal,iscale,x,None,0)
      ## xnorm=x
      self.dxy[iview]=[xnorm,y]
      if ialpha==1:
        Ky=np.dot(y,y.T)
        xdata=[xnorm,None]
        (Kx,dx1,dx2)=kernel_eval_kernel(xdata,None,None,params_spec)
        C=1
        D=0
        xalpha=csolver.mmr_solver(Kx,Ky,C,D,1,1,1,1)
        self.dalpha[iview]=xalpha
      else:
        xalpha=np.ones(len(y))/len(y)
        self.dalpha[iview]=xalpha
      
      self.ddiag[iview]=np.dot(xalpha,np.dot(Kx*Ky,xalpha))

    return
## ----------------------------------------
  def prekernel(self,itrain,itest,params_spec):

    mtrain=len(itrain)
    mtest=len(itest)
    K=np.zeros((mtrain,mtest))
    
    for i0 in range(mtrain):
      i=itrain[i0]
      for j0 in range(mtest):
        j=itest[j0]
        xdata=[self.dxy[i][0],self.dxy[j][0]]
        (Kx,dx1,dx2)=kernel_eval_kernel(xdata,None,None,params_spec)
        Ky=np.dot(self.dxy[i][1],self.dxy[j][1].T)
        K[i0,j0]=np.dot(self.dalpha[i],np.dot((Ky*Kx),self.dalpha[j]))
        
    d1=self.ddiag[itrain]
    d2=self.ddiag[itest]
    
    return(K,d1,d2)
## #################################
        
