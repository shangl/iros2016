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
import time

import numpy as np
## import numpy.linalg as np_lin
## ####################
import mmr_base_classes
## from mmr_kernel_eval import kernel_operator_valued
## #######################################################################
def inverse_knn(Y0,cPredict):

  (k,mtest)=cPredict.knnPredCat.shape
  n=Y0.shape[1]
  Ypred=np.zeros((mtest,n))

  ## p=np.ones(k)/k
  for i in range(mtest):
    inear=cPredict.knnPredCat[:,i]
    Z=Y0[inear]
    ## IZZT=np_lin.pinv(np.dot(Z,Z.T))
    qZ=cPredict.ZTest[inear,i]
    ## qZ=np.dot(IZZT,pZ)    ##  contravariant coordinates
    ineg=np.where(qZ<0)[0]
    qZ[ineg]=0
    sqZ=np.sum(qZ)
    if sqZ>0:
      qZ=qZ/np.sum(qZ)
    else:
      qZ=np.ones(k)/k
    sZ=np.dot(qZ,Z)
    Ypred[i,:]=1*(sZ>=0.5)

  return(Ypred)
## #######################################################################
def inverse_knn_real(Y0,cPredict):

  (k,mtest)=cPredict.knnPredCat.shape
  n=Y0.shape[1]
  Ypred=np.zeros((mtest,n))

  ## p=np.ones(k)/k
  for i in range(mtest):
    inear=cPredict.knnPredCat[:,i]
    Z=Y0[inear]
    ## IZZT=np_lin.pinv(np.dot(Z,Z.T))
    qZ=cPredict.ZTest[inear,i]
    ## qZ=np.dot(IZZT,pZ)    ##  contravariant coordinates
    ineg=np.where(qZ<0)[0]
    qZ[ineg]=0
    sqZ=np.sum(qZ)
    if sqZ>0:
      qZ=qZ/np.sum(qZ)
    else:
      qZ=np.ones(k)/k
    sZ=np.dot(qZ,Z)
    ## Ypred[i,:]=1*(sZ>=0.5)
    Ypred[i,:]=sZ

  return(Ypred)
## ###########################################################
def test_dynamic_prog(cDataTest,cOptDual):

  (mtrain,mtest)=cDataTest.KXcross.shape
  ndim=cDataTest.YTrainNorm.shape[1]

  cPredict=mmr_base_classes.cls_predict()
  cPredict.zPred=np.zeros((mtest,ndim))

  beta=cDataTest.KXcross*np.outer(cOptDual.alpha,np.ones(mtest))
  beta=beta.T

  ## inverse covariance
  ## ymean=np.mean(cDataTest.YTrainNorm,axis=0)
  Y=cDataTest.YTrainNorm
  ## Yn=Y-np.outer(np.ones(mtrain),ymean)
  ## Ycov=np.dot(Yn.T,Yn)/mtrain
  ## Ycovinv=np_lin.pinv(Ycov,rcond=1e-2)/(2*(params.output.ipar1**2))
  Ycovinv=np.eye(ndim)/(2*(cDataTest.kernel_params.ipar1**2))
  CY=np.dot(Ycovinv,Y.T)
  cy0=np.sum(Y*CY.T,axis=1)
  y0=np.zeros(ndim)
  ## y0c=np.zeros(ndim)
  ## e=np.ones(mtrain)

  rtime=time.time()

  ntime=5
  xdynstack0=np.zeros((ntime,ndim))
  xdynstack1=np.zeros((ntime,ndim,ndim))
  xdynstack2=np.zeros((ntime,ndim,mtrain))

  for it in range(mtest):
    ## setup of the dynamic programming
    ## load time 0
    f=np.dot(np.exp(-cy0),beta[it])
    for i in range(ntime):
      for j in range(ndim):
        xdynstack0[i,j]=f
        xdynstack1[i,j,:]=y0
        xdynstack2[i,j,:]=cy0

    ## print('>>>',f)
    for t in range(0,ntime-1):
      if t==0:
        nitem=1
      else:
        nitem=ndim
      for i1 in range(nitem):
        ## print(t,i1)
        f=xdynstack0[t,i1]
        y0=xdynstack1[t,i1,:]
        cy0=xdynstack2[t,i1,:]
        fmax=0
        y0c=np.dot(Ycovinv,y0)
        for i2 in range(ndim):
          cy_new=cy0+Ycovinv[i2,i2]+y0c[i2]-CY[i2]
          fnew=np.dot(np.exp(-cy_new),beta[it])
          if fnew>xdynstack0[t+1,i2]:
            ## print(t,i1,i2,fnew)
            y=np.copy(y0)
            y[i2]=1
            xdynstack0[t+1,i2]=fnew
            xdynstack1[t+1,i2,:]=y
            xdynstack2[t+1,i2,:]=cy_new
            ## else:
            ##   y=np.copy(y0)
            ##   dynstack[t+1][i2]=(f,y,y0c)
          if fnew>fmax:
            fmax=fnew
        ## print(f,fmax)
        ## print(t,i1)        
    ## find the best one
    fbest=0
    ibest=-1
    for i in range(ndim):
      if fbest<xdynstack0[ntime-1,i]:
        fbest=xdynstack0[ntime-1,i]
        ibest=i
    cPredict.zPred[it,:]=xdynstack1[ntime-1,ibest,:]
    if it%10==0:
      print(it,(time.time()-rtime)/(it+1))

  return(cPredict)
## #####################################################
