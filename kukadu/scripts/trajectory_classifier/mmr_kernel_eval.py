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
import sys
import numpy as np
import numpy.linalg as np_lin 
import mmr_kpca
## #####################
## ######################################################################
def kernel_eval(X1,X2,params_spec):
  
#   kernel_type=params_spec.kernel_type
#   ipar1=params_spec.ipar1
#   ipar2=params_spec.ipar2

  m1=X1.shape[0]
  m2=X2.shape[0]

  if len(X1.shape)==1:   ## vector
    X1=X1.reshape((m1,1))
    
  if len(X2.shape)==1:
    X2=X2.reshape((m2,1))

  d1=np.sum(X1**2,axis=1)
  d2=np.sum(X2**2,axis=1)
  K=np.dot(X1,X2.T)

  K=kernel_eval_nl(K,d1,d2,params_spec)  

  return(K)
## ######################################################################
def kernel_eval_nl(K,d1,d2,params_spec):

  kernel_type=params_spec.kernel_type
  ipar1=params_spec.ipar1
  ipar2=params_spec.ipar2

  ## print('>>>',kernel_type,ipar1,ipar2)
  
  (m1,m2)=K.shape
  e1=np.ones(m1)
  e2=np.ones(m2)
## linear input kernel
  if kernel_type==0:     ## linear kernel
    pass
  elif kernel_type in (1,51,61):   ## polynomial kernel
    K=np.sign(K+ipar2)*np.abs(K+ipar2)**ipar1   ##in case of fraction power
##    K=(K+ipar2)**ipar1
  elif kernel_type==2:   ## Sigmoid kernel
    K=np.tanh(ipar1*K+ipar2)
  elif kernel_type in (3,53):   ## Gaussian kernel
    K=np.outer(d1,e2)+np.outer(e1,d2)-2*K
    K=np.exp(-K/(2*ipar1**2))
  elif kernel_type==31:   ## powered Gaussian kernel
    K=np.outer(d1,e2)+np.outer(e1,d2)-2*K
    K=K*(K>=0.0)
    K=np.sqrt(K)**ipar2
    K=np.exp(-K/(2*ipar1**2))
  else:
    pass                    ## default is linear

  return(K)
## ######################################################################
def kernel_eval_gauss(X1,X2,params_spec):
  
  ## kernel_type=params_spec.kernel_type
  ipar1=params_spec.ipar1
  ## ipar2=params_spec.ipar2

  m1=X1.shape[0]
  m2=X2.shape[0]

  e1=np.ones(m1)
  e2=np.ones(m2)

  if len(X1.shape)==1:   ## vector
    X1=X1.reshape((m1,1))
    
  if len(X2.shape)==1:
    X2=X2.reshape((m2,1))

  d1=np.sum(X1**2,axis=1)
  d2=np.sum(X2**2,axis=1)

  xmean=np.mean(X1,axis=0)
  ## ndim=len(xmean)
  X1n=X1-np.outer(e1,xmean)
  Xcov=np.dot(X1n.T,X1n)/m1
  Xcovinv=np_lin.pinv(Xcov)
  ## Ycovinv=np.eye(ndim)
  D1=np.dot(Xcovinv,X1.T)
  D2=np.dot(Xcovinv,X2.T)
  K=np.outer(np.sum(X1*D1.T,axis=1),e2)+np.outer(e1,np.sum(X2*D2.T,axis=1)) \
     -2*np.dot(X1,D2)
  K=np.exp(-K/(2*ipar1**2))
  
  return(K,Xcovinv,d1,d2)
## ######################################################################
"""
Compute the kernel, linear and nonlinear one, of feature matrix, or compute nonlinear kernel of raw kernel 
Input:
      xData         list, if one item then raw kernel
                      if two item then feature matrices,
                          where the second can be None
      itrain        array of indexes of training items
                    if None then all item of xData[0]
                    else xData[0][itrain]
      itest         array of indexes of test items
                      if len(xData)==2 and xData[1] is not None
                        if None then all item of xData[1]
                        else  xData[1][itest]
                      else None
      params_sepc   kernel parameters
      
Output:
      K             kernel
      d1            squared L_2 norm of training items
      d2            squared L_2 norm of test items, if test is None then d2=d1
                      
"""
def kernel_eval_kernel(xData,itrain,itest,params_spec):
  
#   kernel_type=params_spec.kernel_type
#   ipar1=params_spec.ipar1
#   ipar2=params_spec.ipar2
#   ## print('>>>',ipar1,ipar2)

  if itrain is not None:
    m1=len(itrain)
  else:
    m1=len(xData[0])
    itrain=np.arange(m1)
    
  if len(xData)==2:
    if xData[1] is not None:
      if itest is not None:
        m2=len(itest)
      else:
        m2=len(xData[1])
        itest=np.arange(m2)
    else:
      m2=m1
  else:
    if itest is not None:
      m2=len(itest)
    else:
      m2=xData[0].shape[1]
      itest=np.arange(m2)

  if len(xData)==2:
    X1=xData[0][itrain]
    if len(X1.shape)==1:   ## vector
      X1=X1.reshape((m1,1))
    d1=np.sum(X1**2,axis=1)
    if xData[1] is None:
      d2=d1
      K=np.dot(X1,X1.T)
    else:
      X2=xData[1][itest]
      if len(X2.shape)==1:
        X2=X2.reshape((m2,1))
      d2=np.sum(X2**2,axis=1)
      K=np.dot(X1,X2.T)
  else:
    ## K=xData[0][itrain,:][:,itest]
    K=xData[0][itrain.reshape((m1,1)),itest]
    d1=np.diag(xData[0])[itrain]
    d2=np.diag(xData[0])[itest]

  ## if X1.shape[1]>1:
  ##   (K,d1,d2)=kernel_normalize(K,d1,d2)
  
  K=kernel_eval_nl(K,d1,d2,params_spec)  

  return(K,d1,d2)
## ######################################################################
"""
Compute the kernel, linear and nonlinear one, of feature matrix, or compute nonlinear kernel of raw kernel 
Input:
      xData         list, if one item then raw kernel
                      if two item then feature matrices,
                          where the second can be None
      itrain        array of indexes of training items
                    if None then all item of xData[0]
                    else xData[0][itrain]
      itest         array of indexes of test items
                      if len(xData)==2 and xData[1] is not None
                        if None then all item of xData[1]
                        else  xData[1][itest]
                      else None
      params_sepc   kernel parameters
      
Output:
      K             kernel
      d1            squared L_2 norm of training items
      d2            squared L_2 norm of test items, if test is None then d2=d1
                      
"""
def kernel_eval_kernel_cat(xData,itrain,itest,params_spec):
  
#   kernel_type=params_spec.kernel_type
#   ipar1=params_spec.ipar1
#   ipar2=params_spec.ipar2

  if itrain is not None:
    m1=len(itrain)
  else:
    m1=len(xData[0])
    itrain=np.arange(m1)
    
  if len(xData)==2:
    if xData[1] is not None:
      if itest is not None:
        m2=len(itest)
      else:
        m2=len(xData[1])
        itest=np.arange(m2)
    else:
      m2=m1
  else:
    if itest is not None:
      m2=len(itest)
    else:
      m2=xData[0].shape[1]
      itest=np.arange(m2)

  if len(xData)==2:
    X1=xData[0][itrain]
    if len(X1.shape)==1:   ## vector
      X1=X1.reshape((m1,1))
    d1=np.sum(X1**2,axis=1)
    if xData[1] is None:
      d2=d1
      K=np.dot(X1,X1.T)
    else:
      X2=xData[1][itest]
      if len(X2.shape)==1:
        X2=X2.reshape((m2,1))
      d2=np.sum(X2**2,axis=1)
      K=np.dot(X1,X2.T)
  else:
    K=xData[0][itrain,:][:,itest]
    d1=np.diag(xData[0])[itrain]
    d2=np.diag(xData[0])[itest]

  K=kernel_eval_nl(K,d1,d2,params_spec)  

  return(K,d1,d2)
## #######################################################################
## centralization in the feature space
def kernel_center(K):

  (m1,m2)=K.shape

  K=K-np.outer(np.ones(m1),np.mean(K,axis=0)) \
     -np.outer(np.mean(K,axis=1),np.ones(m2)) \
     +np.ones((m1,m2))*np.mean(K)

  return(K)

## #######################################################################
## centralization in the feature space
def kernel_normalize(K,d1,d2):

  (m1,m2)=K.shape

  d1=d1+(d1==0)
  d2=d2+(d2==0)
  d1=np.sqrt(d1)
  d2=np.sqrt(d2)
  K=K/np.outer(d1,d2)

  d1=np.ones(m1)
  d2=np.ones(m2)

  return(K,d1,d2)
## ######################################################################
"""
Compute the kernel, linear and nonlinear one, of feature matrix, or compute nonlinear kernel of raw kernel 
Input:
      KX            input training kernel
      KXcross       input test kernel, can be None
      d1            squared L_2 norm of training items
      d2            squared L_2 norm of test items, if test is None then d2=d1
      params_output   output kernel parameters
                      ( current version assumes linear kernel )
      u             dual variables, can be None for training
Output:
      K             total kernel
      Yu            raw prediction
                      
"""
def kernel_operator_valued(KX,KXcross,d1,d2,Y1,params_spec,u=None):

  nboro=params_spec.nboro
  m1=len(d1)
  if d2 is not None:
    m2=len(d2)
  else:
    d2=d1
    m2=m1
  n=Y1.shape[1]

  if u is None:
    u=np.ones(m1)

  ## KY1=np.dot(Y1,Y1.T)
    
  ## distance between X1 and X2 rows
  DX=np.outer(d1,np.ones(m1))+ \
        np.outer(np.ones(m1),d1)-2*KX
  i1boro=np.argsort(DX,axis=1)[:,:nboro]
  if KXcross is not None:
    DX=np.outer(d1,np.ones(m2))+ \
        np.outer(np.ones(m1),d2)-2*KXcross
    i2boro=np.argsort(DX,axis=0)[0:nboro,:].T
  else:
    i2boro=i1boro

  ## compute  mean and variance of neighborhoods

  e1=np.ones(nboro)
  d1mom=np.zeros((m1,params_spec.neig))
  ytrans1=np.zeros(params_spec.neig)
  for i in range(m1):
    yboro=Y1[i1boro[i,:]]
    ymean=np.mean(yboro,axis=0)
    yboro=yboro-np.outer(e1,ymean)
    KY=np.dot(yboro,yboro.T)
    d1=np.sum(yboro**2,axis=1)
    d2=d1
    KY=kernel_eval_nl(KY,d1,d2,params_spec)
    KY=kernel_center(KY)
    veig=mmr_kpca.kpca(KY,params_spec.neig)[1]
    ytrans=np.dot(veig.T,yboro)
    ytrans1+=u[i]*ytrans
    d1mom[i]=ytrans

  if KXcross is not None:
    d2mom=np.zeros((m2,n))
    for i in range(m2):
      yboro=Y1[i2boro[i,:]]
      ymean=np.mean(yboro,axis=0)
      yboro=yboro-np.outer(e1,ymean)
      ## KY=np.dot(yboro_centr,yboro_centr.T)
      KY=np.dot(yboro,yboro.T)
      d1=np.sum(yboro**2,axis=1)
      d2=d1
      KY=kernel_eval_nl(KY,d1,d2,params_spec)
      KY=kernel_center(KY)
      veig=mmr_kpca.kpca(KY,params_spec.neig)[1]
      Ttarget=np.dot(veig.T,yboro)
      ytrans=np.dot(Ttarget.T,ytrans1)
      d2mom[i]=ytrans
  else:
    d2mom=d1mom
    
  if KXcross is not None:
    Yu=np.zeros((m2,n))
    for i in range(m2):
      Yu[i,:]=d2mom[i]
    ## K=np.zeros((m2,m1))
    K=None
  else:
    Yu=None
    K=np.dot(d1mom,d1mom.T)
    ## if i%1000==0:
      ## print(i)

  return(K,Yu)
## #####################################################3
def cross_table_kernel(X):

  ## print('Cross-table kernel')

  (m,n)=X.shape
  xcolsum=np.sum(X,axis=0)  
  xcovX=np.dot(X.T,X)   ## column intersections

  Q=np.zeros((n,n))
  for i in range(n):
    for j in range(n):
      Q[i,j]=(xcovX[i,j]+1)*(m-xcolsum[i]-xcolsum[j]+xcovX[i,j]+1)/ \
              ((xcolsum[i]-xcovX[i,j]+1)*(xcolsum[j]-xcovX[i,j]+1))

  Q=np.log2(Q)
  Q=np.sign(Q)*np.abs(Q)**2
  dnz={}
  for i in range(m):
    dnz[i]=np.where(X[i]>0)[0]

  K=np.zeros((m,m))
  for i in range(m):
    ir=dnz[i]
    if len(ir)==0:
      continue
    ir=ir.reshape((len(ir),1))
    for j in range(m):
      ic=dnz[j]
      if len(ic)==0:
        continue
      K[i,j]=np.mean(Q[ir,ic])
    if i>0 and i%1000==0:
      print(i)
      sys.stdout.flush()

  return(K)
## ## #################################
## def tanimoto(X):

##   (m,n)=X.shape
##   K=np.dot(X,X.T)   ## column intersections
##   xrowsum=np.sum(X,axis=1)  

##   for i in range(m):
##     for j in range(m):
##       xdenom=xrowsum[i]+xrowsum[j]-K[i,j]
##       if xdenom>0:
##         K[i,j]=K[i,j]/xdenom

##   return(K)
## #################################
def tanimoto(X1,X2):

  m1=X1.shape[0]
  m2=X2.shape[0]
  K=np.dot(X1,X2.T)   ## column intersections
  xrowsum1=np.sum(X1,axis=1)  
  xrowsum2=np.sum(X2,axis=1)  

  for i in range(m1):
    for j in range(m2):
      xdenom=xrowsum1[i]+xrowsum2[j]-K[i,j]
      if xdenom>0:
        ## if xdenom<K[i,j]:
        ##   print(xdenom)
        K[i,j]=K[i,j]/xdenom
      else:
        if i==j:
          K[i,j]=1
      ## if K[i,j]>1:
      ##   print('!!!')

  return(K)
## #########################################################
def kernel_localscale(K,KY0,itrain,itest,params_spec,ilocal=-1,iscale=-1):

  mtra=len(itrain)
  mtes=len(itest)
  ## m=K.shape[0]

  Ktra=K[itrain.reshape(mtra,1),itrain]
  if KY0 is None:
    Kcross=K[itrain.reshape(mtra,1),itest]
  else:
    Kcross=KY0[itrain,:]
    
  dktes=K[itest,itest]
  km=np.mean(Ktra,axis=0)
  kmc=np.mean(Kcross,axis=0)
  km2=np.mean(Ktra)

  if ilocal!=-1:
    Ktra=Ktra-np.outer(np.ones(mtra),km) \
          -np.outer(km,np.ones(mtra))+np.ones((mtra,mtra))*km2

    Kcross=Kcross-np.outer(np.ones(mtra),kmc) \
            -np.outer(km,np.ones(mtes))+np.ones((mtra,mtes))*km2

    dktes+=-2*kmc+np.dot(km,km)
  
  d1=np.diag(Ktra)
  d2=d1
  d1c=d1
  d2c=dktes

  if iscale!=-1:
    d1=d1+(d1==0)
    Ktra/=np.outer(np.sqrt(d1),np.sqrt(d1))
    
    d2c=d2c+(d2c==0)    
    Kcross/=np.outer(np.sqrt(d1c),np.sqrt(d2c))

    d1=np.ones(mtra)
    d2=d1
    d1c=d1
    d2c=np.ones(mtes)

  Ktra=kernel_eval_nl(Ktra,d1,d2,params_spec)  
  Kcross=kernel_eval_nl(Kcross,d1c,d2c,params_spec)  

  return(Ktra,Kcross,d1,d2,d1c,d2c)
## #########################################################
def gaussian_process_type_kernel(X,ipar1):

  m=X.shape[0]
  K=np.zeros((m,m))

  ## for i in range(m):
  ##   for j in range(i,m):
  ##     ip=0
  ##     for k in range(n):
  ##       ## ip+=np.exp(-(X[i,k]-X[j,k])**2/(2*ipar1**2))
  ##       ip+=np.exp(-np.abs(X[i,k]-X[j,k])/(ipar1))
  ##     K[i,j]=ip
  ##     K[j,i]=K[i,j]

  e1=np.ones(m)
  for i in range(m):
    xi=X[i]
    D=np.abs(X-np.outer(e1,xi))
    K[i]=np.sum(np.exp(-D/ipar1),axis=1)

  return(K)
## #########################################################

