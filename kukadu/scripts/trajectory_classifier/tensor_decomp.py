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
import numpy.linalg as np_lin
## ##########################################
## #################################
class cls_tensor_decomp:

## --------------------------  
  def __init__(self):

    self.lcomponents=[]
    self.xmatrix=None
    self.xfactors=None

## --------------------------  
  def load(self,xmatrix,xfactors):
  
    self.xmatrix=xmatrix
    self.xfactors=xfactors

## ---------------------------
  def reorder_in(self,xmat):

    xfactors=self.xfactors
    ## (m,n)=self.xmatrix.shape
    xmatre=np.zeros((xfactors[0,0]*xfactors[0,1],xfactors[1,0]*xfactors[1,1]))
    k=0
    for i1 in range(xfactors[0,0]):
      for j1 in range(xfactors[0,1]):
        xblock=xmat[i1*xfactors[1,0]:(i1+1)*xfactors[1,0], \
                    j1*xfactors[1,1]:(j1+1)*xfactors[1,1]]
        xmatre[k,:]=xblock.ravel()
        k+=1

    return(xmatre)
## ---------------------------
  def reorder_in_x(self,xmat):

    xfactors=self.xfactors
    tdim=self.xmatrix.shape
    ndim=len(tdim)
    mf=xfactors.shape[0]
    xmatredim=np.zeros(mf,dtype=int)
    for i in range(mf):
      xmatredim[i]=np.prod(xfactors[i,:])
    xmatre=np.zeros(xmatredim)

    pdim=np.prod(tdim)
    i_in=np.arange(pdim)
    i_inx=np.array(np.unravel_index(i_in,tdim))
    i_outx=[None]*ndim
    for i in range(ndim):
      i_outx[i]=np.array(np.unravel_index(i_inx[i],xfactors[:,i]))
    i_outx=np.array(i_outx)
    
    i_out=[None]*mf
    for i in range(mf):
      i_out[i]=np.array(np.ravel_multi_index(i_outx[:,i],xfactors[i]))
    i_out=np.array(i_out)

    xmatre[tuple(i_out)]=self.xmatrix[tuple(i_inx)]

    return(xmatre)
## ---------------------------
  def invert_reorder_in_x(self,xmatre,xfactors):

    (mf,nf)=xfactors.shape
    ## tdim2=xmatre.shape
    ## ndim2=len(tdim2)

    ndim=nf
    tdim=np.zeros(nf,dtype=int)
    for i in range(nf):
      tdim[i]=np.prod(xfactors[:,i])
    xmatrix=np.zeros(tdim)

    pdim=np.prod(tdim)
    i_in=np.arange(pdim)
    i_inx=np.array(np.unravel_index(i_in,tdim))

    i_outx=[None]*ndim
    for i in range(ndim):
      i_outx[i]=np.array(np.unravel_index(i_inx[i],xfactors[:,i]))
    i_outx=np.array(i_outx)
    
    i_out=[None]*mf
    for i in range(mf):
      i_out[i]=np.array(np.ravel_multi_index(i_outx[:,i],xfactors[i]))
    i_out=np.array(i_out)

    xmatrix[tuple(i_inx)]=xmatre[tuple(i_out)]
    xmatrix=xmatrix.astype(np.uint8)

    return(xmatrix)
## -------------------------
  def reorder_sort(self,xmat):

    tdim=xmat.shape
    xmat1=xmat.ravel()
    ixsort=np.argsort(xmat1)
    n=len(ixsort)
    ixsort_rev=np.zeros(n)
    for i in range(n):
      ixsort_rev[ixsort[i]]=i
    self.ixsort_rev=ixsort_rev.astype(int)
    self.ixsort=ixsort
    xmat1=xmat1[ixsort]
    xmatre=xmat1.reshape(tdim)

    return(xmatre)
## -------------------------
  def reorder_sort_inv(self,xmat):

    tdim=self.xmatrix.shape
    xmat1=xmat.ravel()
    xmat1=xmat1[self.ixsort_rev]
    xmatre=xmat1.reshape(tdim)

    return(xmatre)
## -------------------------       
  def reorder_out(self,u,v):

    xfactors=self.xfactors
    A=u.reshape((xfactors[0,0],xfactors[0,1]))
    B=v.reshape((xfactors[1,0],xfactors[1,1]))
    
    return(A,B)
## ---------------------------
  def tensor_decomp(self,X,niter=10):
    """
    Jacobi-Gauss-Newton decomposition
    """
  
    tdim=X.shape
    ndim=len(tdim)
  
    ## init singular vectors
    xsingv={}
    for i in range(ndim):
      v=np.random.randn(tdim[i])
      v=v/np.sqrt(np.sum(v**2))
      xsingv[i]=v
      
    dlambda0=-10**6
    derr=0.001
    for iiter in range(niter):
      for i in range(ndim):
        v=np.copy(X)
        k=0
        for j in range(ndim):
          if j!=i:
            v=np.tensordot(v,xsingv[j],axes=([j-k],[0]))
            k+=1 
        dlambda=np.sqrt(np.sum(v**2))
        v=v/dlambda
        xsingv[i]=v
        ## print(dlambda)
      if np.abs(dlambda0-dlambda)<derr:
        break
      dlambda0=dlambda
            
    return(dlambda,xsingv)
    
## --------------------------
  def multi_integral(self,X):

    tdim=X.shape
    ndim=len(tdim)
    Xint=np.copy(X)
    for i in range(ndim):
      X1=np.copy(Xint)
      Xint=np.cumsum(X1,axis=ndim-i-1)

    return(Xint)
## --------------------------
  def multi_diff(self,X):

    tdim=X.shape
    ndim=len(tdim)

      
    Xdif=np.copy(X)
    for i in range(ndim):
      X1=np.copy(Xdif)
      Xdif=np.diff(X1,axis=i)
      lslice=[None]*ndim
      for j in range(ndim):
        lslice[j]=slice(tdim[j])
      lslice[i]=slice(1)
      tslice=tuple(lslice)
      Xdif=np.concatenate((X1[tslice],Xdif),axis=i)

    return(Xdif)
## --------------------------
  def decompose(self,niter):

    xmat0=np.copy(self.xmatrix)
    ## (m,n)=xmat0.shape
    ## k=min(m,n)
    xmat1=self.reorder_in(xmat0)
    (m,n)=xmat1.shape
    ## k=min(m,n)
    (u,s,v)=np_lin.svd(xmat1,full_matrices=0)
    sq=np.sqrt(s)
    u=u*np.outer(np.ones(m),sq)
    v=v*np.outer(sq,np.ones(n))
    for i in range(niter):
      self.lcomponents.append(self.reorder_out(u[:,i],v[i,:]))
## --------------------------
  def decompose2(self,niter):
    """
    mapping for svd: tensor -> vector 
    """
    xfactors=self.xfactors
    xmat0=np.copy(self.xmatrix)
    ## xmat0=self.reorder_sort(xmat0)
    ## (m,n)=xmat0.shape
    xmat1=self.reorder_in_x(xmat0)

    ## xmat2=self.invert_reorder_in_x(xmat1)
    
    tdim=xmat1.shape
    ndim=len(tdim)
    for iiter in range(niter):
      (xlambda,xsingv)=self.tensor_decomp(xmat1)
      v=xsingv[0]
      for j in range(1,ndim):
        v=np.outer(v,xsingv[j])
      v=v.reshape(xmat1.shape)
      xmat1=xmat1-xlambda*v
      lcomp=[]
      for j in range(ndim):
        A=xlambda**(1.0/ndim)*xsingv[j]
        ## lcomp.append(A.reshape((xfactors[j,0],xfactors[j,1])))
        lcomp.append(A.reshape(tuple(xfactors[j,:])))
      self.lcomponents.append(lcomp)
   
    
    
    
