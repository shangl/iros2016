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
## import networkx as nx
## ##########################################
## #################################
def mmr_01_toprobability(X):
  """
  X \in \mathbb{B}^{mn}
  """
  
  n=X.shape[1]
  
  xcolsum=np.sum(X,axis=0)  
  xcovX=np.dot(X.T,X)   ## column intersections
  xcolsum=xcolsum+(xcolsum==0)
  Q=xcovX/np.outer(xcolsum,np.ones(n))  ## P(x_i|x_j)

  return(Q)
## ###################################
def mmr_perceptron_primal(Xlist,Y,margin=1,s=0.1,nrepeat=1):
  """
  Compute the MMR primal perceptron
  Input;    Y       outputs
            X       inputs
            margin  margin value
            s       step size
            nrepeat repeatation
  Output:   W       linear operator
  """

  nview=len(Xlist)
  X=Xlist[0]
  for i in range(1,nview):
    if Xlist[i] is not None:
      X=np.hstack((X,Xlist[i]))

  (m,nx)=X.shape
  ny=Y.shape[1]

  ## print('Perceptron:','margin:',margin,'s:',s,'nrepeat:',nrepeat,'m:',m)

  W=np.random.randn(ny,nx+1)
  wnorm=np.sqrt(np.sum(W**2))
  W=W/wnorm
  
  t=0

  xmean=np.mean(X)
  X=np.hstack((X,np.ones((m,1))*xmean))
  dX=np.sum(X**2,axis=1)
  dY=np.sum(Y**2,axis=1)
  for irepeat in range(nrepeat):
    for i in range(m):
      yWx=np.dot(Y[i],np.dot(W,X[i]))
      if yWx<margin:
        W+=s*np.outer(Y[i],X[i])
        t+=1
        ## wnorm=np.sqrt(np.sum(W**2))
        wnorm=1+s**2*dX[i]*dY[i]+2*yWx
        W=W/wnorm
      if i>0 and i%1000==0:
        print(irepeat,i)
        sys.stdout.flush()

  return(W)

## ######################################################
## ###################################
def mmr_perceptron_dual(X,Y,ipar1,ipar2,ikernel=0,margin=1,s=0.1,nrepeat=1):
  """
  Compute the MMR primal perceptron
  Input;    Y       outputs
            X       inputs
            ipar1   kernel parameter
            ipar2   kernel parameter
            ikernel kernel type =0 linear =1 polynomial =3 Gaussian
            margin  margin value
            s       step size
            nrepeat repeatation
  Output:   xalpha  optimal dual variables
  """

  m=X.shape[0]
  ## ny=Y.shape[1]

  ## t=0

  diagX=np.sum(X**2,axis=1)
  e1=np.ones(m)
  xalpha=np.zeros(m)
  for irepeat in range(nrepeat):
    for i in range(m):
      KYi=np.dot(Y,Y[i])
      KXi=np.dot(X,X[i])
      if ikernel==1:
        KXi=(KXi+ipar2)**ipar1
      elif ikernel==3:
        DXi=diagX+diagX[i]*e1-2*KXi
        KXi=np.exp(-DXi/(2*ipar1**2))
        
      if np.dot(xalpha,KYi*KXi)<margin:
        xalpha+=s*KYi*KXi
        
  return(xalpha)

## ######################################################
