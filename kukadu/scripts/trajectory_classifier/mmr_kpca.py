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
## ##################################################
import numpy as np
import scipy.linalg as sp_linalg

## ###################################################
## Solve the kernel pca problem
##
## input:
## K        2d-array is expected to be centralized kernel
## neig     number of greatest eigenvalues
## output:
## seig     neig greatest eigenvalues
## veig     neig eigen vectors belonging to seig
## #######################################################
def kpca(K,neig):
  m=K.shape[0]
## centralization
  J=np.ones((m,m))
  K=K-np.dot(J,K)/m-np.dot(K,J)/m+np.dot(J,np.dot(K,J))/(m**2)

  hi=m-1
  lo=m-1-neig+1
  if lo<0:
    lo=0
  (seig,Veig)=sp_linalg.eigh(K/m,b=None,eigvals=(lo,hi))

  iseig=np.argsort(-seig)
  Veig=Veig[:,iseig]
  seig=seig[iseig]

  iz=np.where(seig<0)[0]
  seig[iz]=0.0
  
  vnorm=np.sqrt(seig*m)
  vnorm=vnorm+(vnorm==0)
  Veig=Veig/np.tile(vnorm,(m,1))

  return(seig,Veig)

  

  
  
  

  
