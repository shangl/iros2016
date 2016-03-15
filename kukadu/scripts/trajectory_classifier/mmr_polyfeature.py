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
from numpy import zeros, ones, array
from numpy import sum as np_sum
## #############################################################
## ######################################################
## ###################################################
## Compute the explicit polynomial features, all terms up to degree ndegree
## ------------------------------------------------
## Inputs:
## xdata data, rows are the items, columns are the variables
## ndegree, the f=degree of the polynomial
## Outputs:
## xpolydata: the explicit polynomial features
## #####################################################
def mmr_polyfeature_d(xdata,ndegree):

  (m,ndim)=xdata.shape
  ndegree=int(ndegree)
## number of terms  = \binomial(ndegree+ndim,ndim)
  nd=1
  for i in range(ndim):
    nd*=(ndegree+i+1)/(i+1)
  nd=int(nd)

  xpolydir={}
  xpower=zeros(ndim,dtype=int)
  xpolydir[tuple(xpower)]=ones(m)
 
  for i in range(nd):
    for j in range(ndim):
      if xpower[j]<ndegree-np_sum(xpower[j+1:]):
        xterm=xpolydir[tuple(xpower)]
        xpower[j]+=1
        xpolydir[tuple(xpower)]=xterm*xdata[:,j]
        break
      else:
        xpower[j]=0
        
  xpolydata=zeros((m,nd))
  xpolylist=[  xpow for xpow  in xpolydir.keys()]
  xpolylist.sort()

  for i in range(nd):
    xpow=xpolylist[i]
    xpolydata[:,i]=xpolydir[xpow]

  return(xpolydata)
## ########################################################
def mmr_polypower_d(ndim,ndegree):

  ndegree=int(ndegree)
## number of terms  = \binomial(ndegree+ndim,ndim)
  nd=1
  for i in range(ndim):
    nd*=(ndegree+i+1)/(i+1)
  nd=int(nd)

  xpolydir={}
  xpower=zeros(ndim,dtype=int)
  xpolydir[tuple(xpower)]=1
 
  for i in range(nd):
    for j in range(ndim):
      if xpower[j]<ndegree-np_sum(xpower[j+1:]):
        xpower[j]+=1
        xpolydir[tuple(xpower)]=1
        break
      else:
        xpower[j]=0
        
  xpolylist=[  xpow for xpow  in xpolydir.keys()]
  xpolylist.sort()
  xpolypower=array(xpolylist)


  return(xpolypower)
## ########################################################
## ###################################################
## Compute the explicit polynomial features, all terms up to degree ndegree
## ------------------------------------------------
## Inputs:
## xdata data, rows are the items, columns are the variables
## maxdegree, maximum degree of all terms
## ldegree, list of maximum degree of each variables
## Outputs:
## xpolydata: the explicit polynomial features
## #####################################################
def mmr_polyfeature_dn(xdata,maxdegree,ldegree):

  (m,ndim)=xdata.shape
  maxdegree=int(maxdegree)
  if len(ldegree)==0:
    ldegree=[ maxdegree for i in range(ndim)]

  xpolydir={}
  xpower=zeros(ndim,dtype=int)
  xpolydir[tuple(xpower)]=ones(m)

  istate=1
  while istate==1:
    for j in range(ndim):
      if xpower[j]<min(maxdegree-np_sum(xpower[j+1:]),ldegree[j]):
        xterm=xpolydir[tuple(xpower)]
        xpower[j]+=1
        xpolydir[tuple(xpower)]=xterm*xdata[:,j]
        break
      else:
        if j<ndim-1:
          xpower[j]=0
        else:
          istate=0
        
  xpolylist=[ xpow for xpow  in xpolydir.keys()]
  xpolylist.sort()
  nd=len(xpolylist)
  xpolydata=zeros((m,nd))

  for i in range(nd):
    xpow=xpolylist[i]
    xpolydata[:,i]=xpolydir[xpow]

  return(xpolydata)
## ########################################################
def mmr_polypower_dn(ndim,maxdegree,ldegree):

  maxdegree=int(maxdegree)
  if len(ldegree)==0:
    ldegree=[maxdegree]*ndim

  xpolydir={}
  xpower=zeros(ndim,dtype=int)
  xpolydir[tuple(xpower)]=1

  istate=1
  while istate==1:
    for j in range(ndim):
      if xpower[j]<min(maxdegree-np_sum(xpower[j+1:]),ldegree[j]):
        xpower[j]+=1
        xpolydir[tuple(xpower)]=1
        break
      else:
        if j<ndim-1:
          xpower[j]=0
        else:
          istate=0
        
  xpolylist=[  xpow for xpow  in xpolydir.keys()]
  xpolylist.sort()
  xpolypower=array(xpolylist)


  return(xpolypower)
## ########################################################

      
    
    
  
  
