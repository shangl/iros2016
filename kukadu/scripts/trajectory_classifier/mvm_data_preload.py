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

## ####################
## import mvm_classes
## ####################
def mvm_data_preload(sdir,sfile,xdatacls,params):
###############################################################
# Loads 
#  - the data files
#  - transform the row and the col indeces into continues, gap free,
#  range
#  -sort the data items ( row,col,rank) into ascending order 
# ############################################################# 
# inputs
#     sdir        data directory name
#     sfile       data file name
#     xdatacls    data class containing features + kernels
#     params      global system parameters from mvm_setparams
# outputs
#     nrow       number of rows
#     ncol      number of cols
#     ndata       number of ranks  
###############################################################  

  ndatacolumn=3

  if params.idataset==0:
    xdata=np.loadtxt(sdir+sfile)
  else:
    xdata=np.loadtxt(sdir+sfile,delimiter=',')

## use only row, col index and rank
## convert row col index into int  
  xdata=[xdata[:,0].astype(int),xdata[:,1].astype(int),xdata[:,2]]
  
  nrow=int(np.max(xdata[0]))
  ncol=int(np.max(xdata[1]))
  ndata=xdata[0].shape[0]

# transform the row and col indeces into a continous range

  print('compress range of cols and rows')
# cols
  xcol0=np.zeros((ncol,2))
  xrow0=np.zeros((nrow,2))
  for i in range(ndata):
    irow=xdata[0][i]-1
    icol=xdata[1][i]-1
    xcol0[icol,0]+=1
    xrow0[irow,0]+=1
    if i%1000000==0:
      print(i)
# rows  
  j=0
  for i in range(nrow):
    if xrow0[i,0]>0:
      xrow0[i,1]=j
      j+=1
  nrow=j
  
# cols
  j=0
  for i in range(ncol):
    if xcol0[i,0]>0:
      xcol0[i,1]=j
      j+=1
  ncol=j
  
  for i in range(ndata):
    irow=xdata[0][i]-1
    icol=xdata[1][i]-1
    xdata[0][i]=xrow0[irow,1]
    xdata[1][i]=xcol0[icol,1]
    if i%1000000==0:
      print(i)

  print('Sorting begins')
## data ordered row wise   
  ldata=[ [xdata[0][i],xdata[1][i],xdata[2][i]] for i in range(ndata)] 
  ldata.sort()
  xdatacls.xdata_rel=[ None for i in range(ndatacolumn)] 
  xdatacls.xdata_rel[0]=np.array([ xitem[0] for xitem in ldata]).astype(int)
  xdatacls.xdata_rel[1]=np.array([ xitem[1] for xitem in ldata]).astype(int)
  xdatacls.xdata_rel[2]=np.array([ xitem[2] for xitem in ldata])
## ## data orderd col wise
##   ldata=[ [xdata[1][i],xdata[0][i],xdata[2][i]] for i in range(ndata)] 
##   ldata.sort()
##   xdatacls.xdata2=[ None for i in range(ndatacolumn)] 
##   xdatacls.xdata2[0]=np.array([ xitem[0] for xitem in ldata]).astype(int)
##   xdatacls.xdata2[1]=np.array([ xitem[1] for xitem in ldata]).astype(int)
##   xdatacls.xdata2[2]=np.array([ xitem[2] for xitem in ldata])
  
  print('Sorting done')
  
  return(nrow,ncol,ndata)
  
