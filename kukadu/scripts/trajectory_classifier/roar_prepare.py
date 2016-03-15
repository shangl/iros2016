######################
## Version 0.1 #######
## /**********************************************************************
##   Copyright 2014, Sandor Szedmak  
##   email: sandor.szedmak@uibk.ac.at
##
##   This file is part of Maximum Margin Multi-valued Regression code(MMMVR).
##
##   MMMVR is free software: you can redistribute it and/or modify
##   it under the terms of the GNU Lesser General Public License as published by
##   the Free Software Foundation, either version 3 of the License, or
##   (at your option) any later version. 
##
##   MMMVR is distributed in the hope that it will be useful,
##   but WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##   GNU Lesser General Public License for more details.
##
##   You should have received a copy of the GNU Lesser General Public License
##   along with MMMVR.  If not, see <http://www.gnu.org/licenses/>.
##
## ***********************************************************************/
######################
import numpy as np

from scipy import sparse
## ####################
import mmr_base_classes
import roar_db_actions
## ####################

def roar_prepare(xdatacls):
  """
  It loades the postresql database items into the learning class xdatacls
  """

  table_row=('objects','preposition')
  table_column=('actions',)
  table_class='score'
  ctables=roar_db_actions.cls_roar_table(table_row,table_column,table_class)
  xdatadb=ctables.load_tables()

  ##  covert confidence into rank

  ndata=len(ctables.xdatadb[0])
  xdatacls.ipretrain=np.zeros(ndata)
  xdatacls.ipretest=np.zeros(ctables.nrow*ctables.ncol-ndata)
  
  xdata0=np.zeros((ctables.nrow,ctables.ncol))-1
  for i in range(ndata):
    xdata0[xdatadb[0][i],xdatadb[1][i]]= \
                        round(xdatacls.YKernel.ymax*xdatadb[2][i])
   
  xdata_rank=[ [] for i in range(3)]  ## row index, column index , value
  k=0
  ktrain=0
  ktest=0
  for i in range(ctables.nrow):
    for j in range(ctables.ncol):
      xdata_rank[0][k]=i
      xdata_rank[1][k]=j
      if xdata0[i,j]<0:
        xdata_rank[2][k]=0
        xdatacls.ipretest[ktest]=k
        ktest+=1
      else:
        xdata_rank[2][k]=xdata0[i,j]
        xdatacls.ipretrain[ktrain]=k
        ktrain+=1
      k+=1
        
  Y0=np.arange(xdatacls.YKernel.ymax)
  xdatacls.load_data(xdata_rank,ctables.ncategory, \
                     ctables.nrow,ctables.ncol,Y0)

  return

## ###################################################
