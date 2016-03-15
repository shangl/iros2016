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
## ###########################################################
## from mmr_normalization_new import mmr_normalization
import mvm_kernel_eval
import mvm_prepare
## class definitions
## ##################################################
class cls_dataitem:

  def __init__(self):

    self.xdata_tra=None
    self.xdata_tes=None
    self.nrow=None
    self.ncol=None

  ## ---------------------------------------
  def load(self,xdata1,xdata2,nrow,ncol):

    self.xdata_tra=xdata1
    self.xdata_tes=xdata2
    self.nrow=nrow
    self.ncol=ncol
    

## ##################################################
class cls_kernel:

  def __init__(self,ikernelstruct=0):
    """
    ikernelstruct       =0 monolith
                        =1 learner wise

    """
    self.ikernelstruct=ikernelstruct
    self.Kmonolith=None
    self.dkernel=None
    self.dkernel_test=None
    self.dsubsets=None
    self.index2subset=None

  ## -------------------------------------------------
  def create_monolith(self,xdatacls,isymmetric,params_spec,norm_spec):

    self.Kmonolith=mvm_kernel_eval.mvm_kernel_sparse(xdatacls,isymmetric, \
                                     params_spec,norm_spec)

    return
  ## -------------------------------------------------
  def create_diag_kernel(self,xdatacls,params_spec,norm_spec,itraintest=1):
    
    xdata_1=xdatacls.xdata_tra
    nval=xdata_1[2].shape[1]
    xranges_1=xdatacls.xranges_rel
    if itraintest!=1:
      xdata_2=xdatacls.xdata_tes
      xranges_2=xdatacls.xranges_rel_test
    else:
      xdata_2=xdata_1
      xranges_2=xranges_1
      
    nrow=xdatacls.nrow
    ncol=xdatacls.ncol

    ## create column, row order 
    xdata_1_inv=[xdata_1[1],xdata_1[0]]
    xdata_1_inv=mvm_prepare.sort_table(xdata_1_inv,idata=0)
    xranges_1_inv=mvm_prepare.mvm_ranges(xdata_1_inv,ncol,None)
    if itraintest!=1:
      xdata_2_inv=[xdata_2[1],xdata_2[0]]
      xdata_2_inv=mvm_prepare.sort_table(xdata_2_inv,idata=0)
      xranges_2_inv=mvm_prepare.mvm_ranges(xdata_2_inv,ncol,None)
    else:
      xdata_2_inv=xdata_1_inv
      xranges_2_inv=xranges_1_inv
      
    cxdata=cls_dataitem()
    npart=3
    cxdata.xdata_tra=[None]*npart 
    cxdata.xdata_tes=[None]*npart 

    for ilearner in range(nrow):
      (istart,nlength)=xranges_1[ilearner]
      nlength_1=0
      for icol in range(nlength):
        inboro=xdata_1[1][istart+icol]
        nlength_1+=xranges_1_inv[inboro][1]
      cxdata.xdata_tra=[np.zeros(nlength_1,dtype=int), \
                        np.zeros(nlength_1,dtype=int), \
                        np.zeros((nlength_1,nval))]
      if itraintest!=1:
        (istart,nlength)=xranges_2[ilearner]
        nlength_2=0
        for icol in range(nlength):
          inboro=xdata_2[1][istart+icol]
          nlength_2+=xranges_2_inv[inboro][1]
        cxdata.xdata_tes=[np.zeros(nlength_1,dtype=int), \
                          np.zeros(nlength_1,dtype=int), \
                          np.zeros((nlength_1,nval))]
      else:
        nlength_2=nlength_1


    
    for ilearner in range(nrow):
      (istart,nlength_tra)=xranges_1[ilearner]
      for icol in range(nlength_tra):
        inboro=xdata_1[1][istart+icol]
        (istartr,nlengthr)=xranges_1_inv[inboro]
        xilearners=xdata_1_inv[istartr,istartr:nlengthr]
        ipos=0
        for ilearnerc in xilearners:
          (istartc,nlengthc)=xranges_1[ilearnerc]
          for ipart in range(npart):
            cxdata.xdata_tra[ipart][ipos:ipos+nlengthc] \
                  =xdata_1[ipart][istartc,istartc+nlengthc]
            ipos+=nlengthc
      if itraintest!=1:
        (istart,nlength_tes)=xranges_2[ilearner]
        for icol in range(nlength_tes):
          inboro=xdata_2[1][istart+icol]
          (istartr,nlengthr)=xranges_2_inv[inboro]
          xilearners=xdata_2_inv[istartr,istartr:nlengthr]
          ipos=0
          for ilearnerc in xilearners:
            (istartc,nlengthc)=xranges_2[ilearnerc]
            for ipart in range(npart):
              cxdata.xdata_tes[ipart][ipos:ipos+nlengthc] \
                    =xdata_2[ipart][istartc,istartc+nlengthc]
              ipos+=nlengthc
        
      cxdata.nrow=nrow

      if itraintest==1:
        cxdata.nncol=nlength_tra
        self.dkernel[ilearner]=mvm_kernel_eval.mvm_kernel_sparse(cxdata,itraintest, \
                                     params_spec,norm_spec)
      else:
        cxdata.nncol=nlength_tes
        self.dkernel_test[ilearner]=mvm_kernel_eval.mvm_kernel_sparse(cxdata,itraintest, \
                                     params_spec,norm_spec)
        

    return

  ## ## -------------------------------------------------
  ## def load_subsets(self,dubsets):
  ##   """
  ##   dsubsets partitions the row space of xdata
  ##   dsubset={1:set(<partition1>),2:set(<partition2>), ...}
  ##   """
    
  ##   self.dsubsets=dsubsets
  ##   self.index2subset={}

  ##   for ikey,ssubset in self.dsubsets.items():
  ##     for irow in ssubset:
  ##       if irow not in self.index2subset:
  ##         self.index2subset[irow]=set([ikey])
  ##       else:
  ##         self.index2subset[irow].add([ikey])

  ##   return
  ## ## -------------------------------------------------
  ## def create_diagblocks(self,xdatacls,params_spec,norm_spec,itraintest=1):
    
  ##   xdata_1=xdatacls.xdata_tra
  ##   n=xdata_1.shape[1]
  ##   xranges_1=xdatacls.xranges_rel
  ##   if itraintest!=1:
  ##     xdata_2=xdatacls.xdata_tes
  ##     xranges_2=xdatacls.xranges_rel_test
  ##   else:
  ##     xdata_2=xdata_1
  ##     xranges_2=xranges_1
      
  ##   nrow=xdatacls.nrow
  ##   ncol=xdatacls.ncol

  ##   cxdata=cls_dataitem()
  ##   X1=None
  ##   X2=None
   
  ##   for ikey,ssubset in self.dsubsets.items():
  ##     nlength1=0
  ##     nlength2=0
  ##     for irow in ssubset:
  ##       nlength1+=xranges_1[irow,0]
  ##       X1=np.zeros((nlength1,n))
  ##       if itraintest==1:
  ##         nlength2=nlength1
  ##         X2=X1
  ##       else:
  ##         nlength2+=xranges_2[irow,0]
  ##         X2=np.zeros((nlength2,n))
  ##     i1=0
  ##     i2=0
  ##     for irow in ssubset:
  ##       (istart,nlength)=xranges_1[irow,0]
  ##       X1[i1,:]=xdata_1[istart,istart:istart+nlength]
  ##       i1+=nlength
  ##       if itraintest!=1:
  ##         (istart,nlength)=xranges_2[irow,0]
  ##         X2[i2,:]=xdata_2[istart,istart:istart+nlength]
  ##         i2+=nlength
  ##     cxdata.xdata_tra=X1
  ##     cxdata.xdata_tes=X2
  ##     cxdata.nrow=len(ssubset)
  ##     cxdata.ncol=ncol
  ##     if itraintest==1:
  ##       self.dkernel[ikey]=mvm_kernel_sparse(cxdata,itraintest, \
  ##                                    params_spec,norm_spec)
  ##     else:
  ##       self.dkernel_test[ikey]=mvm_kernel_sparse(cxdata,itraintest, \
  ##                                    params_spec,norm_spec)
        

  ##   return

  ## -------------------------------------------------
  def read_block_x(self,xdatacls,ilearner,itraintest=1,isubset=None):

    xdata_1=xdatacls.xdata_tra
    ## n=xdata_1.shape[1]
    xranges_1=xdatacls.xranges_rel
    if itraintest!=1:
      xdata_2=xdatacls.xdata_tes
      xranges_2=xdatacls.xranges_rel_test
    else:
      xdata_2=xdata_1
      xranges_2=xranges_1
    
    if self.ikernelstruct==0: ## monolith kernel
      istart=xranges_1[ilearner,0]       
      nlength=xranges_1[ilearner,1]
      ixrange=xdata_1[istart:istart+nlength]
      if itraintest==1:
        if isubset is None:
          ixsubrange=ixrange
        else:
          ixsubrange=ixrange[isubset]
      else:
        istart=xranges_2[ilearner,0]       
        nlength=xranges_2[ilearner,1]
        ixsubrange=xdata_2[istart:istart+nlength]  

      lx=len(ixrange)
      Kblock=self.Kmonolith[ixrange.reshape((lx,1)),ixsubrange]
    else:
      if itraintest==1:
        if isubset is None:
          Kblock=self.dkernel[ilearner]
        else:
          Kblock=self.dkernel[ilearner][:,isubset]
      else:
        Kblock=self.dkernel_test[ilearner]

    return(Kblock)
  ## ------------------------------------------------
  def swap_row_col(self,ncol,xdata1):
    ## collect neighborhood wise indexes, by swaping row and column
    ## but maintaining the lexical order

    lcols={}           ## dictionary for ncol for training
    xncols=np.zeros(ncol)
    ndata=len(xdata1)
    for idata in range(ndata):
      icol=xdata1[idata]
      xncols[icol]+=1

    for icol in range(ncol):
      lcols[icol]=np.zeros(xncols[icol],dtype=int)
  
    xpcols=np.zeros(ncol)
    for idata in range(ndata):
      icol=xdata1[idata]
      lcols[icol][xpcols[icol]]=idata
      xpcols[icol]+=1

    return(lcols)

    
## ################################################################
