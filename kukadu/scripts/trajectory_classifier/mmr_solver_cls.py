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
## import os, string, re, math, pickle, random
## import math, time
from numpy import sqrt, diag, zeros, dot, abs
## import pylab as lab
## #####################
 
## #######################################################################
class cls_mmr_solver:

  def __init__(self):

    self.niter=1000   ## maximum iteration
    self.normx1=1  ## normalization within the kernel by this power
    self.normy1=1  ## normalization within the kernel by this power
    self.normx2=1  ## normalization of duals bound
    self.normy2=1  ## normalization of duals bound
    self.ilabel=0  ## 1 explicit labels, 0 implicit labels 
    self.ibias=0   ## 0 no bias considered, 1 bias is computed by solver 
    self.ibias_estim=0  ## estimated bias =0 no =1 computed 
    self.i_l2_l1=1   ## =1 l2 norm =0 l1 norm regularization   
    self.report=0

  ## ----------------------------------------------------
  def mmr_solver(self,Kx,Ky,C,D):
  ## solve an unbiased mmr
    err_tolerance=0.001
    maxiter=100
    xeps=10**(-4)

  ## input output norms
    dx=diag(Kx)
    dy=diag(Ky)
    dx=dx+(abs(dx)+xeps)*(dx<=0)
    dy=dy+(abs(dy)+xeps)*(dy<=0)
    dKx=sqrt(dx)
    dKy=sqrt(dy)

    dKxy1=dKx**self.normx1*dKy**self.normy1   ## norm based scale of the margin
    dKxy2=dKx**self.normx2*dKy**self.normy2   ## norm based scale of the loss

    dKxy2+=1.0*(dKxy2==0)    ## to avoid zero

    lB=float(D)/(dKxy2)               ## scale the ranges
    uB=float(C)/(dKxy2)

    Bdiff=uB-lB
    z_eps=err_tolerance*sqrt(sum(Bdiff*Bdiff))

    qs=-dKxy1

    Kxy=Kx*Ky
    m=Kxy.shape[0]
  ## scaling by diagonal elements  
    ## dKxy=diag(Kx)
    ## dKxy=dKxy+(dKxy==0)
    ## Kxy=Kxy/outer(dKxy,dKxy)
    ## qs=qs/dKxy

  ##  xalpha=zeros(m)
    xalpha=lB
    xalpha0=xalpha.copy()
    for irepeat in range(maxiter):
      for irow in range(m):
        t=(-qs[irow]-dot(Kxy[irow],xalpha0))/Kxy[irow,irow]
        ## t=-qs[irow]-dot(Kxy[irow],xalpha0)
        xnew=xalpha0[irow]+t
        lbi=lB[irow]
        ubi=uB[irow]
        if lbi<xnew:
          if ubi>xnew:
            xalpha0[irow]=xnew
          else:
            xalpha0[irow]=ubi
        else:
          xalpha0[irow]=lbi
      xdiff=xalpha0-xalpha
      zerr=sqrt(sum(xdiff*xdiff))     ## L2 norm error
  ##     zerr=max(abs(xdiff))     ## L_infty norm error
      xalpha=xalpha0.copy()
      if zerr<z_eps:
  ##       print irepeat
        break
  ## xalpha the dual solution
    return(xalpha)
  
  ## ----------------------------------------

  def mmr_solver_bias(self,Kx,Ky,C,D):
  ## solve an unbiased mmr

  ##   C=1.0
  ##   D=0.0

    cgrowmax=4.0    ## growing factor of the penalty constant
    ck=2            ## initial penalty
    maxiterout=20   ## max iteration of the Lagrangien
    maxiterin=100   ## maximum iteration of the inner loop
    err_tolerance_out=0.0001;  ## error tolerance of the Lagrangian of the dual
    err_tolerance_in=0.00001;  ## error tolerance of the dual variables
    ilambda=1       ## Lagrangian is considered
    xeps=10**(-4)

    cgrow=cgrowmax

  ## input output norms
    dKx=sqrt(diag(Kx))
    dKy=sqrt(diag(Ky))

    dKxy1=dKx**self.normx1*dKy**self.normy1   ## norm based scale of the margin
    dKxy2=dKx**self.normx2*dKy**self.normy2   ## norm based scale of the loss

    dKxy2+=1.0*(dKxy2==0)    ## to avoid zero

    lB=float(D)/(dKxy2)               ## scale the ranges
    uB=float(C)/(dKxy2)

    Bdiff=uB-lB
    z_eps=err_tolerance_in*sqrt(sum(Bdiff*Bdiff))
    herr=10**10

    Kxy=Kx*Ky
    m=Kxy.shape[0]           ## kernel size

    xlambda=zeros(m)          ## initialization of the Lagrangien
    xlambda0=xlambda.copy()

    Ky2=dot(Ky,Ky)
    qs0=-dKxy1

    xalpha=zeros(m)
    for iouter in range(maxiterout):
      xalpha0=xalpha.copy()
      if ilambda==1:              ## augmented Lagrangian
        qs1=dot(Ky,xlambda)
        qs=qs0+qs1
        KK=Kxy+ck*Ky2
      else:
        qs=qs0
        KK=Kxy

  ## scaling by diagonal elements  
      dKxy=diag(KK)
      dKxy=dKxy+xeps*(dKxy==0)
      KK=KK+diag(dKxy)
      ## KK=KK/outer(dKxy,ones(m))
      ## qs=qs/dKxy

      xalpha0=xalpha.copy()
      for irepeat in range(maxiterin):
        for irow in range(m):
          t=(-qs[irow]-dot(KK[irow],xalpha0))/KK[irow,irow]
          xnew=xalpha0[irow]+t
          if lB[irow]<xnew:
            if uB[irow]>xnew:
              xalpha0[irow]=xnew
            else:
              xalpha0[irow]=uB[irow]
          else:
            xalpha0[irow]=lB[irow]
        xdiff=xalpha0-xalpha
        zerr=sqrt(sum(xdiff*xdiff))     ## L2 norm error
    ##     zerr=max(abs(xdiff))     ## L_infty norm error
        xalpha=xalpha0.copy()
        if zerr<z_eps:
    ##       print irepeat
          break

      h=dot(Ky.transpose(),xalpha)
      ## xlambda0=xlambda.copy()
      xlambda+=ck*h/cgrow
      ck=cgrow*ck
      ## xdiff_lambda=xlambda-xlambda0
      ## lerr=sqrt(sum(xdiff_lambda*xdiff_lambda))
      herr0=herr
      herr=sum(h*h)
      if herr<err_tolerance_out or abs(herr/herr0)-1<err_tolerance_out:
        break
      herr0=herr

  ## xalpha the dual solution
    return(xalpha,xlambda)
## ########################################################################
  

