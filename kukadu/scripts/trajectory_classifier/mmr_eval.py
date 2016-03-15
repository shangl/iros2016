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

import mmr_base_classes 
        
## ######################################################################
def mmr_eval(yTest,yPred):
## yTest           true label
## yPred            prediction
##
## eer         equal error rate
## auc         area under curve
## acc         accuracy
  
## area under curve  
  yy=1*(yTest>0)
  
  n=len(yy)
  np=sum(yy)
  nn=n-np
#  inp=where(yy>0)[0]
#  inn=where(yy==0)[0]
  rp=np.random.permutation(n)
  pres=yy[rp]
  conf=yPred[rp]
  
  si=np.argsort(-conf)
  sp=pres[si]
  xtp=np.cumsum(sp)/np
  xfp=np.cumsum(1-sp)/nn  
  
  ## auc=0
  ## for i in range(np):
  ##   pv=yPred[inp[i]]
  ##   ii=sum(yPred[inn]<pv)
  ##   auc=auc+ii
  ## auc=auc/(np*nn)
## BE AWARE !!!!!!
## it works in the reverse order, trapz(y,x),
##  of the matlab function, trapz(x,y)  
  auc=np.trapz(xtp,xfp)
  
## equal error rate  
  iz=np.argmin(abs(1-xtp-xfp),axis=0)
  eer=(1-xfp[iz]+xtp[iz])/2
## accuracy  
  acc=sum(yTest==np.sign(yPred))/n
  
  xaccur=np.zeros((2,2))
  tp=sum((yTest==1)*(yPred==1))
  fp=sum((yTest==-1)*(yPred==1))
  fn=sum((yTest==1)*(yPred==-1))
  tn=sum((yTest==-1)*(yPred==-1))
  
  xaccur[0,0]=tp
  xaccur[0,1]=fp
  xaccur[1,0]=fn
  xaccur[1,1]=tn

  if tp+fp>0:
    prec=tp/(tp+fp)
  else:
    prec=0
    
  if tp+fn>0:
    recall=tp/(tp+fn)
  else:
    recall=0
  
  if prec+recall>0:
    f1=2*prec*recall/(prec+recall)
  else:
    f1=0
  
  return(acc,eer,auc,prec,recall,f1,xaccur,xtp,xfp)

## #######################################################################
def mmr_eval_binvector(yTest,yPred):

  accuracy=0.0
  precision=0.0
  recall=0.0

  (m,n)=yTest.shape

  cEvaluation=mmr_base_classes.cls_evaluation()
  cEvaluation.classconfusion=np.zeros((n,n))

  ## class confusion
  for i in range(m):
    for j in range(n):
      for k in range(n):
        if yTest[i,j]>0 and yPred[i,k]>0:
          cEvaluation.classconfusion[k,j]+=1

  xtp=(yTest>0)*(yPred>0)
  xfp=(yTest<=0)*(yPred>0)
  xfn=(yTest>0)*(yPred<=0)
  xtn=(yTest<=0)*(yPred<=0)
  
  xctp=np.sum(xtp,axis=0)
  xcfp=np.sum(xfp,axis=0)
  xcfn=np.sum(xfn,axis=0)
  ## xctn=np.sum(xtn,axis=0)


  cEvaluation.cprecision=np.zeros(n)
  cEvaluation.crecall=np.zeros(n)
  cEvaluation.cf1=np.zeros(n)
  for i in range(n):
    if xctp[i]+xcfp[i]>0:
      cEvaluation.cprecision[i]=xctp[i]/(xctp[i]+xcfp[i])
    else:
      cEvaluation.cprecision[i]=0

    if xctp[i]+xcfn[i]>0:
      cEvaluation.crecall[i]=xctp[i]/(xctp[i]+xcfn[i])
    else:
      cEvaluation.crecall[i]=0

    if cEvaluation.cprecision[i]+cEvaluation.crecall[i]>0:
      cEvaluation.cf1[i]=2*cEvaluation.cprecision[i]*cEvaluation.crecall[i] \
                       /(cEvaluation.cprecision[i]+cEvaluation.crecall[i])
    else:
      cEvaluation.cf1[i]=0

  tp=np.sum(xtp)
  fp=np.sum(xfp)
  fn=np.sum(xfn)
  tn=np.sum(xtn)

  for i in range(m):
    if np.sum(yTest[i]*yPred[i])==n:
      accuracy+=1.0

  if tp+fp>0:
    precision=tp/(tp+fp)
  else:
    precision=0
    
  if tp+fn>0:
    recall=tp/(tp+fn)
  else:
    recall=0
  
  if precision+recall>0:
    f1=2*precision*recall/(precision+recall)
  else:
    f1=0

  cEvaluation.accuracy=accuracy/m
  cEvaluation.precision=precision
  cEvaluation.recall=recall
  cEvaluation.f1=f1

  cEvaluation.confusion=np.zeros((2,2))
  cEvaluation.confusion[0,0]=tp
  cEvaluation.confusion[0,1]=fp
  cEvaluation.confusion[1,0]=fn
  cEvaluation.confusion[1,1]=tn


  
  return(cEvaluation)
## #######################################################################
def mmr_eval_angle(yTest,yPred):

#   accuracy=0.0
#   precision=0.0
#   recall=0.0

  ## (m,n)=yTest.shape

  cEvaluation=mmr_base_classes.cls_evaluation()

  xdot=np.sum(yTest*yPred,axis=1)
  xnormt=np.sqrt(np.sum(yTest**2,axis=1))
  xnormt=xnormt+(xnormt==0)
  xnormp=np.sqrt(np.sum(yPred**2,axis=1))
  xnormp=xnormp+(xnormp==0)
  xangle=xdot/(xnormt*xnormp)
  iacc=np.where(xangle>1.0)
  xangle[iacc]=1.0
  iacc=np.where(xangle<-1.0)
  xangle[iacc]=-1.0
  xangle=np.arccos(xangle)
  accuracy=np.sqrt(np.mean(xangle**2))

  cEvaluation.accuracy=accuracy
  cEvaluation.precision=0
  cEvaluation.recall=0
  cEvaluation.f1=0

  cEvaluation.confusion=np.zeros((2,2))
  cEvaluation.confusion[0,0]=0
  cEvaluation.confusion[0,1]=0
  cEvaluation.confusion[1,0]=0
  cEvaluation.confusion[1,1]=0
  
  return(cEvaluation)
## #######################################################################
def mmr_eval_real(yTest,yPred):

  ## (m,n)=yTest.shape

  cEvaluation=mmr_base_classes.cls_evaluation()

  ## RMSE
  cEvaluation.accuracy=np.sqrt(np.mean(np.sum((yTest-yPred)**2,axis=1)))  

  cEvaluation.precision=0
  cEvaluation.recall=0
  cEvaluation.f1=0

  cEvaluation.confusion=np.zeros((2,2))
  return(cEvaluation)
## #################################################33
## #################################################33
