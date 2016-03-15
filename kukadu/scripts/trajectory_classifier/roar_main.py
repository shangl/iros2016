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
import sys, time
import numpy as np
## ################################################
from mmr_base_classes import cls_empty_class
import mvm_mvm_cls
import mmr_setparams
from mvm_prepare import mvm_ranges, mvm_glm, mvm_ygrid, mvm_largest_category
from mvm_eval import mvm_eval
from mvm_validation import mvm_validation
## import roar_db_actions
import roar_prepare
## import plandb_sqlite_classes
## ################################################
def roar_main(workmode):

  params=mmr_setparams.cls_params()
  params.setvalidation()
  params.setsolver()
  params.setgeneral()
  params.setoutput()
  params.setinput()

## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  xdatacls=mvm_mvm_cls.cls_mvm()

  roar_prepare.roar_prepare(xdatacls)

  nfold=xdatacls.nfold
  if xdatacls.itestmode in (0,3):
    nfold0=1        ## active learning
  else:
    nfold0=nfold    ## n-fold cross validation
  nrepeat=xdatacls.nrepeat

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  scombine=''

  if xdatacls.itestmode==0:
    if xdatacls.ibootstrap==0:
      fname='xresultte_rand'+scombine+'.csv'
    elif xdatacls.ibootstrap==1:  
      fname='xresultte_active'+scombine+'.csv'
    elif xdatacls.ibootstrap==2:  
      fname='xresultte_greedy'+scombine+'.csv'
    elif xdatacls.ibootstrap==3:  
      fname='xresultte_act_rand'+scombine+'.csv'
  else:
    fname='xresultte_ncross'+scombine+'.csv'

  ## xdatacls.YKernel.ymax=ctables.ncategory
  # it will be recomputed in mvm_ranges
  xdatacls.YKernel.ymin=0
  xdatacls.YKernel.yrange=100 # it will be recomputed in classcol_ranges
  xdatacls.YKernel.ystep=1  

  # load the databases
  # data file
  ndata=xdatacls.ndata
  
##  set_printoptions(precision=4)
  npar=1   ## number of parameter selected for random subsample
  
  nparam=4    # C,D,par1,par2
  nreport=4   ## accuracy, precision, recall, f1

  if xdatacls.itestmode==0:
    nrepeat0=ndata-1   ## active learning
  else:
    nrepeat0=nrepeat

  if xdatacls.itestmode==0:
    ## initialize the active learning seeds
    ## pzero=0.001
    ## xselector=1*(np.random.rand(ndata)<pzero)

    nzero=100  ## !!!!!!!! initial training size
    xselector=np.zeros(ndata)
    nprime=4999
    ip=0
    for i in range(nzero):
      ip+=nprime
      if ip>ndata:
        ip=ip%ndata
      xselector[ip]=1  

    ndatainit=int(np.sum(xselector))
    mtest=ndata-ndatainit
    xdatacls.itest=np.where(xselector==0)[0]
    icandidate_w=-1
    icandidate_b=-1
    ## nrepeat0=ndata-ndatainit-10
    nrepeat0=min(100000,ndata-ndatainit-1000)  ## !!!!!! test size
    ## nrepeat0=1
  else:   ## n-fold cross validation
    nrepeat0=nrepeat
    
  xresulttr=np.zeros((nrepeat0,nfold0))
  xresultte=np.zeros((nrepeat0,nfold0,nreport))
  xbest_param=np.zeros((nrepeat0,nfold0,nparam))

  # ############################################################

  # number iterations in the optimization
  params.solver.niter=100
  print('niter:',params.solver.niter)

  for ipar in range(npar):

    nval=len(xdatacls.YKernel.valrange)
    xconfusion3=np.zeros((nrepeat0,nfold0,xdatacls.YKernel.ndim,nval,nval))

    ireport=0
    ## for irepeat in range(int(float(ndata)/3)):
    for irepeat in range(nrepeat0):

      if xdatacls.itestmode==0:
        if xdatacls.ibootstrap==0:
          if icandidate_w>=0:
            icandidate_w=np.random.randint(mtest,size=1)
            icandidate_w=xdatacls.itest[icandidate_w]
            xselector[icandidate_w]=1
            ## xselector[icandidate_b]=0     ## delete the best 
        elif xdatacls.ibootstrap==1:  ## worst confidence
          if icandidate_w>=0:
            xselector[icandidate_w]=1
            ## xselector[icandidate_b]=0     ## delete the best 
        elif xdatacls.ibootstrap==2:  ## best confidence
          if icandidate_b>=0:
            xselector[icandidate_b]=1
        elif xdatacls.ibootstrap==3:  ## worst+random
          if icandidate_w>=0:
            pselect=np.random.rand()
            if pselect<0.5:
              icandidate_w=np.random.randint(mtest)
              icandidate_w=xdatacls.itest[icandidate_w]
            xselector[icandidate_w]=1
            ## xselector[icandidate_b]=0     ## delete the best
      elif xdatacls.itestmode==1:   ## n-fold cross-validation
        ## !!! Emre !!!
        xselector=np.floor(np.random.random(ndata)*nfold0)
        xselector=xselector-(xselector==nfold0)

      ## if xdatacls.itestmode==1:  ## n-fold crossvalidation
      ##   xselector=np.random.randint(nfold0, size=ndata)
      ## elif xdatacls.itestmode==2:  ## random subset
      ##   xselector=1*(np.random.rand(ndata)<float(plist[ipar])/100)
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
## for test only
      elif xdatacls.itestmode==-1:
        for i in range(ndata):
          xselector[i]=i%nfold0
## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!          
##        xselector_row=np.floor(nfold0*np.random.rand(nrow))

      for ifold in range(nfold0):

        xdatacls.split_train_test(xselector,ifold)
        mtest=len(xdatacls.itest)
        if mtest<=0:
          print('!!!!!!!')
          break

        print('mtest:',mtest,'mtrain:',len(xdatacls.itrain))

        xdatacls.mvm_datasplit()        

    # sparse matrices of ranks-row_avarage-col_average+total_avarege  
        xdatacls.xranges_rel=mvm_ranges(xdatacls.xdata_tra,xdatacls.nrow, \
                                     params)
        xdatacls.xranges_rel_test=mvm_ranges(xdatacls.xdata_tes, \
                                          xdatacls.nrow,params)
        ## mvm_loadmatrix(xdatacls,isubset_tra,params)
        if xdatacls.category==0:
          mvm_glm(xdatacls,params)
          mvm_ygrid(xdatacls,params)
        elif xdatacls.category==1:
          mvm_largest_category(xdatacls)
        elif xdatacls.category==2:
          mvm_largest_category(xdatacls)

    # validation to choose the best parameters
        print('Validation')
        xdatacls.set_validation()
        params.validation.rkernel=xdatacls.XKernel[0].title
        if params.validation.rkernel in xdatacls.dkernels:
          kernbest=xdatacls.dkernels[params.validation.rkernel].kernel_params
        else:
          kernbest=xdatacls.XKernel[0].kernel_params
        
        if params.validation.ivalid==1:
          best_param=mvm_validation(xdatacls,params)
        else:
          best_param=cls_empty_class()
          best_param.c=xdatacls.penalty.c
          best_param.d=xdatacls.penalty.d
          best_param.par1=kernbest.ipar1
          best_param.par2=kernbest.ipar2

        xdatacls.penalty.c=best_param.c
        xdatacls.penalty.d=best_param.d
        kernbest.ipar1=best_param.par1
        kernbest.ipar2=best_param.par2

        print('Parameters:',xdatacls.penalty.c,xdatacls.penalty.d, \
              kernbest.ipar1,kernbest.ipar2)
        
        print('Best parameters found by validation')
        xbest_param[irepeat,ifold,0]=best_param.c
        xbest_param[irepeat,ifold,1]=best_param.d
        xbest_param[irepeat,ifold,2]=best_param.par1
        xbest_param[irepeat,ifold,3]=best_param.par2

    # training with the best parameters
        print('training')

        time0=time.time()
        cOptDual= xdatacls.mvm_train(params)
        print('Training time:',time.time()-time0)
        
    # cls transfers the dual variables to the test procedure
    # compute test 

    # check the train accuracy
        print('test on training')

    # $$$ # counts the proportion the ones predicted correctly    
    # $$$ # ######################################
    # $$$     deval=col_eval(xdatacls.ieval_type,nrow,isubset_tra, \
    # $$$                      xranges_tra,Zrow)
    # $$$     xresulttr(irepeat,ifold)=deval
    # ######################################     
    # check the test accuracy
        print('test on test')
        time0=time.time()
        cPredict=xdatacls.mvm_test(cOptDual.alpha,params)
        print('Test time:',time.time()-time0)

    # counts the proportion the ones predicted correctly
    # ####################################
        time0=time.time()
        (cEval,icandidate_w,icandidate_b)=mvm_eval(xdatacls.ieval_type, \
                                          xdatacls.nrow,xdatacls,cPredict.Zrow)
        print('Evaluation time:',time.time()-time0)

        if xdatacls.ieval_type==0:
          xresultte[irepeat,ifold,0]=cEval.accuracy
          ## prediction of effective categories
          ## part_accuracy=float(np.sum(np.diag(cEval.xconfusion)[1:]))/ \
          ##           np.sum(cEval.xconfusion[1:,1:])
          ## xresultte[irepeat,ifold,1]=part_accuracy
          xresultte[irepeat,ifold,1]=cEval.precision
          xresultte[irepeat,ifold,2]=cEval.recall
          xresultte[irepeat,ifold,3]=cEval.f1
        elif xdatacls.ieval_type==10:
          xresultte[irepeat,ifold,0]=cEval.accuracy
          xconfusion3[irepeat,ifold]=cEval.xconfusion3
        else:
          xresultte[irepeat,ifold,0]=cEval.deval
        icandidate_w=xdatacls.itest[icandidate_w]
        icandidate_b=xdatacls.itest[icandidate_b]
        ireport+=1

        ## print(cEval.xconfusion)
        if xdatacls.ieval_type!=10:
          for xconfrow in cEval.xconfusion:
            for ditem in xconfrow:
              print('%7.0f'%ditem,end='')
            print()
          print()
        else:
          for xtable in cEval.xconfusion3:
            xsum=np.sum(xtable)
            if xsum==0:
              xsum=1
            xtable=100*xtable/xsum
            for xconfrow in xtable:
              for ditem in xconfrow:
                print('%8.4f'%ditem,end='')
              print()
            print()
          print()
        
    # ####################################    
        print('*** ipar, repeatation, fold ***') 
        print(ipar,irepeat,ifold)

        if xdatacls.itestmode==1: ## n-fold crossvalidation
          print('Result in one fold and one repeatation')
          ## print('Accuracy on train')
          ## print(xresulttr[irepeat,ifold])
          print('Accuracy on test')
          if xdatacls.ieval_type==0:
            print(xresultte[irepeat,ifold])
          else:
            print(xresultte[irepeat,ifold,0])

      print('Result in one repetation')
      print('Mean and std of the accuracy on test')
      if xdatacls.ieval_type==0:
        print(np.mean(xresultte[irepeat,:,0]),
            np.std(xresultte[irepeat,:,0]))
      else:
        print(np.mean(xresultte[irepeat,:,0]),
            np.std(xresultte[irepeat,:,0]))
        
      sys.stdout.flush()
        
      if xdatacls.itestmode==0: ## n-fold crossvalidation
        np.savetxt(fname,xresultte[:ireport,0,:],delimiter=',',fmt='%6.4f')
      else:
        if xdatacls.ieval_type==0:
          np.savetxt(fname,xresultte[:ireport,:,:],delimiter=',',fmt='%6.4f')
        else:
          np.savetxt(fname,xresultte[:ireport,:,0],delimiter=',',fmt='%6.4f')

    print('***** Overall result ****')
    print('Mean and std of the accuracy on test + error')
    if xdatacls.ieval_type==0:
      print(np.mean(xresultte[:,:,0]),
            np.std(xresultte[:,:,0]))
    else:
      print(np.mean(xresultte[:,:,0]),
            np.std(xresultte[:,:,0]))

#     if xdatacls.ieval_type==10:
#       confusion_latex(xconfusion3,lfiles)      
      
    print('Average best parameters')
    ##  sfield=dir(best_param)
    xlabels=('c','d','par1','par2')
    for i in range(nparam):
    ##    print(sfield[i])
      print(xlabels[i],': ',np.mean(xbest_param[:,:,i]), \
              '(',np.std(xbest_param[:,:,i]),')')
  
  ## np.savetxt(fname,xresultte[:ireport,0,:],delimiter=',',fmt='%6.4f')
  print('Bye')    
  
  return

## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  roar_main(iworkmode)
