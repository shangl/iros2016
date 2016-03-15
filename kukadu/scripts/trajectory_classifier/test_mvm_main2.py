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
import sys, time
import numpy as np
## ################################################
import mmr_base_classes
import mvm_mvm_cls
import mmr_setparams
import mvm_random_matrix
from mvm_eval import mvm_eval, confusion_latex, makearray
import mvm_validation_cls
import mmr_report_cls
## ################################################
def test_mvm_main(workmode):

  params=mmr_setparams.cls_params()

  xdatacls=mvm_mvm_cls.cls_mvm()
  nfold=xdatacls.nfold
  if xdatacls.itestmode==0:
    nfold0=1        ## active learning
  else:
    nfold0=nfold    ## n-fold cross validation

  nparacc=2   ## rmse, time
  npar=1
  xsummary=np.zeros((npar,nparacc))
  
  ifile=0
  pselect=0.05
  itrates=1
  print('ifile:',ifile)
  print('itrates:',itrates)
  print('pselect:',pselect)
  lfiles=[]
  
  for ipar in range(npar):

    rmatrix=mvm_random_matrix.cls_label_files()
    (xdata,nrow2,ncol2)=rmatrix.load(ifile,pselect,itrain=itrates)
    xdatacls.load_data(xdata,xdatacls.categorymax, \
                       int(nrow2),int(ncol2),None)
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

    xdatacls.YKernel.ymax=1
    # it will be recomputed in mvm_ranges
    xdatacls.YKernel.ymin=-1
    xdatacls.YKernel.yrange=200 # it will be recomputed in classcol_ranges
    xdatacls.YKernel.ystep=(xdatacls.YKernel.ymax-xdatacls.YKernel.ymin) \
                            /xdatacls.YKernel.yrange
    ##  set_printoptions(precision=4)
    nparam=4    # C,D,par1,par2
    nreport=4   ## accuracy, precision, recall, f1

    xdatacls.prepare_repetition_folding(init_train_size=100)
    nrepeat0=xdatacls.nrepeat0
    nfold0=xdatacls.nfold0

    creport=mmr_report_cls.cls_mmr_report()
    creport.create_xaprf(nrepeat=nrepeat0,nfold=nfold0,nreport=nreport)
    xbest_param=np.zeros((nrepeat0,nfold0,nparam))

    # ############################################################

    nval=max(xdatacls.YKernel.valrange)+1
    xconfusion3=np.zeros((nrepeat0,nfold0,xdatacls.YKernel.ndim,nval,nval))

    xsolvertime=0.0
    ireport=0
    for irepeat in range(nrepeat0):

      xdatacls.prepare_repetition_training()

      for ifold in range(nfold0):

        xdatacls.prepare_fold_training(ifold)

    # validation to choose the best parameters
        print('Validation')
        xdatacls.set_validation()
        cvalidation=mvm_validation_cls.cls_mvm_validation()
        cvalidation.validation_rkernel=xdatacls.XKernel[0].title
        best_param=cvalidation.mvm_validation(xdatacls)

        print('Parameters:',best_param.c,best_param.d, \
              best_param.par1,best_param.par2)

        print('Best parameters found by validation')
        xbest_param[irepeat,ifold,0]=best_param.c
        xbest_param[irepeat,ifold,1]=best_param.d
        xbest_param[irepeat,ifold,2]=best_param.par1
        xbest_param[irepeat,ifold,3]=best_param.par2

    # training with the best parameters
        print('training')

        time0=time.time()
        cOptDual= xdatacls.mvm_train()
        xsolvertime+=xdatacls.solvertime
        print('Training time:',time.time()-time0)
        sys.stdout.flush()

    # check the train accuracy
        print('test on training')

    # check the test accuracy
        print('test on test')
        time0=time.time()
        cPredict=xdatacls.mvm_test()
        print('Test time:',time.time()-time0)
        sys.stdout.flush()

    # counts the proportion the ones predicted correctly
    # ####################################
        time0=time.time()
        (cEval,icandidate_w,icandidate_b)=mvm_eval(xdatacls.ieval_type, \
                                          xdatacls.nrow,xdatacls,cPredict.Zrow)
        print('Evaluation time:',time.time()-time0)
        (qtest,qpred)=makearray(xdatacls,cPredict.Zrow)

        if xdatacls.ieval_type==0:
          creport.set_xaprf(irepeat,ifold,cEval)
        elif xdatacls.ieval_type==10:
          creport.set_xaprf(irepeat,ifold,cEval)
          xconfusion3[irepeat,ifold]=cEval.xconfusion3
        else:
          creport.set_xaprf(irepeat,ifold,cEval)

        xdatacls.icandidate_w=xdatacls.itest[icandidate_w]
        xdatacls.icandidate_b=xdatacls.itest[icandidate_b]
        ireport+=1

        ## print(cEval.xconfusion)
        if xdatacls.ieval_type==0:
          for xconfrow in cEval.xconfusion:
            for ditem in xconfrow:
              print('%7.0f'%ditem,end='')
            print()
          print()
        elif xdatacls.ieval_type==10:
          for xtable in cEval.xconfusion3:
            xsum=np.sum(xtable)
            if xsum==0:
              xsum=1
            xtable=100*xtable/xsum
            for xconfrow in xtable:
              for ditem in xconfrow:
                print('%9.4f'%ditem,end='')
              print()
            print()
          print()
        
    # ####################################    
        print('*** ipar, repeatation, fold ***') 
        print(ipar,irepeat,ifold)
        
        if xdatacls.itestmode==1: ## n-fold crossvalidation

          creport.report_prf(xmask=[irepeat,ifold], \
                             stitle='Result in one fold and one repetation', \
                             ssubtitle='Accuracy on test')

      creport.report_prf(xmask=[irepeat,None], \
                         stitle='Result in one repetation', \
                         ssubtitle='Mean and std of the accuracy on test')

      sys.stdout.flush()

      if xdatacls.itestmode==0: ## n-fold crossvalidation
        np.savetxt(fname,creport.xresulttes[:ireport,0,:],delimiter=',', \
                   fmt='%6.4f')
      else:
        if xdatacls.ieval_type==0:
          np.savetxt(fname,np.squeeze(creport.xaprf),delimiter=',', \
                     fmt='%6.4f')
        else:
          np.savetxt(fname,creport.xaprf[:,:,0],delimiter=',',fmt='%6.4f')

    (xmean,xstd)=creport.report_prf(xmask=[None,None], \
                     stitle='***** Overall result ****', \
                     ssubtitle='Mean and std of the accuracy on test + error')

    xsummary[ipar,0]=xmean[0]
    xsummary[ipar,1]=xsolvertime/(nrepeat0*nfold0)                          

    if xdatacls.ieval_type==10:
      confusion_latex(xconfusion3,lfiles)      
      
    print('Average best parameters')
    xlabels=('c','d','par1','par2')
    for i in range(nparam):
      print(xlabels[i],': ',np.mean(xbest_param[:,:,i]), \
              '(',np.std(xbest_param[:,:,i]),')')

  print('$$$$$$$$$ Summary results:')
  (m,n)=xsummary.shape
  for i in range(m):
    for j in range(n):
      print('%10.4f'%xsummary[i,j],end='')
    print()

  ## np.savetxt(fname,xresultte[:ireport,0,:],delimiter=',',fmt='%6.4f')
  print('Bye')    
  
  return

## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  test_mvm_main(iworkmode)
