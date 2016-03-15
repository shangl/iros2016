######################
## Version 0.1 #######
######################

import sys, time
## import pickle
import numpy as np
## import scipy.io 
## import pylab as plt

## ##########################################
## import mmr_base_classes
import mmr_setparams
import mmr_mmr_cls
import objpart_load_data
import mmr_validation_cls
from mmr_test import inverse_knn
from mmr_report import mmr_report
from mmr_eval import mmr_eval_binvector
## #################################
def mmr_main(iworkmode):

  params=mmr_setparams.cls_params()
  
  np.set_printoptions(precision=4)
  
  dresult={}
## ---------------------------------------------
  list_object=['parts']
  nviews=1
  lfile_in=[[[(3,4,21)]]]
  tfile_out=(3,1,6)

  lresult=[]
  for iobject in range(len(lfile_in)):

    tfile_in=lfile_in[iobject]
    
    for ifeature in range(nviews):

      params.ninputview=len(tfile_in)
      cMMR=mmr_mmr_cls.cls_mmr(params.ninputview)
      nfold=cMMR.nfold
      nrepeat=cMMR.nrepeat
      cMMR.xbias=-0.95-ifeature*0.05
      print('Xbias:',cMMR.xbias)

      nscore=4
      nipar=1
      if cMMR.crossval_mode==0:   ## random
        nfold0=nfold
        xresult_test=np.zeros((nipar,nrepeat,nfold0))
        xresult_train=np.zeros((nipar,nrepeat,nfold0))
        xpr=np.zeros((nipar,nrepeat,nfold0,nscore))
      elif cMMR.crossval_mode==1:  ## predefined trianing and test
        nrepeat=1
        nfold0=1
        xresult_test=np.zeros((nipar,nrepeat,nfold0))
        xresult_train=np.zeros((nipar,nrepeat,nfold0))
        xpr=np.zeros((nipar,nrepeat,nfold0,nscore))

    ## @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

      cdata_store=objpart_load_data.cls_label_files()  
      cdata_store.load_mmr(cMMR,tfile_in,tfile_out)
      mdata=cMMR.mdata

      ## -----------------------------------------------
      print('Output kernel type: ',cMMR.YKernel.kernel_params.kernel_type)
      for i in range(params.ninputview):
        print(i,'Input kernel type: ',cMMR.XKernel[i].kernel_params.kernel_type)
      ## -------------------------------------------------

      xtime=np.zeros(5)
    ## ############################################################
      nparam=4    ## C,D,par1,par2
      xbest_param=np.zeros((nrepeat,nfold0,nparam))

      ## xinpar=[0.2,0.3,0.4,0.5,0.6]
      for iipar in range(nipar):

        print('===================================================')
        for irepeat in range(nrepeat):
        ## split data into training and test
          if cMMR.crossval_mode==0:  ## random selection
            xselector=np.floor(np.random.random(mdata)*nfold0)
            xselector=xselector-(xselector==nfold0)
          elif cMMR.crossval_mode==1: ## preddefined training and test
            xselector=np.zeros(mdata)
            xselector[cMMR.ifixtrain]=1

          for ifold in range(nfold0):
            cMMR.split_train_test(xselector,ifold)

            ## validation to choose the best parameters
            print('Validation')
            t0=time.clock()
            ## select the kernel to be validated
            cMMR.set_validation()

            cvalidation=mmr_validation_cls.cls_mmr_validation()
            cvalidation.validation_rkernel=cMMR.XKernel[0].title
            best_param=cvalidation.mmr_validation(cMMR)
          
            xtime[0]=time.clock()-t0

            print('Best parameters found by validation')
            print('c: ',best_param.c)
            print('d: ',best_param.d)
            print('par1: ',best_param.par1)
            print('par2: ',best_param.par2)
            xbest_param[irepeat,ifold,0]=best_param.c
            xbest_param[irepeat,ifold,1]=best_param.d
            xbest_param[irepeat,ifold,2]=best_param.par1
            xbest_param[irepeat,ifold,3]=best_param.par2

            cMMR.compute_kernels()
            cMMR.Y0=cMMR.YKernel.get_train(cMMR.itrain)   ## candidates

      ## training with the best parameters
            print('Training')

            print(cMMR.YKernel.kernel_params.kernel_type, \
                  cMMR.YKernel.kernel_params.ipar1, \
                  cMMR.YKernel.kernel_params.ipar2)
            for iview in range(cMMR.ninputview):
              print(cMMR.XKernel[iview].kernel_params.kernel_type, \
                    cMMR.XKernel[iview].kernel_params.ipar1, \
                    cMMR.XKernel[iview].kernel_params.ipar2)
              
            
            t0=time.clock()
            cOptDual=cMMR.mmr_train()
            xtime[1]=time.clock()-t0
      ## cls transfers the dual variables to the test procedure
      ## compute tests 
      ## check the train accuracy
            print('Test')
            cPredictTra=cMMR.mmr_test(cOptDual,itraindata=0)
      ## counts the proportion the ones predicted correctly    
      ## ######################################
            if cMMR.itestmode==2:
              print('Test knn')
              ypred=inverse_knn(cMMR.YKernel.get_Y0(cMMR.itrain), \
                                cPredictTra)
            else:
              ypred=cPredictTra.zPred
            cEvaluationTra= \
                  mmr_eval_binvector(cMMR.YKernel.get_train(cMMR.itrain), \
                                     ypred)
            xresult_train[iipar,irepeat,ifold]=cEvaluationTra.accuracy
            print('>>>>>>>>>>>\n',cEvaluationTra.confusion)
      ## ######################################     
      ## check the test accuracy
            t0=time.clock()
            cPredictTes= cMMR.mmr_test(cOptDual,itraindata=1)
      ## counts the proportion the ones predicted correctly
            if cMMR.itestmode==2:
              ypred=inverse_knn(cMMR.YKernel.get_Y0(cMMR.itrain), \
                                cPredictTes)
            else:
              ypred=cPredictTes.zPred
            ## cEvaluationTes=mmr_eval_binvector(cData.YTest,cPredictTes.zPred)
            cEvaluationTes= \
                  mmr_eval_binvector(cMMR.YKernel.get_test(cMMR.itest), \
                                     ypred)

            xtime[2]=time.clock()-t0
            xresult_test[iipar,irepeat,ifold]=cEvaluationTes.accuracy

            xpr[iipar,irepeat,ifold,0]=cEvaluationTes.precision
            xpr[iipar,irepeat,ifold,1]=cEvaluationTes.recall
            xpr[iipar,irepeat,ifold,2]=cEvaluationTes.f1
            xpr[iipar,irepeat,ifold,3]=cEvaluationTes.accuracy

            print(cEvaluationTes.confusion)
            print(cEvaluationTes.classconfusion)
            try:
              xclassconfusion+=cEvaluationTes.classconfusion
            except:
              (n,n)=cEvaluationTes.classconfusion.shape
              xclassconfusion=np.zeros((n,n))
              xclassconfusion+=cEvaluationTes.classconfusion

      ## ####################################
            print('Parameter:',iipar,'Repetition: ',irepeat, \
                  'Fold: ',ifold)
            mmr_report('Result on one fold',
                       xresult_train[iipar,irepeat,ifold], \
                       xresult_test[iipar,irepeat,ifold], \
                       xpr[iipar,irepeat,ifold,:])
            print(np.sum(xpr[iipar,irepeat,:ifold+1,:],0)/(ifold+1))

          mmr_report('Result on one repetition',
                     np.mean(xresult_train[iipar,irepeat,:]), \
                     np.mean(xresult_test[iipar,irepeat,:]), \
                     np.mean(xpr[iipar,irepeat,:,:],0))

        mmr_report('Result on all repetitions @@@@@@@',
                   np.mean(xresult_train[iipar,:,:].flatten()), \
                   np.mean(xresult_test[iipar,:,:].flatten()), \
                   np.mean(np.mean(xpr[iipar,:,:,:],0),0))



        print('Average best parameters')
      ##  sfield=dir(best_param)
        xlabels=('c','d','par1','par2')
        for i in range(nparam):
      ##    print(sfield[i])
          print(xlabels[i],': ',np.mean(xbest_param[:,:,i]), \
                '(',np.std(xbest_param[:,:,i]),')')

        print('xtime:',xtime)
        sys.stdout.flush()

        dresult[ifeature]=(cMMR.xbias,np.mean(np.mean(xpr[iipar,:,:,:],0),0))

    for sfeature_type,tresult in dresult.items():
      ## xhead=cMMR.xbias
      headkey=tfile_in[0][0]
      xhead=cdata_store.dirvar[headkey][0]+', '+cdata_store.dirvar[headkey][1]
      lresult.append((xhead,tresult))

    ## lresult.sort()
    ## for litem in lresult:
    ##   print(litem)

    print('\\begin{tabular}{l|rrr}')
    print('& \\multicolumn{3}{c}{'+'Objects'+'} \\\\')
    print('Feature type & Precision & Recall & F1 \\\\ \\hline')
    for litem in lresult:
      print(litem[0],' & ','%6.4f'%litem[1][1][0], \
            ' & ','%6.4f'%litem[1][1][1],' & ','%6.4f'%litem[1][1][2],' \\\\')
    print('\\end{tabular}')  

  print(xclassconfusion)

  print('Bye')
  
  return
  
## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  mmr_main(iworkmode)

