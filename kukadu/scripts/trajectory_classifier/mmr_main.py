######################
## Version 0.1 #######
## /**********************************************************************
##   Copyright 2015, Sandor Szedmak  
##   email: sandor.szedmak@uibk.ac.at
##          szedmak777@gmail.com
##
##   This file is part of Maximum Margin Regression code(MMR).
##
##   MMR is free software: you can redistribute it and/or modify
##   it under the terms of the GNU General Public License as published by
##   the Free Software Foundation, either version 3 of the License, or
##   (at your option) any later version. 
##
##   MMR is distributed in the hope that it will be useful,
##   but WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##   GNU General Public License for more details.
##
##   You should have received a copy of the GNU General Public License
##   along with MMR.  If not, see <http://www.gnu.org/licenses/>.
##
## ***********************************************************************/
######################
import sys, time
import numpy as np
## ##########################################
import mmr_base_classes
import mmr_setparams
import mmr_load_data
import mmr_mmr_cls
import mmr_validation_cls
from mmr_test import inverse_knn
import mmr_report
from mmr_eval import mmr_eval_binvector
## ---------------------------------
## #################################
def mmr_main(iworkmode):

  params=mmr_setparams.cls_params()



## @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  list_features=["annot","DenseHue","DenseHueV3H1", \
                "DenseSift","DenseSiftV3H1","Gist", \
                "HarrisHue","HarrisHueV3H1","HarrisSift", \
                "HarrisSiftV3H1","Hsv","HsvV3H1","Lab", \
                "LabV3H1","Rgb","RgbV3H1"]

  ## data files in the corresponding directories
  datadirs=['corel5k','espgame','iaprtc12','mirflickr','pascal07']

  ## Example lfeatures=[4,8] means we selected the features:
  ##                                "DenseSiftV3H1" and  "HarrisSift"   
  lfeatures=[4]
  params.ninputview=len(lfeatures)
  idata=0       ## process corel5k data set

  xdatacls=mmr_mmr_cls.cls_mmr(params.ninputview)
  nfold=xdatacls.nfold
  nrepeat=xdatacls.nrepeat
  print('Xbias:',xdatacls.xbias)

  cdata_store=mmr_load_data.cls_data_load()  
  cdata_store.load_data(xdatacls,idata,lfeatures)
  mdata=xdatacls.mdata

  ## initializing the array collecting the results
  nscore=4
  nipar=1
  if xdatacls.crossval_mode==0:   ## random
    nfold0=nfold
    xresult_test=np.zeros((nipar,nrepeat,nfold0))
    xresult_train=np.zeros((nipar,nrepeat,nfold0))
    xpr=np.zeros((nipar,nrepeat,nfold0,nscore))
  elif xdatacls.crossval_mode==1:  ## predefined trianing and test
    nrepeat=1
    nfold0=1
    xresult_test=np.zeros((nipar,nrepeat,nfold0))
    xresult_train=np.zeros((nipar,nrepeat,nfold0))
    xpr=np.zeros((nipar,nrepeat,nfold0,nscore))


  ## -----------------------------------------------
  print('Output kernel type: ',xdatacls.YKernel.kernel_params.kernel_type)
  for i in range(params.ninputview):
    print(i,'Input kernel type: ',xdatacls.XKernel[i].kernel_params.kernel_type)
  ## -------------------------------------------------

  xcross=np.zeros((mdata,mdata))

  xtime=np.zeros(5)
## ############################################################
  nparam=4    ## C,D,par1,par2
  xbest_param=np.zeros((nrepeat,nfold0,nparam))

  for iipar in range(nipar):

    print('===================================================')
    for irepeat in range(nrepeat):

      xdatacls.prepare_repetition_training(nfold0)

      for ifold in range(nfold0):

        xdatacls.prepare_fold_training(ifold)

        ## validation to choose the best parameters
        print('Validation')
        t0=time.clock()
        xdatacls.set_validation()
        cvalidation=mmr_validation_cls.cls_mmr_validation()
        cvalidation.validation_rkernel=xdatacls.XKernel[0].title
        best_param=cvalidation.mmr_validation(xdatacls)

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

        xdatacls.compute_kernels()
        xdatacls.Y0=xdatacls.YKernel.get_train(xdatacls.itrain)   ## candidates

  ## training with the best parameters
        print('Training')

        print(xdatacls.YKernel.kernel_params.kernel_type, \
              xdatacls.YKernel.kernel_params.ipar1, \
              xdatacls.YKernel.kernel_params.ipar2)
        for iview in range(xdatacls.ninputview):
          print(xdatacls.XKernel[iview].kernel_params.kernel_type, \
                xdatacls.XKernel[iview].kernel_params.ipar1, \
                xdatacls.XKernel[iview].kernel_params.ipar2)


        t0=time.clock()
        cOptDual=xdatacls.mmr_train()
        xtime[1]=time.clock()-t0
  ## cls transfers the dual variables to the test procedure
  ## compute tests 
  ## check the train accuracy
        print('Test')
        cPredictTra=xdatacls.mmr_test(cOptDual,itraindata=0)
  ## counts the proportion the ones predicted correctly    
  ## ######################################
        if xdatacls.itestmode==2:
          print('Test knn')
          ypred=inverse_knn(xdatacls.YKernel.get_Y0(xdatacls.itrain), \
                            cPredictTra)
        else:
          ypred=cPredictTra.zPred
        cEvaluationTra= \
              mmr_eval_binvector(xdatacls.YKernel.get_train(xdatacls.itrain), \
                                 ypred)
        xresult_train[iipar,irepeat,ifold]=cEvaluationTra.accuracy
        print('>>>>>>>>>>>\n',cEvaluationTra.confusion)
  ## ######################################     
  ## check the test accuracy
        t0=time.clock()
        cPredictTes= xdatacls.mmr_test(cOptDual,itraindata=1)
  ## counts the proportion the ones predicted correctly
        if xdatacls.itestmode==2:
          ypred=inverse_knn(xdatacls.YKernel.get_Y0(xdatacls.itrain), \
                            cPredictTes)
        else:
          ypred=cPredictTes.zPred
        ## cEvaluationTes=mmr_eval_binvector(cData.YTest,cPredictTes.zPred)
        cEvaluationTes= \
              mmr_eval_binvector(xdatacls.YKernel.get_test(xdatacls.itest), \
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
        ## mmr_eval_label(ZW,iPre,YTesN,Y0,kit_data,itest,params)

  ## ####################################
        print('Parameter:',iipar,'Repetition: ',irepeat, \
              'Fold: ',ifold)
        mmr_report.mmr_report('Result on one fold',
                   xresult_train[iipar,irepeat,ifold], \
                   xresult_test[iipar,irepeat,ifold], \
                   xpr[iipar,irepeat,ifold,:])
        print(np.sum(xpr[iipar,irepeat,:ifold+1,:],0)/(ifold+1))

      mmr_report.mmr_report('Result on one repetition',
                 np.mean(xresult_train[iipar,irepeat,:]), \
                 np.mean(xresult_test[iipar,irepeat,:]), \
                 np.mean(xpr[iipar,irepeat,:,:],0))

    mmr_report.mmr_report('Result on all repetitions @@@@@@@',
               np.mean(xresult_train[iipar,:,:].flatten()), \
               np.mean(xresult_test[iipar,:,:].flatten()), \
               np.mean(np.mean(xpr[iipar,:,:,:],0),0))



    print('Average best parameters')
    xlabels=('c','d','par1','par2')
    for i in range(nparam):
      print(xlabels[i],': ',np.mean(xbest_param[:,:,i]), \
              '(',np.std(xbest_param[:,:,i]),')')

    print('xtime:',xtime)
    sys.stdout.flush()

  print('Bye')
  
  return
  
## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  mmr_main(iworkmode)

