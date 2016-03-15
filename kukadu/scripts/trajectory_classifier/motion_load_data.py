## ##################################################
import numpy as np
import scipy.signal as spsignal
import scipy.io

## import scipy.linalg as sp_linalg
import mmr_kernel_explicit
import mmr_multic_label
## import mmr_kernel_subspace
from mmr_initial_params import cls_initial_params
import mmr_kernel_eval
import tensor_decomp

## ###################################################
## ##################################################
class cls_label_files:

  def __init__(self):
    """
    """

    self.sbasedir='/home/szedmak/data/motion_hand/' 
    self.datafile='test_ST.mat'
    
    return

  ## --------------------------------------------------------------
  def load_mmr(self,cData,lviews):

    dmat=scipy.io.loadmat(self.sbasedir+self.datafile)
    ytrain=dmat['labels']
    xtrain=dmat['his']

    ytrain=ytrain[0,:]
    (m,n)=xtrain.shape
    nmax=ytrain.max()
    nmin=ytrain.min()
    ny=nmax-nmin+1

    ncut=10000
    if ncut<m:
      isample=np.random.permutation(m)
      isample=isample[:ncut]
      m=ncut
    else:
      isample=np.arange(m)

    ytrain=ytrain[isample]
    xtrain=xtrain[isample]
    X_in=[xtrain]

    X_out=np.zeros((m,ny))
    for i in range(m):
      X_out[i,ytrain[i]-nmin]=1

    ## subspace output kernel
    cData.YKernel=mmr_kernel_explicit.cls_feature(ifeature=0)
    cData.YKernel.load_data(X_out,ifeature=0)
    cData.YKernel.ifeature=0
    cData.YKernel.title='output'
    ## setting output parameters
    cparams=cls_initial_params()
    cData.YKernel.kernel_params.set(cparams.get_yparams('kernel',0))
    ## cData.YKernel.prekernel_params.set(cparams.get_yinparams('kernel',0))
    cData.YKernel.crossval.set(cparams.get_yparams('cross',0))
    cData.YKernel.norm.set(cparams.get_yparams('norm',0))

    idata=0
    nview=1
    for iview in range(nview):
      if iview in lviews:
        cData.XKernel[idata]=mmr_kernel_explicit.cls_feature(ifeature=0)
        cData.XKernel[idata].load_data(X_in[iview],ifeature=0)
        cData.XKernel[idata].title='input_'+str(iview)

    ## setting input parameters
        cData.XKernel[idata].kernel_params.set(cparams. \
                                             get_xparams('kernel',idata))
        ## cData.XKernel[idata].prekernel_params.set(cparams. \
        ##                             get_xinparams('kernel',idata))
        cData.XKernel[idata].crossval.set(cparams.get_xparams('cross',idata))
        cData.XKernel[idata].norm.set(cparams.get_xparams('norm',idata))
        idata+=1

    cData.ninputview=idata  ## set active views
    cData.mdata=cData.YKernel.dataraw.shape[0]

    cData.nfold=2
    cData.nrepeat=2
    cData.kmode=1   ## =0 additive (feature concatenation)
                    ## =1 multiplicative (fetaure tensor product)

    return
## ###################################################
