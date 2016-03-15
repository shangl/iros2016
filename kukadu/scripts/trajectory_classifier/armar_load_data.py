## ##################################################
import numpy as np
import scipy.signal as spsignal
import scipy.io

## import scipy.linalg as sp_linalg
import mmr_kernel_explicit
## import mmr_multic_label
import mmr_base_classes as base
## import mmr_kernel_subspace
from mmr_initial_params import cls_initial_params
import mmr_kernel_eval
## import tensor_decomp

## ###################################################
## ##################################################
class cls_label_files:

  def __init__(self,trainOutput,trainInput,dualParams,kernelParams):
    """
    """

    self.sbasedir='data/' 
    self.training_input=trainInput
    self.training_output=trainOutput
    self.dualParams=dualParams
    self.kernelParams=kernelParams
    self.training_center=None

    self.par1=0
    self.par2=0
    
    return

  ## --------------------------------------------------------------
  def load_mmr(self,cData):

    X_in=np.loadtxt(self.training_input)
    X_out=np.loadtxt(self.training_output)
    alpha=np.loadtxt(self.dualParams)
    kernelParams=np.loadtxt(self.kernelParams)

    (m,nx)=X_in.shape
    ny=X_out.shape[1]

    cData.training_center=np.mean(X_in,axis=0)

    ## loading the offline training parameters
    cData.dual=base.cls_dual(alpha,np.zeros(ny)+0.1)

    ## add an empty test row to the trainig,
    ## it will be replaced with the real test inputs 
    X_in=[np.vstack((X_in,np.zeros((1,nx))))] ## list of views
    X_out=np.vstack((X_out,np.zeros((1,ny))))

    ## subspace output kernel
    cData.YKernel=mmr_kernel_explicit.cls_feature(ifeature=0)
    cData.YKernel.load_data(X_out,ifeature=0)
    cData.YKernel.ifeature=0
    cData.YKernel.title='output'
    ## setting output parameters
    cparams=cls_initial_params()
    cData.YKernel.kernel_params.set(cparams.get_yparams('kernel',0))
    cData.YKernel.crossval.set(cparams.get_yparams('cross',0))
    cData.YKernel.norm.set(cparams.get_yparams('norm',0))

    idata=0
    nview=1
    for iview in range(nview):
      cData.XKernel[idata]=mmr_kernel_explicit.cls_feature(ifeature=0)
      cData.XKernel[idata].load_data(X_in[iview],ifeature=0)
      cData.XKernel[idata].title='input_'+str(iview)


      ## setting input parameters
      cData.XKernel[idata].kernel_params.set(cparams. \
                                           get_xparams('kernel',idata))
      ## kernel parameters
      cData.XKernel[idata].kernel_params.ipar1=kernelParams[0]
      cData.XKernel[idata].kernel_params.ipar2=kernelParams[1]

      cData.XKernel[idata].crossval.set(cparams.get_xparams('cross',idata))
      cData.XKernel[idata].norm.set(cparams.get_xparams('norm',idata))
      idata+=1

    cData.ninputview=idata  ## set active views
    cData.mdata=cData.YKernel.dataraw.shape[0]

    cData.nfold=1
    cData.nrepeat=1
    cData.kmode=1   ## =0 additive (feature concatenation)
                    ## =1 multiplicative (fetaure tensor product)

    return
## ###################################################
