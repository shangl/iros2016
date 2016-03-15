######################
## Version 0.1 #######
######################

import numpy as np
## #############################################################
class cls_mmr_report:

  def __init__(self):

    self.xresulttra=None
    self.xresulttes=None
    self.xaprf=None

  ## --------------------------------------------
  def create_xaprf(self,nrepeat=1,nfold=1,nreport=1):

    self.xaprf=np.zeros((nrepeat,nfold,nreport))
  ## --------------------------------------------
  def set_xaprf(self,irepeat,ifold,cEval):

    self.xaprf[irepeat,ifold,0]=cEval.deval
    self.xaprf[irepeat,ifold,1]=cEval.precision
    self.xaprf[irepeat,ifold,2]=cEval.recall
    self.xaprf[irepeat,ifold,3]=cEval.f1
   
  ## --------------------------------------------
  def report(self,stitle):

    print('***** '+stitle+' *****')
    print('Mean and std of the accuracy on train + error')
    print(self.xresulttra)
    print('Mean and std of the accuracy on test + error')
    print(self.xresulttes)
    print('Precision, recall, f1')
    print(self.xpr)

  ## --------------------------------------------
  def report_prf(self,xmask=[],stitle=None,ssubtitle=None,ieval_type=1,iprint=1):
    """
    xaprf     3 dim array,
                firts index is repetition,
                second index is fold
                third index is parameter:
                  accuracy(0), precision(1), recall(2), f1(3)  
    """

    if stitle is not None:
      print('***** '+stitle+' *****')
    if ssubtitle is not None:
      print('***** '+ssubtitle+' *****')

    tdim=self.xaprf.shape
    ndim=len(tdim)

    lmask=[None]*(ndim-1)
    for i in range(len(xmask)):
      if xmask[i] is None:
        lmask[i]=slice(0,tdim[i])
      else:
        lmask[i]=xmask[i]

    xmean=np.zeros(tdim[ndim-1])
    xstd=np.zeros(tdim[ndim-1])
    for i in range(tdim[ndim-1]):
      tmask=tuple(lmask+[i])
      xmean[i]=np.mean(self.xaprf[tmask])
      xstd[i]=np.std(self.xaprf[tmask])

    if iprint==1:
      print(xmean)
      print(xstd)

    return(xmean,xstd)  
    
## #############################################################


