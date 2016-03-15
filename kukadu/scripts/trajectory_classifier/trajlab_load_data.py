## ##################################################
import numpy as np
import scipy.signal as spsignal

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

  def __init__(self, trainingBase, evalFile, performClassification):
    """
    """

    self.sbasedir = trainingBase
    self.evaldir = evalFile
    self.labelfile='labels'
    self.loadFurther = performClassification
    ## self.linputmask=[(0,7),(7,10),(10,13)]
    self.linputmask=[(14,21),(21,24),(24,27)]
    
    return

  ## --------------------------------------------------------------
  def load_mmr(self, cData, lviews):
    
    ldata_labels = self.read_raw_txt(self.sbasedir,self.labelfile)
    if self.loadFurther==1:
      ldata_labels.append([self.evaldir, 1])
    mdata=len(ldata_labels)

    ## !!!!!!!!!!!!!!!!!!!!!!!!!!
    ## fixed number of categories od sides
    ncategory=4
    ## ncategory=3
    category_recode=[0,1,2,3]
    ## ####################################
    X_out=np.zeros((mdata,ncategory))
    for i in range(mdata):
      ## X_out[i,int(ldata_labels[i][1])-1]=1
      icategory=int(ldata_labels[i][1])-1
      if icategory==0:
        X_out[i,category_recode[icategory]]=1
      else:
        if ncategory==3:
          X_out[i,category_recode[icategory]]=1
        else:
          X_out[i,category_recode[icategory]]=1

    ## number of views: joint force, cart force, cart torque
    nview=len(self.linputmask)

    ## find common minimum length of trajectories
    xlength=np.zeros(mdata)
    for i in range(mdata):
      X=np.loadtxt(ldata_labels[i][0],skiprows=1)
      xlength[i]=X.shape[0]

    ixin_type=0   ## =0 simple vectorized =1 angles
    ixin_decompose=0  ## =0 no, =1 tensor decomposition
    icumulant=0   ## =0 no cumulants, =1 cumulants are the feature components 

    nmin=xlength.min()

    ## !!!!!!!!!!!!!!!!!!!!!!
    if nmin>240:
      if ixin_decompose==1:
        nmin=240    ## 8*6*5
    tfactors=(30,8,1,1)
    nfactor=len(tfactors)
    nlayer=1
    
    ncumulant=5
    if ncumulant>tfactors[0]:
      ncumulant=tfactors[0]
    if icumulant==0:
      nitem=tfactors[0]
    else:
      nitem=ncumulant
        

    if ixin_type==0:
      nboot=1
      if ixin_decompose==0:
        X_in=[]
        for i in range(nview):
          nslice=self.linputmask[i][1]-self.linputmask[i][0]
          X_in.append(np.zeros((mdata,nboot*nmin*nslice)))
          
      elif ixin_decompose==1:
        
        X_in=[]
        for i in range(nview):
          nslice=self.linputmask[i][1]-self.linputmask[i][0]
          X_in.append(np.zeros((mdata,nlayer*nitem*nslice)))

      for i in range(mdata):
        ## if i==18:
        ##   print(i)
        X=np.loadtxt(ldata_labels[i][0],skiprows=1)
        X=X[:nmin,:]
        if ixin_decompose==0:
          for j in range(nview):
            i0=self.linputmask[j][0]
            ik=self.linputmask[j][1]
            ## fX=X[:,i0:ik].ravel()
            ## fX=np.sort(fX)
            ## X_in[j][i]=fX
            X_in[j][i]=X[:,i0:ik].ravel()

        elif ixin_decompose==1:
          for j in range(nview):
            i0=self.linputmask[j][0]
            ik=self.linputmask[j][1]
            xmatrix=X[:,i0:ik]
            ## xamtrix=spsignal.savgol_filter(xmatrix,7,5,deriv=0,axis=0, \
            ##                            mode='interp')
            ## xfactors assume 240 rows
            xfactors=np.array([[tfactors[0],ik-i0],[tfactors[1],1], \
                               [tfactors[2],1],[tfactors[3],1]])
            ctensor=tensor_decomp.cls_tensor_decomp()
            ctensor.load(xmatrix,xfactors)
            niter=10
            ctensor.decompose2(niter)

            nslice=self.linputmask[j][1]-self.linputmask[j][0]
            fX=np.zeros(nlayer*nitem*nslice)
            ipos=0
            for ilayer in range(nlayer):
              for ifactor in range(1):
                zfact=ctensor.lcomponents[ilayer][ifactor]
                (mzfact,nzfact)=zfact.shape
                for icol in range(nzfact):
                  if np.mean(zfact[:,icol])<0:
                    zfact[:,icol]=-zfact[:,icol]
                if icumulant==1:
                  for icol in range(nzfact):
                    xcumulants=cumulants(zfact[:,icol],ncumulant,icentral=1)
                    fX[ipos:ipos+nitem]=xcumulants
                    ipos+=nitem
                else:
                  ## (u,s,v)=np.linalg.svd(zfact)
                  ## zfact=np.dot(u[:,:nzfact],np.diag(s[:nzfact]))
                  for icol in range(nzfact):
                    fX[ipos:ipos+mzfact]=zfact[:,icol]
                    ipos+=mzfact

            ## fX=np.sort(fX)
            X_in[j][i]=fX
          
    elif ixin_type==1:
      X_in=[ np.zeros((mdata,nmin-1)) for i in range(nview)]

      for i in range(mdata):
        X=np.loadtxt(self.sbasedir+ldata_labels[i][0],skiprows=1)
        X=X[:nmin,:]
        for j in range(nview):
          i0=self.linputmask[j][0]
          ik=self.linputmask[j][1]
          X_in[j][i]=angle_moment(X[:,i0:ik])
       

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
## ---------------------------
  def read_raw_txt(self,sdir,sfile):

    fin=open(sdir+sfile,'rb')
    xbytes=fin.read()
    fin.close()

    xstring=xbytes.decode("utf-8",'replace')
    xstokens=xstring.split('\n')

    ldata=[]
    for xline in xstokens:
      xline.strip()
      if len(xline)==0:
        continue
      stokens=xline.split()
      #stokens[0] = stokens[0].join(sdir)
      fName = stokens[0]
      stokens[0] = ''.join([sdir, fName])
      ldata.append(stokens)

    return(ldata)
  
## ###################################################
def angle_moment(X):

  (m,n)=X.shape

  xam=np.zeros(m-1)

  for i in range(m-1):
    x1=X[i]
    x2=X[i+1]
    xcorr=np.dot(x1,x2)/np.sqrt(np.dot(x1,x1)*np.dot(x2,x2))
    if xcorr>1:
      xcorr=1
    if xcorr<-1:
      xcorr=-1
    xam[i]=np.arccos(xcorr)
    
  return(xam)  
## ###################################################
def moments(x,nmoment,icentral=0):
  """
  x is vector
  nmoments number of moments
  icentral  =0 moments =1 central moments
  """
  xmoments=np.zeros(nmoment)
  xmean=np.mean(x)
  xmoments[0]=xmean
  for i in range(1,nmoment):
    if icentral==1:
      xmoments[i]=np.mean((x-xmean)**(i+1))
    else:
      xmoments[i]=np.mean(x**(i+1))
      

  return(xmoments)
## ###################################################
def cumulants(x,ncumulant,icentral=0):

  xcumulants=np.zeros(ncumulant)
  if icentral==0:
    xmoments=moments(x,ncumulant,icentral=0)
    nfactor=1
    xcumulants=np.copy(xmoments)
    for i in range(1,ncumulant):
      nfactor=1
      for j in range(1,i):
        xcumulants[i]-=nfactor*xcumulants[j-1]*xmoments[i-j]
        nfactor*=(i+1-j)/j
  else:
    xmoments=moments(x,ncumulant,icentral=1)
    nfactor=1
    xcumulants=np.copy(xmoments)
    xcumulants[0]=0
    xmoments[0]=0
    for i in range(1,ncumulant):
      nfactor=1
      for j in range(1,i):
        xcumulants[i]-=nfactor*xcumulants[j-1]*xmoments[i-j-1]
        nfactor*=(i+1-j)/j
    
  return(xcumulants)


## ###################################################


  
