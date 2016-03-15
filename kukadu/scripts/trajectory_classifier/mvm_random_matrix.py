## ##################################################
import numpy as np
import scipy.io
import PIL.Image as pilimg
## import gzip
## import simplejson

## import scipy.linalg as sp_linalg
import mvm_prepare
import tensor_decomp
## ###################################################
class cls_random_matrix:
  """
  Loads a matrix X and gives back the sparse mvm form:
                (row_index,column_index,value), in list of arrays

                pselect gives the selection probability of the matrix items
  """

  def __init__(self):

    self.X=None
    
    return
  ## -------------------------------------------------
  def load(self,X,imulty=0):

    if imulty==1:
      self.X=np.dot(X,X.T)
    else:
      self.X=X

    return
  ## -----------------------------------------------
  def mvm_form(self,pselect=1.0):

    xdata=[[],[],[]]    ## row index, column index , value

    (m,n)=self.X.shape

    k=0
    for i in range(m):
      px=np.random.rand(n)
      for j in range(n):
        if px[j]<pselect:
          xdata[0].append(i)
          xdata[1].append(j)
          xdata[2].append(self.X[i,j])
          k+=1
          
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)

    return(xdata)
  ## -----------------------------------------------
  def mvm_form_prod(self,X,pselect=1.0):

    xdata=[[],[],[]]    ## row index, column index , value

    (m,n)=X.shape

    X=X.reshape((1*m,n/1))
    ## X=np.dot(X,X.T)
    # n=m
    k=0
    icount=0
    for i in range(m):
      px=np.random.rand(m)
      for j in range(m):
        if px[j]<pselect:
          v=np.dot(X[i],X[j])
          ij=np.ravel_multi_index(([i],[j]),(m,m))[0]
          tij=np.unravel_index([ij],(m*1,m/1))
          xdata[0].append(tij[0][0])
          xdata[1].append(tij[1][0])
          xdata[2].append(v)
          k+=1
        icount+=1
        
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)
          
    return(xdata)

  ## -----------------------------------------------
  def mvm_form_prod_reorder(self,X,xfactors):

    xdata=[[],[],[]]    ## row index, column index , value

    (m,n)=X.shape
    X2=np.dot(X,X.T)

    if m==400:
      print(m,n)
      
    ctensor=tensor_decomp.cls_tensor_decomp()
    ctensor.load(X2,xfactors)
    X2=ctensor.reorder_in_x(X2)
    (m2,n2)=X2.shape
    xmean=np.mean(X2)
    xstd=np.std(X2)
    lbound=xmean-5*xstd
    ubound=xmean+5*xstd
    
    k=0
    icount=0
    for i in range(m2):
      for j in range(n2):
        v=X2[i,j]
        if v>=lbound and v<=ubound:
          xdata[0].append(i)
          xdata[1].append(j)
          xdata[2].append(v)
          k+=1
        icount+=1
        
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)
          
    return(xdata,m2,n2)

## ##################################################
class cls_label_files:
  """
  Loads a image label files and gives back the sparse mvm form:
                (row_index,column_index,value), in list of arrays

  """

  def __init__(self):

    self.sbasedir='/home/szedmak/src/python/algebra/tagging/data/'
    self.ldirs=['corel5k','espgame','iaprtc12','mirflickr','pascal07']
    self.slabels='annot.mat'

  ## --------------------------------------------------
  def load(self,ifile,pselect,itrain=1):

    xdata=[[],[],[]]    ## row index, column index , value

    ddata=scipy.io.loadmat(self.sbasedir+self.ldirs[ifile]+'/'+self.slabels)
    if itrain==1:
      xtrain=ddata['xtrain']
    else:
      xtrain=ddata['xtest']
    m=xtrain.shape[0]

    ## antimatroid(xtrain[:5000])
    
    iselect=np.where(np.random.rand(m)<pselect)[0]

    xtrain0=xtrain[iselect]
    m=xtrain0.shape[0];

    X=np.dot(xtrain0,xtrain.T)

    for i in range(m):
      for j in range(m):
        v=X[i,j]
        if v!=0:
          xdata[0].append(i)
          xdata[1].append(j)
          xdata[2].append(v)
        
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)
          
    return(xdata,m,m)
## ##################################################
class cls_image_files:
  """
  Loads a image files and gives back the sparse mvm form:
                (row_index,column_index,value), in list of arrays

  """

  def __init__(self):

    self.image_scale=256
    
    return

  ## ---------------------------------------------
  def load(self,ifile):  

    if ifile==0:
      sdir='/home/szedmak/data/image_text/labeled_images/'
      sfile='/cambridge/Image007.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((768,512)))  ## 504,756 -> 512,768
      self.xfactors=np.array([[32,48,1],[16,16,1],[1,1,3]]) 
    elif ifile==1:
      sdir='/home/szedmak/src/python/algebra/tensor_decomp/'
      sfile='handwritten.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((1024,1024)))  ## 504,756 -> 512,768
      self.xfactors=np.array([[32,32],[32,32]])
    elif ifile==2:
      sdir='/home/szedmak/src/python/algebra/tensor_decomp/'
      sfile='tilling.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((576,512)))  ## 504,756 -> 512,768
      ## xmatrix=np.mean(xmatrix,axis=2)
      self.xfactors=np.array([[32,36,1],[16,16,1],[1,1,3]])  ## tilling
    elif ifile==3:
      sdir='/home/szedmak/src/python/algebra/tensor_decomp/'
      sfile='poligon.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((512,512)))  ## 504,756 -> 512,768
      self.xfactors=np.array([[32,32,1],[16,16,1],[1,1,3]])  ## poligon
    elif ifile==4:
      sdir='/home/szedmak/src/python/algebra/tensor_decomp/'
      sfile='2008toyroom.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((768,512)))  ## 504,756 -> 512,768
      ## xmatrix=np.mean(xmatrix,axis=2)
      self.xfactors=np.array([[32,48,1],[16,16,1],[1,1,3]]) 
    elif ifile==5:
      sdir='/home/szedmak/src/python/algebra/tensor_decomp/'
      sfile='PLayroom_How_to_Organize_Kids_Room_2013.png'
      xmatpil=pilimg.open(sdir+sfile)
      xmatrix=np.array(xmatpil.resize((768,512)))  ## 504,756 -> 512,768
      self.xfactors=np.array([[32,48,1],[16,16,1],[1,1,3]]) 

    print(sdir,sfile)
    print(self.xfactors)

    self.baseline(xmatrix)

    ctensor=tensor_decomp.cls_tensor_decomp()
    ctensor.load(xmatrix,self.xfactors)
    X=ctensor.reorder_in_x(xmatrix)
    ## X=X.T
    tshape=X.shape
    mi=tshape[0]
    mj=tshape[1]
    ## if len(tshape)>2:
    ##   mk=tshape[2]

    ## X=X/self.image_scale
    
    xdata=[[],[],[]]    ## row index, column index , value
    for i in range(mi):
      for j in range(mj):
        xdata[0].append(i)
        xdata[1].append(int(np.mean(X[i,j])))
        xdata[2].append(X[i,j]/self.image_scale)
        
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)

    m1=mi
    ## m2=mj
    m2=256
          
    return(xdata,m1,m2)
  
  ## -----------------------------------------
  def invert(self,xmatre):

    ctensor=tensor_decomp.cls_tensor_decomp()
    xmatout=ctensor.invert_reorder_in_x(xmatre,self.xfactors)

    return(xmatout)
  ## -----------------------------------------
  def baseline(self,x):

    tdim=x.shape
    mi=tdim[0]
    mj=tdim[1]
    if len(tdim)==2:
      mk=1
    else:
      mk=tdim[2]
    ## assume mi%2==0
    rmse=0.0
    k=4
    k_2=k>>1
    for i in range(0,mj,k):
      for j in range(k):
        ## rmse+=np.sum((x[i+k_2,:]-x[i+j,:])**2)
        rmse+=np.sum((x[:,i+k_2]-x[:,i+j])**2)

    print('Baseline:')
    print(np.sqrt((k-1)*rmse/(mi*mj*mk))/self.image_scale)

    return
  
  ## ----------------------------------------
  ## def resize_power2(self,xmatpil):
  ##   """
  ##   Resize PIL image xmatpil to the greater closets power of 2
  ##   """
  ##   return
## ##############################################3
def antimatroid(X):

  m=X.shape[0]

  K=np.dot(X,X.T)
  dd=np.zeros((m,m))

  dchain={}

  for i in range(m):
    for j in range(m):
      if K[i,j]==K[i,i]:
        if K[i,j]==K[j,j]-1:
          dchain[(i,j)]=K[i,i]
          dd[i,j]=1
      else:
        if K[i,j]==K[j,j]:
          if K[i,j]==K[i,i]-1:
            dchain[(i,j)]=K[j,j]
            dd[i,j]=1

    if  i%100==0:
      print(i)
      
  kmax=np.diag(K).max()

  xstat=np.zeros(kmax+1)
  for tval in dchain.values():
    xstat[tval]+=1

  for i in range(kmax+1):
    print(i,'%5d'%xstat[i])
  print('%6d'%(np.sum(xstat)))

  return

  
    

    
    




  




  
