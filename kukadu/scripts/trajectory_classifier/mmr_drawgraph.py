######################
## Version 0.1 #######
######################
## import pickle, os
## ###############################
import numpy as np
## import scipy.io
## import networkx as nx
import numpy.linalg as np_lin

import pylab as lab
## import mpl_toolkits.mplot3d.axes3d as p3
## import mpl_toolkits.mplot3d.art3d as art3d
## ###############################
## #######################################################
def hist_tra_tes(Ytra,Ytes,sdata):

  n=Ytra.shape[1]
  ytra_pdf=np.sum(Ytra,axis=0)
  ytra_pdf=ytra_pdf/np.sum(ytra_pdf)
  ytes_pdf=np.sum(Ytes,axis=0)
  ytes_pdf=ytes_pdf/np.sum(ytes_pdf)
  x=np.arange(n)

  ylow1=0.0
  yhigh1=0.1
  xlinewidth=3
  yration=(ytes_pdf+0.001)/(ytra_pdf+0.001)

  fig1=lab.figure(figsize=(12,6))
  ax=fig1.add_subplot(1,2,1)
  ax.plot(x,ytra_pdf,label='Training',linewidth=xlinewidth, \
          linestyle='-',color='b')
  ax.plot(x,ytes_pdf,label='Test',linewidth=xlinewidth,
           linestyle=':',color='r')
  ax.legend(loc='upper right')
  ax.set_title('Training and Test distribution:'+sdata ,fontsize=16)
  ax.set_xlim(0,len(x))
  ax.set_ylim(ylow1,yhigh1)
  ax.set_yticks(np.arange(ylow1,yhigh1,0.05))
  ax.set_xlabel('Label indexes', fontsize=14)
  ax.set_ylabel('Estimated probabilities',fontsize=14)
  ax.grid(True)

  ax=fig1.add_subplot(1,2,2)
  ## ax.scatter(ytra_pdf,ytes_pdf,label='Training')
  ax.plot(x,yration,label='Test/Training ration',linewidth=xlinewidth,
           linestyle='-',color='r')
  ax.set_title('Test/Training probability ratios',fontsize=16)
  ax.set_xlabel('Label index', fontsize=14)
  ax.set_ylabel('Test/Training Ratio',fontsize=14)
  ax.grid(True)
    
  lab.show()

  return

## #######################################################
def corr_matrix(Ytra,Ytes,sdata):

  m=Ytra.shape[0]
  mt=Ytes.shape[0]
  
  xmean=np.mean(Ytra,axis=0)
  ytra0=Ytra-np.outer(np.ones(m),xmean)
  xmean=np.mean(Ytes,axis=0)
  ytes0=Ytes-np.outer(np.ones(mt),xmean)

  xnorm=np.sqrt(np.sum(ytra0**2,axis=0))
  Ctra=np.dot(ytra0.T,ytra0)
  xnorm=xnorm+(xnorm==0)
  Ctra=Ctra/np.outer(xnorm,xnorm)
  xnorm=np.sqrt(np.sum(ytes0**2,axis=0))
  Ctes=np.dot(ytes0.T,ytes0)
  xnorm=xnorm+(xnorm==0)
  Ctes=Ctes/np.outer(xnorm,xnorm)

  ## Ctra=np.log(np.abs(Ctra)+1)*np.sign(Ctra)
  ## Ctes=np.log(np.abs(Ctes)+1)*np.sign(Ctes)
  
  fig1=lab.figure(figsize=(12,6))
  ## ax=fig1.add_subplot(1,2,1)
  ## ax.imshow(Ctra,interpolation='none')
  ax1=lab.subplot2grid((10,2),(1,0), rowspan=9)
  ax1.hist(Ctra.ravel(),200,log=True)
  ax1.set_xlim(-0.2,1.2)
  ax1.set_title('Training labels:',fontsize=16)
  ax1.grid(True)

  ## ax=fig1.add_subplot(1,2,2)
  ## ax.imshow(Ctes,interpolation='none')
  ax2=lab.subplot2grid((10,2),(1,1), rowspan=9)
  ax2.hist(Ctes.ravel(),200,log=True)
  ax2.set_xlim(-0.2,1.2)
  ax2.set_title('Test labels:',fontsize=16)
  ax2.grid(True)

  fig1.suptitle('Histograms of label correlation(log scale), data set:'+sdata,fontsize=18  )
  ## fig1.tight_layout()
    
  lab.show()

  return

## #######################################################
def singular(Ytra,Ytes,sdata):

  m=Ytra.shape[0]
  mt=Ytes.shape[0]
  
  xmean=np.mean(Ytra,axis=0)
  ytra0=Ytra-np.outer(np.ones(m),xmean)
  xmean=np.mean(Ytes,axis=0)
  ytes0=Ytes-np.outer(np.ones(mt),xmean)

  xnorm=np.sqrt(np.sum(ytra0**2,axis=0))
  Ctra=np.dot(ytra0.T,ytra0)
  xnorm=xnorm+(xnorm==0)
  Ctra=Ctra/np.outer(xnorm,xnorm)
  xnorm=np.sqrt(np.sum(ytes0**2,axis=0))
  Ctes=np.dot(ytes0.T,ytes0)
  xnorm=xnorm+(xnorm==0)
  Ctes=Ctes/np.outer(xnorm,xnorm)

  stra=np_lin.svd(Ctra)[1]
  stes=np_lin.svd(Ctes)[1]

  xlinewidth=3
  
  fig1=lab.figure(figsize=(12,6))
  ax1=lab.subplot2grid((10,2),(1,0), rowspan=9)
  ## ax=fig1.add_subplot(1,2,1)
  ax1.plot(stra,linewidth=xlinewidth, \
          linestyle='-',color='b')
  ax1.set_xlabel('Label indexes', fontsize=14)
  ax1.set_ylabel('Eigen values',fontsize=14)
  ax1.set_title('Training:',fontsize=16)
  ax1.grid(True)

  ## ax=fig1.add_subplot(1,2,2)
  ax2=lab.subplot2grid((10,2),(1,1), rowspan=9)
  ax2.plot(stes,linewidth=xlinewidth, \
          linestyle='-',color='b')
  ax2.set_xlabel('Label indexes', fontsize=14)
  ax2.set_ylabel('Eigen values',fontsize=14)
  ax2.set_title('Test:',fontsize=16)
  ax2.grid(True)

  fig1.suptitle('Eigen values of correlation matrixes: '+sdata,fontsize=18  )
    
  lab.show()

  return

