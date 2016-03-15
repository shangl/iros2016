## ##################################################
import csv
import numpy as np
import scipy.io
import pickle
import scipy.stats

## import scipy.linalg as sp_linalg
import mvm_prepare
## import tensor_decomp
## ###################################################

## ##################################################
class cls_label_files:
  """
  Loads a image label files and gives back the sparse mvm form:
                (row_index,column_index,value), in list of arrays
  """

  def __init__(self):

    self.sbasedir='data/'
    self.listcsv=['affordances_instrument_for','affordances_patient']
    self.fileext='.csv'
    self.csv_delimiter=','
    self.headerline=3

    self.test_predicted_data='test_prediction.txt'
    self.full_predicted_data='full_prediction.txt'

    self.nfeature=2   ## number of features assigned to each pair of objects
    self.feature_orig=2

    self.dobject={}   ## object name -> object index
    self.dobject_inv={}   ## object index -> object name
    self.daction={}   ## object name -> object index
    self.daction_inv={}   ## object index -> object name

    self.iobject=0
    self.iaction=0

    self.ddata={}

    self.nobject=0  ## number of objects
    self.naction=0  ## number of objects
    self.nrow=0     ## number of internal rows
    self.ncol=0     ## number of internal columns

    self.irowcol=1  ## =0 3D array mapped into 2D array
                    ##      (object,object * feature)
                    ## =1 3D array mapped into 2D array
                    ##  (object * feature,object)

    
  ## --------------------------------------------------
  def load_objobj_act(self,lfiles,ifeature):
    """
    load collected known entities (object, action,feature)
    into the relation table.
    """

    ## first phase collect object labels
    for ifile in lfiles:
      self.collect_entities(ifile,ifeature)
    
    ## second phase load the relation table of known examples
    xdata=[[],[],[]]    ## row index, column index , value
    idata=0
    ifixtrain=[]
    ifixtest=[]

    for iobject1 in range(self.nobject):
      for iaction in self.ddata[iobject1].keys():
        for iobject2 in range(self.nobject):
          if iaction in self.ddata[iobject2]:
            lvalue1=self.ddata[iobject1][iaction]
            lvalue2=self.ddata[iobject2][iaction]
            if len(lvalue1)>0 and len(lvalue2)>0:
              vvalue=np.hstack((np.array(list(lvalue1)), \
                                np.array(list(lvalue2))))
              ## if len(vvalue)>1:
              ##   print(vvalue)
              vvalue=np.mean(vvalue.astype(np.double))
              if self.irowcol==0: 
                xdata[0].append(iobject1*self.nobject+iobject2)
                xdata[1].append(iaction)
                xdata[2].append(vvalue)
              elif self.irowcol==1:
                xdata[0].append(iaction)
                xdata[1].append(iobject1*self.nobject+iobject2)
                xdata[2].append(vvalue)
              ifixtrain.append(idata)
              idata+=1
        
    if self.irowcol==0:
      self.nrow=self.nobject**2
      self.ncol=self.naction
    elif self.irowcol==1:
      self.nrow=self.naction
      self.ncol=self.nobject**2

    xdata=mvm_prepare.sort_table(xdata,ifloat=1)

    ifixtrain=np.array(ifixtrain)
    ifixtest=ifixtrain
    
    return(xdata,self.nrow,self.ncol,ifixtrain,ifixtest)

  ## -----------------------------------------------
  def collect_entities(self,ifile,ifeature):
    """
    collect entities, objects, actions, and feature values
    the tuples (object,action,feature) is loaded into a dictionary:
      self.ddata[object index][action index]=feture value as string
    """

    with open(self.sbasedir+self.listcsv[ifile]+self.fileext, 'r') as infile:
      csv_reader = csv.reader(infile, delimiter=self.csv_delimiter)
      ifirst=self.headerline
      for line in csv_reader:
        if ifirst>0:
          ifirst-=1
          continue
        if len(line)==0:
          continue
        sobject=line[0]
        ## sobject=sobject[1:-1] ## remove '', of the ascii string
        if sobject not in self.dobject:
          self.dobject[sobject]=self.iobject
          self.dobject_inv[self.iobject]=sobject
          self.iobject+=1
        saction=line[1]
        ## saction=saction[1:-1]
        if saction not in self.daction:
          self.daction[saction]=self.iaction
          self.daction_inv[self.iaction]=saction
          self.iaction+=1
    infile.close()

    self.nobject=self.iobject
    self.naction=self.iaction

    with open(self.sbasedir+self.listcsv[ifile]+self.fileext, 'r') as infile:
      csv_reader = csv.reader(infile, delimiter=self.csv_delimiter)
      ifirst=self.headerline
      for line in csv_reader:
        if ifirst>0:
          ifirst-=1
          continue
        if len(line)==0:
          continue
        sobject=line[0]
        ## sobject=sobject[1:-1]
        iobject=self.dobject[sobject]
        saction=line[1]
        ## saction=saction[1:-1]
        iaction=self.daction[saction]
        iposition=ifeature+self.feature_orig
        svalue=line[iposition]
        if iobject not in self.ddata:
          self.ddata[iobject]={}
        if iaction not in self.ddata[iobject]:
          self.ddata[iobject][iaction]=set([svalue])
        else:
          self.ddata[iobject][iaction].add(svalue)
    infile.close()
        
    return
  ## ----------------------------------------------
  def export_full_prediction(self,Zrow):
    """
    Zrow is loaded with continous values of relation array
    """

    if self.irowcol==0:
      nrow=self.nobject
      ncol=self.naction
    elif self.irowcol==1:
      ncol=self.naction
      nrow=self.nobject
    
    fout=open(self.sbasedir+self.full_predicted_data, 'w')
    for irow in range(nrow):
      for icol in range(ncol):
        if self.irowcol==0:
          sline='"'+self.dobject_inv[irow]+'"'+',' \
                 +'"'+self.daction_inv[icol]+'"'
        elif self.irowcol==1:
          sline='"'+self.dobject_inv[icol]+'"'+',' \
                 +'"'+self.daction_inv[irow]+'"'
        vpred=Zrow[irow][0][icol]
        sline+=','+'"'+str('%7.5f'%vpred)+'"'
        
        fout.write(sline+'\n')
    fout.close()

    return
  ## ----------------------------------------------
  def full_test(self):

    ndata=self.nrow*self.ncol
    npart=3
    xdata_tes=[ np.zeros(ndata,dtype=int) for i in range(npart)]
    k=0
    for i in range(self.nrow):
      for j in range(self.ncol):
        xdata_tes[0][k]=i
        xdata_tes[1][k]=j
        k+=1
        
    return(xdata_tes)

  ## --------------------------------------------------
  def order_dict(self,ddictin):

    lkeys=list(ddictin.keys())
    lkeys.sort()

    ddictout={}
    iindex=0
    for key in lkeys:
      ddictout[key]=iindex
      iindex+=1

    return(ddictout)
  ## --------------------------------------------------
  def invert_dict(self,ddictin):

    ddictout={}
    for key,val in ddictin.items():
      if val not in ddictout:
        ddictout[val]=key

    return(ddictout)
  ## --------------------------------------------------
  def export_prediction(self,filename,xdatacls,ZrowT):

    if xdatacls.testontrain==0:
      xranges_tes=xdatacls.xranges_rel_test
      xdata_tes=xdatacls.xdata_tes
    else:
      xranges_tes=xdatacls.xranges_rel
      xdata_tes=xdatacls.xdata_tra

    nrow=xdatacls.nrow
    ncol=xdatacls.ncol

    txdim=xdata_tes[2].shape
    if len(txdim)==1:
      nxdim=1
    else:
      nxdim=txdim[1]

    pred_list=[]
    for irow in range(nrow):
      if xranges_tes[irow,1]>0:
        istart_tes=xranges_tes[irow,0]
        nlength_tes=xranges_tes[irow,1]
        for i in range(nlength_tes):
          ii=istart_tes+i
          vpred=ZrowT[irow][0][i]
          ## vconf=ZrowT[irow][2][i]
          icol=xdata_tes[1][ii]
          if self.irowcol==0:
            iobject1=irow // self.nobject
            iobject2=irow % self.nobject
            iaction=icol
          else:
            iaction=irow
            iobject1=icol // self.nobject
            iobject2=icol % self.nobject
            
          sobject1=self.dobject_inv[iobject1]
          sobject2=self.dobject_inv[iobject2]
          saction=self.daction_inv[iaction]
          ## confidence assumes normal_distribution(0,1)
          vconf=scipy.stats.norm.cdf(vpred, \
                                     loc=xdatacls.confidence_local, \
                                     scale=xdatacls.confidence_scale)
          if vconf>0.9:
            pred_list.append([sobject1,sobject2,saction,vconf])
      if irow % 100 ==0:
        print(irow)

    ## pred_list2=[ [sobject1,saction,vconf,sobject2] \
    ##              for (sobject1,sobject2,saction,vconf) in pred_list]
    pred_list.sort()
    
    fout=open(filename,'w')
    fout.write('# Predicted relations\n')
    fout.write('# Columns: object1, object2, action, score \n')
    fout.write('\n')
    
    for (sobject1,sobject2,saction,vconf) in pred_list:
      sline='"'+sobject1+'"'+','
      sline+='"'+sobject2+'"'+','
      sline+='"'+saction+'"'+','
      sline+='"'+str(vconf)+'"'
      fout.write(sline+'\n')
    fout.close()

    return
## ##################################################


  
