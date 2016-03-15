######################
## Version 0.1 #######
######################
## import pickle
## ###############################
## import psycopg2
import numpy as np
## import scipy.linalg as sp_lin
## ###############################
import psql_db as pdb
## ###############################
class cls_roar_db:

  def __init__(self,dbname='salad_web_test',
               dbuser='szs777',dbhost='localhost',dbpassword='cso7ba8nc'):

    self.dbname=dbname
    self.dbuser=dbuser
    self.dbhost=dbhost
    self.dbpassword=dbpassword

    
    self.dbtable=None         ## dbtable='object_action_web_2'
    
    self.table_row=None       ## list of name of row fields
    self.table_column=None    ## list of name of column fields
    self.table_data=None      ## name of the data field,
                              ## if it is None after load data 
                              ## then the existence or non-existence
                              ## of the entire record is predicted   

    self.dbrows=None
    self.nrow=None
    self.ncol=None
    self.ncategory=1
    

    self.dtable={} ## dict[row_index][column_index]=value
    self.ltable=[None]*3    ## row index, column index, value
    self.xtable=None
    self.drow={}    ## dictionary row string to index
    self.drowinv={}    ## dictionary row index to string
    self.dcol={}    ## dictionary column string to index
    self.dcolinv={}    ## dictionary column index to string

    return

  ## ------------------------------------
  def load_table_mvm(self,dbtable,table_row,table_column,table_data=None):

    self.table_row=table_row        ## list of the name of row fields
    self.table_column=table_column  ## list of the name of column fields

    lfields=table_row.copy()
    lfields.extend(table_column)

    self.load_table_raw(dbtable,lfields)
    
    ## convert db rows into doc-array
    if self.dbrows is not None:
      self.rows2dict()
    ## covert py dictionary into sparse array of (row,column,value)
      xdata_rel=self.dict2relation_rable()    
    else:
      xdata_rel=None

    return(xdata_rel)
  ## ------------------------------------
  def load_table_mmr(self,dbtable,table_row,table_column,table_data=None):

    self.table_row=table_row        ## list of the name of row fields
    self.table_column=table_column  ## list of the name of column fields

    lfields=table_row.copy()
    lfields.extend(table_column)

    self.load_table_raw(dbtable,lfields)

    m=len(self.dbrows)
    nx=len(self.dbrows[0][table_row[0]])
    ny=len(self.dbrows[0][table_column[0]])
    X=np.zeros((m,nx))
    Y=np.zeros((m,ny))
    i=0
    for row in self.dbrows:
      X[i,:]=np.array(row[table_row[0]])
      Y[i,:]=np.array(row[table_column[0]])
      i+=1

    return(X,Y)
  ## ----------------------------------------------
  def pyarrays2sqlarray(self,table_row,table_column,X,Y):

    (m,nx)=X.shape
    (m,ny)=Y.shape

    datavalues=[]
    drow={}
    for i in range(m):
      lz=[ int(X[i,j]) for j in range(nx)]
      drow[table_row[0]]=lz
      lz=[ int(Y[i,j]) for j in range(ny)]
      drow[table_column[0]]=lz
      datavalues.append(drow)

    return(tuple(datavalues))
  ## -----------------------------------------------

  def rows2dict(self):
    """
    converts rows of SQL select into table of dictionary format
    Input: self.
          rows          list of lists as output of SQL select
          table_row     selects field as table row
          table_column  selects field as table column
          table_data   != -1 select filed as table value
                        =-1 table value =1 for all row-column pair
    """

    iindex=0

    self.drow={}
    self.drowinv={}
    self.dcol={}
    self.dcolinv={}

    self.dtable={}

    nitem=0
    irowdb=0
    icoldb=0
    for row in self.dbrows:
      ## collect indexes from fields
      lrow=[]
      for sitem in self.table_row:
        lrow.append(row[sitem])
      trow=tuple(lrow)
      irow=irowdb
      if trow not in self.drow:
        self.drow[trow]=[irowdb,1]
        self.drowinv[irowdb]=trow
        irowdb+=1
      else:
        self.drow[trow][1]+=1
      lrow=[]

      lcol=[]
      for sitem in self.table_column:
        lcol.append(row[sitem])
      tcol=tuple(lcol)
      icol=icoldb
      if tcol not in self.dcol:
        self.dcol[tcol]=[icoldb,1]
        self.dcolinv[icoldb]=tcol
        icoldb+=1
      else:
        self.dcol[tcol][1]+=1

      ## collect the values belonging to a pair of (irow,icol)
      if self.table_data is not None:
        lvalue=[]
        for sitem in self.table_data:
          lvalue.append(row[sitem])

      ## process the case when only indexes are given in a record
      if self.table_data is None:
        if irow not in self.dtable:
          self.dtable[irow]={}
        if icol not in self.dtable[irow]:
          ## the record containing fields from table_row and table_column
          ## is exists 
          self.dtable[irow][icol]=1
          nitem+=1
      else:     ## there is data value in the DB table
        if irow not in self.dtable:
          self.dtable[irow]={}
        if icol not in self.dtable[irow]:
          ## the record containing fields from table_row and table_column
          ## is exists 
          self.dtable[irow][icol]=lvalue
          nitem+=1

      self.nitem=nitem
    return

  ## -----------------------------------------------------------
  def dict2relation_rable(self):

    nitem=self.nitem

    if self.table_data is None:
      ndata=1
    else:
      ndata=len(self.table_data)
      
    ## row index, column index, value
    ltable=[np.zeros(nitem,dtype=int),np.zeros(nitem,dtype=int), \
            np.zeros((nitem,ndata))] 

    iitem=0
    for irow,dcolval in self.dtable.items():
      for icol,xvalue in dcolval.items():
        ltable[0][iitem]=irow
        ltable[1][iitem]=icol
        ltable[2][iitem]=xvalue
        iitem+=1

    ldata=[ [ltable[0][i],ltable[1][i],i] for i in range(nitem)]
    ldata.sort()
    xdata_rel=[None]*3    ## row, column, value 
    xdata_rel[0]=np.array([ xitem[0] for xitem in ldata]).astype(int)
    xdata_rel[1]=np.array([ xitem[1] for xitem in ldata]).astype(int)
    xdata_rel[2]=np.array([ ltable[2][xitem[2]]  \
                            for xitem in ldata]).astype(float)
    
    return(xdata_rel,len(self.drow),len(self.dcol))

  ## ---------------------------------------------------

  def create_table(self,dbtable,dfields):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor()

    ## update(insert) table
    iret=roar_db.create_table(dbtable,dfields)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()

    return(iret)
    
  ## ---------------------------------------------------
  def insert_table(self,dbtable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor()

    ## update(insert) table
    roar_db.insert_many(dbtable,lfields,datavalues)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()
  ## ---------------------------------------------------
  def load_table_raw(self,dbtable,lfieldsin):

    self.dbtable=dbtable            ## name of the table

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor_dict()

    lfields=lfieldsin.copy()
    self.dbrows=roar_db.select(self.dbtable,lfields)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()

    return
  ## ----------------------------------------------
  ## ---------------------------------------------------

  def update_table(self,dbtable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor()

    ## update(insert) table
    roar_db.update_many_2(dbtable,lfields,datavalues)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()
    
  ## -----------------------------------------------------
  def rewrite_tables(self,coo_table):

    ## covert row, column, value indeces into original sql db values
    ndata=len(coo_table[0])
    converted_table=[]
    if self.cdbtable.table_class==-1:
      for i in range(ndata):
        converted_table.append((self.cdbtable.drowinv[coo_table[i][0]], \
                             self.cdbtable.dcolinv[coo_table[i][1]], \
                             coo_table[i][2]))
    else:
      for i in range(ndata):
        converted_table.append((self.cdbtable.drowinv[coo_table[i][0]], \
                             self.cdbtable.dcolinv[coo_table[i][1]], \
                             self.cdbtable.dclassinv[coo_table[i][2]]))
    self.cdbtable.update(converted_table)

    return
  ## =====================================================
  ## Private internal functions
  def fields2index(self,lfields,dfields2index,dindex2fields):

    dfieldtoindex={}
    dindextofield={}
    
    iindex=0
    
    for row in self.dbrows:
      ## collect indexes from fields
      lvalues=[]
      for sitem in self.lfields:
        lvalues.append(row[sitem])
      tvalues=tuple(lvalues)
      if tvalues not in dfields2index:
        dfields2index[tvalues]=[iindex,1]
        iindex+=1
      else:
        dfields2index[tvalues][1]+=1
      
  ## ---------------------------------------------
  ## ---------------------------------------------
  ## ---------------------------------------------
  def read_mmr(self,dtable,lfields):
    
    


    return
  ## ---------------------------------------------
  def write_mmr(self):


    return
  
## ####################################################
