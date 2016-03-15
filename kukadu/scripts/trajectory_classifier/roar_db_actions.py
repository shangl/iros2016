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
class cls_roar_table:

  def __init__(self,table_row,table_column,table_class=None, \
               dbname='salad_web_test',dbtable='object_action_web_2',
               dbuser='szs777',dbhost='localhost',dbpassword='cso7ba8nc'):

    self.dbname=dbname
    self.dbuser=dbuser
    self.dbhost=dbhost
    self.dbpassword=dbpassword
    
    self.dbtable=dbtable
    self.table_row=table_row
    self.table_column=table_column
    self.table_class=table_class
    self.rows=None
    self.nrow=None
    self.ncol=None
    self.ncategory=1
    

    self.dtable={} ## dict[row_index][column_index]=value_index
    self.ltable=[None]*3    ## row index, column index, value
    self.xtable=None
    self.drow={}    ## dictionary row string to index
    self.dcol={}    ## dictionary column string to index
    self.dclass={}  ## dictionary value string to index, multi class indicators 
    self.drowinv={}    ## dictionary row index to string
    self.dcolinv={}    ## dictionary column index to string
    self.dclassinv={}  ## dictionary value index to string

    self.ltables=['object_action','object_object','object_object_action', \
                  'objects']
    self.dfields={ 'object_action' : \
                   [ 'object','action','preposition','score'],
                   'object_object' : \
                   ['object1','object2','score'],
                   'object_object_action' : \
                   ['object1','object2','action','preposition','score'],
                   'objects' : \
                   ['object','affordance','confidence']}
    self.dtypes={ 'object_action' : \
                   [ 'char','char','num','num'],
                   'object_object' : \
                   [ 'char','char','num'],
                    'object_object_action' : \
                   [ 'char','char','char','char','num'],
                   'objects' : \
                   [ 'char','char','num']}
    return

  ## ------------------------------------

  def load_table(self):

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor_dict()

    ## read the dbtable
    ## sfields='object, object_feature, value'
    sfields=''
    for sitem in self.table_row:
      sfields=sfields+sitem+', '
    for sitem in self.table_column:
      sfields=sfields+sitem+', '
    sfields=sfields[:-2]+' '
    
    self.rows=roar_db.select(self.dbtable,sfields)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()

    ## convert db rows into mmmvr list of 1D arrays
    xdata_rel=self.rows2dtable()
    
    ## covert py dictionary into sparse array of (row,column,value)
    ## coo_ijv=self.dtable2ijv_coo()    

    return(xdata_rel)
  ## ----------------------------------------------
  def rows2dtable(self):
    """
    converts rows of SQL select into table of dictionary format
    Input: self.
          rows          list of lists as output of SQL select
          table_row     selects field as table row
          table_column  selects field as table column
          table_class   != -1 select filed as table value
                        =-1 table value =1 for all row-column pair
    """

    irow=0
    icol=0
    nclass=0
    drowcol={}
    irecord=0
    for row in self.rows:
      trowcol=[]
      trow=[]
      for sitem in self.table_row:
        trow.append(row[sitem])
      trowcol.extend(trow)
      trow=tuple(trow)
      tcol=[]
      for sitem in self.table_column:
        tcol.append(row[sitem])
      trowcol.extend(tcol)
      tcol=tuple(tcol)
      trowcol=tuple(trowcol)
      if trowcol not in drowcol:
        drowcol[trowcol]=[1,irecord]
      else:
        drowcol[trowcol][0]+=1

      if self.table_class!=None:
        sclass=row[self.table_class]
        if sclass not in self.dclass:
          self.dclass[sclass]=nclass
          self.dclassinv[nclass]=sclass
          nclass+=1

      if trow not in self.drow:
        self.drow[trow]=irow
        self.drowinv[irow]=trow
        irow+=1
      if tcol not in self.dcol:
        self.dcol[tcol]=icol
        self.dcolinv[icol]=tcol
        icol+=1

      irecord+=1

    ## nitem=0  
    ## for tkey,xitem in drowcol.items():
    ##   if xitem[0]>1:
    ##     nitem+=1
    ##     print(tkey,xitem[0],self.rows[xitem[1]])
    ## print(nitem)

    self.nrow=len(self.drowinv)
    self.ncol=len(self.dcolinv)

    ltable=[ [] for i in range(3)] ## row index, column index, value

    for row in self.rows:
      trow=[]
      for sitem in self.table_row:
        trow.append(row[sitem])
      trow=tuple(trow)
      tcol=[]
      for sitem in self.table_column:
        tcol.append(row[sitem])
      tcol=tuple(tcol)
      irow=self.drow[trow]
      icol=self.dcol[tcol]
      if self.table_class!=None:
        iclass=self.dclass[row[self.table_class]]
      else:
        iclass=0
      ltable[0].append(irow)
      ltable[1].append(icol)
      if self.table_class==None:
        ltable[2].append(1)
      else:
        ltable[2].append(iclass)

    ndata=len(ltable[0])
    ldata=[ [ltable[0][i],ltable[0][1],ltable[2][i]] for i in range(ndata)]
    ldata.sort()

    xdata_rel=[None]*3 
    xdata_rel[0]=np.array([ xitem[0] for xitem in ldata]).astype(int)
    xdata_rel[1]=np.array([ xitem[1] for xitem in ldata]).astype(int)
    xdata_rel[2]=np.array([ xitem[2] for xitem in ldata])
        
    return(xdata_rel)

  ## ---------------------------------------------------

  def update(self,xdatapred,sfield):

    roar_db=pdb.cls_sql_db(self.dbname,self.dbuser,self.dbhost, \
                           self.dbpassword)
    roar_db.connect()
    roar_db.cursor()

    ## update table

    m=len(xdatapred[0])
    for i in range(m):
      datavalue=float(xdatapred[2][i])
      dcondition={}
      trow=self.drowinv[xdatapred[0][i]]
      tcol=self.dcolinv[xdatapred[1][i]]
      iitem=0
      for sitem in self.table_row:
        dcondition[sitem]=trow[iitem]
        iitem+=1
      iitem=0
      for sitem in self.table_column:
        dcondition[sitem]=tcol[iitem]
        iitem+=1

      print(sfield)
      print(datavalue)
      print(dcondition)
      if datavalue==15.0:
        roar_db.update(self.dbtable,sfield,datavalue,dcondition)

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
## ####################################################
