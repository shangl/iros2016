######################
## Version 0.1 #######
######################
## import pickle
## ###############################
import psycopg2
import numpy as np
## import scipy.linalg as sp_lin
## ###############################
import psql_db as pdb
## ###############################
class cls_table_store:

  def __init__(self,table_name,table_row,table_column,table_class=-1):

    self.table_name=table_name
    self.table_row=table_row
    self.table_column=table_column
    self.table_class=table_class
    self.rows=None

    self.dtable={} ## dict[row_index][column_index]=value_index
    self.drow={}    ## dictionary row string to index
    self.dcol={}    ## dictionary column string to index
    self.dclass={}  ## dictionary value string to index, multi class indicators 
    self.drowinv={}    ## dictionary row index to string
    self.dcolinv={}    ## dictionary column index to string
    self.dclassinv={}  ## dictionary value index to string

    return
  ## ------------------------------------
    

## ##############################################
class cls_db_salad(pdb.cls_sql_db):

  def __init__(self,sdbname,suser,shost,spassword):
    """
    dbname='object_action'
    user='szs777'
    host='loaclhost'
    password='******'
    """

    pdb.cls_sql_db.__init__(self,spassword,sdbname='salad_scenario', \
                        suser='szs777', shost='localhost')

    self.dbname=sdbname
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
    self.user=suser
    self.host=shost
    self.password=spassword
    self.dtable_store={}

  ## -------------------------------------------------------
  def db_add_table(self,stable,table_name,table_row,table_column, \
                   table_class=-1):
    
    ctable_store=cls_table_store(table_name,table_row,table_column,table_class)
    self.dtable_store[stable]=ctable_store

    return

  ## -------------------------------------------
  def db_read_join(self,table1,field1,table2,field2):

    roar_db=pdb.cls_sql_db(self.dbname,self.user,self.host,self.password)

    roar_db.connect()
    roar_db.cursor()
    ssql='SELECT'+' '+'*' \
          +'FROM'+table1+' ' \
          +'INNER JOIN'+' '+table2+' ' \
          +'ON'+' '+table1+'.'+field1+' '+'+'+' '+table2+'.'+field2 \
          +';'
    rows=roar_db.execute_select(self,ssql)
    roar_db.cursor_close()
    roar_db.connect_close()
    
    return(rows)
  ## -------------------------------------------
  def table2sparse(self,xtable,xtable_struct):

    ## select column_name, data_type from information_schema.columns where table_name='aor';

    return()
  ## -------------------------------------------
  def db_load(self,stable,suser,shost,spassword,):

    ## dobject2index={}
    ## dindex2object={}
    ## dfeature2index={}
    ## dindex2feature={}


  ## load database roar with table roar of fields object,object_feature,value
    ## roar_db=pdb.cls_sql_db('roar','szs777','localhost','cso7ba8nc')
    sdatabase='salad_scenario'
    roar_db=pdb.cls_sql_db(sdatabase,suser,shost,spassword)
    roar_db.connect()
    roar_db.cursor()

    ## sfields='object, object_feature, value'
    sfields='*'
    
    rows=roar_db.select(self.dtable_store[stable].table_name,sfields)

    ## roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()

    self.dtable_store[stable].rows=rows

    return()

  ## -------------------------------------------
  def db_insert(self,stable,suser,shost,spassword,datavalues):


  ## load database roar with table roar of fields object,object_feature,value
    ## roar_db=pdb.cls_sql_db('roar','szs777','localhost','cso7ba8nc')
    sdatabase='salad_scenario'
    roar_db=pdb.cls_sql_db(sdatabase,suser,shost,spassword)
    roar_db.connect()
    roar_db.cursor()

    ## sfields='object, object_feature, value'
    sfields='*'
    
    rows=roar_db.insert_many(stable,sfields,datavalues)

    ## args_str = ','.join(cur.mogrify("(%s,%s,%s,%s,%s,%s,%s,%s,%s)", \
    ##                                x) for x in tup)
    ## cur.execute("INSERT INTO table VALUES " + args_str) 

    roar_db.commit()
    roar_db.close_cursor()
    roar_db.close_connect()

    return(rows)

  ## ----------------------------------------------
  def rows2dtable(self,stable):
    """
    converts rows of SQL select into table of dictionary format
    Input:
          rows          list of lists as output of SQL select
          table_row     selects field as table row
          table_column  selects field as table column
          table_class   != -1 select filed as table value
                        =-1 table value =1 for all row-column pair
    """

    ctable=self.dtable_store[stable] 

    irow=0
    icol=0
    nclass=0
    for row in ctable.rows:
      srow=row[ctable.table_row]
      scol=row[ctable.table_column]
      if ctable.table_class!=-1:
        sclass=row[ctable.table_class]
        if sclass not in ctable.dclass:
          ctable.dclass[sclass]=nclass
          ctable.dclassinv[nclass]=sclass
          nclass+=1
      if srow not in ctable.drow:
        ctable.drow[srow]=irow
        ctable.drowinv[irow]=srow
        irow+=1
      if scol not in ctable.dcolumn:
        ctable.dcol[scol]=icol
        ctable.dcolinv[icol]=scol
        icol+=1

    for row in ctable.rows:
      irow=ctable.drow[row[ctable.table_row]]
      icol=ctable.dcolumn[row[ctable.table_column]]
      if ctable.table_class!=-1:
        iclass=ctable.dclass[row[ctable.table_class]]
      else:
        iclass=0
      if irow not in ctable.dtable:
        ctable.dtable[irow]={}
      if icol not in ctable.dtable[irow]:
        if ctable.table_class!=-1:
          ctable.dtable[irow][icol]=np.zeros(nclass)
        else:
          ctable.dtable[irow][icol]=np.zeros(1)
      ctable.dtable[irow][icol][iclass]=1

    return

  ## ---------------------------------------------------
  def dtable2ijv_coo(self,dtable):

    ndatacolumn=3
    lijv=[]   
    for rowkey, dcol in dtable.items():
      for colkey, value in dcol.items():
        lijv.append([rowkey,colkey,value])

    ndata=len(lijv)
    ## sort items by row and column
    lijv.sort()

    lxijv=[None]*ndatacolumn  ## row,column, value which could be vector
    xijv=np.array(lijv)
    lxijv[0]=xijv[:,0].astype(int)
    lxijv[1]=xijv[:,1].astype(int)
    lxijv[2]=xijv[:,2:]
    
    return(lxijv)

## ####################################################  
## ###################################################### 

