######################
## Version 0.1 #######
######################
## import pickle
## ###############################
import psycopg2
import psycopg2.extras 
import numpy as np
## import scipy.linalg as sp_lin
## ###############################
## ###############################
## ###################################################### 
class cls_sql_db:

  def __init__(self,sdbname,suser,shost,spassword):

    self.sdbname=sdbname
    self.suser=suser
    self.shost=shost
    self.spassword=spassword
    self.dbconnect=None
    self.dbcursor=None

  ## ---------------------------------------------
  def connect(self):

    try:
      ## self.dbconnect = psycopg2.connect("dbname='template1' \
      ##                                    user='dbuser' \
      ##                                    host='localhost' \
      ##                                    password='dbpass'")

      ## dsn = 'dbname=%s host=%s user=%s password=%s' % \
      ##       (self.sdbname, self.shost, self.suser, self.spassword)
      dsn='dbname'+"="+self.sdbname+' ' \
                      +'user'+'='+self.suser+' ' \
                      +'host'+'='+self.shost+' ' \
                      +'password'+'='+self.spassword
      self.dbconnect = psycopg2.connect(dsn)
      ierr=0
    except:
      print('I am unable to connect to database:' \
            +self.sdbname+','+self.shost+','+self.suser)
      ierr=1
      
    return(ierr)
  ## -------------------------------------------
  def close_connect(self):
    self.dbconnect.close()
    
  ## -------------------------------------------
  def cursor(self):

    try:
      self.dbcursor=self.dbconnect.cursor()
      ierr=0
    except:
      print('Create cursor was unsuccessful')
      ierr=1
      
    return(ierr)

  ## -------------------------------------------
  def cursor_dict(self):

    try:
      self.dbcursor=self.dbconnect.cursor(cursor_factory \
                                          =psycopg2.extras.DictCursor)
      ierr=0
    except:
      print('Create cursor was unsuccessful')
      ierr=1
      
    return(ierr)

  ## -------------------------------------------
  def close_cursor(self):
    self.dbcursor.close()
  ## -------------------------------------------
  def commit(self):
    self.dbconnect.commit()
    
  ## ---------------------------------------------
  def execute_sql(self,ssql):

    try:
      sql_string='"""'+ssql+'"""'
      self.dbcursor.execute(sql_string)
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    return(iret)

  ## ---------------------------------------------
  def insert(self,stable,sfield,stype,datavalue):

    sql_string="INSERT INTO "+stable+' '+sfield+" VALUES "+stype
    try:
      self.dbcursor.execute(sql_string,datavalue)
      self.dbconnect.commit()
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    return(iret)

  ## ---------------------------------------------
  def update(self,stable,sfield,datavalue,dconditions):

    ldatavalue=[datavalue]
    sconditions=''
    for skey,sval in dconditions.items():
      sconditions=sconditions+' '+skey+' = (%s) and'  
      ldatavalue.append(sval)
    sconditions=sconditions[:-3]   ## drop the last 'and'
    ldatavalue=tuple(ldatavalue)
    
    sql_string="UPDATE "+stable+' SET '+sfield+"=(%s)"  \
                     +" WHERE "+sconditions 
    try:
      self.dbcursor.execute(sql_string,ldatavalue)
      self.dbconnect.commit()
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    return(iret)

  ## ---------------------------------------------
  def update_many(self,stable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """

    sfields_cond=lfields[0]
    for sfield in lfields[1:-1]:
      sfields_cond+=","+sfield
    if lfields[-1] is None:
      sfield_val=""
    else:
      sfield_val=lfields[-1]

    for datablock in datavalues:
      ldatavalue=[str(datablock[sfield_val])]
      sconditions=' '+lfields[0]+' = '+'(%s)'
      ldatavalue.append(str(datablock[lfields[0]]))
      for sfield in lfields[1:-1]:
        sconditions+=' and '+sfield+' = '+ '(%s)'
        ldatavalue.append(str(datablock[sfield]))
        
      tdatavalue=tuple(ldatavalue)
    
      sql_string="UPDATE "+stable+' SET '+sfield_val+"=(%s)"  \
                     +" WHERE "+sconditions 
      try:
        self.dbcursor.execute(sql_string,tdatavalue)
        self.dbconnect.commit()
        iret=0
      except:
        print("Execution of "+sql_string+'failed!')
        iret=1
        break
      
    return(iret)

  ## ---------------------------------------------
  def update_many_2(self,stable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 

    Al alternative solution:

    
    update users as u set
      email = u2.email,
      first_name = u2.first_name,
      last_name = u2.last_name
    from (values
      (1, 'hollis@weimann.biz', 'Hollis', 'O\'Connell'),
      (2, 'robert@duncan.info', 'Robert', 'Duncan')
    ) as u2(id, email, first_name, last_name)
    where u2.id = u.id;

    """

    sfields_cond=lfields[0]
    for sfield in lfields[1:-1]:
      sfields_cond+=","+sfield
    if lfields[-1] is None:
      sfield_val=""
    else:
      sfield_val=lfields[-1]

    sconditions=' '+'u2.'+lfields[0]+' = '+'u.'+lfields[0]
    for sfield in lfields[1:-1]:
      sconditions+=' and '+'u2.'+sfield+' = '+'u.'+sfield

    sql_string="UPDATE "+stable+' AS u '+'SET '
    sql_string+=lfields[-1]+'='+'u2.'+lfields[-1]
    sql_string+=' FROM ( VALUES ('
    sql_string+='%('+lfields[0]+')s,'
    for sfield in lfields[1:]:
      sql_string+='%('+sfield+')s,'
    sql_string=sql_string[:-1]
    ## for datarow in datavalues:
    ##   sql_string+='('
    ##   for sfield in lfields:
    ##     sql_string+=datarow[sfield]+','
    ##   sql_string+=')'
    sql_string+=') )'+' AS '+'u2('
    for sfield in lfields:
      sql_string+=sfield+','
    sql_string=sql_string[:-1]
    sql_string+=') '
    sql_string+='WHERE '+sconditions

    try:
      self.dbcursor.executemany(sql_string,datavalues)
      self.dbconnect.commit()
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    return(iret)

  ## ---------------------------------------------
  def insert_many(self,stable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """
   
    sfields=lfields[0]
    for sfield in lfields[1:]:
      sfields+=","+sfield

    sql_string='INSERT INTO '+stable+' ('+sfields+')'+' VALUES ('
    for xfield in lfields:
      sql_string=sql_string+'%('+xfield+')s, '
    sql_string=sql_string[:-2]  ## cut last ', '
    sql_string+=')'
    
    try:
      self.dbcursor.executemany(sql_string,datavalues)
      self.dbconnect.commit()
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    return(iret)

  ## ------------------------------------------------
  def create_table(self,stable,dfields,temporal=0):
    
    sql_string="CREATE"
    if temporal!=0:
      sql_string+=" TEMPORARY "
    sql_string+=" TABLE "
    ## sql_string+=" IF NOT EXIST "
    sql_string+=stable+" ("
    for sfield,stype in dfields.items():
      sql_string+=" "+sfield+" "+stype+","
    sql_string=sql_string[:-1]
    sql_string+=" );"

    try:
      self.dbcursor.execute(sql_string)
      iret=0
    except:
      iret=1

    self.dbconnect.commit()

    return(iret)
    
  ## ---------------------------------------------
  def insert_many_temporal(self,stable,lfields,datavalues):
    """
    datavalues = tuple of dictionaries ( { column_name : value, ... } ) 
    """

    sfields=lfields[0]
    for sfield in lfields[1:]:
      sfields+=","+sfield

    stabletemp=stable+"_temp"    
    ## self.dbcursor.execute("CREATE TEMPORARY TABLE "+stable+"_temp " \
    ##                       +"("+sfields+")"+";")   
    ## cur.execute("INSERT INTO table VALUES " + args_str) 
    ## sql_string="CREATE TEMPORARY TABLE "

    sql_string='DROP TABLE IF EXISTS '+stabletemp+';'
    self.dbcursor.execute(sql_string)
    self.dbconnect.commit()

    sql_string="CREATE TABLE "
    sql_string+=stabletemp+" AS "
    sql_string+="SELECT "+sfields+" FROM "+stable+" WHERE 1=2;"
    try:
      self.dbcursor.execute(sql_string)
      iret=0
    except:
      iret=1
    self.dbconnect.commit()

    sql_string='INSERT INTO '+stabletemp+' ('+sfields+')'+' VALUES ('
    for xfield in lfields:
      sql_string=sql_string+'%('+xfield+')s, '
    sql_string=sql_string[:-2]  ## cut last ', '
    sql_string+=')'
    
    try:
      self.dbcursor.executemany(sql_string,datavalues)
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1
      
    self.dbconnect.commit()

    sql_string='CREATE UNIQUE INDEX ON '+stabletemp+' '+'('+sfields+');'
    self.dbcursor.execute(sql_string)
    self.dbconnect.commit()
    
    return(iret)



  ## ---------------------------------------------
  ## def update_many(self,stable,tfields,datavalues):
  ##   """
  ##   update users as u set -- postgres FTW
  ##     email = u2.email,
  ##     first_name = u2.first_name,
  ##     last_name = u2.last_name
  ##   from (values
  ##     (1, 'hollis@weimann.biz', 'Hollis', 'O\'Connell'),
  ##     (2, 'robert@duncan.info', 'Robert', 'Duncan')
  ##   ) as u2(id, email, first_name, last_name)
  ##   where u2.id = u.id;
  ##   """

  ##   nfields=len(tfields)
  ##   sql_string="UPDATE "+stable+" as t "+"SET"+"\n"
  ##   for i in range(nfields):
  ##     sql_string=sql_string+" "+tfields[i]+"="+"s."+tfields[i]+"\n"
  ##   sql_string=sql_string+"from (values"+"\n"
    
    
    
  ##   try:
  ##     self.dbcursor.execute(sql_string)
  ##     self.dbcursor.commit()
  ##     iret=0
  ##   except:
  ##     print("Execution of "+sql_string+'failed!')
  ##     iret=1
      
  ##   return(iret)

## ---------------------------------------------
  def select(self,stable,lfields=None,scondition=''):

    if lfields is None:
      sfields='*'
    else:
      if len(lfields)>0:
        sfields=lfields[0]
        for sfield in lfields[1:]:
          sfields+=","+sfield
      else:
        sfields='*'

    sql_string="SELECT "+sfields+" FROM "+stable
    if len(scondition)>0:
      sql_string=sql_string+" WHERE "+scondition
    sql_string=sql_string+';'
    
    try:
      self.dbcursor.execute(sql_string)
      ierr=0
    except:
      print("Execution of "+sql_string+'failed!')
      ierr=1

    if ierr==0:
      rows=self.dbcursor.fetchall()
    else:
      rows=None   ## empty list
      
    return(rows)
  ## -----------------------------------------------
  def insert_update(self,stable,lfields,datavalues=None):
    """
    stable      string, table name
    lfields     list of strings, names of the fields,
                the last name referes to the output variable if there is one,
                                                            otherwise is None
    datavalues  tuple of directories: each row is an element of the tuple,
                within a row a directory assigns values to field names
                { field_name : value }
    
      Stackoverflow.com:
      http://stackoverflow.com/questions/4069718/postgres-insert-if-does-not-exist-already  
       
      Create temporary table. See docs here.

      CREATE TEMPORARY TABLE temp_data(name, name_slug, status);
      INSERT INTO temp_data(name, name_slug, status); 

      Add any indexes to the temp table.

      Do main table insert.

      INSERT INTO hundred(name, name_slug, status) 
          SELECT DISTINCT name, name_slug, status
          FROM hundred
          WHERE NOT EXISTS (
              SELECT 'X' 
              FROM temp_data
              WHERE 
                  temp_data.name          = hundred.name
                  AND temp_data.name_slug = hundred.name_slug
                  AND temp_data.status    = status
          );
    """

    sfields_cond=lfields[0]
    for sfield in lfields[1:-1]:
      sfields_cond+=","+sfield
    if lfields[-1] is None:
      sfield_val=""
    else:
      sfield_val=lfields[-1]
    

    if lfields[-1] is None:
      iret=self.insert_many_temporal(stable,lfields[:-1])
    else:
      iret=self.insert_many_temporal(stable,lfields,datavalues)

    ## ## !!!!!!!!!!!!!!!!!!!!!!!!!1
    ## self.dbcursor.execute("CREATE"+stable+"_temp " \
    ##                       +"("+sfields_cond+","+sfields_val+")"+";")   
    ## ## !!!!!!!!!!!!!!!!!!!!!!!!!1
    
    ## ldatavalue=[datavalues]
    stabletemp=stable+"_temp"
    sql_string="INSERT INTO "+stable+"("+sfields_cond+","+sfield_val+") "
    sql_string+="SELECT DISTINCT "+sfields_cond+","+sfield_val+" "
    sql_string+="FROM "+stable+" WHERE NOT EXISTS ( SELECT "+"'X'"+" "
    sql_string+="FROM "+stabletemp+" WHERE "

    sconditions=stable+"_temp"+"."+lfields[0]+" = "+stable+"."+lfields[0]
    for sfield in lfields[1:-1]:
      sconditions+=" AND "+stable+"_temp"+"."+sfield+" = "+stable+"."+sfield
    ## sconditions+=" AND "+stable+"_temp"+"."+sfield_val+" = "+stable \
    ##               +"."+sfield_val   

    sql_string+=sconditions+" );"

    try:
      self.dbcursor.execute(sql_string)
      iret=0
    except:
      print("Execution of "+sql_string+'failed!')
      iret=1

    self.dbconnect.commit()

    sql_string='DROP TABLE IF EXISTS '+stabletemp+';'
    self.dbcursor.execute(sql_string)
    self.dbconnect.commit()
     
    return(iret)
  
  ## ###################################################### 

