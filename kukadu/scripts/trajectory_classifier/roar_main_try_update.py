######################
## Version 0.1 #######
######################

import sys
import numpy as np
import roar_db_cls
## ---------------------------------
## #################################
def roar_main(iworkmode):

  croardb=roar_db_cls.cls_roar_db()
  ## xdatarel=croardb.load_table('object_action_2', \
  ##                             ['object','preposition'],['action']) 

  dbtable='test1'
  ## dfields={'fieldx1' : 'varchar(20)', 'fieldx2' : 'varchar(20)', 'fieldy' : 'integer[3]'}
  ## croardb.create_table(dbtable,dfields)
  
  ## lfields=list(dfields.keys())
  ## lfields.sort()
  ## datavalues=({'fieldx1' : '11', 'fieldx2' : '12', 'fieldy' : [1,None,3] },
  ##             {'fieldx1' : '21', 'fieldx2' : '22', 'fieldy' : [1,None,3] },
  ##             {'fieldx1' : '31', 'fieldx2' : '32', 'fieldy' : [1,2,3] })
  
  ## ## datavalues=({'fieldx1' : 11, 'fieldx2' : 12, 'fieldy' : '{1,2,3}'},
  ## ##             {'fieldx1' : 21, 'fieldx2' : 22, 'fieldy' : '{1,2,3}'},
  ## ##             {'fieldx1' : 31, 'fieldx2' : 32, 'fieldy' : '{1,2,3}'})

  ## ## croardb.insert_table(dbtable,lfields,datavalues)

  ## datavalues=({'fieldx1' : '11', 'fieldx2' : '12', 'fieldy' : [3,4,5] },
  ##             {'fieldx1' : '21', 'fieldx2' : '22', 'fieldy' : [3,4,5] },
  ##             {'fieldx1' : '31', 'fieldx2' : '32', 'fieldy' : [3,4,5] })
  
  ## ## datavalues=({'fieldx1' : '11', 'fieldx2' : '12', 'fieldy' : '{4,5,6}'},
  ## ##             {'fieldx1' : '21', 'fieldx2' : '22', 'fieldy' : '{4,5,6}'},
  ## ##             {'fieldx1' : '31', 'fieldx2' : '32', 'fieldy' : '{4,5,6}' })

  ## ## croardb.update_table(dbtable,lfields,datavalues)

  ## ## croardb.load_table(dbtable,lfields[:2],lfields[2:])
  ## croardb.load_table_mmr(dbtable,lfields)


  dfields={'fieldx' : 'integer[3]', 'fieldy' : 'integer[3]'}
  croardb.create_table(dbtable,dfields)
  
  lfields=list(dfields.keys())
  lfields.sort()
  datavalues=({'fieldx' : [4,5,6],  'fieldy' : [1,2,3] },
              {'fieldx' : [4,5,6],  'fieldy' : [1,2,3] },
              {'fieldx' : [4,5,6],  'fieldy' : [1,2,3] })
  
  croardb.insert_table(dbtable,lfields,datavalues)

  datavalues=({'fieldx' : [4,5,6],  'fieldy' : [5,2,3] },
              {'fieldx' : [4,5,6],  'fieldy' : [5,2,3] },
              {'fieldx' : [4,5,6],  'fieldy' : [5,2,3] })

  croardb.update_table(dbtable,lfields,datavalues)

  (X,Y)=croardb.load_table_mmr(dbtable,lfields[:1],lfields[1:])
  dval=croardb.pyarrays2sqlarray(lfields[:1],lfields[1:],X,Y)
  croardb.update_table(dbtable,lfields,dval)

  print('Bye')
  return

## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  roar_main(iworkmode)
