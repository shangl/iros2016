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
  xdatarel=croardb.load_table('object_action_2', \
                              ['object','preposition'],['action']) 

  nitem=len(xdatarel[0])

  pval=0.1
  for i in range(nitem):
    ## qval=np.random.rand(1)[0]
    ## if qval<pval:
    if i==100:
      xdatarel[2][i]=15.0

  ## croardb.update(xdatarel,'score')

  print('Bye')
  return

## ################################################################
if __name__ == "__main__":
  if len(sys.argv)==1:
    iworkmode=0
  elif len(sys.argv)>=2:
    iworkmode=eval(sys.argv[1])
  roar_main(iworkmode)
