import os
import sys

cmd = 'rosrun drive_gen3 do_action.py --velocity 0.01 --force_lim 10 --action r'
os.system(cmd)
cmd = 'rosrun drive_gen3 do_action.py --velocity 0.01 --force_lim 10 --action r'
os.system(cmd)
cmd = 'rosrun drive_gen3 do_action.py --velocity 0.01 --force_lim 10 --action u'
os.system(cmd)
cmd = 'rosrun drive_gen3 do_action.py --velocity 0.01 --force_lim 13 --action d'
os.system(cmd)
cmd = 'rosrun drive_gen3 do_action.py --velocity 0.01 --force_lim 10 --action l'
os.system(cmd)
