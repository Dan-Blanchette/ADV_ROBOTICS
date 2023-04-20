# RoboDK Python Intermediate file to generate robot programs.
# Program name: PaintDemo
# This file requires the post processor: 
#   Fanuc_RJ3
# to generate your robot program.
# This is a temporary file and you can delete it once you have generated your program.
# 
# Post processor documentation: https://robodk.com/doc/en/PythonAPI/postprocessor.html

import sys
import os
sys.path.append(os.path.abspath(r"""/home/tarnished-dan22/RoboDK/Posts/""")) # temporarily add path to POSTS folder

from Fanuc_RJ3 import *

try:
  from robodk.robomath import PosePP as p
except:
  # This will be removed in future versions of RoboDK
  from robodk import PosePP as p


print('Total instructions: 6')a
r = RobotPost(r"""Fanuc_RJ3""",r"""Fanuc P-50iB/10L""",6, axes_type=['R','R','R','R','R','R'], native_name=r"""Fanuc P-50iB/10L""", ip_com=r"""127.0.0.1""", api_port=20500, prog_ptr=94362479431040, robot_ptr=94362445670576)

r.ProgStart(r"""PaintDemo""")
r.RunMessage(r"""Program generated by RoboDK v5.5.4 for Fanuc P-50iB/10L on 30/03/2023 12:24:00""",True)
r.RunMessage(r"""Using nominal kinematics.""",True)
r.setFrame(p(1847.5,25.562,-163.784,0,0,0),2,r"""Frame 2""")
r.setTool(p(50,0,450,0,30,0),-1,r"""Paint gun""")
r.MoveJ(p(-512.995,-434.817,681.29,176.847,29.434,-177.272),[-12.7769,34.2168,-8.53046,77.2163,55.6271,89.7235],[0,0,1])
r.MoveJ(p(-212.062,-43.8675,115.509,176.847,-1.40372e-06,-180),[3.80634,37.8354,-17.3326,86.8643,45.8729,71.9941],[0,0,1])
r.MoveJ(p(-196.297,-42.9612,243.332,176.81,4.86844,179.397),[3.8,34.28,-12.43,86.86,45.87,71.99],[0,0,1])
r.MoveJ(p(-215.461,184.898,115.509,176.847,-1.70944e-06,-180),[12.99,38.7529,-17.421,93.0247,47.1473,59.1249],[0,0,1])
r.MoveJ(p(-201.933,188.076,223.024,176.801,3.96276,178.841),[12.99,35.77,-13.3,93.02,47.14,59.12],[0,0,1])
r.MoveJ(p(-512.995,-434.817,681.29,176.847,29.434,-177.272),[-12.7769,34.2168,-8.53046,77.2163,55.6271,89.7235],[0,0,1])
r.ProgFinish(r"""PaintDemo""")
r.ProgSave(r"""/home/tarnished-dan22/Documents/RoboDK/Programs""",r"""PaintDemo""",True,[r"""gedit"""])