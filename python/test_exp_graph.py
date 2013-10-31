#example to check pyton binding

# from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
# from dynamic_graph.sot.core.meta_tasks_kine import *
# from numpy import *
# 
# ## Create the robot romeo.
# from dynamic_graph.sot.romeo.robot import *
# robot = Robot('romeo', device=RobotSimu('romeo'))
# 
# ## Binds with ROS. assert that roscore is running.
# from dynamic_graph.ros import *
# ros = Ros(robot)
# 
# # Create a simple kinematic solver.
# from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
# solver = initialize ( robot )
# 
# 
# #-------------------------------------------------------------------------------
# #----- MAIN LOOP ---------------------------------------------------------------
# #-------------------------------------------------------------------------------
# # define the macro allowing to run the simulation.
# from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
# dt=5e-3
# @loopInThread
# def inc():
#     robot.device.increment(dt)
# 
# runner=inc()
# [go,stop,next,n]=loopShortcuts(runner)
# 
# # ---- TASKS -------------------------------------------------------------------
# # --- CONTACTS
# # define contactLF and contactRF
# for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
#     contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
#     contact.feature.frame('desired')
#     contact.gain.setConstant(10)
#     contact.keep()
#     locals()['contact'+name] = contact
# # ---- TASKS -------------------------------------------------------------------
# # ---- TASKS -------------------------------------------------------------------



from dynamic_graph.sot.expression_graph.expression_graph import *

expg = FeatureExpressionGraph('expg')
expg.displaySignals()

