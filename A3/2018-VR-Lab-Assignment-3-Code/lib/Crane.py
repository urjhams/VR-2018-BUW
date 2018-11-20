#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua


### import application libraries
from lib.KeyboardInput import KeyboardInput
from lib.Hinge import Hinge
from lib.Arm import Arm
from lib.Hook import Hook


class Crane:
  
    ## constructor
    def __init__(self,
        PARENT_NODE = None,
        TARGET_LIST = [], # required for bounding box intersection in Hook class 
        ):
        

        ## init base node for whole crane
        self.base_node = avango.gua.nodes.TransformNode(Name = "base_node")
        self.base_node.Transform.value = avango.gua.make_trans_mat(0.0,-0.1,0.0)
        PARENT_NODE.Children.value.append(self.base_node)


        ## init internal sub-classes
        self.input = KeyboardInput()

        # 3.1
                     
        ## first hinge
        self.hinge0 = Hinge(
            PARENT_NODE = self.base_node,
            DIAMETER = 0.1,
            HEIGHT = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_identity_mat(),
            SF_ROT_INPUT = self.input.sf_rot_input0,
            MIN_ANGLE = 0,
            MAX_ANGLE = 360,
            )
        
        ## first arm segment
        self.arm0 = Arm(
            PARENT_NODE = self.hinge0.hinge_node,
            LENGTH = 0.12,
            DIAMETER = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_identity_mat(),
            )

        ## second hinge
        self.hinge1 = Hinge(
            PARENT_NODE = self.arm0.arm_end_node,
            DIAMETER = 0.04,
            HEIGHT = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_rot_mat(90.0,1,0,0),
            SF_ROT_INPUT = self.input.sf_rot_input1,
            MIN_ANGLE = -90,
            MAX_ANGLE = 0,
            )
        
        ## second arm segment
        self.arm1 = Arm(
            PARENT_NODE = self.hinge1.hinge_node,
            LENGTH = 0.08,
            DIAMETER = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_rot_mat(90.0,0,0,1),
            )

        ## second hinge
        self.hinge2 = Hinge(
            PARENT_NODE = self.arm1.arm_end_node,
            DIAMETER = 0.04,
            HEIGHT = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_rot_mat(90,0,0,1),
            SF_ROT_INPUT = self.input.sf_rot_input2,
            MIN_ANGLE = -90,
            MAX_ANGLE = 90,
            )
            
        ## second arm segment
        self.arm2 = Arm(
            PARENT_NODE = self.hinge2.hinge_node,
            LENGTH = 0.08,
            DIAMETER = 0.01,
            ROT_OFFSET_MAT = avango.gua.make_rot_mat(-90.0,0,0,1),
            )   

        self.hook = Hook().my_constructor(
            PARENT_NODE = self.arm2.arm_end_node,
            SIZE = 0.03,
            TARGET_LIST = TARGET_LIST,
            ) 