#MAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_01.png
#USER REQUEST (U): You must stack green objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_3 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_3 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_3"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_3"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_3 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_3 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "is_at_home",
     "type": "Condition"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_01.png
#USER REQUEST (U): You must stack blue parallelepipeds and yellow cylinder
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "blue_parallelepiped_4"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "blue_parallelepiped_4"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_0 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_0 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_0"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_0"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_0 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_0 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_01.png
#USER REQUEST (U): You must stack parallelepiped objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_02.png
#USER REQUEST (U): You must stack cube objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "blue_parallelepiped_4"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "blue_parallelepiped_4 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "temp_pose_0 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "home",
     "type": "Action"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_02.png
#USER REQUEST (U): You must stack green cube objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_cylinder_3 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_cylinder_3"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_cylinder_3 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "green_semi_cylinder_2 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "args": [
      "home"
     ],
     "type": "Action"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_02.png
#USER REQUEST (U): You must stack parallelepipeds
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "blue_parallelepiped_4 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_1 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_03.png
#USER REQUEST (U): You must stack cylinders
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_03.png
#USER REQUEST (U): You must stack green objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_1 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "temp_pose_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "is_at_home",
     "type": "Condition"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): You must stack green objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_6"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_6"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "is_at_home",
     "type": "Condition"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): You must pile green objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_parallelepiped_6"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "name": "MovePose",
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "type": "Action"
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "name": "MovePose",
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "args": [
      "home"
     ],
     "type": "Action"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): Stack one green object and one pink
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_5 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_5 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_5"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_5"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_parallelepiped_5 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_parallelepiped_5 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): Stack the pink objects
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_3 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_3 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_3"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_3"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "pink_parallelepiped_3 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "pink_parallelepiped_3 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "pink_parallelepiped_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "is_at_home",
     "type": "Condition"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): Lift a green object, then lift the other one
bt = {
 "children": [
  {
   "args": [],
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "args": [],
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "args": [],
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "args": [],
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "name": "MovePose",
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "args": [
            "green_parallelepiped_6"
           ],
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "args": [
            "green_parallelepiped_6"
           ],
           "name": "MovePose",
           "type": "Action"
          }
         ],
         "type": "Parallel"
        },
        {
         "args": [],
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "name": "is_at_pose",
           "type": "Condition"
          },
          {
           "args": [
            "green_parallelepiped_6 + z_offset"
           ],
           "name": "MovePose",
           "type": "Action"
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "args": [],
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "args": [],
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [],
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "name": "is_at_pose",
       "type": "Condition"
      },
      {
       "args": [
        "green_parallelepiped_5 + z_offset"
       ],
       "name": "MovePose",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "args": [
      "home"
     ],
     "name": "MoveJoints",
     "type": "Action"
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}

#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_04.png
#USER REQUEST (U): Stack the yellow parallelepipeds
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_8 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_8 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_8"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_8"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_8 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_8 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_9 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_9 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_9 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_9 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_10 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_10 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_10"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_10"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_10 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_10 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_8 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_8 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_8 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_8 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_11 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_11 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_11"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_11"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_parallelepiped_11 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_parallelepiped_11 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_10 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_10 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_parallelepiped_10 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_parallelepiped_10 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}


#IMAGE PATH (I):  Pix_FT_datasets/real_samples/real_images/real_05.png
#USER REQUEST (U): Stack all the cylinders blocks
bt = {
 "children": [
  {
   "name": "is_at_home",
   "type": "Condition"
  },
  {
   "children": [
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_6"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_6"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "yellow_cylinder_6 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "yellow_cylinder_6 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_0 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_5 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_5 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_5"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_5"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "green_cylinder_5 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "green_cylinder_5 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_6 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_6 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "yellow_cylinder_6 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "yellow_cylinder_6 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_grasped",
       "type": "Condition"
      },
      {
       "children": [
        {
         "children": [
          {
           "name": "is_gripper_open",
           "type": "Condition"
          },
          {
           "name": "OpenGripper",
           "type": "Action"
          }
         ],
         "type": "Selector"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_cylinder_7 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_cylinder_7 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_cylinder_7"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_cylinder_7"
           ]
          }
         ],
         "type": "Parallel"
        },
        {
         "name": "CloseGripper",
         "type": "Action"
        },
        {
         "children": [
          {
           "name": "is_at_pose",
           "type": "Condition",
           "args": [
            "red_cylinder_7 + z_offset"
           ]
          },
          {
           "name": "MovePose",
           "type": "Action",
           "args": [
            "red_cylinder_7 + z_offset"
           ]
          }
         ],
         "type": "Parallel"
        }
       ],
       "type": "Sequence"
      }
     ],
     "type": "Selector"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_cylinder_5 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_cylinder_5 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "children": [
      {
       "name": "is_contact_detected",
       "type": "Condition"
      },
      {
       "name": "MoveDownUntillContact",
       "type": "Action"
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "OpenGripper",
     "type": "Action"
    },
    {
     "children": [
      {
       "name": "is_at_pose",
       "type": "Condition",
       "args": [
        "green_cylinder_5 + z_offset"
       ]
      },
      {
       "name": "MovePose",
       "type": "Action",
       "args": [
        "green_cylinder_5 + z_offset"
       ]
      }
     ],
     "type": "Parallel"
    },
    {
     "name": "MoveJoints",
     "type": "Action",
     "args": [
      "home"
     ]
    }
   ],
   "type": "Sequence"
  }
 ],
 "type": "Selector"
}
