import py_trees
from franka_actions import actions, checks

class BehaviorTreeManager:
    def __init__(self, pose_controller, joints_controller, data_listener):
        self.pose_controller = pose_controller
        self.joints_controller = joints_controller
        self.data_listener = data_listener

        self.action_map = {
            "MovePose": actions.MovePose,
            "MoveJoints": actions.MoveJoints,
            "OpenGripper": actions.OpenGripper,
            "CloseGripper": actions.CloseGripper,
            "MoveDownUntillContact": actions.MoveDownUntillContact,
            "MeasureGripperSites": actions.MeasureGripperSites,
            "MeasureMassWithTorque": actions.MeasureMassWithTorque
        }
        
        self.condition_map = {
            "is_at_home": checks.IsAtHome,
            "is_at_pose": checks.IsAtPose,
            "is_grasped": checks.IsGrasped,
            "is_gripper_open": checks.IsGripperOpen,
            "is_contact_detected": checks.IsContactDetected
        }

    def build_tree_from_json(self, node_data):
        """
        Recursively builds the tree from the dictionary structure.
        """
        node_type = node_data.get("type")
        
        if node_type == "Sequence":
            node = py_trees.composites.Sequence(name="Sequence", memory=True)
            for child in node_data.get("children", []):
                node.add_child(self.build_tree_from_json(child))
            return node

        elif node_type == "Selector":
            node = py_trees.composites.Selector(name="Selector", memory=False)
            for child in node_data.get("children", []):
                node.add_child(self.build_tree_from_json(child))
            return node

        elif node_type == "Parallel":
            node = py_trees.composites.Parallel(name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
            for child in node_data.get("children", []):
                node.add_child(self.build_tree_from_json(child))
            return node

        node_name = node_data.get("name")
        args = node_data.get("args", [])

        if node_type == "Action":
            return self._create_action(node_name, args)
        elif node_type == "Condition":
            return self._create_condition(node_name, args)
        
        raise ValueError(f"Unknown node configuration: {node_data}")

    def _create_action(self, name, args):
        cls = self.action_map.get(name)
        if not cls:
            raise ValueError(f"Action {name} not found in map.")
        
        if name == "MovePose" or name == "MoveDownUntillContact":
            return cls(name, self.pose_controller, *args)
        elif name == "MoveJoints":
            return cls(name, self.joints_controller, *args)
        elif "Measure" in name:
            return cls(name, self.data_listener, *args)
        else:
            return cls(name, *args)

    def _create_condition(self, name, args):
        cls = self.condition_map.get(name)
        if not cls:
            raise ValueError(f"Condition {name} not found in map.")
        
        return cls(name, self.data_listener, *args)

    def build_tree(self, json_structure):
        if "O_BT" in json_structure:
            root_data = json_structure["O_BT"]
        else:
            root_data = json_structure
            
        self.root_node = py_trees.trees.BehaviourTree(self.build_tree_from_json(root_data))
        return self.root_node

    def tick(self, display_tree=False):
        if display_tree:
            print(py_trees.display.unicode_tree(self.root_node.root, show_status=True))
        
        
        self.root_node.tick()
        return self.root_node.root.status