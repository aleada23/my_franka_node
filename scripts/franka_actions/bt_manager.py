import py_trees
from franka_actions import actions
class BehaviorTreeManager:
    def __init__(self):

        # Map action names to classes
        self.action_map = {
            "MovePose": actions.MovePose,
            "MoveJoints": actions.MoveJoints,
            "OpenGripper": actions.OpenGripper,
            "CloseGripper": actions.CloseGripper,
            "MoveDownUntillContact": actions.MoveDownUntillContact,
            "MeasureGripperSites": actions.MeasureGripperSites,
            "MeasureAppliedHE": actions.MeasureAppliedHE,
            "MeasureGripperOpnening": actions.MeasureGripperOpnening,
            "MeasureMassWithTorque": actions.MeasureMassWithTorque
        }
        self.condition_map = {
            
        }

    def build_tree_from_list(self, bt_list):
        if isinstance(bt_list, list) and len(bt_list) > 0:
            node_type = bt_list[0]
            children = bt_list[1:] if len(bt_list) > 1 else []

            if node_type.startswith("Sequence"):
                seq_node = py_trees.composites.Sequence(name="Sequence", memory=True)
                for child in children:
                    seq_node.add_child(self.build_tree_from_list(child))
                return seq_node
            
            elif node_type.startswith("Selector"):
                seq_node = py_trees.composites.Selector(name="Selector", memory=True)
                for child in children:
                    seq_node.add_child(self.build_tree_from_list(child))
                return seq_node

            elif node_type.startswith("Parallel"):
                fb_node = py_trees.composites.Parallel(name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
                for child in children:
                    fb_node.add_child(self.build_tree_from_list(child))
                return fb_node

            else:
                # Leaf node: action or condition
                # bt_list can be like ["MovePose", {"desired_pose": "object1_pose"}]
                if isinstance(bt_list, list) and len(bt_list) == 2 and isinstance(bt_list[1], dict):
                    node_name = bt_list[0]
                    params = bt_list[1]
                else:
                    node_name = bt_list
                    params = {}
                if node_name in self.action_map.keys():
                    return self.action_map[node_name](node_name, **params)
                elif node_name in self.condition_map.keys():
                    return self.condition_map[node_name](node_name, **params)
                else:
                    raise ValueError(f"Unknown node type: {node_name}")
        else:
            raise ValueError(f"Invalid BT list: {bt_list}")

    def build_tree(self, bt_list):
        self.root_node = py_trees.trees.BehaviourTree(self.build_tree_from_list(bt_list))
        return self.root_node


    def tick(self, display_tree = False, display_blackboard = False):
        if display_tree:
            print(py_trees.display.unicode_tree(self.root_node.root, show_status=True))
        if display_blackboard:
            print(py_trees.display.unicode_blackboard())
        if self.root_node.root.status in [py_trees.common.Status.RUNNING,py_trees.common.Status.INVALID]:
            self.root_node.tick()
        
        return self.root_node.root.status

    def print_tree(self):
        py_trees.display.render_dot_tree(self.root_node.root)
