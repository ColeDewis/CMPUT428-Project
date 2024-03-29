import libtmux
import time
from os import path

if __name__ == "__main__":
    server = libtmux.Server(
        config_file=path.expandvars("/home/user/project428/scripts/.tmux.conf")
    )
    if server.has_session("sim"):
        exit()
    else:
        session = server.new_session("sim", start_directory="/home/user/project428", attach=False)
        
    # terminals for the simulation to start
    terminals = {
        "core": "roscore",
        "rqt": "rqt",
        "camera1": "rosrun camera_node camera_node.py _cam_idx:=0", # TODO: change this to take in idx somehow. 
        "camera2": "rosrun camera_node camera_node.py _cam_idx:=2", # TODO: change this to take in idx somehow. 
        "tracking": "rosrun camera_node tracking_node.py _idxs:=\"[0, 2]\"", 
    }
    
    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)