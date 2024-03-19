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
        "rqt": "rqt",
        "kortex_bringup": "roslaunch kortex_bringup kortex_bringup.launch", # launch kortex - note that this starts a roscore
        "camera": "rosrun camera_node camera_node.py _cam_idx:=2", # TODO: change this to take in idx somehow. 
        "visual_servoing": "rosrun visual_servoing visual_servo_node.py"
    }
    
    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)