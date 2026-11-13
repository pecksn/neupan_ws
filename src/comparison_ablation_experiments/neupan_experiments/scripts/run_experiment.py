#!/usr/bin/env python3
"""Compose an opt-in launch IN MEMORY. Never edit the production launch/configs."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml


def param(node, name, value):
    for old in list(node.findall('param')):
        if old.get('name')==name:node.remove(old)
    return ET.SubElement(node,'param',name=name,value=str(value).lower() if isinstance(value,bool) else str(value))


def compose(mode, workspace, navigation_file):
    if mode not in ('gps_always','teb','neupan'):raise ValueError('unknown experiment')
    root=ET.parse(workspace/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch').getroot()
    settings=dict(outdoor_mode='true',low_precision_startup='true',enable_rviz='true',
        enable_camera_right_path='false',enable_camera_right_control='false',
        enable_hybridnets_overlay='false',enable_system_log='true',gps_transport_limit_sec='4.0',
        gps_common_utc_verified='true',gps_raw_capture_sec='60',neupan_result_publish_timeout='0.60',
        neupan_result_compute_timeout='0.50',max_forward_speed_mps='1.5',max_reverse_speed_mps='1.5')
    if mode=='gps_always':settings.update(use_global_ekf='true',low_precision_startup='false')
    for name,value in settings.items():
        node=next(v for v in root.findall('arg') if v.get('name')==name)
        node.set('default',value)
    for node in root.iter('node'):
        if node.get('name')=='system_run_logger':param(node,'label','system_run_experiment_'+mode)
    navigation=None
    if mode=='gps_always':
        navigation=ET.parse(workspace/'src/robot_base_driver/launch/navigation_core.launch').getroot()
        # This isolated branch supplies a lag-aware EKF and an independent GPS
        # projector; it is NOT the unsupported production global-EKF branch.
        for node in navigation.findall('arg'):
            if node.get('name')=='gps_time_mode_validated':node.set('default','experiment_lag_aware_ekf')
        for node in navigation.iter('node'):
            if node.get('name')=='gps_fix_to_map_odom':
                node.set('pkg','neupan_experiments');node.set('type','gps_always_projection.py')
                param(node,'heading_rotation_deg','$(arg gps_rotation)')
                param(node,'heading_source_is_north_cw','$(arg gps_heading_source_is_north_cw)')
            if node.get('name')=='ekf_global':
                # Differential local XY+yaw continue between real GPS messages;
                # every finite GPS position is an observation, irrespective of quality.
                p=ET.SubElement(node,'rosparam',param='odom0_config')
                p.text=yaml.safe_dump([True,True,False,False,False,True]+[False]*9)
                param(node,'smooth_lagged_data',True);param(node,'history_length',8.0)
                param(node,'predict_to_current_time',True)
                param(node,'odom2','/experiments/gnss_heading')
                heading=ET.SubElement(node,'rosparam',param='odom2_config')
                heading.text=yaml.safe_dump([False]*5+[True]+[False]*9)
                param(node,'odom2_differential',False)
                param(node,'odom2_queue_size',10)
                param(node,'odom2_pose_rejection_threshold',1000000.)
        for include in root.iter('include'):
            if include.get('file')=='$(find robot_base_driver)/launch/navigation_core.launch':
                include.set('file',str(navigation_file))
        for node in root.iter('node'):
            if node.get('name')=='control_mode_mux':param(node,'enable_auto_localization_safety_stop',True)
    elif mode=='teb':
        for parent in root.iter():
            for node in list(parent):
                if node.tag=='node' and node.get('pkg')=='neupan_ros' and node.get('type')=='neupan_node.py':
                    parent.remove(node)
        for node in root.iter('node'):
            if node.get('name')=='outdoor_global_astar':
                ET.SubElement(node,'remap',{'from':'/move_base_simple/goal','to':'/teb_test/osm_goal'})
        move=ET.SubElement(root,'node',pkg='move_base',type='move_base',name='move_base',output='screen')
        ET.SubElement(move,'remap',{'from':'cmd_vel','to':'/experiments/teb_cmd_raw'})
        for namespace in ('global_costmap','local_costmap'):
            ET.SubElement(move,'rosparam',file='$(find teb_test)/config/common_costmap.yaml',command='load',ns=namespace)
        for name in ('global_costmap','local_costmap','move_base','teb_local_planner'):
            ET.SubElement(move,'rosparam',file='$(find teb_test)/config/'+name+'.yaml',command='load')
        robot=yaml.safe_load((workspace/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml').read_text())['robot']
        length,width,wheelbase=robot['length'],robot['width'],robot['wheelbase']
        rear=(length-wheelbase)/2.;front=wheelbase+rear
        footprint=[[front,width/2.],[front,-width/2.],[-rear,-width/2.],[-rear,width/2.]]
        for namespace in ('global_costmap','local_costmap','TebLocalPlannerROS/footprint_model'):
            item=ET.SubElement(move,'rosparam',param=namespace+('/vertices' if 'footprint_model' in namespace else '/footprint'))
            item.text=yaml.safe_dump(footprint)
        for name,value in {'wheelbase':wheelbase,'max_vel_x':.8,'max_vel_x_backwards':.45,
                           'cmd_angle_instead_rotvel':True}.items():param(move,'TebLocalPlannerROS/'+name,value)
        param(move,'OSMPathPlanner/debug_log',False)
        ET.SubElement(root,'node',pkg='neupan_experiments',type='teb_command_gate.py',
                      name='teb_experiment_command_gate',output='screen')
    return root,navigation


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=('gps_always','teb','neupan'))
    parser.add_argument('--validate-only',action='store_true')
    args,_=parser.parse_known_args()
    import rospkg
    import roslaunch
    workspace=Path(rospkg.RosPack().get_path('neupan_ros')).parents[1]
    with tempfile.TemporaryDirectory(prefix='neupan_experiment_') as folder:
        child=Path(folder)/'navigation.launch'
        root,navigation=compose(args.mode,workspace,child)
        if navigation is not None:ET.ElementTree(navigation).write(str(child),encoding='unicode')
        xml=ET.tostring(root,encoding='unicode')
        uuid=roslaunch.rlutil.get_or_generate_uuid(None,False)
        if args.validate_only:
            with contextlib.redirect_stdout(io.StringIO()):
                config=roslaunch.config.load_config_default([],None,roslaunch_strs=[xml],verbose=False)
            names=[v.name for v in config.nodes]
            assert ('move_base' in names)==(args.mode=='teb')
            assert ('neupan_control' in names)==(args.mode!='teb')
            assert ('ekf_global' in names)==(args.mode=='gps_always')
            assert config.params['/control_mode_mux/enable_auto_localization_safety_stop'].value is True
            print(json.dumps(dict(mode=args.mode,nodes=[v.name for v in config.nodes],
                                 scope='parse only; no nodes started'),ensure_ascii=False))
            return
        # Do not replace any live vehicle process, reopen its serial ports or
        # publish into an already running controller's command path.
        import rosnode
        existing=set(rosnode.get_node_names())
        with contextlib.redirect_stdout(io.StringIO()):
            config=roslaunch.config.load_config_default([],None,roslaunch_strs=[xml],verbose=False)
        conflicts=sorted(existing & {v.namespace.rstrip('/')+'/'+v.name for v in config.nodes})
        if conflicts:raise RuntimeError('Stop the previous system explicitly first; conflicting nodes: '+str(conflicts))
        roslaunch.configure_logging(uuid)
        parent=roslaunch.parent.ROSLaunchParent(uuid,[],roslaunch_strs=[xml])
        try:
            parent.start();parent.spin()
        finally:parent.shutdown()


if __name__=='__main__':main()
