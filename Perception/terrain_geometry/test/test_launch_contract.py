import os
from pathlib import Path
import pytest
from ament_index_python.packages import get_package_share_directory,PackageNotFoundError
from launch import Action,LaunchDescription,LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions,perform_substitutions
from launch_ros.utilities import normalize_parameters,evaluate_parameters
import launch_ros.actions


@pytest.fixture
def expand(monkeypatch):
    captured={}
    class CaptureNode(Action):
        def __init__(self,*,package,executable,parameters=None,**kwargs):
            super().__init__()
            self.package,self.executable,self.parameters=package,executable,parameters or []
            self.arguments=kwargs.get('arguments',[])
        def execute(self,context):
            parameters=evaluate_parameters(context,normalize_parameters(self.parameters))
            arguments=[perform_substitutions(context,normalize_to_list_of_substitutions(v)) for v in self.arguments]
            captured[self.executable]=(self.package,parameters,arguments)
    monkeypatch.setattr(launch_ros.actions,'Node',CaptureNode)
    def run(options=None,package='terrain_geometry',filename='perception_system.launch.py'):
        captured.clear()
        service=LaunchService()
        path=os.path.join(get_package_share_directory(package),'launch',filename)
        service.include_launch_description(LaunchDescription([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),launch_arguments=(options or {}).items())]))
        assert service.run()==0
        return dict(captured)
    return run


def test_default_master_contains_only_real_processing(expand):
    assert set(expand())=={'terrain_node','marker_detection_node','marker_tf_broadcaster','marker_action_interface_node'}


def test_terrain_and_marker_pipelines_independent(expand):
    assert set(expand({'launch_marker_branch':'false'}))=={'terrain_node'}
    assert 'terrain_node' not in expand({'launch_terrain':'false'})
    assert expand({'launch_terrain':'false','launch_marker_branch':'false'})=={}


def test_clock_topic_and_exclusive_memory_ownership(expand):
    nodes=expand({'use_sim_time':'false','launch_memory':'true','rgb_topic':'/test/rgb'})
    terrain={k:v for entry in nodes['terrain_node'][1] if isinstance(entry,dict) for k,v in entry.items()}
    assert terrain['use_sim_time'] is False
    assert terrain['publish_to_obstacles_only'] is False
    assert 'obstacle_memory' in nodes
    assert Path(nodes['obstacle_memory'][1][0]).name=='obstacle_memory.yaml'
    detector={k:v for entry in nodes['marker_detection_node'][1] if isinstance(entry,dict) for k,v in entry.items()}
    assert detector['rgb_topic']=='/test/rgb'


def test_parameter_files_do_not_leak_between_stages(expand):
    nodes=expand({'launch_marker_mapping':'true','launch_landmark_adapter':'true'})
    for executable,stem in [('marker_detection_node','marker_detection'),('marker_tf_broadcaster','marker_tf'),
        ('marker_map_node','marker_mapping'),('marker_action_interface_node','marker_action_interface'),
        ('marker_landmark_adapter','landmark_adapter')]:
        assert Path(nodes[executable][1][0]).name==stem+'.yaml'


def test_master_uses_combined_rviz_and_standalone_frame(expand):
    nodes=expand({'launch_rviz':'true','rviz_fixed_frame':'test_frame'})
    arguments=nodes['rviz2'][2]
    assert Path(arguments[1]).name=='perception_system_view.rviz'
    assert arguments[-2:]==['-f','test_frame']


def test_testing_launch_selects_isolated_sensor_topics(expand):
    try:
        get_package_share_directory('perception_test_tools')
    except PackageNotFoundError:
        pytest.skip('Optional test-only package was not built')
    nodes=expand({'pipeline':'terrain','launch_rviz':'false'},'perception_test_tools','test_stack.launch.py')
    assert set(nodes)=={'sensor_inputs','terrain_node'}
    terrain={k:v for entry in nodes['terrain_node'][1] if isinstance(entry,dict) for k,v in entry.items()}
    assert terrain['input_pointcloud_topic'].startswith('/testing/perception/')
    assert terrain['use_sim_time'] is False
