from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['moveit_ork']
d['scripts'] = ['bin/moveit_planning_scene_from_ork.py']
d['package_dir'] = {'': 'src'}

setup(**d)
