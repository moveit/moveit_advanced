moveit_workspace_analysis
=========================

How to use
----------

You need to start your robot moveit configuration before starting moveit workspace analysis, for example 

```
roslaunch  ur5_moveit_config demo.launch
```

And start analysis program

```
roslaunch moveit_workspace_analysis workspace_analysis.launch group_name:=manipulator
```

The `group name` argument must be exist in the moveit configuration settings you include above.


To visualize the result, start

```
roslaunch moveit_workspace_analysis reader.launch
```

and enable `Display` -> `Marker` -> `/workspace_analysis_results/moveit_workspace_reader/workspace` in your `rviz`


Troubleshooting
---------------

```
[ERROR] [1516790240.275392209]: No kinematics solver instantiated for group 'right_arm'
```

This problem occurs when one of the following problems:

- the kinematics.yaml has a typo or does not specify a plugin that can be found for the group you instantiate.
- the srdf defines your group in a way that is not a chain. Right now there is a check that your group is a chain before trying to run IK for it. I would try defining the group using the <chain> tag in the srdf.

c.f. https://groups.google.com/forum/#!topic/moveit-users/NpbzaXp97KE