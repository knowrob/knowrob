knowrob_map_tools
===

Additional predicates for manipulating semantic maps and reasoning about them


### urdf_to_sem

  A script to convert URDF to knowrob:SemanticEnvironmentMap

  ``` bash
$ rosrun knowrob_map_tools urdf_to_sem

usage: urdf_to_sem [-h] [-n NAMESPACE] [-s SUFFIX] [-f] urdf [sem]

positional arguments:
  urdf                  Path to URDF file. (required)
  sem                   Path to output SemanticEnvironmentMap file. (default: <urdf file name>.owl)

optional arguments:
  -h, --help            show this help message and exit
  -n NAMESPACE, --namespace NAMESPACE
                        Namespace of output map. (default: file name of urdf)
  -s SUFFIX, --suffix SUFFIX
                        Suffix for SemanticEnvironmentMap instance. (default: unique string)
  -f, --overwrite       Overwrite output file if exists. (default: false)
```

  - Example
  
    - 1. Convert URDF model to SemanticEnvironmentMap
  
    ```bash
    $ rosrun knowrob_map_tools urdf_to_sem -s PM580j -n iai-map kitchen.urdf kitchen.owl
    ```

    - 2. Load Ontology
    
    ```prolog
    ?- register_ros_package('knowrob_vis').
    ?- owl_parse('kitchen.owl').
    ?- owl_individual_of(M, knowrob:'SemanticEnvironmentMap'), marker_update(object(M)).
    ```
