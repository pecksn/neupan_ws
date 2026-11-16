#!/usr/bin/env python3
# Module contract: Executable ROS entry point for camera-derived reference refinement.
# The implementation remains in the importable OSM module.

"""ROS entry point; path refinement lives in the osm_map_handler package."""
from osm_camera_path_refinement import main


if __name__ == '__main__':
    main()
