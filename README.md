# Drake

Pre-compiled drake files.

The provided drake binary do not have the plugin activated. These plugins are however visible when you compile from source using bazel. This is a temporary fix to make all the plugins work on the binary as well. The idea is pretty simple. I have copied all the files from the drake/workspace/tools/drake_visualizer/plugin to the drake/bin/folder, edited the singleton funcitions as suggested by (here)[https://github.com/RobotLocomotion/drake/issues/10486#issuecomment-618148956]. 

To get all plugins activated, clone this repository, or just take the `drake_plugin` folder and the `drakeVisualizerPluginEnabled.sh` from the bin directory of this repo, copy it your local `drake/bin` folder. Then run the `drakeVisualizerPluginEnabled.sh` to see all the plugins activated in the visualizer.
