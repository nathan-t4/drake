# Quadruped-Drake
A passive simulation of mini-cheetah (similar to the [atlas](https://github.com/RobotLocomotion/drake/tree/master/examples/atlas) example in the [Drake](https://github.com/RobotLocomotion/drake) repository). 

Build:
```
$ cd path_to_drake/examples/quadruped
$ bazel build ...
```

Start visualizer:
```
$ cd path_to_drake
$ bazel run //tools:meldis -- --open-window
```

Run simulation:
```
$ cd path_to_drake
$ bazel-bin/examples/quadruped/mini_cheetah_sim
```
