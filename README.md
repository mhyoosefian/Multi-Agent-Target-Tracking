# Multi-Agent Target Tracking with Obstacle Avoidance

MATLAB implementation of multi-agent target localization and tracking with range-only sensors. Three agents track a moving target in the formation that is optimal for localization (derived from the Fisher information matrix), estimate the target's state with a distributed information EKF, and follow their desired trajectories with a Lyapunov-based controller in two scenarios:

1. **External disturbances:** an EKF on each agent estimates the disturbance, which is fed back to the controller to reject it.
2. **Obstacles:** a control barrier function (CBF) safety filter, solved as a quadratic program, guarantees collision-free motion around static obstacles and other agents.

**For an explanation of the method and the results, see the project page:**
**https://mhyoosefian.github.io/projects/multi-agent-target-tracking/**

## Requirements

- MATLAB R2020a or newer (for `exportgraphics`)
- Optimization Toolbox (for `quadprog`, used by the CBF safety filter)

## Repository structure

```
scenario1-disturbance/   agents under external disturbances, with disturbance estimation
  runMe.m                main script
  estimate_follower_disturbance.m   EKF for each agent's state and disturbance
  estimate_leader_state.m           information EKF for the target
  compute_control_action.m          Lyapunov-based tracking controller
  utils/                 dynamics, measurement model and Jacobians
scenario2-obstacles/     disturbance-free agents with static obstacles
  runMe.m                main script
  cbf_controller.m       CBF-QP safety filter
```

Each folder also contains the figures produced by its script (PDF).

## How to use the code

1. Open MATLAB in `scenario1-disturbance` or `scenario2-obstacles`.
2. Run `runMe.m`.

The script simulates the target and the three agents, then plots the trajectories and the estimation errors and saves them as PDF files in the same folder. Simulation settings (sampling time, duration, number of agents, obstacle positions and radii) are at the top of `runMe.m`. To watch an animation of the simulation, set `show_animation = 1`.

## Citation

If you use the code in your research work, please cite the following papers:

```
@article{nooshabadi2024stealthy,
  title={Stealthy Optimal Range-Sensor Placement for Target Localization},
  author={Nooshabadi, Mohammad Hussein Yoosefian and Sipahi, Rifat and Lessard, Laurent},
  journal={IEEE Control Systems Letters},
  volume={8},
  pages={2763--2768},
  year={2024},
  publisher={IEEE}
}
```

```
@inproceedings{shahraki2025spacecraft,
  title={Spacecraft attitude control under reaction wheel constraints using control lyapunov and control barrier functions},
  author={Shahraki, Milad Alipour and Lessard, Laurent},
  booktitle={2025 American Control Conference (ACC)},
  pages={947--952},
  year={2025},
  organization={IEEE}
}
```
