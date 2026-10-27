# AAA agent routing

This repository is an umbrella snapshot, not one buildable workspace. Keep context small.

## Route first

- Robot research, sensing, mapping, Nav2, perception, Isaac: `robot_research_ws/`
- Jetson/Humble chassis, CAN, swerve controller, web console: `mobile_base_ws/`
- Planner comparisons and lattice generation only: `omni_lattice_benchmark_ws/`
- Mechanical/URDF reference: `chassis_4wis/`
- Third-party examples: `abc/`; do not edit unless explicitly requested.
- Thesis: `greenhouse_4wis_thesis/`; hardware installers: `配置软件文档等/`.

Read the nearest child `AGENTS.md` after choosing a scope. Do not inventory the whole repository again unless the task is explicitly repository-wide.

## Context budget

- Search only the selected workspace/package. Prefer `rg` with a path and narrow globs.
- Do not scan `build/`, `install/`, `log/`, `.git/`, `.venvs/`, `datasets/`, `maps/`, `models/`, `results/`, `runs/`, `weights/`, binary meshes, installers, or archives by default.
- Read `PROJECTS.md` for structure; read package README/config/tests before broad documentation.
- Treat dated plans as history. Runtime configuration and tests are authoritative.
- Large data inspection must be explicitly required by the task.
- For an explicitly requested excluded directory, use `rg --no-ignore <pattern> <exact-path>` rather than widening the global search.

## Git

The root and some child workspaces have separate Git histories. Run `git rev-parse --show-toplevel` before staging. Never stage unrelated child-workspace changes.

## Current navigation

The main mobile base uses Smac State Lattice with the official Omni lattice, MPPI `motion_model: Omni`, an omnidirectional velocity smoother, Collision Monitor, and the swerve controller. Planner benchmarks are independent from runtime navigation.
