# Contributing to WayBionic Ground Station

This is the ROS 2 workspace for the WayBionic robotic ground station. This guide
covers how we work: how to set up, how to name branches, how to write commits,
and what a pull request needs before it can merge. Read it once before your first
contribution — it keeps everyone's work consistent and keeps `main` stable.

## Access & workflow

Everyone on the team has **Write** access, so you don't need to fork. You branch
directly off this repo, push your branch, and open a pull request.

`main` is protected — you can't push to it directly, and every change lands through
a reviewed PR. The loop:

```bash
git checkout main
git pull origin main
git checkout -b feature/your-thing-name   # see naming below
# ... make changes, commit ...
git push -u origin feature/your-thing-name
# then open a Pull Request on GitHub with base = main
```

## Branch naming

Use `type/short-description` in kebab-case. Add your first name at the end when a
branch is clearly "yours," so it's easy to tell apart at a glance.

```
type/short-description[-yourname]
```

Types we use:

| Type        | For                                     |
| ----------- | --------------------------------------- |
| `feature/`  | New functionality                       |
| `fix/`      | Bug fixes                               |
| `docs/`     | Documentation only                      |
| `refactor/` | Restructuring, no behavior change       |
| `research/` | Spikes, experiments, prototypes         |
| `chore/`    | Tooling, dependencies, cleanup          |

Real examples from this repo:

```
feature/ground-station-ui-khuzaymah-v2
feature/real-arm-urdf-import
research/doctor-camera-pipeline-gianna
```

## Commit messages

- Write the subject in the imperative: "Add diagnostics panel," not "Added" or "Adds."
- Keep the subject short (~50-72 chars). Put details in the body if you need them.
- Explain the *why*, not just the *what*, when it isn't obvious.
- Reference issues or PRs when relevant (e.g. `Refs #2`).

```
Add RViz diagnostics panel to bringup

Wires the waybionic_rviz_plugins panel into the default bringup
config so operators see joint state at launch.
Refs #2
```

## Before you open a PR

There's no CI yet, so validation is on you. From the workspace root, on a clean build:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
colcon test        # if the package has tests
```

If you touched Python, lint it:

```bash
flake8 .           # or: colcon test --packages-select <pkg>  (runs ament_flake8)
```

Make sure:

- [ ] Branch is up to date with `main` (`git pull origin main` or rebase).
- [ ] `colcon build` succeeds from a clean tree.
- [ ] Tests pass (if the package has any).
- [ ] No lint errors on Python you changed.
- [ ] No build artifacts committed (`build/`, `install/`, `log/` are gitignored — keep it that way).
- [ ] No personal or machine-specific paths committed (absolute paths, local `.vscode` settings, etc.).

## Opening the PR

- Keep it focused. One feature or fix per PR — small PRs get reviewed faster.
- Base the PR on `main`.
- In the description, cover **what** changed, **why**, and **how you tested it**.
- For anything visual (RViz, GUI), drop in a screenshot or a short clip.
- Link related issues and PRs.

## Review & merge

- Every PR needs approval from the code owner (**@yassinsolim**) before it can merge —
  this is requested automatically via [`.github/CODEOWNERS`](./.github/CODEOWNERS).
- Resolve all review conversations before merging.
- Pushing new commits dismisses stale approvals, so get the branch green before final review.
- We **squash merge** to keep `main`'s history clean (one PR = one commit).
- `main` is protected: no direct pushes, no force-pushes, no branch deletion.

## Development environment

Full build and launch steps are in [BuildInstructions.md](./BuildInstructions.md). Quick notes:

- **Target:** ROS 2 **Jazzy** on **Ubuntu 24.04**.
- **Windows:** use **WSL2** with Ubuntu 24.04.
- **Linux:** native, no extra setup.
- **macOS (Apple Silicon):** first-class support is still being worked out (Docker / VM).
  Ask in the team channel before starting, and **don't commit Mac-specific local paths**
  into shared config.

## Questions

Not sure about something? Ask in the team channel, or open a draft PR early and tag
@yassinsolim. Better to ask than to guess.
