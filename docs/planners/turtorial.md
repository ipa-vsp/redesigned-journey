# cuRobo Tutorial (Beginner to Advanced)

This guide walks you from a first run to deeper customization and performance work.

## 0) Prerequisites

1. NVIDIA GPU with recent drivers and CUDA toolkit.
2. Python 3.8+ and `pip`.
3. Git and a C++ toolchain (needed for CUDA extensions).

Optional:
- Isaac Sim, if you want to use the simulation examples.

## 1) Clone and create a virtual environment (Beginner)

```bash
git clone https://github.com/NVlabs/curobo.git
cd curobo
python -m venv .venv
source .venv/bin/activate
```

## 2) Install PyTorch with CUDA (Beginner)

Install a PyTorch build that matches your CUDA version. Follow the official selector:
https://pytorch.org/get-started/locally/

Example (adjust for your CUDA version):

```bash
pip install torch --index-url https://download.pytorch.org/whl/cu118
```

## 3) Install cuRobo (Beginner)

```bash
pip install -e .
```

Verify the install and CUDA:

```bash
python -c "import torch, curobo; print('torch cuda:', torch.cuda.is_available()); print('curobo:', curobo.__version__)"
```

## 4) Run your first examples (Beginner)

Start with small examples that do not require extra assets:

```bash
python examples/kinematics_example.py
python examples/ik_example.py
python examples/collision_check_example.py
```

What to expect:
- Kinematics and IK examples print or plot results.
- Collision check prints distances for world and self-collisions.

## 5) Understand configs and swap robots (Intermediate)

Robot and world configuration files live under:
- `src/curobo/content/configs/robot`
- `src/curobo/content/configs/world`

Try editing or swapping robot models in examples:

1. Open `examples/collision_check_example.py`.
2. Change `robot_file = "franka.yml"` to `robot_file = "iiwa.yml"`.
3. Re-run the example.

If you want your own robot:
1. Copy one of the existing YAML files in `src/curobo/content/configs/robot`.
2. Update joint limits, link names, and collision geometry.
3. Point your example or script to the new YAML.

## 6) Motion generation and trajectory optimization (Intermediate)

Run motion generation and trajectory optimization:

```bash
python examples/motion_gen_example.py
python examples/trajopt_example.py
```

Notes:
- These examples show how to build a world, set a goal, and generate a collision-free trajectory.
- You can switch robots and world configs as in the previous step.

## 7) Advanced: performance, profiling, and MPC

1. Profiling and performance:

```bash
python examples/motion_gen_profile.py
```

2. Model predictive control:

```bash
python examples/mpc_example.py
```

3. World representation and mesh handling:

```bash
python examples/world_representation_example.py
python examples/mesh_dataset.py
```

## 8) Advanced: Isaac Sim integration (Optional)

If you want the Isaac Sim examples:

1. Install cuRobo with extras:

```bash
pip install -e ".[isaacsim]"
```

2. Run an example under `examples/isaac_sim` within an Isaac Sim environment.

## 9) Troubleshooting tips

- If CUDA extensions fail to build, check that your CUDA toolkit matches your PyTorch CUDA version.
- If `torch.cuda.is_available()` is `False`, verify drivers and environment variables.
- For larger examples, ensure you have enough GPU memory (VRAM).

## 10) Next steps (Advanced)

1. Create your own task config under `src/curobo/content/configs/task`.
2. Integrate with a simulator or real robot using your preferred middleware.
3. Batch-run experiments and log performance metrics from the examples.
