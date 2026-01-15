# [CuRobo v0.7.7](https://curobo.org/)

### Python 3.11 
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.11 python3.11-venv python3.11-dev
```

## Installation 

- Install cuda toolkit from [NVIDIA](https://developer.nvidia.com/cuda-downloads)
- Ubuntu 20.04 or 22.04. Other linux environments might also work. We have added experimental support for Windows.
- NVIDIA GPU with VOLTA or newer architecture and at least 4 GB of VRAM. NVIDIA Jetson Orin is also supported.
- Python 3.8 to 3.10 is supported. 3.10 is recommended. Python >= 3.11 might work but is not tested.
- PyTorch 1.15 or newer. PyTorch 2.0+ is recommended. from [pytorch.org](https://pytorch.org/get-started/locally/)
- Start with a pytorch docker or a python environment with pytorch >= 1.10.
- Install git lfs with `sudo apt install git-lfs`.
- Clone cuRobo repository with `git clone -b v0.7.7 https://github.com/NVlabs/curobo.git`, move inside the cloned repo with `cd curobo`, and run `pip install -e . --no-build-isolation`. This will take 20 minutes to install.
- Run `pip install pytest`
- `python3 -m pytest .` to verify that all unit tests pass.
- Look at python scripts in examples/ path for examples.

### Docker Instructions (x86_64 only)
```bash
cd /path/to/curobo/docs/planners/docker
USER_ID=$(id -g "$USER") docker compose build
```
Start base image:
```bash
USER_ID=$(id -g "$USER") docker compose run --rm curobo
```
Start user image (bind-mounts `/home/$USER/code`):
```bash
USER_ID=$(id -g "$USER") docker compose run --rm curobo_user
```