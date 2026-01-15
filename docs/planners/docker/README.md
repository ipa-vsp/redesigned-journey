<!--
Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
property and proprietary rights in and to this material, related
documentation and any modifications thereto. Any use, reproduction,
disclosure or distribution of this material and related documentation
without an express license agreement from NVIDIA CORPORATION or
its affiliates is strictly prohibited.
-->
# Docker Instructions (x86_64 only)

This folder contains the x86_64-only dockerfiles and a docker-compose setup.
Check [Docker Development](https://curobo.org/source/getting_started/5_docker_development.html) for
general instructions.

Build:
```
USER_ID=$(id -g "$USER") docker compose build
```

Start base image:
```
USER_ID=$(id -g "$USER") docker compose run --rm curobo
```

Start user image (bind-mounts `/home/$USER/code`):
```
USER_ID=$(id -g "$USER") docker compose run --rm curobo_user
```

To check your GPU's compute capability, run:
```
nvidia-smi --query-gpu=compute_cap --format=csv,noheader
```