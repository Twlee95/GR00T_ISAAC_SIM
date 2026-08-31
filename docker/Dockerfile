FROM nvidia/cuda:12.8.0-runtime-ubuntu24.04
ENV DEBIAN_FRONTEND=noninteractive ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES

RUN apt-get update && apt-get install -y \
    python3.12 python3.12-venv python3.12-dev \
    git git-lfs wget curl \
    libgl1 libglu1-mesa libegl1 libglib2.0-0 libxrandr2 libxinerama1 libxcursor1 libxi6 \
 && rm -rf /var/lib/apt/lists/*

RUN python3.12 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --upgrade pip

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install "isaacsim[all,extscache]==6.0.1.0" --extra-index-url https://pypi.nvidia.com

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install -U torch torchvision --index-url https://download.pytorch.org/whl/cu128

RUN git clone --branch v3.0.0-beta2 --depth 1 https://github.com/isaac-sim/IsaacLab.git /opt/IsaacLab \
 && cd /opt/IsaacLab && ./isaaclab.sh -i

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install uv==0.12.5 && uv python install 3.10

COPY requirements.txt /tmp/
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install -r /tmp/requirements.txt --extra-index-url https://pypi.nvidia.com

WORKDIR /opt/IsaacLab
