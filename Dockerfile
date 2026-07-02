FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    FMS_HOST=127.0.0.1 \
    FMS_PORT=8787 \
    FMS_ENABLE_ROS=1 \
    ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils \
    ffmpeg \
    git \
    lsof \
    psmisc \
    pulseaudio-utils \
    python3-opencv \
    python3-pip \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-sensor-msgs-py \
    ros-jazzy-web-video-server \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    fastapi==0.115.0 \
    python-multipart==0.0.9 \
    uvicorn[standard]==0.30.6

WORKDIR /app
COPY . /app

EXPOSE 8787
CMD ["/app/scripts/run_local.sh"]
