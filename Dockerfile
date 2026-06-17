FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    FMS_HOST=127.0.0.1 \
    FMS_PORT=8787 \
    FMS_ENABLE_ROS=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    lsof \
    psmisc \
    python3-opencv \
    python3-pip \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    fastapi==0.115.0 \
    uvicorn[standard]==0.30.6

WORKDIR /app
COPY . /app

EXPOSE 8787
CMD ["/app/scripts/run_local.sh"]

