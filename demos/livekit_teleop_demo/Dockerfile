FROM python:3.10-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir livekit livekit-api opencv-python-headless numpy
WORKDIR /app
COPY . /app
CMD ["python3", "robot_livekit_agent.py"]
