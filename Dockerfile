
FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc g++ make \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-docker.txt .
RUN pip install --no-cache-dir -r requirements-docker.txt

COPY pyproject.toml README.md LICENSE ./
COPY gym_pybullet_drones ./gym_pybullet_drones
COPY voice_drone.py ./

RUN pip install --no-cache-dir --no-deps .

RUN mkdir -p /app/scan_images

CMD ["python", "voice_drone.py"]