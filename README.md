# Voice-Controlled Drone Simulation

A Python-based drone simulation built using **gym-pybullet-drones** and **PyBullet**. The project allows users to control a simulated drone through terminal commands, navigate a custom environment, and capture images while scanning the surroundings.

## Features

* **Drone simulation:** Uses PyBullet physics through `gym-pybullet-drones`.
* **Terminal-based control:** Control the drone using commands entered in the terminal.
* **Flight movements:** Take off, move forward/backward/left/right/up/down, and land.
* **Environment scanning:** Rotate the drone to scan its surroundings.
* **Image capture:** Save scan images as PNG files.
* **Custom environment:** Includes obstacles and a simulated environment.
* **Docker support:** Includes Docker configuration for containerized execution.

## Tech Stack

* Python
* PyBullet
* gym-pybullet-drones
* NumPy
* Pillow
* Docker

## Project Structure

```text
gym-pybullet-drones/
├── voice_drone.py
├── requirements.txt
├── requirements-docker.txt
├── Dockerfile
├── .dockerignore
├── .gitignore
├── gym_pybullet_drones/
│   └── examples/
│       └── voice_drone.py
└── scan_images/
```

`scan_images/` is generated when scan images are captured and may be excluded from version control.

## Requirements

* Python 3.12
* Git
* A compatible Windows, Linux, or macOS environment
* PyBullet and the required Python dependencies

Docker is optional for local development.

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/dhruvaharish005/Drones-Project.git
cd Drones-Project
```

### 2. Create a virtual environment

**Windows PowerShell:**

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### 3. Install dependencies

```powershell
pip install -r requirements.txt
```

If the package itself is not installed in your environment, install it from the project directory:

```powershell
pip install -e .
```

## Running the Simulation

Run the main simulation script:

```powershell
python voice_drone.py
```

The simulation starts and accepts commands through the terminal.

## Available Commands

| Command    | Description                                          |
| ---------- | ---------------------------------------------------- |
| `takeoff`  | Initiates takeoff                                    |
| `forward`  | Moves forward                                        |
| `backward` | Moves backward                                       |
| `left`     | Moves left                                           |
| `right`    | Moves right                                          |
| `up`       | Moves upward                                         |
| `down`     | Moves downward                                       |
| `scan`     | Rotates to scan the surroundings and captures images |
| `stop`     | Stops the current movement                           |
| `land`     | Lands the drone                                      |
| `quit`     | Exits the simulation                                 |

## Scan Image Output

When the `scan` command is executed, captured images are saved as PNG files in a timestamped directory under:

```text
scan_images/
```

This allows captured frames to be organized by scan session.

## Docker

The project includes a Dockerfile and a lightweight Docker dependency list.

### Build the image

```powershell
docker build -t voice-drone-sim .
```

### Run the container

```powershell
docker run --rm -it voice-drone-sim
```

**Note:** The current simulation uses a graphical PyBullet interface. Running the container with a visible GUI on Windows requires additional display-server configuration. Building the Docker image successfully does not automatically provide GUI access.

For normal interactive GUI development, running the project directly in the Python virtual environment is the simpler approach.

## Current Limitations

* Control is currently terminal-command-based; speech recognition is not implemented.
* The project operates in simulation and does not control a physical drone.
* Docker GUI display requires additional configuration on Windows.

## Future Improvements

* Integrate speech recognition for voice-based commands.
* Improve obstacle detection and autonomous navigation.
* Add more advanced scanning and image-processing capabilities.
* Support headless simulation and automated testing.
* Explore integration with real drone hardware.

## Acknowledgements

This project builds on the [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) simulation framework.

## License

Refer to the project's existing `LICENSE` file for licensing terms.
