# 2D Multi-Robot Simulator

A lightweight Python kinematic simulator for multi-robot trajectory planning, designed to mirror ROS 2 architecture for easy deployment to physical hardware.

## Quickstart

1. **System Dependencies (Linux/Ubuntu):**
   The C++ solver requires native build tools and Python development headers.
   ```bash
   sudo apt update
   sudo apt install -y cmake build-essential python3-dev git

2. **Clone and enter the repository:**
   ```bash
   git clone [https://github.com/Abderraoufkhellas/Multi_Robot_Interaction.git](https://github.com/Abderraoufkhellas/Multi_Robot_Interaction.git)
   cd Multi_Robot_Interaction


3. **Setup the virtual environment:**
    ```bash
    python3 -m venv venv
    source venv/bin/activate

    pip install --upgrade pip setuptools wheel
    pip install Cython
    pip install --no-build-isolation git+[https://github.com/sybrenstuvel/Python-RVO2.git](https://github.com/sybrenstuvel/Python-RVO2.git)
    pip install -r requirements.txt

4. **Run the master loop:**
    ```bash
    python main.py