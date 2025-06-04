# Safe outdoor flight with air speed sensor

Fixed-wing Uncrewed Aerial Vehicles (UAVs) cannot hover or return to a stationary state, making outdoor testing inherently risky. 
Existing approaches often rely on indoor tethered setups in front of wind tunnels, which constrain manoeuvrability and fail to replicate realistic environmental conditions. 
This paper presents the Winged Aircraft Safety Platform (WASP): a motorized, cable-guided system that enables safe, tethered outdoor flight of fixed-wing UAVs. 
The WASP autonomously tracks the UAV using onboard vision and logs local wind conditions using a 3D wind sensor. In the event of a malfunction during
testing, a safety tether prevents the UAV from crashing.


<!-- # SSH onnection

1. Connect the Raspberry Pi to a wifi network (preferably a smartphone's hotspot if planning on using the device outside)

2. Connect the laptop to the same wifi network then connect to the Raspberry Pi (password: lislis)
    ```
   ssh lis@raspberrypi.local
    ```
3. Optional: In VSCode on the laptop, use the extansion SSH Remote to access the Raspberry Pi's code

# Installation

1. Clone the repository:
    ```
    git clone https://github.com/your-username/safe-outdoor-flight.git
    ```

2. Navigate to the project directory:

3. Create a virtual environment and activate it
    ```
    python -m venv myenv
    source myenv/bin/activate 
    ```

4. Install the required dependencies in the virtual environment:
    ```
    pip install -r requirements.txt
    ```

5. Configure the system by editing the configuration file `config.js` with your desired settings.


# Use
## Tracking
1. Run calibration script

```
python vision.py --calibrate
```

2. Run main script

```
python main.py
```


## Move forward or backward without tracking

1. Run main script for forward

```
python main.py --forward
```

2. Run main script for backward

```
python main.py --backward
``` -->
