# Winged Aircraft Safety Platform (WASP) Enables Safe Outdoor Flight Test

Outdoor testing of fixed-wing uncrewed aerial vehicles (UAVs) remains inherently risky, as winged platforms cannot rapidly transition to a safe hover state following malfunction. 
Existing safety approaches, including wind-tunnel confinement and onboard parachute systems, either restrict the flight envelope or are impractical for lightweight aircraft. 
We present the Winged Aircraft Safety Platform (WASP), a motorized, cable-guided robotic system designed to enable safe, tethered outdoor flight testing of fixed-wing UAVs. 
The WASP autonomously tracks a UAV using onboard vision and positions itself above the aircraft, maintaining a lightweight safety tether (< 5 g) capable of preventing ground impact in the event of failure.
Beyond safety, the system functions as a mobile measurement platform. 
By integrating encoder-based motion sensing, vision-based pose estimation, and real-time three-dimensional wind measurements from an ultrasonic anemometer, the WASP reconstructs the UAV’s full flight state, including position, velocity, attitude, airspeed, angle of attack, and sideslip angle. 
Outdoor experiments conducted over a 70 m cable span demonstrate stable tracking at speeds up to 14 m/s with accelerations of 8 m/s², and close agreement between WASP-derived aerodynamic estimates and onboard sensor measurements.
By combining safety and data collection, WASP accelerates the development of next-generation winged UAVs.

![Demo](docs/success_flight.gif)


Add Youtube video link?

---



## Hardware Setup

---



## Sofware Setup

---



## Use case

---



## Data analysis

---



## Acknowledgements
- **Authors:** Simon Jeger & Cyril Goffin
- **Affiliation:** [Laboratory of Intelligent Systems (LIS)](https://www.epfl.ch/labs/lis/), EPFL
- **Publication:** [Winged Aircraft Safety Platform (WASP) Enables Safe Outdoor Flight Test](docs/Winged_Aircraft_Safety_Platform_(WASP)_Enables_Safe_Outdoor_Flight_Test.pdf)

---




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
