# Winged Aircraft Safety Platform (WASP) Enables Safe Outdoor Flight Test

Outdoor testing of fixed-wing uncrewed aerial vehicles (UAVs) remains inherently risky, as winged platforms cannot rapidly transition to a safe hover state following malfunction. 
Existing safety approaches, including wind-tunnel confinement and onboard parachute systems, either restrict the flight envelope or are impractical for lightweight aircraft. 
We present the Winged Aircraft Safety Platform (WASP), a motorized, cable-guided robotic system designed to enable safe, tethered outdoor flight testing of fixed-wing UAVs. 
The WASP autonomously tracks a UAV using onboard vision and positions itself above the aircraft, maintaining a lightweight safety tether (< 5 g) capable of preventing ground impact in the event of failure.
Beyond safety, the system functions as a mobile measurement platform. 
By integrating encoder-based motion sensing, vision-based pose estimation, and real-time three-dimensional wind measurements from an ultrasonic anemometer, the WASP reconstructs the UAV’s full flight state, including position, velocity, attitude, airspeed, angle of attack, and sideslip angle. 
Outdoor experiments conducted over a 70 m cable span demonstrate stable tracking at speeds up to 14 m/s with accelerations of 8 m/s², and close agreement between WASP-derived aerodynamic estimates and onboard sensor measurements.
By combining safety and data collection, WASP accelerates the development of next-generation winged UAVs.

<p align="center">
  <img src="docs/success_flight.gif" width="800">
</p>

${\color{red} \text{add link to youtube video}}$




## Hardware Setup

The full [CAD model]((https://cad.onshape.com/documents/e720400fb6a2299e38239c2c/v/94bfd332e710a20f5be1e874/e/eed919d2f4453d865657b7de)) of the WASP is available online on
Onshape. 
The robot structure is primarily made from 2 mm folded aluminum sheets, while the more technical parts are 3D-printed in [PLA](https://ultimaker.com/materials/s-series-tough-pla/). 
The outer ring of the drive wheel is printed in [TPU](https://ultimaker.com/materials/s-series-tpu-95a/), which provides friction between the drivetrain and the 2 mm diameter
steel cable. Because this TPU layer gradually wears out after repeated
runs, it may have to be occasionally replaced.
Without listing all standard hardware components such as screws, nuts,
bearings, hooks, and magnets, the main elements of the system are
summarized below:

| | | |
|-----------------------|--------------------------|-------------------------|
| [ODrive S1](https://eu.odriverobotics.com/shop/odrive-s1) | [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) | [GT6 EVO Radio Controller](https://www.conrad.ch/fr/p/reely-gt6-evo-radiocommande-a-poignee-pistolet-2-4-ghz-nombre-de-canaux-6-avec-recepteur-1780646.html) |
| [ODrive Encoder OA1](https://eu.odriverobotics.com/shop/odrive-encoder-oa1-on-axis-magnetic-encoder-with-rs485) | [Raspberry Pi Global Shutter Camera](https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/) | [NeoPixel LEDs](https://www.adafruit.com/product/1312) |
| [ODrive Motor D5312s 330KV](https://eu.odriverobotics.com/shop/dual-shaft-motor-d5212s-300kv) | [TriSonica LI-550](https://www.licor.com/products/trisonica/LI-550-mini?category=Meteorology) | [4S LiPo Battery](https://www.galaxus.ch/en/s5/product/nvision-lipo-battery-148v-2500mah-35c-1480-v-2500-mah-rc-batteries-7946624) |
| [ODrive USB Isolator](https://eu.odriverobotics.com/shop/usb-isolator) | [TriSonica USB Adapter](https://www.licor.com/products/trisonica/accessories) | [MTTEC Keto HV BEC](https://www.mttec.de/MTTEC-KETO-HV-BEC-12s-10A-20A-Peak-V2) |

Once fully assembled, the platform looks as follows:

${\color{red} \text{Add front and back photos of WASP}}$





## Sofware Setup

### Raspberry Pi setup






## Use case





## Data analysis





## Acknowledgements
- **Authors:** Simon Jeger & Cyril Goffin
- **Affiliation:** [Laboratory of Intelligent Systems (LIS)](https://www.epfl.ch/labs/lis/), EPFL
- **Publication:** [Winged Aircraft Safety Platform (WASP) Enables Safe Outdoor Flight Test](docs/Winged_Aircraft_Safety_Platform_(WASP)_Enables_Safe_Outdoor_Flight_Test.pdf)






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
