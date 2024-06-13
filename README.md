# Safe Outdoor flight

Winged uncrewed aerial vehicles (UAVs) can not easily return to a safe state like rotary-winged drones can, hence drastically increasing their risk of crashing [[1](https://www.nature.com/articles/nature14542)].
Current solutions that enable safe testing can be split into two groups: Static indoor tests allow winged UAVs to fly against constant airflow from a wind tunnel whilst being tethered.
This ensures safety but constrains movement to a small flight envelope the size of the wind tunnel [[2](http://arxiv.org/abs/2403.08598)]. Existing Outdoor safety mechanisms involve parachutes or blow-up pads which are too heavy for small UAVs, weighing at least 80g [[3](https://fruitychutes.com/uav_rpv_drone_recovery_parachutes)].
To address this gap, we propose a novel cable-guided mobile safety system (CGMSS) that detects and moves in tandem with the UAV, allowing tethered outdoor flight. It tracks the UAV using a camera, detects the end of the line using distance sensors and communicates via LEDs with the user.
![CGMSS](/docs/concept.png)

# Installation

To install the Safe Outdoor Flight system, follow these steps:

1. Clone the repository:
    ```
    git clone https://github.com/your-username/safe-outdoor-flight.git
    ```

2. Navigate to the project directory:
    ```
    cd safe-outdoor-flight/src
    ```
3. Create a virtual environment and activate it
    ```
    python -m venv myenv
    source myenv/bin/activate 
    ```

4. Install the required dependencies:
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
```
