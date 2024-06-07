# Safe Outdoor flight

Winged uncrewed aerial vehicles (UAVs) can not easily return to a safe state like rotary-winged drones can, hence drastically increasing their risk of crashing [[1](https://www.nature.com/articles/nature14542)].
Current solutions that enable safe testing can be split into two groups: Static indoor tests allow winged UAVs to fly against constant airflow from a wind tunnel whilst being tethered.
This ensures safety but constrains movement to a small flight envelope the size of the wind tunnel [[2](http://arxiv.org/abs/2403.08598)]. Existing Outdoor safety mechanisms involve parachutes or blow-up pads which are too heavy for small UAVs, weighing at least 80g [[3](https://fruitychutes.com/uav_rpv_drone_recovery_parachutes)].
To address this gap, we propose a novel cable-guided mobile safety system (CGMSS) that detects and moves in tandem with the UAV, allowing tethered outdoor flight. It tracks the UAV using a camera, detects the end of the line using distance sensors and communicates via LEDs with the user.
