# robosub_2025

Team Inspiration's repository for the RoboSub 2025 competition. We are using ROS, python, and C++.

## Getting Started

### Prerequisites

We have two different subs, Onyx and Graey, they both run on similar software setups.

#### Onyx

* Ubuntu 20.04
* ROS Noetic
* Python 3.8.10
* OpenCV 4.7.0 (CUDA)
* CVbridge (Noetic)

#### Graey

* Ubuntu 18.04
* Ros Melodic
* Python 3.8.0 (non-native)
* OpenCV 4.7.0 (CUDA)
* CVbridge (Noetic modified)

Because of this, when we design our code, we need to make sure that it is compatible between both subs. Most compatibility issues have been resolved; the main issue is tuning power to the thrusters differently if using dead-reckoning without sensor input.

### Installing

#### For Onyx or Graey

Note: If using installScripts the first two steps will be done for you, you just need to install.
The installScripts repository is used to install all necessary dependencies -- the link is here: https://github.com/InspirationRobotics/installScripts
Make sure to follow the README to install the dependencies in the correct order.

(For the following, you can also use the SSH link instead of the HTTPS link -- HTTPS is usually simpler)

```bash
git clone https://github.com/InspirationRobotics/robosub_2025
mv robosub_2023 auv && cd auv
python3 -m pip install -e .
```

#### On your own computer

```bash
git clone https://github.com/InspirationRobotics/robosub_2025
cd robosub_2025
python3 -m pip install -e ."[cv,dev]"
# or 
python2 -m pip install -e ."[cv,dev]"
```

### Repo Structure

In order not to have a messy repo, we have a specific structure for the repo. Please follow this structure when adding new features.

```bash
robosub_2025
├── auv
│   ├── device
│   │   ├── # everything related to the sensors
│   │   ├── # each device should have its own file/folder
│   ├── localization 
│   │   ├── # EKF node fused IMU, DVL, and barometer
│   ├── mission 
│   │   ├── # Mission classes
│   │   ├── # see auv/mission/template_mission.py for an example
│   ├── motion
│   │   ├── # Actuators code
│   │   ├── # each actuator should have its own file/folder
│   ├── cv
│   │   ├── # All CV classes and functions
│   │   ├── # see auv/cv/template_cv.py for an example
│   ├── utils
│   │   ├── # utility functions
├── scripts
│   ├── # bash scripts
├── mission
│   ├── # Mission files (calling mission classes)
├── tests
│   ├── # Unit tests (pytest)
├── config.json # where all the config values used in mission files are stored
├── setup.py # setup file for pip install
├── .gitignore
├── README.md
```

### Running

#### Tests

This will run all the tests in the tests folder using pytest.
Any file that has the name `test_*.py` will be run.

```bash
pytest
```

#### Mission

We have a mission class for each individual mission/task.
Those classes are then called in a mission file (see `mission/` folder).
The mission file will be the one giving the mission class the necessary parameters to run the mission (for examples variables from the config file).

You can run individually mission classes for debugging purpose (if they have a `if __name__ == "__main__"` block) by running:

```bash
python3 -m auv.mission.<mission_name>
# or
python2 -m auv.mission.<mission_name>
```

Else, you can import the mission class in a mission file and run it from there.

```py
from auv.mission.<mission_name> import <MissionClass>

# initialize mission
mission = <MissionClass>() 
# run mission
mission.run() 
# cleanup mission after it is done
mission.cleanup()
```

#### Mission Planner

There exists a mission planner that allows you to load and run missions from a yaml file. This is useful to craft a sequence of missions to run in a competition while avoiding the need to modify the code. It is a graph based planner that allows you to define the sequence of missions and the conditions to run them. On top of that, it does catch exceptions (and thus not leave the sub in an unknown state).
You can also visualize with the option `-v` the graph and double check that the missions are in the right order.

To run the mission planner, you can use the following command:

```bash
python3 -m auv.mission.mission_planner -p <path_to_yaml_file> (-v optional for graph visualization)
```

You can find an example of a yaml file in the `missions/` folder here: [plan_template.yaml](missions/plan_template.yaml).
To fully understand how to use the mission planner, you can take a look at the [mission_planner.py](auv/mission/mission_planner.py) file.

#### CV

The idea behind the CV classes is that they are run in a separate thread and they publish their results to a ROS topic.
You can run individually CV classes for debugging purpose (if they have a `if __name__ == "__main__"` block) by running:

```bash
python3 -m auv.cv.<cv_name>
```

## Contributing

### Pull requests

In order to contribute to the repo, you will need to open pull requests. To do so, create a new branch with ideally this format: `<your_name>/<feature_name>`.
Then, when you are done with your feature, open a pull request on github.
Opening a pull request will trigger a code review from one of the team members. This team member will be in charge of reviewing your code and making sure that it is working as intended. Please provide as much information as possible in your pull request (what you changed, why you changed it, etc...). Keep up the good practices !

> What if the default branch has changed since I started working on my feature ? No problem, just rebase your branch on the default branch and push it again. This will update your branch with the latest changes from the default branch.
> ```bash
> git checkout <your_branch>
> git rebase origin/master
> git push origin <your_branch>
> ```

If it's a small change / fix or in a period of rush, of course you can push directly to master, but please try to avoid it as much as possible, we don't want to push something that is not working into the sub *especially right before testings*.

### How to test your code

**soon**, we will have a simulator that will act as a digital clone for the sub, you will be able to spinup a docker container and test your code in it.

Additionally, you can use pytest for unit tests. Please write unit tests for your code and make sure that they pass before pushing your code.
This is to make sure that we don't break anything when we add new features.
Tests are mainly there to make sure that there is no syntax error and that the code runs without crashing, not necessarily to make sure that the code is doing what it is supposed to do (although it is a plus).

### Code Style (optionnal)

It is always pleasant to read code that is well formatted and well commented.
Ideally we would want to format our code using black, so try to run regularly on your code:

```bash
black .
```

If you have trouble running this command, you can use the following command instead:

```bash
python3 -m black .
```

(bonus) Code analysis is done using pylint. we will omit C0115 and C0116 for now, but try to fix the other errors.

```bash
pylint --disable=C0115,C0116 auv
```

## Running the AUVs

### Graey and Onyx

1. Git pull if necessary

    ```bash
    cd auv
    git pull
    ```

2. First, start running roscore and mavros. This script will launch both at the same time in separate screens.

    ```bash
    bash ./scripts/mavros.sh
    ```

3. Now run the cams scripts:

    ```bash
    screen -S cams
    python3 -m auv.device.camsVersatile.py
    ```

    Follow the prompts, enter which camera id you are using and then enter "start" or "stop"

4. If you would like camera feed

    ```bash
    screen -S bash
    cd ~/rtsp
    ./start_video.sh <camera_id>
    ```

5. Run pix_standalone for sensor and thruster capabilities.

    ```bash
    screen -S pix
    python3 auv.device.pix_standalone.py
    ```

    This will arm the sub by default. To disarm, run `python3 -m auv.utils.disarm`, to arm it back again use: `python3 -m auv.utils.arm`

6. Run your mission program

    ```bash
    cd auv/missions/
    python3 yourMission.py
    ```
