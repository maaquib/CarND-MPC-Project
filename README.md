# CarND-Controls-MPC ![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)
Self-Driving Car Engineer Nanodegree Program

### The Model

The kinematic model used in the project is the one described in the lessons and uses the vehicles position (`x, y`), orientation (`psi`) and velocity (`v`). Using these variables to construct a state vector we can predict where it will be in the future after timestep `dt`. Additionally, we also want to capture how errors affect the state of the vehicle. We include the cross track error (`cte`) and the orientation error (`epsi`) in the state, to minimize the difference between the actual and reference trajectories.

```cpp
x[t + 1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t + 1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t + 1] = psi[t] - (v[t] / Lf) * delta[t] * dt;
v[t + 1] = v[t] + a[t] * dt;
cte[t + 1] = (f[t] - y[t]) + (v[t] * sin(epsi[t]) * dt);
epsi[t + 1] = (psi[t] - psides[t]) - ((v[t] / Lf) * delta[t] * dt);
```

`L​f` measures the distance between the front of the vehicle and its center of gravity in our `psi` and `cte` equations. Since the perception of rotation in the simulator is reversed, we use `- Lf` instead of what was provided (`+ Lf`) as part of the model equations in the lessons.

Using the above set of model equations we can provide the system with actuator values to control and change the state of the vehicle.

### Timestep Length and Elapsed Duration (N & dt)
The values chosen for Timestep Length are `N = 10` and for Elapsed Duration `dt = 0.1`.

I tried using `N = 15` but did not see much improvement so decided to stick with N = 10. Using a value smaller than 0.1 dt (`dt = 0.05`) caused larger oscillations around the track center with the vehicle rolling over the curb at times. Although, I believe modifying the cost factor multipliers would have led to a better performance, I did not explore this further in the interest of time.

### Polynomial Fitting and MPC Preprocessing
Before fitting the polynomials, there is preprocessing that is done on the provided waypoints. Since the waypoints are in map perspective, they are transformed to vehicle perspective using the approach described [here](https://discussions.udacity.com/t/waypoints-going-crazy/270597/2).

```cpp
for (int i = 0; i < ptsx.size(); ++i) {
  waypoints_x_vals.push_back(cos(-psi) * (ptsx[i] - px)
                        - sin(-psi) * (ptsy[i] - py));
  waypoints_y_vals.push_back(sin(-psi) * (ptsx[i] - px)
                        + cos(-psi) * (ptsy[i] - py));
}
```

Following this, a third order polynomial is fitted (`polyfit` function) through the transformed waypoints and the `coeffs` obtained are used to predict the error in actual and reference trajectories. The initial state vector is then constructed and used for MPC processing.

### Model Predictive Control with Latency
To deal with latency in applying actuator controls to the vehicle (100ms), I implemented the suggestion provided [here](https://discussions.udacity.com/t/how-to-take-into-account-latency-of-the-system/248671/2). Rather than applying the current actuator control values, their is a delay introduced artificially by applying the actuator values from an older timestep. This makes the vehicle handle latency in correctly.

### Screen Recording of Lap
[![MPC-Project](http://img.youtube.com/vi/0q3NUuvYhmQ/0.jpg)](http://www.youtube.com/watch?v=0q3NUuvYhmQ "MPC-Project")

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
