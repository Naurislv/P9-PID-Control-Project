# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Solution

PID controller consists of 3 parts: 3  coefficients which are manually tuned and 3 errors which are actual feedback from a system (car).

PID = - Position + Integral - Differential = Kp * p_error - Ki * i_error + Kd * d_error

p_error: Proportional to cross track error (CTE). CTE is a distance from car and trajectory point where a car should be. We are using PID for steering so p_error = CTE. As CTE increases so as p_error increases and PID value increases - we want to steer more back to center as we are further away (CTE is bigger).

i_error += CTE. With i_error we can balance imbalanced system. For example, if our steering wheel is not perfect and it tends to steer more to right than left, then i_error would solve that issue.

d_error: current CTE - previous CTE. d_error greater when the difference between current CTE and previous CTE is greater. It helps to recover faster when CTE is greater and helps to follow straight line where p_error won't do that because of fluctuations.

```
double PID::Predict_steer(){
    return - _p[0] * p_error - _p[1] * i_error - _p[2] * d_error;
}
```

To also control speed another PID like controller is created which also consists of three parts. Current speed, Current Steering angle, and CTE.

Speed PID controller:

```
double PID::Predict_throttle(){
    return _p2[0] * (1 / (fabs(p_error) + 0.0001)) +
           _p2[1] * (1 / (steer_error + 0.0001)) +
           _p2[2] * (1 / (speed_error + 0.0001));
}
```

The hardest part of the project is to find automatically coefficients for both PID controllers (6 coefficients). To find coefficients I used Twiddle method, which basically change parameter by parameter and then car drive again, then by checking cost function it decides whether it was good or not. For both PID controllers Twiddle was done simultaneously using different cost functions.

Steering PID:

```
cte_sum += cte * cte;
```

Throttle PID:

Notice 0.1 in the formula. With this parameter, it is possible to set safety while Twiddling. The lower this value, the more car will try to stick to the perfect position but this also will affect how a car can drive - lowest CTE at all time does not necessarily mean best lap speed.

```
speed_sum += speed_error * (1 / (fabs(cte) + 0.1)) / 100;
```

Results are quite good. Some parameters are changed to fit Project Specifications - car must drive what is considered to be safe for human driver, so there shouldn't be much fluctuations around trajectory which was achieved with solid average speed. When I tested without such constraints I was able to get more than >90 speed and even better average speed in track.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
