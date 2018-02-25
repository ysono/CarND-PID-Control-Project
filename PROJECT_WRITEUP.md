## Describe the effect of the P, I, D component of the PID algorithm in their implementation.

As described below, the 3 hyperparameters were tuned with semi-autonomous twiddle cycles. But when manually tuning, it was done with the consideration that:

- Increased P should quicken correction
- Increased D should decrease overshoot
- Increased I should effectively remember the past error longer

and vice versa.

Then final chosen hyperparameters were `0.2, 2.8, 0.001`.

## How the final hyperparameters were chosen

### Deterministic twiddle tuning using python script

Before the Unity simulator was used at all, the initial set of hypereparameters were approximated by twiddle optimization on the bicycle model, using this [python script](doc/initial-hyperparams.py) that was modified from the code provided as course material, in these ways:

- Instead of a step function as the setpoint, use sigmoid, followed by multiple step changes. The aim is to optimize the hyperparameters for curves. Steps were made long enough to give time for the process value (ie cross track error) to settle; otherwise, mean squared error (MSE) of a time series of process values that have not settled yet would be meaningless.
- The control variable (ie steering) is added to error, after being arbitrarily normalized by pi. The aim is to prefer smoother steering.

With the script as committed, the initial optimized set of hyperparameters is `[0.8392463661865858, 3.476015002672399, 0.009660602550421216]`. Also see the script's [output image](doc/initial-hyperparams.svg) for the process value response.

### Stochastic twiddle tuning using the simulator

A semi-autonomous twiddle tuning was implemented for the use with the simulator, so that the above initial hyperparameters could be fed in and be further optimized.

[`Twiddle` class](src/twiddle.cpp) is instantiated as a stateful singleton object that creates and keeps a stateful and disposable (ie non-singleton) PID objects for as long as it is evaluating the MSE of the same set of hyperparameters. Consumer code calls it with process values (ie cross track error) and receives control variable (ie steering), just as the consumer would with a [`PID` controller class](src/PID.h); and in addition, `Twiddle` notifies the consumer code when sufficient process values have been received (hard coded to 2400 events which is approx half or third the circuit) to evaluate the MSE for the current set of hyperparameters.

[`TwiddleDriver` class](src/twiddle_driver.cpp) is the consumer of `Twiddle`. `TwiddleDriver` is also instantiated as a singleton. `TwiddleDriver` attempts to reset the car to the center of the road and to a near-stationary speed (and ideally also to the straight ahead direction) at the beginning of runtime and when notified by `Twiddle` of the end of a tuning phase. After `TwiddleDriver` resets the car, it then calls `Twiddle` to begin a new tuning phase.

`TwiddleDriver` can also inform `Twiddle` to abort the tuning of the current set of hyperparameters when the car has gone off bounds and hence continuing the current evaluation would be meaningless.

The resetting uses a pre-determined and hard-coded PID controller (see `TwiddleDeriver.pid_while_resetting`). The hyperparameters for this PID is iteratively set (and recompiled) to whatever seems to work the best so far. Because this PID is in general expected to be non-optimal, the human user is expected to intervene by iteracting with the simulator to either drive the car back to the center or press `esc` and restart the simulator. Even if the reset PID works well, often the car ends up in a circle outside the road (because the control variable becomes constant maximal steering in one direction) or gets stuck on the (ridiculously tall!) kerb, so human intervention is expected. (If `esc`aping and restarting the simulator, do NOT terminate the twiddle driver server, or else you lose `Twiddle.dK`, ie delta hyperparameters. I should have added these as command line args.)

`Twiddle` has hard-coded criteria for a sufficiently good set of hyperparameters, based on the MSE and the sum of the scalar values of the hyperparameters (`if (mse < final_K_mse_thresh || K[0] + K[1] + K[2] < final_K_sum_thresh)`). If this condition is encountered, `Twiddle` continues using the same hyperparameters forever and never again informs the consumer code about the end of a tuning phase.

In summary, in a happy path, with the simulator started and the semi-autonomus twiddle tuner started with `./pid twiddle 0.8392463661865858, 3.476015002672399, 0.009660602550421216`, the simulator continues evaluating new hyperparameters until a sufficiently good one is discovered, at which point the simulator continues running with these hyperparameters. In reality, many human interventions are required. For demonstration of how it works at the beginning, see [this sample clip](https://youtu.be/Kbyl5A2b-Ro).

### More details about the twiddle driver

In the `main` method, the steering is bound to `[-1, 1]`. This becomes part of the plant process. The plant's bicycle model is non-linear anyway even without this binding.

`Twiddle` declines to evaluate any hyperparameter that is <= 0. We know beforehand that we want all 3 types of control. This is documented in `Twiddle`'s constructor.

The main objective of `TwiddleDriver` is to reset the car's position in order to approach a similar starting point for each twiddle evaluation. However, this is obviously flawed, since `TwiddleDriver` is not able to bring the car back to the exact same spot at each beginning. Hence this twiddle tuning is stochastic. One alternative is to require human intervention to press `esc` and restart the simulator at the end of every twiddle evaluation including aborts. Some advantages with this alternative are 1) more consistent evaluation of MSE, 2) the simulator's UI makes it quicker to restart than to correct the car's position. Some advantages with `TwiddleDriver` are 1) more autonomy with less human intervention, 2) `Twiddle` is more likely to evaluate later parts of the circuit with more tricky turns, 3) this is more realistic simulation of real-life PID tuning on non-closed roads (ok maybe this is a stretch; I don't know if you'd want to sit in a twiddle-tuned car lol).

During resetting, the angle telemtry often reads a constant 0.4363. Therefore `TwiddleDriver` is NOT resetting the car to point straight ahead. This further adds to the stochastic nature of twiddle tuning.

Unlike in the python script above, `Twiddle` does NOT add steering to the MSE. This was an oversight. If added, because steering is normalized to `[-1, 1]` and cte is bound by approx `[-2, 2]`, steering should be multiplied by 2, and summed squared error should be `sse += cte ^ 2 + steering_factor * (steering * 2) ^ 2`, where steering_factor indicates relative importance of steering wrt cte.

In the end, the MSE figure was less useful than qualitative analysis of how smooth or violent the path was. Therefore, after twiddle reached a reasonably good set of hyperparameters, I killed twiddle, ran the whole circuit with these hyperparameters (`pid pdi <Kp> <Kd> <Ki>`), and tuned manually by making small changes. If reaction to turns was too slow, Kp was increased. If the movement was too jerky, Kp was increased. Ki was kept at 0.001, because it was satisfactory for whatever visually undetectable steering drift there was, and also values smaller than 0.001 tended to cause offsets that could cascade to wide swerving.

## Usage

Run `./pid help` to print usage.

Running `./pid` should complete the circuit counter-clockwise. It is admittedly somewhat wobbly, and that's because it's using PID for steering as tuned with the use of a constant throttle of `0.3`, whereas `./pid` runs with throttle that controls the speed to 40mph. To specify speed, use `./pid <speed>`. At speeds of 30mph or lower, it looks smoother: `./pid 30`. To see the condition under which the PID parameters were chosen, use speed of zero (`./pid 0`); this uses a constant throttle of `0.3` (from `TwiddleDriver.throttle_during_tuning`), and in the context of the specific track, the steady state speed is approx 34mph.

To try the autonomous twiddle tuner, and use the final chosen hyperparameters as the starting values, run `./pid twiddle 0.2 2.8 0.001`.
