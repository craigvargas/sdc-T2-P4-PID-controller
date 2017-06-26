Craig Vargas
SDC Nanodegree
PID Controller
Term 2: Project 2: Reflection




Effects of each component in the PID controller:

Control = - (P * Error) – (I * SumOfErrors) – (D * delta(Error))
	Where: delta(x) := dx/dt

P: 

The ‘P’ component of the PID controller is the coefficient that scales the raw error of the object being controlled.  In our case this error was the distance the car was away from the desired path of the car.  As seen in the equation above, as long P > 0, as the car gets further and further from the desired path the more the controller steers in the opposite direction.

For example, if the car is driving straight and it approaches a turn, as the car’s trajectory remains straight it will veer of off the desired path more and more.  The P coefficient will be the first contributing factor for the car to initially steer in the direction of the turn.  The larger the coefficient the more drastic and abrupt the steering is.

**Video: P: 0.1	I: 0.0		D: 0.0**

<img src='gifs/Pval.gif' title='Example: Only the P value set' width='' alt='Video Walkthrough' />

*You can see above that as the car approaches the turn its error increases and the steering control adjusts in the opposite direction.*


I:

The ‘I’ component of the PID controller is the coefficient that scales the sum of the errors of the object being controlled.  This helps to add further steering correction for an object, or in our case vehicle, that is spending an extended period of time off of the desired path.  If the car is positioned to the right of the desired path and is not correcting itself enough, the sum of those errors will continue to build up causing the “( I * SumOfErrors )” term to get larger and add further steering correction to the vehicle to get it back to the desired path.

**Video: P: 0.0	I: 0.00005		D: 0.0**

<img src='gifs/Ival.gif' title='Example: Only the I value set' width='' alt='Video Walkthrough' />

We can see here that as the car spends extended time away from the desired path the I coefficient helps to correct its path.  However, the correction is not reactive enough because it takes time for the errors to build up.


D:

The ‘D’ componenet of the PID controller is the coefficient that scales the differential of the error term.  The concept here is that if the vehicle is way off to the right of the desired path, and its steering instructions make a hard left turn to get back to the desired path, there is likely to be a decent amount of overshooting.  Even though a hard left is required to get the car back on path quickly, if it is not counteracted in the proper way it will just end up on the far left of the desired path.  

The D coefficient helps to stabilize the steering of the car by steering in the opposite direction after the car has corrected itself.  In the example given of steering hard to the left to get back to the center of the road, if the error was 5 and the hard left brought the error down to 2, even though the car needs to continue to steer left to get the error back to zero, the ‘D’ component will cause the steering to adjust to the right so that the car is not steering as hard to the left since it is no longer needs the same type of adjustment to get back on course.

To demonstrate the effect of the D coefficient I’ll show a sequence of videos

**Video: P: 5.0	I: 0.0		D: 0.0**

<img src='gifs/PvalHigh.gif' title='Example: Only the P value set and to an exaggerated value' width='' alt='Video Walkthrough' />

As we can see here the car is very reactive and sends itself into uncontrolled oscillations


**Video: P: 5.0	I: 0.0		D: 50.0**

<img src='gifs/DvalHigh.gif' title='Example: Both the P and D values set and to an exaggerated value' width='' alt='Video Walkthrough' />

Here, even though both parameters are too high and the car is not steering smoothly, we can see that the D parameter helps to stabilize the car after the P parameter makes abrupt turns


Final Parameters:

Methodology:

I concluded on the final parameters by utilizing the twiddle methodology to adjust each parameter slightly and accept the adjustment if the result was better than the best set of parameters thus far.  

Since I could not control the experimental trials of the car driving around the track within the PID class in the C++ code, I had to simulate the twiddle loop structure by using variables to keep track of which coefficient was being “twiddled” with, and where within the conditional block structure of the twiddle algorithm we were last in before the current trial. 

I set up code in the main function that tested if the car was a certain distance off of the desired path, and if it was then ended the trial, evaluated the car’s performance, updated the twiddle simulation tracking variables, and then reset the simulator to continue the twiddle algorithm.  I chose 2.0 as the error threshold to reevaluate and reset the simulator.  This value was chosen by “eyeballing” what error translated to the car being at an unacceptable position within the track, i.e. at a position that would be dangerous to passengers.

I had to choose the performance parameter so that it rewarded a low error as well as a controller that were able to keep the car on the track for a long time.  I did not know how to extract how long the car had driven on the track so instead I kept track of how many steering updates were requested during each trial.  This served as a proxy for the distance driven since trials where the car stayed on track for a long time would request more steering instructions than trials that could not make it past the first turn.  I called this variable numUpdates.  The performance function was chosen as follows:

Utility := numUpdates – sumOfSquaredErrors

This utility function allowed the twiddle algorithm to reward parameter changes that resulted in longer times of staying on the track while also giving preference to a trial that swerved around less on the road but was still able to drive a significant distance.

After setting up this structure I started the twiddle algorithm with all coefficients initialized to zero and all coefficient update parameters set to one.  After many trials I was a bit confused because I was getting relative success with many different combos of parameters.  I did, however, noticed a couple of patterns in successful trials.  In the successful trials the ‘I’ parameter was always set to zero and the ‘D’ parameter was always scaled an order of magnitude higher than the ‘P’ parameter.  To save time and refine the process I began changing my coefficient update parameters so that dP was an order of magnitude smaller than dD, and dI was much, much, smaller (dI < 0.0001).

After many trials with the update parameters “jury rigged”, I was able to settle on the following coefficients:

**P = 0.2		I = 0.00005		D = 5.0**



