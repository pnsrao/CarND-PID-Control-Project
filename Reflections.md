# Reflections

I decided to use the twiddle approach to tune the hyper parameters. 
So I had to modify the code architecture to facilitate the twiddle algorithm as well as reset the simulator in between two
iterations of twiddle.

My first attempt at driving with the embedded twiddle algorithm resulted in the car driving off the road during
the twidle stages. Since it got stuck at the edge of the road, the steering commands had no effect on the CTE and so
the twiddle algorithm was not working.
* In particular, the integral error was accumulating very rapidly when driving at higher speeds. One option ws to lower the coefficient but I didn't follow it.

As a second step, I decided to reduce the throttle to a very low value during the twiddle stages and enabled it to 
 a higher value later. In this case, the car stayed within the track at a very low speed but the twiddle algorithm never seemed
 to converge.

Ignore teh above. Stick to high speed and maipulate the constants.

Tries dp from 1 to 0.1 to 0.01. Improved but didn't get there.

If it went off track, get it back by assuming high error

After fixing a bug, did  alot of training with dp = 0.1. That seemed to give good results for P and D but
I term remained at 0.

Then I manually did a trial run with the P and D coefficients and tuned the I coefficient to a comparable value for I error.

Finally it turned out that the car was going off track at steep turns since it was not reacting fast enough. Fundamentally 
it is an issue if the track where coefficients are trained on is different from the track where the test drive is being conducted.
However, I was not aware of a way to begin training at the steep part of the track. The smulator may not be able to reset at specific points in the track.
One way to deal with the issue is also to have a PID cntrol for speed that enables the car to slow down when error increases. This is similar to human driving.

