# Reflections

I decided to use the twiddle approach to tune the hyper parameters. 
So I had to modify the code architecture to facilitate the twiddle algorithm as well as reset the simulator in between two
iterations of twiddle.

My first attempt at driving with the embedded twiddle algorithm resulted in the car driving off the road during
the twidle stages. Since it got stuck at the edge of the road, the steering commands had no effect on the CTE and so
the twiddle algorithm was not working.

As a second step, I decided to reduce the throttle to a very low value during the twiddle stages and enabled it to 
 a higher value later. In this case, the car stayed within the track at a very low speed but the twiddle algorithm never seemed
 to converge.
