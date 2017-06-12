# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

### Effects of Individual Gains
The P (proportional), I (integral), and D (derivative) gains of the controller all had different effects on the performance of the controller.

Increasing the proportional gain helped the car return to the center of the track quickly, and also helped it turn sharp corners. However, it also made it more likely to get into out-of-control oscillations.

Increasing the integral gain helped the car manage turns, especially smoother turns, gently and without the oscillations about center that increasing the proportional gain produces. However, if the integral gain is too high, the car runs right off the track.

Increasing the derivative gain helps the car smooth out oscillations about the center, but it also inhibits turning about sharp corners.

### Final Values
The final values for the individual gains were:

- Kp = 2.0
- Ki = 6.0
- Kd = 15.0

These allow the car to traverse the track fairly smoothly at approximately 50 mph.

### Method for Optimization

Optimization was initially manual. I started with low values for each gain, modified them individually, and watched the effect they had on the performance of the controller (see above for the effects of gains that are too high).

Once I felt the gains were in the right ballpark, I ran the Twiddle algorithm from lecture to automatically adjust and fine-tune the gains. If the Twiddle algorithm found a minimum after running for a while, I would reset the initial gain values to what it found, and then run Twiddle again. This is how I arrived at the values listed above.