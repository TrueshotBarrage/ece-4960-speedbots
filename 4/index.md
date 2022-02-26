# Lab 4

## Summary

Lab 4 is finally when the car comes in. I was able to get my hands on the car and spec out basic characteristics of the car that we will be using as our robot for the rest of the course. The overall goal of the lab was to gain useful insights about our car that will be good to keep in mind for future labs.

## Procedure

Last semester, I took a course on embedded operating systems, where the final project was an open-ended project of my choice using a Raspberry Pi 4. My team actually built a self-driving robot, and a few valuable lessons that I learned in the process were the motivating factors for which characteristics I wanted to test about the car.

The main characteristics I was interested in right off the bat were car dimensions, car weight, and battery. These are all important factors in designing a robot because they determine the active runtime of the car. If the car is too bulky or heavy for the battery, it may not even be able to power the motors at all to move. On the other hand, if the battery is unnecessarily large, then it might be a non-negligible load on the car. Anyways, here are the details:

- Dimensions: 18cm x 14.5cm x 8cm
- Weight: 500g
- Battery: 8-15 minutes of runtime, 15-20 minutes to full charge

Another important feature of the car that I was surprised by (and consequently one of the characteristics I measured) was its durability. It could pretty much handle any sort of surfaces that it came across, including marble floors, the regular type of floor in Duffield and campus buildings, and even carpets (with a slight impediment on its speed and reduced control).

Unfortunately I ran into some issues with my lab 3 sensors because of a complete design overhaul (soldering did not work out for me too well the first time around) so I was not able to test whether the sensor readings were accurate enough on the car to detect different types of surfaces and floors, but I suspect that it should not matter too much for what I want to achieve with the robot.

However, this did not stop me from measuring the car's speed the old-fashioned way, by repeated trials using a set distance and the time it took to complete laps. I found that the car was extremely fast on regular Duffield floors, with around an average of 2.3 meters per second. That is super quick!

### Tricks

Personally, I don't think the tricks are going to be super important for later labs, but they seemed fun to try, so here is my favorite one of the ones I recorded. It is the car doing a flip by running against an obstacle and having its forward input become a backward input due to changing direction (like torque). Please enjoy:

<p align="center">
  <img src="images/car_flip.gif" />
</p>

## Conclusion

Overall, this lab was much easier than the last one (and much less stressful!). I got to familiarize myself with the car that I will be using for future labs. It is a nifty little bot that has great potential!

## References

- [Lab handout](https://cei-lab.github.io/ECE4960-2022/Lab4.html)
- [RC car tricks](https://racenrcs.com/how-to-do-tricks-with-rc-cars-an-interactive-guide/)

[Back to main](../index.md)
