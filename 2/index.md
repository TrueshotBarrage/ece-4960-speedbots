## Lab 2

### Summary

In this lab, I learned to connect to the Artemis board without a wired
connection. In other words, I had to set up and use Bluetooth for the board so
that it could communicate with my computer wirelessly! The lab delves into the
basics of Python on Jupyter Notebook, setting up a Python virtualenv, and
exploring communication protocols using BLE (Bluetooth Low Energy).

### Procedure

As stated in the [Summary](#Summary), this lab is quite extensive in both
preparation and the actual lab content, but thankfully, I had previous exposure
to most of the setup. First, Python was already installed on my MacBook, so no
worries there, and I had already installed virtualenv before. So a simple
`python3 -m venv ece4960_ble` was enough to get started.

After installing the necessary libraries on the environment and downloading the
lab source code, I was able to load the Jupyter notebook. Once loaded, I had to
change the values of the `connection.yaml` file to match the MAC address of my
Artemis board.

Finally, I could run through the different cells on Jupyter, which allowed me to
see the different types of messages that I could communicate over the ArduinoBLE
library. These types include the conventional int, float, string, etc. but are
customized for the Arduino BLE library. To learn to use these more proficiently,
here are the tasks I had to complete:

- Sending a string and receiving a modified version of the original string
- Sending multiple float values
- Setting up a notification handler with a function to be called on a certain
  event trigger

These tasks were simple, yet they required an understanding of how to use the
BLE library in order to make them work. For the first task of transmitting a
string, I modified the ECHO command to receive the string into a queue and
return the output back from the queue after modification. Here is how it works:

(some code)

[task1.png](images/task1.png)

For the second task of transmitting multiple float values, the procedure was the
same as handling a single float value (using `receive_float()`) but invoked
multiple times. Since we wanted three floats, it was called three times:

(some code)

[task2.png](images/task2.png)

The third task took a bit more consideration in order to complete. The main
issue was figuring out how to set up the notification handler, since it required
using other library functions that the lab introduced. I also had to figure out
what sort of callback function was necessary, since the callback requires two
arguments, but only ends up using one of them:

(code to demonstrate callback function)

[task3.png](images/task3.png)

In addition, I had to consider the difference between the following two
approaches:

1. Receiving a float value from Arduino to the computer (Python) using
   `receive_float()`
2. Receiving a float value in the same way but using `receive_string()`, such
   that the Arduino C-string (BLECStringCharactersitic) contains a float value
   to be casted into a Python float

The primary difference is how the data is handled. To the end user interface,
there should be no functional difference, since the input/output are floats in
both scenarios. However, what does change is that the latter method requires a
cast from string to float on Python. This also means that the data type carried
on from Arduino could possibly have a different memory allocation from a float
on the Arduino side. It also means that there is room for loss of precision of
the float value that could occur when casting, in addition to the possibility
for a casting error, since not all strings will be valid floats.

### Conclusion

I learned how to use a non-wired connection to communicate between the Artemis
board and a computer, which I think is a seriously useful skill that I cannot
believe I didn't learn in other embedded systems classes! Bluetooth is a great
protocol with a nice supporting framework of Arduino libraries that allows our
Artemis board to go from a nice, powerful device with the help of a computer to
a slightly nicer (and quite a bit more useful) gadget that can do a lot more
thanks to being wire free!

## References
