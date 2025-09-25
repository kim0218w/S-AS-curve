# S-AS-curve
개인 연구
Traceback (most recent call last):
  File "/home/raspberrypi/Desktop/s-curve/test.py", line 133, in <module>
    logs = move_stepper_scurve_with_logging(
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/raspberrypi/Desktop/s-curve/scurve.py", line 219, in move_stepper_scurve_with_logging
    gpio.enable_motor(ena_pin, True)
    ^^^^^^^^^^^^^^^^^
AttributeError: 'GPIOHelper' object has no attribute 'enable_motor'
