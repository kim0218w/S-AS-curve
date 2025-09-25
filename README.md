# S-AS-curve
Encoder : I2C - AS5600
개인 연구

CTraceback (most recent call last):
  File "/home/raspberrypi/Desktop/s-curve/main.py", line 75, in <module>
    main()
  File "/home/raspberrypi/Desktop/s-curve/main.py", line 20, in main
    data_log = run_motor_scurve(
               ^^^^^^^^^^^^^^^^^
  File "/home/raspberrypi/Desktop/s-curve/scurve.py", line 100, in run_motor_scurve
    gpio.pulse_step(motor_id, high_time=0.00002, low_time=pulse_interval - 0.00002)
  File "/home/raspberrypi/Desktop/s-curve/encoder.py", line 58, in pulse_step
    time.sleep(low_time)
KeyboardInterrupt


