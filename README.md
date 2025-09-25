# S-AS-curve
개인 연구
    self.run()
  File "/usr/lib/python3.11/threading.py", line 975, in run
    self._target(*self._args, **self._kwargs)
  File "/home/raspberrypi/Desktop/s-curve/encoder.py", line 79, in _poll_loop
    prev_a, prev_b = self._read_ab()
                     ^^^^^^^^^^^^^^^
  File "/home/raspberrypi/Desktop/s-curve/encoder.py", line 74, in _read_ab
    a = lgpio.gpio_read(self.gpio.h, ENCODER_A_PIN)
                        ^^^^^^^^^
AttributeError: 'Encoder' object has no attribute 'gpio'
^CTraceback (most recent call last):
  File "/home/raspberrypi/Desktop/s-curve/test.py", line 9, in <module>
    mode = input("실행 모드 선택 (1: 모터 실행, 2: 파라미터 계산): ").strip()
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
KeyboardInterrupt
