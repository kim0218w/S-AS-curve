# S-AS-curve
개인 연구
Exception in thread Thread-2 (_poll_loop):
Traceback (most recent call last):
  File "/usr/lib/python3.11/threading.py", line 1038, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.11/threading.py", line 975, in run
실행 모드 선택 (1: 모터 실행, 2: 파라미터 계산):     self._target(*self._args, **self._kwargs)
  File "/home/raspberrypi/Desktop/s-curve/encoder.py", line 79, in _poll_loop
    prev_a, prev_b = self._read_ab()
                     ^^^^^^^^^^^^^^^
  File "/home/raspberrypi/Desktop/s-curve/encoder.py", line 74, in _read_ab
    a = lgpio.gpio_read(lgpio.h,0, ENCODER_A_PIN)
                        ^^^^^^^
AttributeError: module 'lgpio' has no attribute 'h'
