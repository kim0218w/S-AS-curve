# S-AS-curve
개인 연구
Exception in thread Thread-2 (_poll_loop):
Traceback (most recent call last):
Traceback (most recent call last):
  File "/usr/lib/python3.11/threading.py", line 1038, in _bootstrap_inner
  File "/home/raspberrypi/Desktop/s-curve/test.py", line 2, in <module>
    self.run()
    from encoder import Encoder, gpio
  File "/usr/lib/python3.11/threading.py", line 975, in run
ImportError: cannot import name 'gpio' from 'encoder' (/home/raspberrypi/Desktop/s-curve/encoder.py)
    self._target(*self._args, **self._kwargs)
Fatal Python error: _enter_buffered_busy: could not acquire lock for <_io.BufferedWriter name='<stderr>'> at interpreter shutdown, possibly due to daemon threads
Python runtime state: finalizing (tstate=0x0000000000a5ef50)

Current thread 0x00007fff1b954700 (most recent call first):
  <no Python frame>

Extension modules: numpy.core._multiarray_umath, numpy.core._multiarray_tests, numpy.linalg._umath_linalg, numpy.fft._pocketfft_internal, numpy.random._common, numpy.random.bit_generator, numpy.random._bounded_integers, numpy.random._mt19937, numpy.random.mtrand, numpy.random._philox, numpy.random._pcg64, numpy.random._sfc64, numpy.random._generator, pandas._libs.tslibs.np_datetime, pandas._libs.tslibs.dtypes, pandas._libs.tslibs.base, pandas._libs.tslibs.nattype, pandas._libs.tslibs.timezones, pandas._libs.tslibs.tzconversion, pandas._libs.tslibs.ccalendar, pandas._libs.tslibs.fields, pandas._libs.tslibs.timedeltas, pandas._libs.tslibs.timestamps, pandas._libs.properties, pandas._libs.tslibs.offsets, pandas._libs.tslibs.parsing, pandas._libs.tslibs.conversion, pandas._libs.tslibs.period, pandas._libs.tslibs.vectorized, pandas._libs.ops_dispatch, pandas._libs.missing, pandas._libs.hashtable, pandas._libs.algos, pandas._libs.interval, pandas._libs.tslib, pandas._libs.lib, pandas._libs.hashing, pandas._libs.ops, numexpr.interpreter, bottleneck.move, bottleneck.nonreduce, bottleneck.nonreduce_axis, bottleneck.reduce, pandas._libs.arrays, pandas._libs.index, pandas._libs.join, pandas._libs.sparse, pandas._libs.reduction, pandas._libs.indexing, pandas._libs.internals, pandas._libs.writers, pandas._libs.window.aggregations, pandas._libs.window.indexers, pandas._libs.reshape, pandas._libs.tslibs.strptime, pandas._libs.groupby, pandas._libs.testing, pandas._libs.parsers, pandas._libs.json, _lgpio, matplotlib._c_internal_utils, PIL._imaging, matplotlib._path, kiwisolver._cext, matplotlib._image (total: 65)
중지됨
