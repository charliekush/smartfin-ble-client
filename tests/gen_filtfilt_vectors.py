"""
Generates ground-truth filtfilt vectors using SciPy (method='gust').
Prints labeled space-separated lines consumed by filtfilt_gust_test.cpp.
"""
import numpy as np
from scipy.signal import butter, filtfilt


def emit(name, arr):
    print(name, ' '.join(f'{v:.17g}' for v in np.asarray(arr).ravel()))


def make_case(label, order, cutoff, signal, irlen=None):
    b, a = butter(order, cutoff)
    y = filtfilt(b, a, signal, method='gust', irlen=irlen)
    print(f'case {label}')
    emit('b', b)
    emit('a', a)
    emit('x', signal)
    emit('y', y)
    if irlen is not None:
        print(f'irlen {irlen}')
    print('end')


rng = np.random.default_rng(42)
n = 200

make_case('random_noise',    4, 0.1,  rng.standard_normal(n))
make_case('sine_wave',       4, 0.1,  np.sin(2 * np.pi * 0.02 * np.arange(n)))
make_case('step_function',   4, 0.1,  np.concatenate([np.zeros(n // 2), np.ones(n // 2)]))
make_case('high_order',      6, 0.05, rng.standard_normal(n))
make_case('irlen_truncated', 4, 0.1,  rng.standard_normal(n), irlen=50)
