from scipy import signal

def setup_filter(highcut, fs, order=3):  # カットオフ周波数は元々12Hz，dock_processingで変更可
    """Get coeffs for filter"""
    nyq = 0.5 * fs
    high = highcut / nyq
    return signal.butter(order, high, btype="lowpass", output="ba")


def apply_filter(coefs, data):
    return signal.filtfilt(*coefs, data)


def setup_realtime_filter(high_cut, fs, order=3):
    b, a = setup_filter(highcut=high_cut, fs=fs, order=order)
    z = signal.lfilter_zi(b, a)
    return b, a, z


def realtime_filter(data, b, z, a=0):
    realtime_data, z = signal.lfilter(b, a, data, zi=z)
    return realtime_data, z