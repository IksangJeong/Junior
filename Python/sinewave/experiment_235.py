"""
신호처리 실험 #235
주파수 분석 및 필터링
"""
import numpy as np

def generate_signal_235(duration=1.0, sample_rate=1000):
    """테스트 신호 생성 #235"""
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    freq1 = 2400
    freq2 = 1745
    signal = np.sin(2 * np.pi * freq1 * t) + 0.5 * np.sin(2 * np.pi * freq2 * t)
    noise = np.random.normal(0, 0.1, len(t))
    return t, signal + noise

def analyze_235(signal, sample_rate=1000):
    """주파수 분석"""
    fft_result = np.fft.fft(signal)
    freqs = np.fft.fftfreq(len(signal), 1/sample_rate)
    magnitude = np.abs(fft_result[:len(signal)//2])
    freqs = freqs[:len(signal)//2]
    peak_idx = np.argmax(magnitude)
    return freqs[peak_idx], magnitude[peak_idx]

if __name__ == "__main__":
    t, sig = generate_signal_235()
    peak_freq, peak_mag = analyze_235(sig)
    print(f"실험 #235")
    print(f"신호 길이: {len(sig)}")
    print(f"주요 주파수: {peak_freq:.1f} Hz")
    print(f"크기: {peak_mag:.2f}")
    print(f"RMS: {np.sqrt(np.mean(sig**2)):.4f}")
