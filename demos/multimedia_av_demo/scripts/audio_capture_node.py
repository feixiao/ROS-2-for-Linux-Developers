#!/usr/bin/env python3
"""
Audio Capture and Streaming Node for ROS 2.
Captures 16kHz 16-bit linear PCM audio for Speech Recognition (ASR / Whisper).
Falls back cleanly to synthetic audio frames if physical microphone is absent.
"""

import math
import struct
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    import pyaudio
    HAS_PYAUDIO = True
except ImportError:
    HAS_PYAUDIO = False


class AudioCaptureNode(Node):

    def __init__(self):
        super().__init__('audio_capture_node')

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 1024)

        self.rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk = self.get_parameter('chunk_size').value

        self.pub_ = self.create_publisher(UInt8MultiArray, 'audio/pcm_raw', 10)

        self.py_audio = None
        self.stream = None
        self.phase = 0.0

        if HAS_PYAUDIO:
            try:
                self.py_audio = pyaudio.PyAudio()
                self.stream = self.py_audio.open(
                    format=pyaudio.paInt16,
                    channels=self.channels,
                    rate=self.rate,
                    input=True,
                    frames_per_buffer=self.chunk
                )
                self.get_logger().info(f'Hardware microphone opened at {self.rate}Hz')
            except Exception as e:
                self.get_logger().warn(f'Could not open physical microphone ({e}), falling back to synthetic audio generator.')
                self.stream = None
        else:
            self.get_logger().info('PyAudio not installed; operating in synthetic audio stream generator mode.')

        # 采样周期: 1024 samples / 16000 samples/sec ≈ 0.064s (15.6Hz)
        period = self.chunk / float(self.rate)
        self.timer = self.create_timer(period, self.capture_or_generate_audio)

    def capture_or_generate_audio(self):
        if self.stream is not None and self.stream.is_active():
            try:
                raw_bytes = self.stream.read(self.chunk, exception_on_overflow=False)
            except Exception:
                raw_bytes = self.generate_synthetic_pcm()
        else:
            raw_bytes = self.generate_synthetic_pcm()

        msg = UInt8MultiArray()
        msg.data = list(raw_bytes)
        self.pub_.publish(msg)

    def generate_synthetic_pcm(self):
        # 生成 440Hz 纯音 (A4 音高) 16-bit 线性 PCM 信号
        freq = 440.0
        samples = []
        for _ in range(self.chunk):
            val = int(16000.0 * math.sin(self.phase))
            samples.append(val)
            self.phase += 2.0 * math.pi * freq / float(self.rate)
            if self.phase > 2.0 * math.pi:
                self.phase -= 2.0 * math.pi
        return struct.pack(f'<{self.chunk}h', *samples)

    def destroy_node(self):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        if self.py_audio is not None:
            self.py_audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
