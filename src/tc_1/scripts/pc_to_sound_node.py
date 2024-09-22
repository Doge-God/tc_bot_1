import rospy
import pyaudio
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudToAudio():
    def __init__(self) -> None:
        self.stream = None

    def compress_data(self, raw_data, target_size):
        current_size = len(raw_data)
    
        # Calculate step size to compress data
        step = current_size // target_size

        if step > 1:
            # Average every 'step' bytes to create the compressed data
            compressed_data = np.mean(np.reshape(raw_data[:step * target_size], (-1, step)), axis=1).astype(np.uint8)
        else:
            # If no compression needed, just truncate or pad to target_size
            compressed_data = raw_data[:target_size]
        
        return compressed_data
    
    def run(self):
        rospy.init_node('pc_to_audio')
        p = pyaudio.PyAudio()
        self.stream = p.open(format=pyaudio.paInt16, channels=1, rate=44100, output=True)
        self.stream.start_stream()

        def callback(point_cloud:PointCloud2):
            
            raw_data = np.frombuffer(point_cloud.data, dtype=np.uint8)  # Convert the data to a numpy array
    
            # Calculate the required audio buffer size
            sample_rate = 44100
            target_duration = 1.0 / 30  # 30 Hz scan rate = 33.33 ms per scan
            target_samples = int(sample_rate * target_duration)  # Number of audio samples we need for each scan
            
            # Each audio sample is 2 bytes (16-bit audio), so we need half the samples in bytes
            target_size = target_samples * 2
            
            # Compress the point cloud data to fit in the audio buffer
            compressed_data = self.compress_data(raw_data, target_size)
            
            # Play the compressed data as audio
            self.stream.write(compressed_data.tobytes())

        rospy.Subscriber('/camera/depth/points', PointCloud2, callback)

        rospy.spin()
        self.stream.stop_stream()

# Execute if main
if __name__ == '__main__':
    try:
        pc_to_audio = PointCloudToAudio()
        pc_to_audio.run()
    except rospy.ROSInterruptException:
        pass