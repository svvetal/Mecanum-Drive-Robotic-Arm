import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
from flask import Flask, jsonify, render_template

app = Flask(__name__)

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.subscription = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        self.cv_bridge = CvBridge()
        self.marker_size = 0.05  # Size of the ArUco marker in meters
        self.detected_ids = set()  # Set to store detected ArUco marker IDs

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting ROS to CV Image: %s" % str(e))
            return 
        
        # Detect ArUco markers
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # You can change the dictionary and marker size as needed
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(cv_image, dictionary, parameters=parameters)

        if ids is not None:
            new_ids = [id[0] for id in ids if id[0] not in self.detected_ids]
            for new_id in new_ids:
                print("Detected ArUco ID:", new_id)
                self.detected_ids.add(new_id)

            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        #cv2.imshow('output video', cv_image)
        cv2.waitKey(1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/aruco_ids', methods=['GET'])
def get_aruco_ids():
    global aruco_detector
    ids = [int(id) for id in aruco_detector.detected_ids]  # Convert int32 IDs to Python integers
    return jsonify(ids)


def main(args=None):
    rclpy.init(args=args)

    global aruco_detector
    aruco_detector = ArUcoDetector()

    # Start Flask server in a separate thread
    import threading
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0'))
    flask_thread.start()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
