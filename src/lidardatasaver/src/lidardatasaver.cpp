import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os 
import time
import shutil

# Node for saving images from a camera to a specified folder
class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # create directory structure for data to be saved in
        self.createDirStructure()

        # init counter for name of images
        self.counter = 0

        # listen to input from /image_raw and process it in callback function listener_callback
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()

    def createDirStructure(self):
        # create new directory for dara to be saved in
        baseTargetDir = os.path.dirname(os.path.realpath(__file__ + "/../../../../../../../")) + '/noetic-slam/sampledata/raw/'

        # new directory name for files (= last directory name + 1)
        self.newDir = 0
        for dir in os.listdir(baseTargetDir):
            if dir != 'meta.yaml':
                if int(dir) > self.newDir:
                    self.newDir = int(dir)
        self.newDir += 1
        self.newDir = baseTargetDir + str(self.newDir).zfill(8) + '/'

        # create image target directory
        cam0Dir = self.newDir + 'cam_00000000/'
        self.imgTargetDir =  cam0Dir + '00000000/'

        if not os.path.exists(self.imgTargetDir):
            os.makedirs(self.imgTargetDir)

        # create scan target directory
        lidar0Dir = self.newDir + 'lidar_00000000/'
        self.scanTargetDir = lidar0Dir + '00000000/'

        if not os.path.exists(self.scanTargetDir):
            os.makedirs(self.scanTargetDir)

        # copy meta.yaml to raw dir
        samplestructureDir = os.path.dirname(os.path.realpath(__file__ + "/../../../../../../../")) + '/noetic-slam/sampledata/samplestructure/'
        shutil.copy(samplestructureDir + 'raw/meta.yaml', baseTargetDir)

        # TODO: copy/create all meta.yaml

    def listener_callback(self, msg):
        # write current camera image to specified folder & name
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_name = self.imgTargetDir + str(self.counter).zfill(8) + '.png'

        cv2.imwrite(image_name, cv_image) # TODO: In Zeitraum2: Datei-Namen anpassen
        self.get_logger().info('Image saved to folder ' + self.imgTargetDir)
        
        # TODO: In Zeitraum 2: Hier können weitere Dateien erzuegt werden

        # increase counter for next image name
        self.counter += 1

        # delay 1 sec
        time.sleep(1) # TODO: In Zeitraum 2: an Frequenz von Sensor anpassen

def main(args=None):
    # start node
    rclpy.init(args=args)
    image_saver = ImageSaverNode()
    rclpy.spin(image_saver)

    # stop node
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
