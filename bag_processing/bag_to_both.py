#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ping360_sonar.msg import SonarEcho 
import rosbag
from bagSonImProcessing import ImageSonarProcessor1
from sonar_processing import SonarProcessor


# USABLE FILE
# Uses realtime (so you'd have to run the bagfile and republish the uncompressed image)
# to get associations

# assuming you run this without pasuing
class image_converter:
    
    def __init__(self):

        # rosbag info to get initial times:
        self.input_bag_path = '../bagfiles/2024_barbados_02_15_rock_6.bag'
        self.input_bag = rosbag.Bag(self.input_bag_path)
        self.prev_cam_time = None
        self.prev_sonar_time = None # initial times

        # For Images
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, self.camera_callback)
        self.image_set = []

        # For Sonar
        self.sonar_sub = rospy.Subscriber('/ping360_node/sonar/data', SonarEcho, self.sonar_callback)
        self.sonar_queue = []


        # initial times
        self.prev_cam_time = None
        self.prev_sonar_time = None
        self.cur_duration = rospy.Time(0) # starting at 0 seconds
        self.duration_threshold = rospy.Time(1)
        self.duration_min_threshold = rospy.Time(0.9)

        self.associations = {}
        # for processing
        self.sonar_processor = SonarProcessor(400)
        self.sonar_image_saver = ImageSonarProcessor1()
        # self.sonar_image_saver.associations = self.associations

    # getting the initial times to start the coupling process 
    def get_first_times(self):

        #image
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed']):
            self.prev_cam_time = msg.header.stamp
            break

        # sonar
        for topic, msg, t in self.input_bag.read_messages(topics=['/ping360_node/sonar/data']):
            self.prev_sonar_time = msg.header.stamp
            break

    # callback for sonar data
    def sonar_callback(self, sonar_data):
        self.sonar_queue.append(sonar_data)


    # callback for image
    def camera_callback(self,image):

        # print("is this working??")
        cur_cam_time = image.header.stamp
        time_duration = cur_cam_time - self.prev_cam_time 
        self.cur_duration += time_duration
        self.prev_cam_time = image.header.stamp


                # once we reach 1 seconds
                # making it into a set of images instead: so if its between 0.9 seconds and as soon as the image
                # is over 1 second
        if self.duration_min_threshold <= self.cur_duration:
            self.image_set.append(msg)
                    
            if self.cur_duration >= self.duration_threshold:
                # using the timestamp as the key to hashmap
                # key: time, value: [ camera message, [sonar_msg1,..., sonar_msgN] ]
                time_used = str(msg.header.stamp)
                # Processing only Sonar Data and images in sight of camera
                possible_sight = self.sonar_processor.check_in_sight(self.sonar_queue)

                if possible_sight:
                    self.associations[time_used] = [self.image_set, self.sonar_queue]
                    self.sonar_image_saver.associations = self.associations

                    direction_vectors = []
                    set_areas = []
                    angles_here = {}
                    
                    # this is the processed image
                    # for all the sonar queues, we are trying to see if the point is in the processed image
                    image_distorted, new_image = self.sonar_image_saver.image_to_cv2_both(self.image_set[0], False) # not compressed
                    
                    for i in range(0,len(self.sonar_queue)): 
                        cur_msg = self.sonar_queue[i]
                        int_intensities = [ord(b) for b in cur_msg.intensities]
                        # we get the max intensity to get the distance for the nearest read point in this cur msg
                        (max_intensity, max_index) = self.sonar_processor.max_intensity(int_intensities)
                        distance = self.sonar_processor.get_intensity_distance(int_intensities, max_index)
                
                        # TRYING TO MINIMIZE IN ERROR IN REPROJECTION-> 
                        # yaw should be within range of -90 (+/- 10 degrees), given how the camera and sonar are configured
                        rotation_matrix = self.sonar_processor.rotation_matrix_3d(90, 'z')

                        # elevation needs to be changed
                        sonar_3d = self.sonar_processor.sonar_rectangle(distance, cur_msg.angle, 30, rotation_matrix)
                        possible_centers, possible_rectangles, possible_elevations = self.sonar_processor.elevation_ranges(distance, cur_msg.angle, rotation_matrix)


                        # already in cam reference
                        direction_vector = sonar_3d[0]
                        sonar_rectangle = sonar_3d[1:]

                        on_image = self.sonar_image_saver.check_point_image(new_image, direction_vector, cur_msg.angle, distance)
                               
                        if on_image:
                            if cur_msg.angle not in self.angles_in_view:
                                self.angles_in_view.append(cur_msg.angle)
                                
                            # making the rectangle
                            angles_here[cur_msg.angle] = distance
                                    

                            # transformed_sonar_3d = self.sonar_processor.transform_rectangle(sonar_3d)
                            direction_vectors.append(direction_vector) # already in float 32 
                            set_areas.append(sonar_rectangle) # set of rectangles for possible areas

                            # checking the arc of possible elevations for direction vectors
                            # TRYING TO MINIMIZE IN ERROR IN REPROJECTION
                            # img = self.sonar_image_saver.project_possible_direction_vectors_notCV(new_image, possible_centers, possible_elevations)
                            # caption = str(cur_msg.angle)
                            # cv2.imshow(caption, img)
                            # cv2.waitKey(0)
                            # cv2.destroyAllWindows()

                    set_areas = np.array(set_areas, np.float32)
                    direction_vectors = np.array(direction_vectors, np.float32)
                            
                    # convert rectangles and direction vectors into image reference
                    if len(direction_vectors) > 0:
                        
                        # Using CV Project Points
                        # img = self.sonar_image_saver.project_direction_vectors_CV(new_image, direction_vectors, angles_here)
                        # img = self.sonar_image_saver.project_CV(new_image,set_areas)


                        # not projectPoints Function
                        img = self.sonar_image_saver.project_direction_vectors_notCV(new_image, direction_vectors, angles_here)
                        img = self.sonar_image_saver.project_notCV(img, set_areas)
                        # caption = str(angles_here)
                        # cv2.imshow(caption, img)
                        # cv2.waitKey(1)
                        # cv2.destroyAllWindows()

                        # Trying to save the images ->  timestep, realtime of camera image

                        realtime_cam = self.image_set[0]
                        self.sonar_image_saver.save_both_timestep(time_used, True)
                        self.sonar_image_saver.save_image(img, time_used, str(realtime_cam.header.stamp))

                            
                        # img = self.sonar_image_saver.project_notCV(img, set_areas)

                # also restarting the sonar queue and image set
                self.cur_duration = rospy.Time(0)
                self.sonar_queue = []
                self.image_set = []



def main(args):
    ic = image_converter()
    ic.get_first_times()
    rospy.init_node('image_sonar_info', anonymous=True)
    
    try:
        rospy.spin()

        # for time in ic.associations:

        #     val = ic.associations[time]
        #     if len(val[1]) > 1:
        #         print("{} {}".format(time, len(val[1])))
        
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



