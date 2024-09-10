import rosbag
import sys
import rospy
from bagSonImProcessing import *
from sonar_processing import *
from camera_pose import *




# USABLE FILE
# Primarily Uses Rosbag to get association data structure

# goal: to couple a group of sonar messages with one image, 1 sec per image difference
class ImageSonarCoupler:
    
    def __init__(self):
        #rosbag info
        self.input_bag_path = '../bagfiles/2024_barbados_02_15_rock_6.bag'
        self.input_bag = rosbag.Bag(self.input_bag_path)


        # initial times
        self.prev_cam_time = None
        self.prev_sonar_time = None

        # data structs to couple them
        self.sonar_queue = []
        self.associations = {}
        self.image_set = []

        self.cur_duration = rospy.Time(0) # starting at 0 seconds
        self.duration_threshold = rospy.Time(1)
        self.duration_min_threshold = rospy.Time(0.9)

        # for processing
        self.sonar_processor = SonarProcessor(400)
        self.sonar_image_saver = ImageSonarProcessor1()

        # just to check angle range
        self.angles_in_view = []
        self.prev_image = None

        # for camera pose estimation
        self.camera_pose = CameraProcessor()


    def get_first_times(self):

        #image
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed']):
            self.prev_cam_time = msg.header.stamp
            self.prev_image = msg
            break

        # sonar
        for topic, msg, t in self.input_bag.read_messages(topics=['/ping360_node/sonar/data']):
            self.prev_sonar_time = msg.header.stamp
            break

    def time_coupler(self):
        cur_cam_time = None
        cur_sonar_time = None
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed', '/ping360_node/sonar/data']):
            
            if topic == '/usb_cam/image_raw/compressed':
                cur_cam_time = msg.header.stamp
                time_duration = cur_cam_time - self.prev_cam_time 
                # print("Time difference camera {}".format(time_duration.to_sec())) # each camera reading is about 0.06 secs apart
                self.prev_cam_time = msg.header.stamp
    
            elif topic == '/ping360_node/sonar/data':
                cur_sonar_time = msg.header.stamp
                time_duration = cur_sonar_time - self.prev_sonar_time 
                # print("Time difference sonar {}".format(time_duration.to_sec())) # each camera reading is about 0.06 secs apart
                self.prev_sonar_time = msg.header.stamp  # each sonar reading is about 0.08-0.1 seconds apart


    # we are trying to add an image to the associations dictionary for every 2 seconds 
    # in between those 1 seconds, we couple all the incoming sonar messages to the list
    # once those 1 seconds are approximately reached, we add the sonar queue to the image and clear the queue for the
    # next
    # def time_coupler2(self):
    #     cur_cam_time = None
    #     cur_sonar_time = None

    #     self.get_first_times()
    #     for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed', '/ping360_node/sonar/data']):
            
    #         if topic == '/usb_cam/image_raw/compressed':
    #             cur_cam_time = msg.header.stamp
    #             time_duration = cur_cam_time - self.prev_cam_time 

    #             self.cur_duration += time_duration
    #             # print("cur duration {} and total duration {} ".format(time_duration.to_sec(), self.cur_duration.to_sec()))
    #             self.prev_cam_time = msg.header.stamp

    #             # once we reach 1 seconds
    #             # making it into a set of images instead: so if its between 0.9 seconds and as soon as the image
    #             # is over 1 second
    #             if self.duration_min_threshold <= self.cur_duration:
    #                 self.image_set.append(msg)
                    
    #                 if self.cur_duration >= self.duration_threshold:
    #                     # using the timestamp as the key to hashmap
    #                     # key: time, value: [ camera message, [sonar_msg1,..., sonar_msgN] ]

                        
    #                     time_used = str(msg.header.stamp)
    #                     # Processing only Sonar Data and images in sight of camera
    #                     in_sight = self.sonar_processor.check_in_sight(self.sonar_queue)

    #                     if in_sight:
    #                         self.associations[time_used] = [self.image_set, self.sonar_queue]
    #                         self.sonar_image_saver.associations = self.associations
    #                         # self.sonar_image_saver.save_both_timestep(time_used, True) #compressed data

    #                         # # get max intensity-> just use most recent msg
    #                         # most_recent_sonar_msg = self.sonar_queue[-1]
    #                         # int_intensities = [ord(b) for b in most_recent_sonar_msg.intensities]

    #                         # # we call max intensities first-> max intensity for a distance
    #                         # # then we call sonar rectangle sonar_rectangle(self, distance, angle) to get list of 3D points
    #                         # # using those 3D points, we call projected points-> then draw them at a corresponding image

                            
    #                         # (max_intensity, max_index) = self.sonar_processor.max_intensity(int_intensities)
    #                         # distance = self.sonar_processor.get_intensity_distance(int_intensities, max_index)
    #                         # # print(int_intensities, "max index ", max_index, "max intensity ", max_intensity, "\ndistance ", distance)
    #                         # sonar_3d = self.sonar_processor.sonar_rectangle(distance, most_recent_sonar_msg.angle)
    #                         # print(sonar_3d)

    #                         # we further refine the outputs, by checking if any sonar reading in the queue is in the image
    #                         # if it is, then we can print the heading and output
    #                         # if not, we dont print out the heading and output 

    #                         # all sonar readings for one image
    #                         center_points = []
    #                         set_areas = []

    #                         # for each sonar reading from the set for the current image
    #                         for i in range(0,len(self.sonar_queue)): # or doing it up to paramter
    #                             cur_msg = self.sonar_queue[i]
    #                             int_intensities = [ord(b) for b in cur_msg.intensities]

    #                             # we get the max intensity to get the distance for the nearest read point
    #                             (max_intensity, max_index) = self.sonar_processor.max_intensity(int_intensities)
    #                             distance = self.sonar_processor.get_intensity_distance(int_intensities, max_index)

    #                             # we get the central point of the reading of where the center of the beam would be
    #                             # as well as the rectangles of possible areas
    #                             sonar_3d = self.sonar_processor.sonar_rectangle(distance, cur_msg.angle)
    #                             center_point = sonar_3d[0]

    #                             # we check if the center point is in the image

    #                             # if not: continue

    #                             # if so: we want to print the projected image (even if one sonar reading out of the
    #                             # queue is the only reading)
    #                             # and print the outputs
    #                             cam_center_point = self.sonar_processor.sonar_to_cam_reference(center_point[0], center_point[1],center_point[2])
    #                             transformed_sonar_3d = self.sonar_processor.transform_rectangle(sonar_3d)

    #                             center_points.append(cam_center_point) # center point i would correspond to rectangle i 
    #                             set_areas.append(transformed_sonar_3d) # set of rectangles for possible areas

    #                         # drawing the rectangles
    #                         set_areas = np.array(set_areas, np.float32)

    #                         # areas_on_image(self, timestep, raw_image, set_rectangles, compressed)
    #                         # self.sonar_image_saver.areas_on_image(time_used, self.image_set[0],center_points,set_areas, True)
    #                         new_image = self.sonar_image_saver.image_to_cv2(self.image_set[0], True)



    #                     # also restarting the sonar queue and image set
    #                     self.cur_duration = rospy.Time(0)
    #                     self.sonar_queue = []
    #                     self.image_set = []

    #         elif topic == '/ping360_node/sonar/data':
    #             #print(msg)
    #             self.sonar_queue.append(msg)


    def sonar_direction_vectors(self):
        cur_cam_time = None
        cur_sonar_time = None

        self.get_first_times()
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed', '/ping360_node/sonar/data']):
            
            if topic == '/usb_cam/image_raw/compressed':
                cur_cam_time = msg.header.stamp
                time_duration = cur_cam_time - self.prev_cam_time 

                self.cur_duration += time_duration
                # print("cur duration {} and total duration {} ".format(time_duration.to_sec(), self.cur_duration.to_sec()))
                self.prev_cam_time = msg.header.stamp

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
                            image_distorted, new_image = self.sonar_image_saver.image_to_cv2_both(self.image_set[0], True) # compressed

                            for i in range(0,len(self.sonar_queue)): 
                                cur_msg = self.sonar_queue[i]
                                int_intensities = [ord(b) for b in cur_msg.intensities]

                                # we get the max intensity to get the distance for the nearest read point in this cur msg
                                (max_intensity, max_index) = self.sonar_processor.max_intensity(int_intensities)
                                distance = self.sonar_processor.get_intensity_distance(int_intensities, max_index)

                                # we get the central point of the reading of where the center of the beam would be
                                # PHI is 30
                                # sonar_3d = self.sonar_processor.sonar_rectangle(1, cur_msg.angle, 30) # phi is 30


                                # TRYING TO MINIMIZE IN ERROR IN REPROJECTION-> 
                                # yaw should be within range of 90 (+/- 10 degrees), given how the camera and sonar are configured, trying to correct sonar to camera
                                rotation_matrix = self.sonar_processor.rotation_matrix_3d(90, 'z')

                                # YOU HAVEN"T CHANGE THE ANGLE
                                acutal_angle = self.sonar_processor.get_angle_degrees(cur_msg.angle)

                                # sonar_3d = self.sonar_processor.sonar_to_cam_reference_updated(distance, cur_msg.angle, 30, rotation_matrix)
                                sonar_3d = self.sonar_processor.sonar_rectangle(distance, acutal_angle, 90, 90)



                                # sonar_3d = self.sonar_processor.sonar_rectangle(distance, cur_msg.angle, 30) # phi is 30
                                possible_centers, possible_rectangles, possible_elevations = self.sonar_processor.elevation_ranges(distance, acutal_angle, 90)


                                # already in cam reference
                                direction_vector = sonar_3d[0]
                                sonar_rectangle = sonar_3d[1:]

                                print("projected vector in cam reference", direction_vector)
                                



                                on_image = self.sonar_image_saver.check_point_image(new_image, direction_vector, acutal_angle, distance)
                               
                                if on_image:
                                    # self.sonar_processor.plot_intensities(int_intensities, time_used)
                                    if acutal_angle not in self.angles_in_view:
                                        self.angles_in_view.append(acutal_angle)
                                        # print("Actual angle in view", acutal_angle)

                                    # making the rectangle
                                    angles_here[acutal_angle] = distance
                                    

                                    # transformed_sonar_3d = self.sonar_processor.transform_rectangle(sonar_3d)
                                    direction_vectors.append(direction_vector) # already in float 32 
                                    set_areas.append(sonar_rectangle) # set of rectangles for possible areas

                                    # checking the arc of possible elevations for direction vectors
                                    # # TRYING TO MINIMIZE IN ERROR IN REPROJECTION
                                    img = self.sonar_image_saver.project_possible_direction_vectors_notCV(new_image, possible_centers, possible_elevations)

                                    caption = str(acutal_angle) + " degrees"
                                    cv2.imshow(caption, img)
                                    cv2.waitKey(0)
                                    cv2.destroyAllWindows()

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
                                # ('height', 480, 'width', 640)
                                # ('new x,y', x: 312.08172607421875, y: 241.51197814941406)

                                # caption = str(angles_here)
                                # cv2.imshow(caption, img)
                                # cv2.waitKey(0)
                                # cv2.destroyAllWindows()

                                # Trying to save the images ->  timestep, realtime of camera image

                                # realtime_cam = self.image_set[0]
                                # self.sonar_image_saver.save_both_timestep(time_used, True)
                                # self.sonar_image_saver.save_image(img, time_used, str(realtime_cam.header.stamp))

                            
                            # img = self.sonar_image_saver.project_notCV(img, set_areas)

                        # also restarting the sonar queue and image set
                        self.cur_duration = rospy.Time(0)
                        self.sonar_queue = []
                        self.image_set = []

            elif topic == '/ping360_node/sonar/data':
                #print(msg)
                # print("Gain",msg.gain)
                # print("Transmit Frequency in kHz",msg.transmit_frequency)
                # print("Speed of Sound",msg.speed_of_sound)
                # print("Number of samples",msg.number_of_samples)
                self.sonar_queue.append(msg)
                
        print("camera angle view with rotation",sorted(self.angles_in_view))

    def checking_frequencies(self):
        cur_cam_time = None
        cur_sonar_time = None
        self.get_first_times()
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed', '/ping360_node/sonar/data']):
            
            if topic == '/usb_cam/image_raw/compressed':
                cur_cam_time = msg.header.stamp
                if self.prev_cam_time is not None:
                    time_duration = cur_cam_time - self.prev_cam_time 
                    self.cur_duration += time_duration
                    print("Camera time duration: {} seconds".format(time_duration.to_sec()))
                self.prev_cam_time = cur_cam_time

            elif topic == '/ping360_node/sonar/data':
                cur_sonar_time = msg.header.stamp
                if self.prev_sonar_time is not None:
                    sonar_time_diff = cur_sonar_time - self.prev_sonar_time
                    print("Sonar time difference: {} seconds".format(sonar_time_diff.to_sec()))
                self.prev_sonar_time = cur_sonar_time
                self.sonar_queue.append(msg)


    def camera_pose_test(self):
        cur_cam_time = None
        cur_sonar_time = None

        self.get_first_times()
        for topic, msg, t in self.input_bag.read_messages(topics=['/usb_cam/image_raw/compressed', '/ping360_node/sonar/data']):
            
            if topic == '/usb_cam/image_raw/compressed':
                cur_cam_time = msg.header.stamp
                time_duration = cur_cam_time - self.prev_cam_time 

                self.cur_duration += time_duration
                # print("cur duration {} and total duration {} ".format(time_duration.to_sec(), self.cur_duration.to_sec()))
                self.prev_cam_time = msg.header.stamp

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
                            image_distorted, new_image = self.sonar_image_saver.image_to_cv2_both(self.image_set[0], True) # compressed


                            # new IMAGE vs old image
                            prev_image_distorted, prev_image = self.sonar_image_saver.image_to_cv2_both(self.prev_image, True)

                            # just getting the depth for the last sonar
                            cur_msg = self.sonar_queue[-1] # most recent sonar
                            int_intensities = [ord(b) for b in cur_msg.intensities]
                            (max_intensity, max_index) = self.sonar_processor.max_intensity(int_intensities)
                            distance = self.sonar_processor.get_intensity_distance(int_intensities, max_index)

                            rotation_matrix = self.sonar_processor.rotation_matrix_3d(90, 'z')
                            acutal_angle = self.sonar_processor.get_angle_degrees(cur_msg.angle)
                            sonar_3d = self.sonar_processor.sonar_rectangle(distance, acutal_angle, 90, 90) #  to make 180

                            # already in cam reference
                            direction_vector = sonar_3d[0]
                            sonar_rectangle = sonar_3d[1:]
                            on_image = self.sonar_image_saver.check_point_image(new_image, direction_vector, acutal_angle, distance)





                            if on_image:
                                image_points1 = self.sonar_image_saver.rectangle_notCV(sonar_rectangle)
                                image_points_int321 = np.array(image_points1, dtype=np.int32).reshape((-1, 1, 2))
                                print("image points int 32", image_points_int321)

                                # we compute the image 
                                img = self.sonar_image_saver.project_direction_vectors_notCV(new_image, direction_vectors, angles_here)
                                img = self.sonar_image_saver.project_notCV(img, set_areas)

                                # getting matches from images
                                matches, locs1, locs2 = self.camera_pose.matchPics(prev_image, new_image)
                                corr_matches1, corr_matches2 = self.camera_pose.recover_match_locs(matches, locs1, locs2)
                                points_within_rectangle = self.camera_pose.within_projection(image_points_int321, corr_matches1, corr_matches2)
                                print("points within rectangle", points_within_rectangle)

                                # print("matches", matches, "locs1",locs1, "locs2", locs2)
                                (R, t) = self.camera_pose.get_Rt(matches, locs1, locs2)
                                print("R", R, "t", t)

                            



                                # points_within_rectangle = self.camera_pose.within_projection(image_points_int321, locs1, locs2)


                                




                        # also restarting the sonar queue and image set
                        self.prev_image = self.image_set[0] # using the image used
                        self.cur_duration = rospy.Time(0)
                        self.sonar_queue = []
                        self.image_set = []

            elif topic == '/ping360_node/sonar/data':
                #print(msg)
                # print("Gain",msg.gain)
                # print("Transmit Frequency in kHz",msg.transmit_frequency)
                # print("Speed of Sound",msg.speed_of_sound)
                # print("Number of samples",msg.number_of_samples)
                self.sonar_queue.append(msg)
                
        print("camera angle view with rotation",sorted(self.angles_in_view))



    
    def check_associations(self):

        print(len(self.associations))
        for time in self.associations:

            val = self.associations[time]
            print("time {} amount of sonar {}".format(time, len(val[1])))

    
# testing purposes
test = ImageSonarCoupler()
test.camera_pose_test()



