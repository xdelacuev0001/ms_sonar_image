import cv2
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import yaml

import numpy as np
import math
import tf
import cv2
import yaml


class ImageSonarProcessor1:

    def __init__(self):
        self.associations = None # the {} that gives camera message and sonar image together
        with open("cam.yaml", "r") as file:
            # Load the YAML data
            yaml_data = yaml.safe_load(file)

        self.camera_name = yaml_data['camera_name']
        self.image_width = yaml_data['image_width']
        self.camera_matrix = yaml_data['camera_matrix'] # dictionary
        self.projection_matrix = yaml_data['projection_matrix'] # dictionary
        self.image_height = yaml_data['image_height']
        self.rectification_matrix = yaml_data['rectification_matrix'] # dictionary
        self.distortion_model = yaml_data['distortion_model']
        self.distortion_coefficients = yaml_data['distortion_coefficients']
        self.tvec=np.zeros((3, 1), np.float32)
        self.rvec=np.array([0,0,1], np.float32)

    # get the m most recent sonar data
    def get_sonar_data(self, timestep, m=0):

        cur_time = self.associations[timestep] # returns [ [list images], [list sonar]]
        sonar_list = cur_time[1] # getting list of sonar data
        
        # default just retrieving all of the sonar data itself
        if m == 0:
            return sonar_list

        if m > len(sonar_list):
            raise ValueError("The number of requested items exceeds the length of the sonar data list.")
            
        return sonar_list[-m:]

    def get_image_data(self, timestep):

        cur_time = self.associations[timestep] # returns [ [list images], [list sonar]]
        images = cur_time[0] # getting list of images

        return images


    def sonar_to_txt(self, timestep, output_directory, m=0):

        # all the sonar data
        if m == 0:
            sonar_list = self.get_sonar_data(timestep)

        else:
            sonar_list = self.get_sonar_data(timestep, m)

        if not os.path.exists(output_directory):
            os.makedirs(output_directory)

        # this is a set of sonar data in one text file
        sonar_filename = "sonar_"+ str(timestep) + ".txt"
        filename = os.path.join(output_directory, sonar_filename)
        file = open(filename,'w')
        for msg in sonar_list:
            file.write(str(msg)+"\n")
        file.close()

        print("saved sonar")

    def cam_to_file(self, timestep, output_directory, compressed):
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)

        images = self.get_image_data(timestep)

        bridge = CvBridge()
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        try:
            if compressed == False:
                for i in range(0, len(images)):
                    msg = images[i]
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    h,w = cv_image.shape[:2]
                    new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 0, (w, h)) 
                    updated_cv_image = cv2.undistort(cv_image, camera_matrix, distortion_coefficients, None, new_cam_matrix)
                    filename = "image"+ str(i)+ '_'+ str(msg.header.stamp)+'.png' # the true timestep
                    image_filename = os.path.join(output_directory, filename)
                    cv2.imwrite(image_filename, updated_cv_image)
                    print("Saved camera image {}".format(image_filename))
            
            # for compressed images
            else:
                for i in range(0, len(images)):
                    msg = images[i]
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough') # for compressed
                    h,w = cv_image.shape[:2]
                    new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 0, (w, h)) 
                    updated_cv_image = cv2.undistort(cv_image, camera_matrix, distortion_coeffs, None, new_cam_matrix)
                    filename = "image"+ str(i)+ '_'+ str(msg.header.stamp)+'.png' # the true timestep
                    image_filename = os.path.join(output_directory, filename)
                    cv2.imwrite(image_filename, updated_cv_image)
                    print("Saved camera image {}".format(image_filename))

        except CvBridgeError as e:
            print(e)
    

    def return_directory(self, timestep):
        output_directory = '/root/catkin_ws/src/outputs' + '/' + str(timestep)

        return output_directory


    def save_both_timestep(self, timestep, realtime):

        # every timestep will have its own directory with the group of photos and the sonar txt
        output_directory = self.return_directory(timestep)
        image_directory = output_directory + '/image_undistorted'
        # sonar_directory = '/root/catkin_ws/src/outputs/sonar_outputs'
        # m = 5
        # sub_image_directory = image_directory + '/' + str(timestep)

        self.sonar_to_txt(timestep, output_directory)
        self.cam_to_file(timestep, image_directory, realtime) #realtime boolean-> not compressed, if false, compressed

    def save_image(self, image, timestep, cam_time):
        output_directory = self.return_directory(timestep)
        filename = "sonar_image"+ '_'+ cam_time+'.png' # the true timestep
        image_filename = os.path.join(output_directory, filename)
        cv2.imwrite(image_filename,image)
        print("Saved camera with sonar image {}".format(image_filename))




    def image_to_cv2_both(self,raw_image, compressed ):
        bridge = CvBridge()
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        try:
            if compressed == False:
                cv_image = bridge.imgmsg_to_cv2(raw_image, "bgr8")

                # undistort:
                h,w = cv_image.shape[:2]
                # print("height", h, "width", w)
                # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
                # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
                new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 0, (w, h)) 
                updated_cv_image = cv2.undistort(cv_image, camera_matrix, distortion_coefficients, None, new_cam_matrix)
                return (cv_image, updated_cv_image)
            else:
                cv_image = bridge.compressed_imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

                # undistort
                h,w = cv_image.shape[:2]
                # print("height", h, "width", w)
                # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
                # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
                new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 0, (w, h)) 
                updated_cv_image = cv2.undistort(cv_image, camera_matrix, distortion_coeffs, None, new_cam_matrix)
                return (cv_image,updated_cv_image)
                # cv.getOptimalNewCameraMatrix(	cameraMatrix, distCoeffs, imageSize, alpha[, newImgSize[, centerPrincipalPoint]]	) 

        except CvBridgeError as e:
            print(e)


 

    def check_point_image(self, image, center_point, angle, distance):
        rows, cols, _ = image.shape
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        image_center_point, _ = cv2.projectPoints(np.array([center_point],np.float32 ), self.rvec, self.tvec, camera_matrix, distortion_coeffs)

        # also need to undistort them
        image_center_point_int32 = np.array(image_center_point, dtype=np.int32).reshape((-1, 1, 2))

        x, y = image_center_point_int32[0][0]
        # print("CENTER POINT", center_point, "from distance", distance, "VS PROJECTED POINT", x, y)
        #print(x,y)

        if 0 <= x < cols and 0 <= y < rows:
            return True
        else:
            return False


    def project_point_notCV(self, real_x, real_y, real_z):

        # print("rotation_matrix", rotation_matrix[0])
        # rows, cols, _ = image.shape
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))

        # assuming that point is already in camera reference frame:
        real_vector = np.array([real_x, real_y, real_z])

        print("real_vector in cam reference", real_vector)
        # real_vector = np.array([0, 0, 1])
        image_vector_unscaled = camera_matrix.dot(real_vector)
        # print(image_vector_unscaled)
        #print("Image vector", image_vector_unscaled)
        # int_cam_matrix = np.hstack([camera_matrix, np.zeros((3, 1))])
        
        # # extrinsic cam assuming 90 degree yaw
        # rot = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(-90))
        # Rt = tf.transformations.quaternion_matrix(rot)

        # real_T_image = int_cam_matrix.dot(Rt)

        # real_vector = np.array([real_x, real_y, real_z] + [1])

        

        # image_vector_unscaled = real_T_image.dot(real_vector)
        z_scale = image_vector_unscaled[2]
        x = image_vector_unscaled[0] /z_scale
        y = image_vector_unscaled[1] /z_scale
        z = image_vector_unscaled[2] /z_scale

        print("new x,y", x, y)

        image_vector = np.array([x,y], np.float32)
        return image_vector

    def rectangle_notCV(self, rectangle):

        image_vectors = []

        for i in range(0, len(rectangle)):
            cur_point = rectangle[i]
            new_vector = self.project_point_notCV(cur_point[0], cur_point[1], cur_point[2])
            image_vectors.append(new_vector)

        return image_vectors

    def project_direction_vectors_notCV(self, image, direction_vectors, angles_here):
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        img = image
        image_points = []

        for i in range(0, len(direction_vectors)):
            cur_point = direction_vectors[i]
            # image_points, _ = cv2.projectPoints(cur_points, self.rvec, self.tvec, camera_matrix, distortion_coeffs)
            image_points1 = self.project_point_notCV(cur_point[0], cur_point[1], cur_point[2])
            image_points_int321 = np.array(image_points1, dtype=np.int32).reshape((-1, 1, 2))

            # print("image_points_int321 direction vector", image_points_int321)
            x, y = image_points_int321[0][0]

            image_points.append([x,y])

            # image = cv2.circle(image, center_coordinates, radius, color, thickness) 
            img = cv2.circle(img, (x,y), 3, (255, 0, 0) , -1)

        return img

    # Trying to get arc of possible central points using elevations from 0-180 in increments of 5
    # NOT USING cv function projectPoints
    def project_possible_direction_vectors_notCV(self, image, direction_vectors, elevations):
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        img = image
        image_points = []

        for i in range(0, len(direction_vectors)):
            cur_point = direction_vectors[i]
            label = str(elevations[i])
            # image_points, _ = cv2.projectPoints(cur_points, self.rvec, self.tvec, camera_matrix, distortion_coeffs)
            image_points1 = self.project_point_notCV(cur_point[0], cur_point[1], cur_point[2])
            image_points_int321 = np.array(image_points1, dtype=np.int32).reshape((-1, 1, 2))

            # print("image_points_int321 direction vector", image_points_int321)
            x, y = image_points_int321[0][0]

            image_points.append([x,y])


            # image = cv2.circle(image, center_coordinates, radius, color, thickness) 
            # img = cv2.circle(img, (x,y), 3, (255, 0, 0) , -1)
            img = cv2.putText(img, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0) , 1)

        # print("direction vectors", direction_vectors, "corresponding image points", image_points, "corresponding to angles", angles_here)

        return img


    # Projecting direction vectors using cv's Project Points
    # parameter assumes point is within image
    def project_direction_vectors_CV(self, image, center_points, angles_here):

        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        img = image

        for i in range(0, len(center_points)):

            cur_point = center_points[i]
            # print("cur point",cur_point)
            # image_points, _ = cv2.projectPoints(np.array([cur_point],np.float32), self.rvec, self.tvec, camera_matrix, None)
            image_points, _ = cv2.projectPoints(np.array([cur_point],np.float32), np.array([0,0,0], np.float32), self.tvec, camera_matrix, None)

            # print("IMAGE Points", image_points)
            image_points_int32 = np.array(image_points, dtype=np.int32).reshape((-1, 1, 2))
            x, y = image_points_int32[0][0]
            img = cv2.circle(img, (x,y), 3, (255, 0, 0) , -1)

        print("direction vectors", center_points, "corresponding image points", image_points, "corresponding to angles", angles_here)

        return img



    # projecting areas using CV function projectPoints
    def project_CV(self, image,rectangle_points):

        rows, cols, _ = image.shape
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        # distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        
        img = image
        indiv = []
        for i in range(0, len(rectangle_points)):
            cur_points = rectangle_points[i]
            image_points, _ = cv2.projectPoints(cur_points, np.array([0,0,0], np.float32), self.tvec, camera_matrix, None)
            image_points_int32 = np.array(image_points, dtype=np.int32).reshape((-1, 1, 2))
            # print("image not reshaped", image_points, "reshaped", image_points_int32)
            # print("rectangle points", rectangle_points, "image_points_int32", image_points_int32)
            indiv.append(image_points_int32)
            img = cv2.polylines(img, indiv, isClosed=True, color=(255,0,0), thickness = 2)

        return img

    # projecting areas using own matrices and transformation 
    def project_notCV(self, image, rectangle_points):
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))

        rectangles = []
        img = image

        for i in range(0, len(rectangle_points)):
            cur_rectangle = rectangle_points[i]
            image_points1 = self.rectangle_notCV(cur_rectangle)
            image_points_int321 = np.array(image_points1, dtype=np.int32).reshape((-1, 1, 2))
            img = cv2.polylines(image, [image_points_int321], isClosed=True, color=(255,0,0), thickness = 2)

        return img


# DRAFTTSSS
    def testing_out_image(self, image):
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)

        rotation_matrix = tf.transformations.euler_matrix()

        # test point
        test_point = [0,0,1]

        # print("Camera matrix",camera_matrix)
        h,w = image.shape[:2]
        [dst,jacobian] = cv2.Rodrigues(self.rvec)

        # print("Rotation vector", jacobian)


        image_points, _ = cv2.projectPoints(np.array([test_point],np.float32), self.rvec, self.tvec, camera_matrix, distortion_coeffs)
        # print("Image width", w, "image_height", h)

        # print("IMAGE Points for center float", image_points)
        image_points_int32 = np.array(image_points, dtype=np.int32).reshape((-1, 1, 2))
        x, y = image_points_int32[0][0]
        # print("IMAGE Points for center", x, y)
        img = cv2.circle(image, (x,y), 3, (255, 0, 0) , -1)

        return img










