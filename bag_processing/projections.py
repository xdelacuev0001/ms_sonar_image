import matplotlib.pyplot as plt
import numpy as np
import math
import tf
import cv2
import yaml


# goal-> getting 3d points onto image as rectangle

class ProjectSonar():
    def __init__(self):

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
        self.rvec=np.zeros((3, 1), np.float32)
        self.tvec=np.array([0,0,1], np.float32)


    def points_on_image(self, image_path,rectangle_points):
        image = cv2.imread(image_path)

        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))

        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        # print(camera_matrix)
        image_points, _ = cv2.projectPoints(rectangle_points, self.rvec, self.tvec, camera_matrix, distortion_coeffs)
        # image_points = np.array
        # print("Image_points",image_points)
        image_points_int32 = np.array(image_points, dtype=np.int32).reshape((-1, 1, 2))
        print("Image_points", image_points_int32)


        img = cv2.polylines(image, [image_points_int32], isClosed=True, color=(255,0,0), thickness = 2)

        # cv2.imshow("Rectangle?", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
                      
        # # Displaying the image
        while(1):
            cv2.imshow('rectangle', img)
            if cv2.waitKey(20) & 0xFF == 27:
                break

    def points_on_image2(self, image_path,rectangle_points):
        image = cv2.imread(image_path)
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
        camera_matrix = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))

        distortion_coeffs = np.array(self.distortion_coefficients['data'],np.float32)
        indiv = []

        img = image

        for i in range(0, len(rectangle_points)):

            cur_points = rectangle_points[i]
            image_points, _ = cv2.projectPoints(cur_points, self.rvec, self.tvec, camera_matrix, distortion_coeffs)
            image_points_int32 = np.array(image_points, dtype=np.int32).reshape((-1, 1, 2))
            indiv.append(image_points_int32)
            img = cv2.polylines(img, indiv, isClosed=True, (255,0,0), thickness = 2)

        cv2.imshow("Rectangle(s)", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
                      
        # # Displaying the image
        # while(1):
        #     cv2.imshow('rectangle', img)
        #     if cv2.waitKey(20) & 0xFF == 27:
        #         break
         
# cv2.destroyAllWindows()

        # for point in image_points.astype(int): 
        #     print("hi")

        #     # image = cv2.circle(image, center_coordinates, radius, color, thickness) 
        #     img = cv2.circle(image, tuple(point[0]), 4, 255, 4) 

        # cv2.imshow('Image', img) 
        # cv2.waitKey(0) 
        # cv2.destroyAllWindows() 

        # Draw the projected points on the image
        # for i in range(0,len(image_points)):
        #     print("this is index i", image_points[i])
        #     point = image_points[i]
        # # for point in image_points:
        #     cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)  # Draw green circles at projected points

        #cv2.polylines(image, [rectangle_points], isClosed=True, color=(0, 255, 0), thickness=1)


# 

image_path = "/root/catkin_ws/src/outputs/1708044520551553035/image_outputs/image0_1708044520431521388.png"



test_point = np.array([0,0,0], np.float32)
test_point1 = np.array([300,200,200], np.float32)
test_list = np.array([test_point, test_point1], np.float32)
test = ProjectSonar()
sample_rectangle_center = [[-3.53553391e+00, -3.53553391e+00, 3.06161700e-16],
[-4.22748572e+00, -2.44074002e+00, -1.08219807e+00],[-2.44074002e+00 ,-4.22748572e+00 ,-1.08219807e+00],[-4.22748572e+00 ,-2.44074002e+00,  1.08219807e+00],[-2.44074002e+00, -4.22748572e+00 , 1.08219807e+00]]


sample_rectangle_center = [[0,0,0],[0, 0, 0],[.5, 0,0],[.5 ,.5 ,0],[0 ,.5,  0]]
sample_rectangle = []
for i in range(1, len(sample_rectangle_center)):
    sample_rectangle.append(np.array(sample_rectangle_center[i],np.float32 ))
sample_rectangle = np.array(sample_rectangle,np.float32)



sample_rectangle_center2 = [[0,0,0],[0, 0, 0],[.2, 0,0],[.2 ,.2 ,0],[0 ,0.2,  0]]
sample_rectangle2 = []
for i in range(1, len(sample_rectangle_center2)):
    sample_rectangle2.append(np.array(sample_rectangle_center2[i],np.float32 ))
sample_rectangle2 = np.array(sample_rectangle2,np.float32)

sample_set = np.array([sample_rectangle, sample_rectangle2], np.float32)

test.points_on_image2(image_path, sample_set)

# {'camera_name': 'narrow_stereo',
#  'image_width': 640, 
# 'camera_matrix': {'rows': 3, 'data': [446.73978, 0.0, 312.08173, 0.0, 448.81162, 241.51198, 0.0, 0.0, 1.0], 'cols': 3}, 
# 'projection_matrix': {'rows': 3, 'data': [448.85345, 0.0, 308.68809, 0.0, 0.0, 451.23367, 238.08258, 0.0, 0.0, 0.0, 1.0, 0.0], 'cols': 4}, 
# 'image_height': 480, 
# 'distortion_coefficients': {'rows': 1, 'data': [-0.001485, 0.000128, -0.006106, -0.004205, 0.0], 'cols': 5}, 
# 'rectification_matrix': {'rows': 3, 'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], 'cols': 3}, 
# 'distortion_model': 'plumb_bob'}

# def project_3d_points_on_image(image_path, object_points, rvec=np.zeros((3, 1)), tvec=np.zeros((3, 1)),
#                                focal_length=1000, distortion_coeffs=np.zeros((4, 1))):
#     # Read the image
#     image = cv2.imread(image_path)

#     # Define camera parameters
#     principal_point = (image.shape[1] / 2, image.shape[0] / 2)  # Image center
#     camera_matrix = np.array([[focal_length, 0, principal_point[0]],
#                               [0, focal_length, principal_point[1]],
#                               [0, 0, 1]])  # Example camera matrix

#     # Project 3D points to image plane
#     image_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coeffs)

#     # Draw the projected points on the image
#     for point in image_points.squeeze():
#         cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)  # Draw green circles at projected points

#     # Display the image with projected points
#     cv2.imshow("Projected Points", image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# # Example usage
# image_path = "your_image_path.jpg"
# object_points = np.array([[10, 5, 20],    # Example 3D point 1
#                           [15, -8, 25],   # Example 3D point 2
#                           [0, 10, 15]])  # Example 3D point 3

# project_3d_points_on_image(image_path, object_points)
