# Xenia Dela Cueva

# Library to Get relative motion between image sequences


# functions

# Feature Extraction: Detect and describe keypoints in both images using feature detectors like SIFT, SURF, ORB, etc.
# Feature Matching: Match the features between the two images.
# Find Essential or Fundamental Matrix: Use the matched features to compute the essential or fundamental matrix.
# Recover Pose: From the essential matrix, recover the relative pose (rotation and translation) between the images.


import numpy as np
import cv2
import scipy.io as sio
from matplotlib import pyplot as plt
import skimage.feature
import skimage.color
import tf
import yaml
import matplotlib.path as mplPath

from helper_cv import *

class CameraProcessor:
	def __init__(self):
		
		# with open("cam.yaml", "r") as file:
		# 	yaml_data = yaml.safe_load(file)
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

	def matchPics(self, I1, I2):
		#Convert Images to GrayScale
		I1_gray = cv2.cvtColor(I1, cv2.COLOR_BGR2GRAY)
		I2_gray = cv2.cvtColor(I2, cv2.COLOR_BGR2GRAY)
	
		#Detect Features in Both Images
		locs1 = corner_detection(I1_gray) # sigma = 0.15
		locs2 = corner_detection(I2_gray)
	
		#Obtain descriptors for the computed feature locations

		# x, y format for locs
		desc1, locs1 = computeBrief(I1_gray, locs1) # using Brief Descriptors
		desc2, locs2 = computeBrief(I2_gray, locs2)
	
		#Match features using the descriptors
		matches = briefMatch(desc1, desc2)
	
		return matches, locs1, locs2

	def recover_match_locs(self,matches, locs1, locs2):

		new_locs1 = []
		new_locs2 = []

		for i in range(0, len(matches)):
			pair = matches[i]
			index1 = pair[0]
			index2 = pair[1]

			new_locs1.append(locs1[index1])
			new_locs2.append(locs2[index2])

		return (np.array(new_locs1), np.array(new_locs2))

	# input: 2 images
	def get_Rt(self, matches, locs1, locs2):
		new_locs1, new_locs2 = self.recover_match_locs(matches, locs1, locs2)
		K = np.array(self.camera_matrix['data'],dtype=np.float32).reshape((3, 3))
		# print("new locs2", new_locs2, "new locs1", new_locs1)
		# print("new locs1", new_locs1, "new locs2", new_locs2)
		[E ,mask]= cv2.findEssentialMat(new_locs1, new_locs2, K)
		print("E", E, "new locs2", new_locs2)
		[R, t, _ , mask] = cv2.recoverPose(E, new_locs1, new_locs2)

		return (R, t)


	# rectangle_notCV(self, rectangle) order : upper_left, upper_right, lower_right, lower_left

	def within_projection(self, image_rectangle_array, match_locs1, match_locs2):
		# for height
		upper_left = image_rectangle_array[0]
		upper_right = image_rectangle_array[1]
		lower_right = image_rectangle_array[2]
		lower_left = image_rectangle_array[3]

		features = []

		quad_points = [tuple(point[0]) for point in image_rectangle_array]
		print("Quad points", quad_points)

		quad_path = mplPath.Path(quad_points)


		#print("upper left", upper_left, "upper right", upper_right, "lower right", lower_right, "lower left", lower_left)

		for i in range(0, len(match_locs2)):
			cur_feature = match_locs2[i]
			if quad_path.contains_point(cur_feature):
				print("True")
				features.append(cur_feature)

		return cur_feature






	

# given the rotation and translation between 2 images as well as matching points, and their
# locations

# we can use image 2's depth reading within the rectangle-> given points
# we check if any matching points from image 2 are within the rectangle


# points3D = []
# for i in known_depth_indices:
#     x, y = pts1[i]
#     depth = depths[i]
#     X = (x - K[0, 2]) * depth / K[0, 0]
#     Y = (y - K[1, 2]) * depth / K[1, 1]
#     Z = depth
#     points3D.append([X, Y, Z])
# points3D = np.array(points3D)



# # Function to compute the re-projection error
# def reprojection_error(params, K, points3D, pts2):
#     R_vec = params[:3]
#     t_vec = params[3:6]
#     R, _ = cv2.Rodrigues(R_vec)
#     t = t_vec.reshape((3, 1))
    
#     # Project points3D to the second image
#     points2D_proj, _ = cv2.projectPoints(points3D, R, t, K, distCoeffs=None)
#     points2D_proj = points2D_proj.reshape(-1, 2)
    
#     # Compute re-projection error
#     error = pts2 - points2D_proj
#     return error.ravel()

# # Initial parameters for optimization: Rotation vector and translation vector
# R_vec, _ = cv2.Rodrigues(R)
# t_vec = t.flatten()
# initial_params = np.hstack((R_vec, t_vec))

# # Perform optimization
# result = least_squares(reprojection_error, initial_params, args=(K, points3D, pts2))

# # Extract optimized rotation and translation
# R_vec_optimized = result.x[:3]
# t_vec_optimized = result.x[3:]
# R_optimized, _ = cv2.Rodrigues(R_vec_optimized)
# t_optimized = t_vec_optimized.reshape((3, 1))

# print("Optimized Rotation matrix:\n", R_optimized)
# print("Optimized Translation vector:\n", t_optimized)




# [R, t, good] = cv.recoverPose(E, points1, points2)
# [R, t, good, mask, triangulatedPoints] = cv.recoverPose(...)
# [...] = cv.recoverPose(..., 'OptionName', optionValue, ...)