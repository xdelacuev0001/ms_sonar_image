import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import tf
import cv2 

#   Header header            #header info
#   float32 angle               # the measurement angle [rad]
#   uint8 gain  # Sonar Gain
#   uint16 number_of_samples 
#   uint16 transmit_frequency # [kHz]
#   uint16 speed_of_sound # [m/s]
#   uint8 range      #  range value [m]
#   uint8[] intensities    # intensity data [0-255].  This is the actual data received from the sonar

# hand measured translations from the camera (in meters)
x_translation = .13
y_translation = -0.15
z_translation = -0.18

class SonarProcessor:

    def __init__(self, clicks):

        self.associations = None
        self.intens_threshold = 20  # values for intensities that we would consider
        self.vertical_beamwidth = 25
        self.horizontal_beamwidth = 2
        # self.msg = msg
        # self.intensities = [ord(b) for b in msg.intensities]
        # self.range = msg.range


        # the sonar's transformations based in reference to camera (hand measured)
        
        # orientations
        self.or_x = 0
        self.or_y = 0
        self.or_z = 0

        # sonar position
        self.x = 0
        self.y = 0
        self.z = 0


    # we need to break down intensities 
    def get_angle_degrees(self, angle):
        # adjuster = round(360,, 3)
        adjuster = np.divide(np.float32(360), np.float32(400)).astype(np.float32)
        adjuster = round(adjuster, 2)
        
        real_angle = angle * adjuster
        real_angle = round(real_angle, 3)
        # print("Adjuster", adjuster, "Click", angle, "adjusted", real_angle)
        return real_angle

    def get_angle_radians(self, angle):
        angle_deg = self.get_angle_degrees(angle)
        return math.radians(angle_deg)

    def plot_sonar_datas(self, msg_list, timestamp):

        for each in msg_list:
            intensities = np.array(each.intensities)
            plt.plot(intensities)

        title = "Sonar Data Intensities at Time " + str(timestamp)
        plt.title(title)
        plt.xlabel("Index")
        plt.ylabel("Intensity")
        plt.ylim(0, 260)  # Set y-axis limit to 0-255
        plt.show()

    def plot_intensities(self, intensities, timestamp):
        intensities = np.array(intensities)
        plt.plot(intensities)
        title = "Sonar Data Intensities at Time " + str(timestamp)
        plt.title(title)
        plt.xlabel("Index")
        plt.ylabel("Intensity")
        plt.ylim(0, 260)  # Set y-axis limit to 0-255
        plt.show()

    # checking if in the sight of camera
    def check_in_sight(self, msg_list):
        first_item = msg_list[0]
        last_item = msg_list[-1]
        print("angle out first " + str(first_item.angle) + " angle out last" + str(last_item.angle))


        # ADJUSTED BACK
        if first_item.angle < 200 and last_item.angle < 200:
        # if first_item.angle < 215 and last_item.angle < 265:
            print("disregarded")
            return False
        else:
            print("usable")
            return True



    # given an array so parameter should be msg.intensities
    # given an intensity index, we get the value
    def get_intensity_distance(self, data, i):
        max_dis = float(10)
        range_inc = round(max_dis / float(len(data)), 2)
    
        # Calculate the distance corresponding to the given index
        distance = i * range_inc + range_inc # we assume the first one not 0 but the first increment
        distance = round(distance, 2)
    
        return distance

    # given an array so parameter should be msg.intensities
    # getting the max value in set of intensities, as well as disregarding if below threshold
    def max_intensity(self, data):

        max_intensity = 0
        max_index = 0 

        # 14 is 0.75 meters, it increments from 0.05
        for i in range(14, len(data)):
            if data[i] < self.intens_threshold:
                continue
            else:
                # we'd rather keep the intensity closer, so its not >=
                if data[i] > max_intensity: 
                    max_intensity = data[i]
                    max_index = i

        # print('max intensity: ', max_intensity, " at distance ", self.get_intensity_distance(data, max_index))

        return (max_intensity, max_index)

    def transform_point(self,point):
 
        transformed_point = self.sonar_to_cam_reference(point[0], point[1], point[2])
        return np.array(transformed_point, np.float32)

    # # python implementation of projecting sonar onto image
    # def get_point(self, distance, og_x, og_y, og_z, azimuth, elevation):
    #     phi = np.deg2rad(elevation)
    #     theta= np.deg2rad(azimuth)

    #     x = og_x + distance * np.sin(phi) * np.cos(theta)
    #     y = og_y + distance * np.sin(phi) * np.sin(theta)
    #     z = og_z + distance * np.cos(phi)

    #     return [x,y,z]

    
    # def sonar_to_cam_reference(self, x, y, z):
    #     global x_translation, y_translation, z_translation
    #     # Map origin from sonar origin in refernce to camera frame
    #     trans = [x_translation, y_translation, z_translation]

    #     # converting my euler angles to quaternion -> [0,0,0] for roll, pitch, yaw since no initial orientation change
    #     initial_ors = [0, 0, 0]

    #     # # USING THE ROTATION FUNCTION
    #     # rotation_matrix = self.rotation_matrix()
    #     # initial_ors = tf.transformations.quaternion_from_matrix(rotation_matrix)
    #     rot = tf.transformations.quaternion_from_euler(initial_ors[0], initial_ors[1], initial_ors[2])

    #     # making my transofrmation matrix for occupancy grid in reference to odom
    #     t = tf.transformations.translation_matrix(trans)
    #     R = tf.transformations.quaternion_matrix(rot)


    #     # Transformation: sonar reference to camera
    #     son_T_cam = t.dot(R) # sonar point in reference to camera-> Using this
    #     # cam_T_son = np.linalg.inv(son_T_cam) # camera point in reference to sonar

    #     sonar_frame_vector = [x, y, z, 1]
    #     sonar_frame_vector = np.array(sonar_frame_vector) 
    #     cam_frame_vector = son_T_cam.dot(sonar_frame_vector)

    #     # gives a new x, y, z in terms of camera frame -> turned to be vectors for cv image
    #     return np.array([cam_frame_vector[0], cam_frame_vector[1], cam_frame_vector[2]], np.float32)
    def plot_points_3d(self, points):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Extract coordinates of the points
        x_coords = [point[0] for point in points]
        y_coords = [point[1] for point in points]
        z_coords = [point[2] for point in points]

        # Plot the points
        ax.scatter(x_coords, y_coords, z_coords, c='b', marker='o')

        # Connect points to the projected center
        projected_center = points[0]
        for point in points[1:]:
            ax.plot([point[0], projected_center[0]], [point[1], projected_center[1]], [point[2], projected_center[2]], linestyle='--', color='gray')

        # Set labels and display
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Projected Points in 3D')
        plt.show()

    # What we are trying to minimize in reprojection
    def rotation_matrix_3d(self, theta, axis):
        # Convert angle from degrees to radians
        theta = np.radians(theta)
        if axis == 'x':
            rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])
        elif axis == 'y':
            rotation_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                    [0, 1, 0],
                                    [-np.sin(theta), 0, np.cos(theta)]])
        elif axis == 'z':
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 1]])
        else:
            raise ValueError("Axis type must be 'x', 'y', or 'z'")
    
        return rotation_matrix





    def sonar_to_cam_reference_updated(self, distance, azimuth, elevation, yaw_deg):
        # rotation is 1X3 vector for roll pitch yaw
        global x_translation, y_translation, z_translation

        trans = [x_translation, y_translation, z_translation]

        # Convert azimuth and elevation angles to radians
        phi = np.deg2rad(elevation)
        theta= np.deg2rad(azimuth)

        # Calculate the point in the sonar frame, assuming origin of sonar is 0 0 0 
        x = distance * np.sin(phi) * np.cos(theta)
        y = distance * np.sin(phi) * np.sin(theta)
        z = distance * np.cos(phi)

        # Make transformation matrix for sonar reference to camera frame
        t = tf.transformations.translation_matrix(trans)
        # assuming euler angles for rotation

        # the roll is -90 to make z forward
        around_x = self.rotation_matrix_3d(-90, 'x')

        around_z = self.rotation_matrix_3d(90 + yaw_deg, 'z')

        rotation_matrix =  around_x.dot(around_z)

        rotation = tf.transformations.euler_from_matrix(rotation_matrix, axes='sxyz')
        rot = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])

        R = tf.transformations.quaternion_matrix(rot)



        # Transformation: sonar reference to camera
        son_T_cam = t.dot(R) # sonar point in reference to camera -> Using this
        # print('son to cam', son_T_cam)

        # print("sonar to cam transformation matrix", son_T_cam)

        # Get point in the sonar frame
        sonar_point = [x, y, z]


        # Convert the sonar frame vector to a homogeneous vector
        sonar_frame_vector = np.array(sonar_point + [1])

        # Transform the sonar frame vector to the camera frame
        cam_frame_vector = son_T_cam.dot(sonar_frame_vector)

        return np.array([cam_frame_vector[0], cam_frame_vector[1], cam_frame_vector[2]], np.float32)

    def sonar_to_cam_reference(self, distance, og_x, og_y, og_z, azimuth, elevation):
        global x_translation, y_translation, z_translation

        trans = [x_translation, y_translation, z_translation]

        # Convert azimuth and elevation angles to radians
        phi = np.deg2rad(elevation)
        theta= np.deg2rad(azimuth)

        # Calculate the point in the sonar frame
        x = og_x + distance * np.sin(phi) * np.cos(theta)
        y = og_y + distance * np.sin(phi) * np.sin(theta)
        z = og_z + distance * np.cos(phi)

        # Make transformation matrix for sonar reference to camera frame
        t = tf.transformations.translation_matrix(trans)

        # Transformation: sonar reference to camera
        son_T_cam = t # sonar point in reference to camera -> Using this

        # Get point in the sonar frame
        sonar_point = [x, y, z]

        # Convert the sonar frame vector to a homogeneous vector
        sonar_frame_vector = np.array(sonar_point + [1])

        # Transform the sonar frame vector to the camera frame
        cam_frame_vector = son_T_cam.dot(sonar_frame_vector)

        return np.array([cam_frame_vector[0], cam_frame_vector[1], cam_frame_vector[2]], np.float32)


    def sonar_rectangle(self, distance, angle, phi, rotation_matrix):
        # getting the projected center
        # projected_center = self.sonar_to_cam_reference(distance, self.x, self.y, self.z, angle, phi )

        # projected_center = self.sonar_to_cam_reference_updated(distance, self.x, self.y, self.z, angle, phi, rotation_matrix )
        projected_center = self.sonar_to_cam_reference_updated(distance,angle, phi, rotation_matrix)
        # projected_center = self.get_point(distance, self.x, self.y, self.z, angle, phi) # in form of [x,y,z]
        # print(projected_center)

        # print("projected center", projected_center, "from angle", angle, "at elevation", phi,"at distance", distance)

        theta_offset = self.horizontal_beamwidth /2  # sonar's horizontal beamwidth for x,y
        phi_offset = self.vertical_beamwidth /2 # sonar's vertical beamwith for z,y

        # distance from sonar origin should be same as center point
        # (self, distance, og_x, og_y, og_z, azimuth, elevation)

        # left (-) in azimuth, above (-) in elevation from center
        # upper_left = self.sonar_to_cam_reference_updated(distance, self.x, self.y, self.z, angle - theta_offset, phi - phi_offset, rotation_matrix)

        # sonar_to_cam_reference_updated(self, distance, azimuth, elevation, rotation_matrix)
        upper_left = self.sonar_to_cam_reference_updated(distance, angle - theta_offset, phi - phi_offset, rotation_matrix)
        # right (+) in azimuth, above (-) in elevation from center 
        # upper_right = self.sonar_to_cam_reference_updated(distance, self.x, self.y, self.z, angle + theta_offset, phi - phi_offset, rotation_matrix)
        upper_right = self.sonar_to_cam_reference_updated(distance, angle + theta_offset, phi - phi_offset, rotation_matrix) 

        # left (-) in azimuth, below (+) in elevation from center
        # lower_left = self.sonar_to_cam_reference_updated(distance, self.x, self.y, self.z, angle - theta_offset, phi + phi_offset, rotation_matrix)
        lower_left = self.sonar_to_cam_reference_updated(distance, angle - theta_offset, phi + phi_offset, rotation_matrix)
        

        # right (+) in azimuth, below (+) in elevation from center
        # lower_right = self.sonar_to_cam_reference_updated(distance, self.x, self.y, self.z, angle + theta_offset, phi + phi_offset, rotation_matrix)
        lower_right = self.sonar_to_cam_reference_updated(distance, angle + theta_offset, phi + phi_offset, rotation_matrix)

        # print("cam vectors", [projected_center, upper_left, upper_right, lower_right, lower_left])

        return [projected_center, upper_left, upper_right, lower_right, lower_left]


    # This is one of the things we are trying to minimize in error for reprojection
    def elevation_ranges(self, distance, angle, yaw_deg):
        # centers
        centers = []

        # just rectangles
        rectangles = []
        phis = []
        
        for phi in range(0, 181, 5):
            cen_rectangle = self.sonar_rectangle(distance, angle, phi, yaw_deg)
            center = cen_rectangle[0]
            rectangle = cen_rectangle[1:]
            centers.append(center)
            rectangles.append(rectangle)
            phis.append(phi)

        return(np.array(centers, np.float32), np.array(rectangles, np.float32), np.array(phis, np.float32))


    
    # def sonar_rectangle_draft(self, distance, angle, phi):
    #     # getting the projected center
    #     projected_center = self.get_point(distance, self.x, self.y, self.z, angle, phi) # in form of [x,y,z]
    #     # print(projected_center)

    #     theta_offset = self.horizontal_beamwidth /2  # sonar's horizontal beamwidth for x,y
    #     phi_offset = self.vertical_beamwidth /2 # sonar's vertical beamwith for z,y

    #     # distance from sonar origin should be same as center point
    #     # (self, distance, og_x, og_y, og_z, azimuth, elevation)

    #     # left (-) in azimuth, above (+) in elevation from center
    #     upper_left = self.get_point(distance, self.x, self.y, self.z, angle - theta_offset, phi + phi_offset)

    #     # right (+) in azimuth, above (+) in elevation from center 
    #     upper_right = self.get_point(distance, self.x, self.y, self.z, angle + theta_offset, phi + phi_offset) 

    #     # left (-) in azimuth, below (-) in elevation from center
    #     lower_left = self.get_point(distance, self.x, self.y, self.z, angle - theta_offset, phi - phi_offset)

    #     # right (+) in azimuth, below (-) in elevation from center
    #     lower_right = self.get_point(distance, self.x, self.y, self.z, angle + theta_offset, phi - phi_offset)

    #     return [projected_center, upper_left, upper_right, lower_right, lower_left]


    def transform_rectangle(self,points):
        transformed_set = []
        for i in range(0, len(points)):
            cur_point = points[i]
            transformed_point = self.sonar_to_cam_reference(cur_point[0], cur_point[1], cur_point[2])
            transformed_set.append(transformed_point)
            # each points[i] is 3d vector [x,y,z]

        return np.array(transformed_set, np.float32)


    # getting the sonar area's points
    def get_width(self, distance):
        half_theta = self.horizontal_beamwidth / 2.0
        width = 2 * math.sin(math.radians(half_theta)) * distance  # converted to radians
        return width  # in meters

    def get_height(self, distance):
        half_theta = self.vertical_beamwidth / 2.0
        height = 2 * math.sin(math.radians(half_theta)) * distance  # converted to radians
        return height  # in meters



    # we call max intensities first-> max intensity for a distance
    # then we call sonar rectangle sonar_rectangle(self, distance, angle) to get list of 3D points
    # using those 3D points, we call projected points-> then draw them at a corresponding image


    # possible problems: since its using real time, would want to save that max insity distance 


    # goal: given a list of distances, we call sonar_rectangle(self, distance, angle) per distance -> we get 

# # Example usage:

test = SonarProcessor(10)

# timestamp = 1708044554
# intensities = [255, 255, 209, 144, 112, 130, 109, 93, 122, 144, 201, 135, 120, 107, 97, 92, 73, 51, 44, 29, 47, 21, 27, 0, 2, 8, 0, 2, 8, 14, 0, 0, 2, 8, 13, 2, 7, 19, 17, 29, 7, 10, 10, 14, 0, 12, 0, 5, 16, 0, 5, 0, 8, 0, 10, 0, 0, 6, 2, 0, 35, 37, 0, 13, 16, 13, 9, 10, 30, 62, 54, 37, 46, 57, 106, 85, 76, 46, 33, 9, 30, 25, 18, 18, 0, 34, 27, 14, 18, 24, 18, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# test.plot_intensities(intensities, timestamp)


# at phi = 25:
# ('camera angle view no rotation', [194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 206.0, 207.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0, 215.0, 216.0, 219.0, 221.0, 228.0, 229.0, 230.0, 231.0, 232.0, 233.0, 234.0, 235.0, 236.0, 237.0, 238.0, 239.0, 240.0, 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0, 262.0, 263.0, 264.0, 275.0, 277.0, 288.0, 289.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 302.0, 303.0, 304.0, 305.0, 311.0, 314.0, 317.0, 318.0, 319.0, 324.0, 325.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 338.0, 341.0, 346.0, 347.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 362.0, 363.0, 364.0, 366.0, 367.0, 371.0, 377.0, 387.0, 399.0, 400.0])

# at 45, there isn't any



