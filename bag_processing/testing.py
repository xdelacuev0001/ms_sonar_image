import matplotlib.pyplot as plt
import numpy as np
import math
import tf

# sonar translations from camera
x_translation = 13
y_translation = -0.15
z_translation = -0.18


def convert_to_cam_reference(x, y,z):
        # Map origin from sonar origin in refernce to camera frame
        trans = [x_translation, y_translation, z_translation]

        # converting my euler angles to quaternion -> [0,0,0] for roll, pitch, yaw since no initial orientation change
        initial_ors = [0, 0, 0]
        rot = tf.transformations.quaternion_from_euler(initial_ors[0], initial_ors[1], initial_ors[2])

        # making my transofrmation matrix for occupancy grid in reference to odom
        t = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)


        # Transformation: sonar reference to camera
        son_T_cam = t.dot(R) # sonar point in reference to camera-> Using this
        # cam_T_son = np.linalg.inv(o_T_om) # camera point in reference to sonar

        sonar_frame_vector = [x, y, z, 1]
        sonar_frame_vector = np.array(sonar_frame_vector) 


        cam_frame_vector = son_T_cam.dot(sonar_frame_vector)

        # gives a new x, y, z in terms of camera frame
        return (cam_frame_vector[0], cam_frame_vector[1], cam_frame_vector[2])


print("Sample if sonar reads at 0,0, 1", convert_to_map_reference(0,0,1))


print("0 phi",len([194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0, 215.0, 216.0, 217.0, 218.0, 219.0, 220.0, 221.0, 222.0, 223.0, 224.0, 225.0, 226.0, 227.0, 228.0, 229.0, 230.0, 231.0, 232.0, 233.0, 234.0, 235.0, 236.0, 237.0, 238.0, 239.0, 240.0, 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0, 262.0, 263.0, 264.0, 265.0, 266.0, 267.0, 268.0, 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0, 305.0, 306.0, 307.0, 308.0, 309.0, 310.0, 311.0, 312.0, 313.0, 314.0, 315.0, 316.0, 317.0, 318.0, 319.0, 320.0, 321.0, 322.0, 323.0, 324.0, 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 337.0, 338.0, 339.0, 340.0, 341.0, 342.0, 343.0, 344.0, 345.0, 346.0, 347.0, 348.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0, 370.0, 371.0, 372.0, 373.0, 374.0, 375.0, 376.0, 377.0, 378.0, 379.0, 380.0, 381.0, 382.0, 383.0, 384.0, 385.0, 386.0, 387.0, 388.0, 389.0, 390.0, 391.0, 392.0, 393.0, 394.0, 395.0, 396.0, 397.0, 398.0, 399.0, 400.0]))
print("25 phi", len([194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 206.0, 207.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0, 215.0, 216.0, 219.0, 221.0, 228.0, 229.0, 230.0, 231.0, 232.0, 233.0, 234.0, 235.0, 236.0, 237.0, 238.0, 239.0, 240.0, 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0, 262.0, 263.0, 264.0, 275.0, 277.0, 288.0, 289.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 302.0, 303.0, 304.0, 305.0, 311.0, 314.0, 317.0, 318.0, 319.0, 324.0, 325.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 338.0, 341.0, 346.0, 347.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 362.0, 363.0, 364.0, 366.0, 367.0, 371.0, 377.0, 387.0, 399.0, 400.0]))