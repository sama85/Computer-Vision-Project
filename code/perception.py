from collections import namedtuple

import numpy as np
import cv2


def rock_thresh(img, low_levels=(130, 105, 0),high_levels=(220,190,70)):
    rockpix = ( (img[:,:,0] > low_levels[0])\
            & (img[:,:,1] > low_levels[1])  \
            & (img[:,:,2] > low_levels [2]) \
            & (img[:,:,0] < high_levels [0])\
            & (img[:,:,1] < high_levels[1]) \
            & (img[:,:,2] < high_levels [2]))

    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1
    return color_select


def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))

    return warped, mask


def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float32)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float32)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel using pythagoream theorem
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel using arc tangent
    rad2deg = 180./np.pi
    angles = np.arctan2(y_pixel, x_pixel)*rad2deg
    return dist, angles


def rotate_pixels(pixels, angle):
    
    deg2rad = np.pi/180.
    angle_rad = angle*deg2rad
    x_pixels, y_pixels = pixels

    x_pixels_rotated = x_pixels*np.cos(angle_rad) - y_pixels*np.sin(angle_rad)
    y_pixels_rotated = x_pixels*np.sin(angle_rad) + y_pixels*np.cos(angle_rad)

    return x_pixels_rotated,y_pixels_rotated

def inv_rotate_pixels(pixels, angle):
    
    deg2rad = np.pi/180.
    angle_rad = angle*deg2rad
    x_pixels, y_pixels = pixels

    x_pixels_rotated = x_pixels*np.cos(angle_rad) + y_pixels*np.sin(angle_rad)
    y_pixels_rotated = -x_pixels*np.sin(angle_rad) + y_pixels*np.cos(angle_rad)

    return x_pixels_rotated,y_pixels_rotated

def translate_pixels(pixels, translation, scale_factor=10):
    
    translation_x, translation_y = translation
    x_pixels, y_pixels = pixels
    x_pixels_translated = x_pixels/scale_factor + translation_x
    y_pixels_translated = y_pixels/scale_factor + translation_y

    return x_pixels_translated,y_pixels_translated

def inv_translate_pixels(pixels, translation, scale_factor=10):
    
    translation_x, translation_y = translation
    x_pixels, y_pixels = pixels

    x_pixels_translated = (x_pixels - translation_x)*scale_factor
    y_pixels_translated = (y_pixels - translation_y)*scale_factor

    return x_pixels_translated,y_pixels_translated

def pix_to_world(pixels_rover, rover_pos, rover_yaw, world_size=200):
    
    # Apply rotation and translation
    pixels_rotated = rotate_pixels(pixels_rover, rover_yaw)
    pixels_translated_x, pixels_translated_y = translate_pixels(pixels_rotated, rover_pos)

    # Clip pixels to be within world size
    x_pixels_world = np.clip(np.int_(pixels_translated_x), 0, world_size-1)
    y_pixels_world = np.clip(np.int_(pixels_translated_y), 0, world_size-1)

    return x_pixels_world, y_pixels_world


def pix_to_rover(pixels_world, rover_pos, rover_yaw):
    
    # Apply inverse translation and rotation
    pixels_rotated = inv_translate_pixels(pixels_world, rover_pos)
    pixels_rover = inv_rotate_pixels(pixels_rotated, rover_yaw)

    return pixels_rover


def perception_step(Rover, R=0, G=1, B=2):
    
    img = Rover.img
    dst_size = 5
    bottom_offset = 6

    src = np.float32([[14, 140], [300, 140], [200, 96], [118, 96]])
    dst = np.float32([
        [img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
        [img.shape[1] / 2 + dst_size, img.shape[0] - bottom_offset],
        [img.shape[1] / 2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
        [img.shape[1] / 2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset]])
    # Apply perspective transform to get 2D overhead view of rover cam
    warped_img , mask = perspect_transform(img=img, src=src, dst=dst)

    Rover.vision_warped = warped_img
    Rover.vision_mask = mask
    # Apply color thresholds to extract pixels of navigable/obstacles/rocks
    navigable_pixels = color_thresh(warped_img)
    Rover.vision_threshed = navigable_pixels
    obstacle_pixels = np.abs(np.float32(navigable_pixels) - 1) * mask
    rock_pixels = rock_thresh(img=warped_img)

    # Update rover vision image with each ROI assigned to one of
    # the RGB color channels (to be displayed on left side of sim screen)
    Rover.vision_image[:, :, R] = obstacle_pixels * 255
    Rover.vision_image[:, :, G] = rock_pixels * 255
    Rover.vision_image[:, :, B] = navigable_pixels * 255

    # Transform pixel coordinates from perspective frame to rover frame
    x_nav, y_nav = rover_coords(binary_img=navigable_pixels)
    x_obs, y_obs = rover_coords(binary_img=obstacle_pixels)
    x_rock, y_rock = rover_coords(binary_img=rock_pixels)

    Rover.x_nav = x_nav
    Rover.y_nav = y_nav

    # Convert above cartesian coordinates to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords( x_nav, y_nav)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(x_obs, y_obs)
    Rover.rock_dists = to_polar_coords(x_rock, y_rock)[0]
    # Extract subset of nav_angles that are left of rover angle
    Rover.nav_angles_left = Rover.nav_angles[Rover.nav_angles > 0]

    # Only include pixels within certain distances from rover (for fidelity)
    nav_pixels_rover = [pts[Rover.nav_dists < 60] for pts in (x_nav, y_nav)]
    obs_pixels_rover = [pts[Rover.obs_dists < 80] for pts in (x_obs, y_obs)]
    rock_pixels_rover = [pts[Rover.rock_dists < 70] for pts in (x_rock, y_rock)]

    # Convert rock cartesian coords to polar coords
    rock_pixels_rover_x, rock_pixels_rover_y = rock_pixels_rover
    Rover.rock_angles = to_polar_coords(rock_pixels_rover_x, rock_pixels_rover_y)[1]

    # Transform pixel points of ROIs from rover frame to world frame
    nav_pixels_world_x,nav_pixels_world_y = pix_to_world(nav_pixels_rover, Rover.pos, Rover.yaw)
    obs_pixels_world_x,obs_pixels_world_y = pix_to_world(obs_pixels_rover, Rover.pos, Rover.yaw)
    rock_pixels_world_x,rock_pixels_world_y = pix_to_world(rock_pixels_rover, Rover.pos, Rover.yaw)

    # Only update worldmap (displayed on right) if rover has a stable drive
    # High pitch/rolls cause inaccurate 3D to 2D mapping and low fidelity
    is_stable = ((Rover.pitch > 359 or Rover.pitch < 0.25)
                 and (Rover.roll > 359 or Rover.roll < 0.37))
    
    if is_stable:  # Update map with each ROI assigned to an RGB color channel
        Rover.worldmap[obs_pixels_world_y, obs_pixels_world_x, R] = 255
        Rover.worldmap[rock_pixels_world_y, rock_pixels_world_x, G] = 255
        Rover.worldmap[nav_pixels_world_y, nav_pixels_world_x, B] = 255

    return Rover
