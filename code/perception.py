import time
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


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
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


def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel using pythagoream theorem
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel using arc tangent
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw degrees to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a matrix rotation counter-clockwise by the yaw radians
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpos + (xpix_rot / scale))
    ypix_translated = (ypos + (ypix_rot / scale))
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale=10):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix=xpix, ypix=ypix, yaw=yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot=xpix_rot, ypix_rot=ypix_rot,
                                         xpos=xpos, ypos=ypos, scale=scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world




# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))

    return warped, mask

# # Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Camera image from the current Rover state (Rover.img)
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    # Define calibration box in source and destination coordintates.
    #   These source and destination points are defined to warp the image
    #   to a grid where each 10x10 pixel square represents 1 square meter
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    #   is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [300, 140], [200, 96], [118, 96]])
    dst = np.float32([
        [img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
        [img.shape[1] / 2 + dst_size, img.shape[0] - bottom_offset],
        [img.shape[1] / 2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
        [img.shape[1] / 2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset]])

    # 2) Apply perspective transform
    warped, mask = perspect_transform(img=img, src=src, dst=dst)
    
    Rover.vision_warped = warped
    Rover.vision_mask = mask

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_pixels = color_thresh(warped)
    Rover.vision_threshed = navigable_pixels
    obstacle_pixels = np.abs(np.float32(navigable_pixels) - 1) * mask
    rock_pixels = rock_thresh(img=warped)

    # 4) Update Rover.vision_image (displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obstacle_pixels * 255
    Rover.vision_image[:, :, 1] = rock_pixels * 255
    Rover.vision_image[:, :, 2] = navigable_pixels * 255

    # 5) Convert map image pixel values to rover centric coordinates
    x_nav, y_nav = rover_coords(binary_img=navigable_pixels)
    x_obs, y_obs = rover_coords(binary_img=obstacle_pixels)
    x_rock, y_rock = rover_coords(binary_img=rock_pixels)
    Rover.x_nav = x_nav
    Rover.y_nav = y_nav

    # 6) Convert rover centric pixel values to world coordinates
    scale = dst_size * 2
    x_rov_pos, y_rov_pos = Rover.pos
    yaw = Rover.yaw
    worldmap_size = Rover.worldmap.shape[0]

    x_nav_world, y_nav_world = pix_to_world(
        xpix=x_nav, ypix=y_nav,
        xpos=x_rov_pos, ypos=y_rov_pos, yaw=yaw, world_size=worldmap_size, scale=scale)
    x_obs_world, y_obs_world = pix_to_world(
        xpix= x_obs, ypix= y_obs,
        xpos=x_rov_pos, ypos=y_rov_pos, yaw=yaw, world_size=worldmap_size, scale=scale)
    x_rock_world, y_rock_world = pix_to_world(
        xpix=x_rock, ypix=y_rock,
        xpos=x_rov_pos, ypos=y_rov_pos, yaw=yaw, world_size=worldmap_size, scale=scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # print(Rover.is_Stable())
    if Rover.is_Stable():
        #Update the Rover worldmap only when the rover is stable
        Rover.worldmap[y_obs_world, x_obs_world, 0] += 1
        Rover.worldmap[y_nav_world, x_nav_world, 2] += 1
        if rock_pixels.any():
            Rover.worldmap[y_rock_world, x_rock_world, :] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dists, angles = to_polar_coords(x_pixel=x_nav, y_pixel=y_nav)

    # Update Rover pixel distances and angles
    Rover.nav_dists = dists
    Rover.nav_angles = angles

    return Rover
