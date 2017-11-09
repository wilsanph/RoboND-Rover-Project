import numpy as np
import math
import cv2
from NavigationStructures import *

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_obstacles( img, rgb_thresh=( 160, 160, 160 ) ) :
    color_select = np.zeros_like( img[:, :, 0] )
    below_thresh = ( img[:, :, 0] < rgb_thresh[0] ) \
                 & ( img[:, :, 1] < rgb_thresh[1] ) \
                 & ( img[:, :, 2 ] < rgb_thresh[2] )
    color_select[below_thresh] = 1
    return color_select

def color_thresh_samples( img, 
                          min_hsv_threshold = np.array( [18, 200, 100] ), 
                          max_hsv_threshold = np.array( [28, 255, 255] ) ) :
    img_hsv = cv2.cvtColor( img, cv2.COLOR_RGB2HSV )
    color_select = cv2.inRange( img_hsv, min_hsv_threshold, max_hsv_threshold )
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

def to_rover_coords( x, y, dh ) :
    _x = np.absolute( y - dh )
    _y = -( x - dh )
    return _x, _y

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw = np.radians( yaw )
    # Apply a rotation
    xpix_rotated = np.cos( yaw ) * xpix - np.sin( yaw ) * ypix
    ypix_rotated = np.sin( yaw ) * xpix + np.cos( yaw ) * ypix
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = np.int_( xpos + ( xpix_rot / scale ) )
    ypix_translated = np.int_( ypos + ( ypix_rot / scale ) )
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    _delta = 10
    _bottom_offset = 6
    _world_pix_scale = 10
    ptsSrc = np.float32( [[14, 140], [301 ,140],[200, 96], [118, 96]] )
    ptsDst = np.float32( [[Rover.img.shape[1]/2 - _delta / 2, Rover.img.shape[0] - _bottom_offset],
                          [Rover.img.shape[1]/2 + _delta / 2, Rover.img.shape[0] - _bottom_offset],
                          [Rover.img.shape[1]/2 + _delta / 2, Rover.img.shape[0] - _delta - _bottom_offset], 
                          [Rover.img.shape[1]/2 - _delta / 2, Rover.img.shape[0] - _delta - _bottom_offset]
                         ])
    # 2) Apply perspective transform
    _img_warped = perspect_transform( Rover.img, ptsSrc, ptsDst )
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    _img_thresh_nav = color_thresh( _img_warped )
    _img_thresh_obs = color_thresh_obstacles( _img_warped )
    _img_thresh_rock = color_thresh_samples( _img_warped )

    # 3.5) Find a rock
    cnts = cv2.findContours( _img_thresh_rock, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[-2]

    if len( cnts ) > 0 :

        c = max( cnts, key = cv2.contourArea )
        ( ( x, y ), radius ) = cv2.minEnclosingCircle( c )
        if radius > 5 :
            pt_obs = np.array([[x, y]])
            pt_obs = np.array([pt_obs])
            M = cv2.getPerspectiveTransform( ptsSrc, ptsDst )
            pt_obs_warp = ( cv2.perspectiveTransform( pt_obs, M ) )[0][0]

            Rover.sample_in_range['exists'] = True
            Rover.sample_in_range['position'][0] = pt_obs_warp[0]
            Rover.sample_in_range['position'][1] = pt_obs_warp[1]
        else :
            Rover.sample_in_range['exists'] = False
    else :
        Rover.sample_in_range['exists'] = False

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = _img_thresh_obs
    Rover.vision_image[:, :, 1] = _img_thresh_rock
    Rover.vision_image[:, :, 2] = _img_thresh_nav

    # 5) Convert map image pixel values to rover-centric coords
    _xpix_nav, _ypix_nav = rover_coords( _img_thresh_nav )
    _xpix_obs, _ypix_obs = rover_coords( _img_thresh_obs )
    _xpix_rock, _ypix_rock = rover_coords( _img_thresh_rock )

    # 5.5) Do the same for the sample position
    if Rover.sample_in_range['exists'] :
        _x, _y = to_rover_coords( Rover.sample_in_range['position'][0],
                                  Rover.sample_in_range['position'][1],
                                  _img_thresh_nav.shape[0] )
        Rover.sample_in_range['position'][0] = _x
        Rover.sample_in_range['position'][1] = _y

        _xx, _yy = pix_to_world( np.array( [ Rover.sample_in_range['position'][0] ] ),
                                 np.array( [ Rover.sample_in_range['position'][1] ] ), 
                                 Rover.pos[0], 
                                 Rover.pos[1], 
                                 Rover.yaw, 
                                 Rover.worldmap.shape[0], _world_pix_scale )
        ## print( '_xx, _yy: ', _xx, _yy )
        Rover.sample_in_range['position'][0] = _xx[0]
        Rover.sample_in_range['position'][1] = _yy[0]

        ## print( "found rock at: ", 
        ##        Rover.sample_in_range['position'][0], 
        ##        Rover.sample_in_range['position'][1] )
        ## print( "Rover.samples_pos: ", Rover.samples_pos )
        ## print( "Rover.pos: ", Rover.pos )

        ## check if hasn't been found yet
        isNewSample = True
        for q in range( len( Rover.samples_found_pos ) ) :
            dx = Rover.sample_in_range['position'][0] - Rover.samples_found_pos[q][0]
            dy = Rover.sample_in_range['position'][1] - Rover.samples_found_pos[q][1]
            dist = math.sqrt( dx ** 2 + dy ** 2 )
            if dist < 3 :
                isNewSample = False
                ## print( 'Already found' )
                break

        if isNewSample :
            _xySample = ( Rover.sample_in_range['position'][0],
                          Rover.sample_in_range['position'][1] )
            Rover.samples_found_pos.append( _xySample )
            print( 'Added new sample at: ', _xySample )
            Rover.samples_found += 1

    # 6) Convert rover-centric pixel values to world coordinates
    _x_world_nav, _y_world_nav = pix_to_world( _xpix_nav, _ypix_nav, 
                                               Rover.pos[0], 
                                               Rover.pos[1], 
                                               Rover.yaw, 
                                               Rover.worldmap.shape[0], _world_pix_scale )
    _x_world_obs, _y_world_obs = pix_to_world( _xpix_obs, _ypix_obs, 
                                               Rover.pos[0], 
                                               Rover.pos[1], 
                                               Rover.yaw, 
                                               Rover.worldmap.shape[0], _world_pix_scale )
    _x_world_rock, _y_world_rock = pix_to_world( _xpix_rock, _ypix_rock, 
                                                 Rover.pos[0], 
                                                 Rover.pos[1], 
                                                 Rover.yaw, 
                                                 Rover.worldmap.shape[0], _world_pix_scale )
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    _isRollOk = ( Rover.roll < 2 and Rover.roll >= 0 ) or ( Rover.roll <= 360 and Rover.roll > 358 )
    _isPitchOk = ( Rover.pitch < 2 and Rover.pitch >= 0 ) or ( Rover.pitch <= 360 and Rover.pitch > 358 )
    if _isRollOk and _isPitchOk :
        Rover.worldmap[_y_world_obs, _x_world_obs, 0] += 1
        Rover.worldmap[_y_world_rock, _x_world_rock, 1] += 1
        Rover.worldmap[_y_world_nav, _x_world_nav, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    _dists, _angles = to_polar_coords( _xpix_nav, _ypix_nav )
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = _dists
    Rover.nav_angles = _angles

    # updating navigation structures
    Rover.navigationPath.update( Rover.time_struct['delta'], Rover )

    return Rover