## *********************************************************
##
## File autogenerated for the face_detection package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 290, 'description': 'Subscribe to this image topic', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imageInput', 'edit_method': '', 'default': '/ardrone/image_raw', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 290, 'description': 'Publish to this image topic', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imageOutput', 'edit_method': '', 'default': '/facerec/image_raw', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 290, 'description': 'Skip frames that are used for detection', 'max': 20, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'skipFrames', 'edit_method': '', 'default': 1, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Select a scaling factor for the detection image', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'imgScale', 'edit_method': "{'enum_description': 'The detection image will be resized by this value. (great performance increase).', 'enum': [{'srcline': 18, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 1.0, 'ctype': 'double', 'type': 'double', 'name': 'No_Resize'}, {'srcline': 19, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.875, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_875'}, {'srcline': 20, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.75, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_75'}, {'srcline': 21, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.625, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_625'}, {'srcline': 22, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.5, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_5'}, {'srcline': 23, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.375, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_375'}, {'srcline': 24, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.25, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_25'}, {'srcline': 25, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const double', 'value': 0.125, 'ctype': 'double', 'type': 'double', 'name': 'Resize_by_0_125'}]}", 'default': 0.625, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 290, 'description': 'The number of neiboring detections required for a sucessfull detection', 'max': 8, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'neighborsValue', 'edit_method': '', 'default': 2, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'Multiplicator for each step of the detection', 'max': 1.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'scaleValue', 'edit_method': '', 'default': 1.2, 'level': 0, 'min': 1.01, 'type': 'double'}, {'srcline': 290, 'description': 'Minimum size for the search window.', 'max': 1024, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'minSize', 'edit_method': '', 'default': 40, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Maximum size for the search window.', 'max': 1024, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'maxSize', 'edit_method': '', 'default': 200, 'level': 0, 'min': 5, 'type': 'int'}, {'srcline': 290, 'description': 'Select a Cascade', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'cascadeValue', 'edit_method': "{'enum_description': 'An enum to set size', 'enum': [{'srcline': 43, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'haarcascade_frontalface_alt'}, {'srcline': 44, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'haarcascade_frontalface_alt2'}, {'srcline': 45, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'haarcascade_frontalface_alt_tree'}, {'srcline': 46, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'haarcascade_frontalface_default'}, {'srcline': 47, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'lbpcascade_frontalface'}]}", 'default': 2, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'Select a type of detection', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'myflag', 'edit_method': "{'enum_description': 'An enum to set size', 'enum': [{'srcline': 51, 'description': 'Standart type', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Scale'}, {'srcline': 52, 'description': 'Only returns the biggest detection', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Biggest'}, {'srcline': 53, 'description': 'Reduces Canny pruning to reduce the number of false detections', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Canny'}, {'srcline': 54, 'description': 'Rought Detection Search', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Rough'}]}", 'default': 2, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'Select type of debugging', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'displayed_Image', 'edit_method': "{'enum_description': 'An enum to set debugging', 'enum': [{'srcline': 59, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Display_Nothing'}, {'srcline': 60, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Displays_Image'}, {'srcline': 61, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Displays_Detection_Image'}, {'srcline': 62, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Displays_With_Overlay'}]}", 'default': 3, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'Select what is published', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'publish', 'edit_method': "{'enum_description': 'An enum to set publishing', 'enum': [{'srcline': 66, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'No_Publishing'}, {'srcline': 67, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Publish_Image'}, {'srcline': 68, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Publish_Data'}, {'srcline': 69, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Publish_Image_and_Data'}]}", 'default': 0, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'Select a detection display type', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'pixelSwitch', 'edit_method': "{'enum_description': 'An enum to set pixelSwitch', 'enum': [{'srcline': 73, 'description': 'Draws Detection Boxes', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Show_Detection_Boxes'}, {'srcline': 74, 'description': 'Pixelises the detected Faces', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Pixelise_Faces'}]}", 'default': 0, 'level': 1, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'The contrast factor applied to the image to improve detection', 'max': 2.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'contrastFactor', 'edit_method': '', 'default': 1.5, 'level': 0, 'min': 0.2, 'type': 'double'}, {'srcline': 290, 'description': 'Select a detection display type', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'histOnOff', 'edit_method': "{'enum_description': 'An enum to set pixelSwitch', 'enum': [{'srcline': 83, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'Histogram_Equalisation_OFF'}, {'srcline': 84, 'description': '', 'srcfile': '/home/robotis/christmann_ws/src/face_detection/cfg/face_track.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Histogram_Equalisation_ON'}]}", 'default': 0, 'level': 1, 'min': -2147483648, 'type': 'int'}, {'srcline': 290, 'description': 'blurs the image a little, used to reduce noise on full resolution images (imageScaleValue = 1)', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'blurFactor', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'brightens up the image', 'max': 5, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'brightnessFactor', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': "start skipping frames from the input after X frames, generally use '1' for realtime feedback, or much higher values for saving image sequences to disk", 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'inputSkipp', 'edit_method': '', 'default': 1, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Number of features used to track each face', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'maxNumFeatures', 'edit_method': '', 'default': 15, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Number of times a face is tracked after being detected again', 'max': 200, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'maxTrackingNum', 'edit_method': '', 'default': 15, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Number of times a face is tracked after the first detection', 'max': 10, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'initialDetectionNum', 'edit_method': '', 'default': 3, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 290, 'description': 'Number of times a face is tracked after the first detection', 'max': 200, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'trackSearchWinSize', 'edit_method': '', 'default': 30, 'level': 0, 'min': 5, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

face_track_No_Resize = 1.0
face_track_Resize_by_0_875 = 0.875
face_track_Resize_by_0_75 = 0.75
face_track_Resize_by_0_625 = 0.625
face_track_Resize_by_0_5 = 0.5
face_track_Resize_by_0_375 = 0.375
face_track_Resize_by_0_25 = 0.25
face_track_Resize_by_0_125 = 0.125
face_track_haarcascade_frontalface_alt = 0
face_track_haarcascade_frontalface_alt2 = 1
face_track_haarcascade_frontalface_alt_tree = 2
face_track_haarcascade_frontalface_default = 3
face_track_lbpcascade_frontalface = 4
face_track_Scale = 0
face_track_Biggest = 1
face_track_Canny = 2
face_track_Rough = 3
face_track_Display_Nothing = 0
face_track_Displays_Image = 1
face_track_Displays_Detection_Image = 2
face_track_Displays_With_Overlay = 3
face_track_No_Publishing = 0
face_track_Publish_Image = 1
face_track_Publish_Data = 2
face_track_Publish_Image_and_Data = 3
face_track_Show_Detection_Boxes = 0
face_track_Pixelise_Faces = 1
face_track_Histogram_Equalisation_OFF = 0
face_track_Histogram_Equalisation_ON = 1
