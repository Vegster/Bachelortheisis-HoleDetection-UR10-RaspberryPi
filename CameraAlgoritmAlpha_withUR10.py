'''

    Bachelorporject Ostold University College 2018 BO18-G11 
    Holedetection algorithm 
    last upate 12.05.2018
    made by Vegard Strand


    This algorithm was made to detect holes on a metall plate and calculate center points
    and send this data to an universial robot 10. coded on a raspberry pi 3 with python 2.7.13

    more information : bo18g11.com/
    
        
'''

#imported libraries 
import skimage
import numpy
import copy
import time
import math
import socket
from picamera import PiCamera
from skimage import io
from scipy import ndimage
from skimage import color
from skimage import feature
from skimage import io
from skimage import img_as_uint
from skimage import exposure




''' ==================
        Global parameters
    =================='''

#Variables for scikit-image filters
ThresholdLevel = 0.4
gaussian_sigma = 4

#Set rapberry pi camera image resolution
image_resolution = [int(700),int(700)]

# varibales holde detection
hole_diameter_in_meters = 0.02
search_lenght = 100
min_stack_size = 90
number_of_backtrackings = 1
min_noise_lenght = 100
max_noise_lenght = 120

#coordinate list that will send data to UR10 robot
coordlist = []

#Variables used to calibrate camera
avarage_diameter_list = []
pixel_accuracy = 0


''' ==================
        Methods
    =================='''

#Simple threshold method
def threshold(thres_image_list, _height, _widht):
    for y in range(_height):
        for x in range(_widht):

            if thres_image_list[y][x] <= ThresholdLevel :
                thres_image_list[y][x] = 0
            else:
                thres_image_list[y][x] = 1

    return thres_image_list


# Selfmade filter by Vegard Strand
def Search_for_hole(image_list, _height, _width):
    # Traversing through image
    for y in range(_height):
        for x in range(_width):

            if image_list[y][x] == 1:
                image_list = Holedetection(image_list, y, x)

    return




def Filter_hole(image_list, search_stack):

    y_sum = 0
    x_sum = 0

    y_farest_up = 9999
    y_farest_down = 0
    x_farest_right  = 9999
    x_farest_left = 0

    #calculate hole diameter
    for i in range(len(search_stack)):
        if search_stack[i][0] < y_farest_up :
            y_farest_up = search_stack[i][0]
            
        if search_stack[i][0] > y_farest_down:
            y_farest_down = search_stack[i][0]

        if search_stack[i][1] < x_farest_right:
            x_farest_right = search_stack[i][1]

        if search_stack[i][1] > x_farest_left :
            x_farest_left = search_stack[i][1]
        
        y_sum = y_sum + search_stack[i][0]
        x_sum = x_sum + search_stack[i][1]
        image_list[search_stack[i][0]][search_stack[i][1]] = 0.6



    #callocate diameter of hole
    gjennomsnitt_diameter = int(math.ceil((y_farest_down - y_farest_up + x_farest_left - x_farest_right )/2))
    avarage_diameter_list.append(gjennomsnitt_diameter)

    #Print to console for debug purposes
    print('average_diameter: ', gjennomsnitt_diameter)
    print('y_sum: ', int(1 + math.ceil(y_sum/len(search_stack))))
    print('x_sum: ', int(1 + math.ceil(x_sum/len(search_stack))))

    #Calculate centerpoints
    y_temp = int(2 + math.ceil(y_sum/len(search_stack)))
    x_temp = int(2 + math.ceil(x_sum/len(search_stack)))

    #Mark hole borders as visited    
    image_list[y_temp][x_temp] = 0.6

    #add centerpoint to coordinate list
    coordlist.append([y_temp,x_temp])


    return image_list

def Holedetection(image_list, y, x):

    # Save startposition
    start_position = [y, x]

    # Search stack
    search_stack = []

    search_y = y
    search_x = x

    search_stack.append([search_y, search_x])

    image_list[y,x] = 0

    backtrack_list = [0,0]

    # Local method for checking if the search loop is outside the boundary of the picture
    def boundary_check(_y, _x):
        if _y < 0 or _x < 0 or (image_list.shape[0] - 1) < _y or (image_list.shape[1] - 1) < _x:
            return False
        else:
            return True

    # Start the search while loop
    searching_bool = True
    
    while searching_bool:

        if(len(backtrack_list) == 0):
            backtrack_list.append([0,0])

        if(len(search_stack) > 6):
            image_list[start_position[0]][start_position[1]] = 1

        #When method have detected a hole
        if search_stack[len(search_stack ) - 1] == start_position and len(search_stack) > min_stack_size and len(search_stack) < max_noise_lenght:
            print('search_stack[len(search_stack ) - 1]: ', search_stack[len(search_stack ) - 1],  'start_position', start_position )
            print('stack_size: ', len(search_stack))
            print('-----------------')
            image_list = Filter_hole(image_list, search_stack)
            break
        
        elif search_stack[len(search_stack ) - 1] == start_position and len(search_stack) > min_noise_lenght:
            image_list[start_position[0]][start_position[1]] = 0
            print('Filtered out: ')
            print('search_stack[len(search_stack ) - 1]: ', search_stack[len(search_stack ) - 1],  'start_position', start_position )
            print('stack_size: ', len(search_stack))
            print('-----------------')

            break


        # Right
        if boundary_check(search_y, search_x + 1):
            if image_list[search_y][search_x + 1] == 1 and not([search_y, search_x + 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y][search_x + 1] = 0
                search_x = search_x + 1
                search_stack.append([search_y, search_x])
                continue

        # Down right
        if boundary_check(search_y + 1, search_x + 1):
            if image_list[search_y + 1][search_x + 1] == 1 and not([search_y + 1, search_x + 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list)> 0 :
                    backtrack_list = []
                image_list[search_y + 1][search_x + 1] = 0
                search_y = search_y + 1
                search_x = search_x + 1
                search_stack.append([search_y, search_x])
                continue

        # Down
        if boundary_check(search_y + 1, search_x):
            if image_list[search_y + 1][search_x] == 1 and not([search_y + 1, search_x] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y + 1][search_x] = 0
                search_y = search_y + 1
                search_stack.append([search_y, search_x])
                continue

        # DownLeft
        if boundary_check(search_y + 1, search_x - 1):
            if image_list[search_y + 1][search_x - 1] == 1 and not([search_y + 1, search_x - 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y + 1][search_x - 1] = 0
                search_y = search_y + 1
                search_x = search_x - 1
                search_stack.append([search_y, search_x])
                continue

        # Left
        if boundary_check(search_y, search_x - 1):
            if image_list[search_y][search_x - 1] == 1 and not([search_y, search_x - 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y][search_x - 1] = 0
                search_x = search_x - 1
                search_stack.append([search_y, search_x])
                continue

        # UpLeft
        if boundary_check(search_y - 1, search_x - 1):
            if image_list[search_y - 1][search_x - 1] == 1 and not([search_y - 1, search_x - 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y - 1][search_x - 1] = 0
                search_y = search_y - 1
                search_x = search_x - 1
                search_stack.append([search_y, search_x])
                continue


        # Up
        if boundary_check(search_y - 1, search_x):
            if image_list[search_y - 1][search_x] == 1 and not([search_y - 1, search_x] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y - 1][search_x] = 0
                search_y = search_y - 1
                search_stack.append([search_y, search_x])
                continue

        # Up right
        if boundary_check(search_y - 1, search_x + 1):
            if image_list[search_y - 1][search_x + 1] == 1 and not([search_y - 1, search_x + 1] == backtrack_list[len(backtrack_list) - 1]):
                if len(backtrack_list) > 0:
                    backtrack_list = []
                image_list[search_y - 1][search_x + 1] = 0
                search_y = search_y - 1
                search_x = search_x + 1
                search_stack.append([search_y, search_x])
                continue


        #backtracking
        if len(backtrack_list) <= number_of_backtrackings and len(search_stack)> number_of_backtrackings:
            search_x = search_stack[len(search_stack) - 1][1]
            search_y = search_stack[len(search_stack) - 1][0]
            backtrack_list.append(search_stack.pop())
            image_list[search_stack[len(search_stack) - 1][0]][search_stack[len(search_stack) - 1][1]] = 1
            continue

        image_list[start_position[0]][start_position[1]] = 0

        searching_bool = False

    return image_list




''' ==================
        Code starts here!
    =================='''

#Take picture with raspberry pi camera and save picture on disk
camera = PiCamera()
camera.vflip = True
camera.hflip = True
camera.resolution = (image_resolution[0],image_resolution[1])
camera.capture('/home/pi/Documents/bilder/img.jpg')

#Open picture from disk
image = skimage.img_as_float(color.rgb2gray(io.imread('bilder/img.jpg')))

#Gets height and width of picture
height, width = image.shape

#Uses scikit-image histogram equalization
image_eq = exposure.equalize_hist(image)
#Save image to disk, only to visualize effect of filter for debug purposes
io.imsave('bilder/exposure_equlize.png', image_eq)

#Uses scikit-image gaussian filter
filtered_image = ndimage.gaussian_filter(image_eq, sigma=gaussian_sigma)
#Save image to disk, only to visualize effect of filter for debug purposes
io.imsave('bilder/gaussian_filtered.png', filtered_image)

#Uses scikit-image canny filter
canny_filtered = feature.canny(filtered_image)
#Save image to disk canny image changes the array structure of image array, easiest to save to disk and reload the image
io.imsave('bilder/canny_filtered_out_img.png', img_as_uint(canny_filtered))


#reread the image
canny_filtered_in = skimage.img_as_float(color.rgb2gray(io.imread('bilder/canny_filtered_out_img.png')))
#Threshold the image so all white pixels are 1
threshold(canny_filtered, height, width)

#Start the search_for_hole_algorithm
Search_for_hole(canny_filtered_in, image.shape[0], image.shape[1])
#Save image to disk, only to visualize effect of filter for debug purposes
io.imsave('bilder/result.png', img_as_uint(canny_filtered_in))
    

#Transfer listCorrdiantes in image coordinate system to coordinate system where center is zero
for i in range(len(coordlist)):
    coordlist[i] = [ -1 * (coordlist[i][0] - int(height/2)), (coordlist[i][1] - int(width/2))] 

print(coordlist)

#Transfer avarage_diameter_list to find pixel accuracy. Uses metallplate holes with a fixed diameter of 2 cm and callibarte camera with that fixed value
sum_avarage = 0
for i in range(len(avarage_diameter_list) - 1):
    sum_avarage = avarage_diameter_list[i] + sum_avarage


#Calibrate pixel accuracy
pixel_accuracy = (hole_diameter_in_meters/( sum_avarage / (len(avarage_diameter_list) - 1))) 

#Print to log for debug purposes
print('pixel accuracy in meters: ', pixel_accuracy)
print('pixel accuracy in millimeters: ', pixel_accuracy * 1000)

user_input = raw_input("Continue to robot communication press y ")

if user_input == 'y' : 
    print("run data communication!")

    #variables for socket communication with UR10 robot
    #Set raspberry pi ip adress here
    TCP_IP = 'xxx.xxx.xxx.xxx'
    TCP_PORT = 5005
    BUFFER_SIZE = 50

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)

    #send coordinate length and pixel accuracy as first datapages
    config_data = str('('+str(len(coordlist))+','+str(pixel_accuracy)+')+\n')
    first_run = True

    #Count number of coordinates raspberry pi and robot is working on at the moment
    i = 0

    print('listening...')
    #wating for communication
    conn, addr = s.accept()
    print ('Connection address: ', addr)
    while 1:
        if first_run: 
            data = conn.recv(BUFFER_SIZE)
            if not data: break;
            #print out recived data from UR10
            print ('recived data:', config_data)
            conn.send(config_data.encode('ascii'))
            first_run = False
        else:
            data = conn.recv(BUFFER_SIZE)
            if not data: break;
            #print out recived data from UR10
            print ('UR10 starts subprogram for hole number: ', data)
            #sends coodinates to UR10 robot
            dataToSend = str('('+str(coordlist[i][1])+','+str(coordlist[i][0])+','+str(0)+')+\n')
            #encode datapages as ascii string
            conn.send(dataToSend.encode('ascii'))
            i = i + 1
            if i == len(coordlist):
                break
    #Ends communication with UR10        
    conn.close()

    
else:
    print('exit program!')

