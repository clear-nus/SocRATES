import cv2
import numpy as np
import pickle 
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--img", required=True, help="path to image file")
parser.add_argument("--out", required=True, help="output folder") 
args = parser.parse_args()
# define an image (black) on which the circle to be drawn
img = cv2.imread('/home/shashank/Downloads/room-planning-software.png')
subimage = np.zeros((50, 50, 3), np.uint8)
counter_a = 0
counter_big = 1
counter_small = 'A'
sub_points = []
dictionary = dict()
def draw_circle(event, x, y, flags, param):
    global subimage, sub_points, ix, iy, counter_small
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(subimage, (x, y), 3, (0, 0, 255), -1)
        cv2.putText(subimage, counter_small, (x+5,y+5), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,255), 1)
        counter_small = chr(ord(counter_small)+1)
        sub_loc_x = (x+ix-100)* 0.050000 + -7.000
        sub_loc_y = -1*((y+iy-100) * 0.050000 + -10.500000)
        sub_points.append((sub_loc_x,sub_loc_y))

# define mouse callback function to draw circle
def sub_draw_circle(event, x, y, flags, param):
      global subimage, sub_points, ix, iy, loc_x,loc_y,a,counter_big, counter_small
      if event == cv2.EVENT_LBUTTONDOWN:
         print("Please select from the following node types:")
         print("x: Doorway")
         print("y: Passageway")
         print("z: Intersection")
         print("Type your selection now:")
         a = input()
         print("you have selected "+str(a))
         loc_x = x * 0.050000 + -7.000
         loc_y = -1*(y * 0.050000 + -10.500000)
         ix = x
         iy = y
         cv2.circle(img, (x, y), 3, (255, 0, 0), -1)
         cv2.putText(img, str(counter_big), (x+5,y+5), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,0,0), 1)
         ay = max(y-100, 0)
         by = min(y+100, img.shape[0])
         ax = max(x-100,0)
         bx = min(x+100, img.shape[1])
             
         subimage = img[ay:by, ax:bx].copy()
         counter_small = 'A'
         counter_big = counter_big + 1
         sub_points = []


# Create a window
cv2.namedWindow("Circle Window")

# bind the callback function to the window
cv2.setMouseCallback("Circle Window", sub_draw_circle)

cv2.namedWindow("Sub Circle Window")
cv2.setMouseCallback("Sub Circle Window", draw_circle)

# display the image
while True:
   cv2.imshow("Circle Window", img)
   cv2.imshow("Sub Circle Window", subimage)
   if cv2.waitKey(20) & 0xFF == 27:
      cv2.imwrite("final_image.png",img)
      break
   elif cv2.waitKey(33) == ord('a'):
       cv2.imwrite("image"+str(counter_a)+".png",subimage)
       counter_a = counter_a + 1
       dictionary.update({(loc_x,loc_y,a):sub_points})
cv2.destroyAllWindows()
print(dictionary)
with open('saved_dictionary.pkl', 'wb') as f:
    pickle.dump(dictionary, f)