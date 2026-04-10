import rosbag
import cv2, os
from cv_bridge import CvBridge

# realsense
# DataBag_2026-04-08-17-42-25  
# DataBag_2026-04-08-17-44-50  
# DataBag_2026-04-08-17-49-25  
# DataBag_2026-04-08-17-52-41  
# DataBag_2026-04-08-17-55-17  
# DataBag_2026-04-09-10-09-06  
# DataBag_2026-04-09-10-11-17  
# DataBag_2026-04-09-10-14-25  

# fisheye
# DataBag_2026-04-09-10-20-34  
# DataBag_2026-04-09-10-22-54  
# DataBag_2026-04-09-10-25-55
# DataBag_2026-04-09-11-07-33
# DataBag_2026-04-09-11-12-40

# --- Configuration ---
BAG_PATH = './calib_data/DataBag_2026-04-09-11-12-40/'

BAG_FILE = 'data.bag'
#TOPIC = '/camera/color/image_raw/compressed'  # compressed topic by realsense
TOPIC = '/right_camera/image/compressed'  # compressed topic by fisheye

OUTPUT_NAME = 'image.png'

def save_first_frame():
    bridge = CvBridge()
    BAG_FILE_PATH = os.path.join(BAG_PATH, BAG_FILE)

    with rosbag.Bag(BAG_FILE_PATH, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            try:
                # Use compressed_imgmsg_to_cv2 for CompressedImage types
                # You do not need to specify a 'desired_encoding' here usually
                cv_img = bridge.compressed_imgmsg_to_cv2(msg)
                
                OUTPUT_FILE_PATH = os.path.join(BAG_PATH, OUTPUT_NAME)
                cv2.imwrite(OUTPUT_FILE_PATH, cv_img)
                print(f"Successfully saved compressed frame to {OUTPUT_FILE_PATH}")
                break 
            except Exception as e:
                print(f"Failed to convert image: {e}")
                break

if __name__ == '__main__':
    save_first_frame()