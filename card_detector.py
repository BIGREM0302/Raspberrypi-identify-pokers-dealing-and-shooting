import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from picamera2 import Picamera2
from libcamera import Transform

picam2=Picamera2()

img_width=420*2
img_height=308*2

camera_config = picam2.create_still_configuration(
	main={"size": (1680, 1232), "format": "RGB888"},
	controls={"ExposureTime":50000, "AnalogueGain": 1.0},
	transform=Transform(vflip=True, hflip=True)
)

#some parameters
threshvalue=200
black_value=133
black_value_suit=150
x=675
y=199
w=1078-740
h=685-242
x_=733
y_=630
w_=1005-738
h_=1008-664

#debug photo
def show(img):
    '''
    while True:
        
        cv2.imshow('name',cv2.resize(img, (img_width, img_height)))
        key=cv2.waitKey(1)
        if key == 27:
            break
    '''
    return


# 讀取模板圖像
def load_templates(template_folder):
    templates = {}
    for filename in os.listdir(template_folder):
        if filename.endswith(".png"):
            template_name = filename[:-4]  # 去掉文件擴展名
            #print(template_name)
            templates[template_name] = cv2.imread(os.path.join(template_folder, filename), 0)
    return templates

# 匹配模板
def match_template(image, templates):
    print("Start each matching")
    best_match = None
    best_match_value = float('inf')
    for template_name, template in templates.items():
        # Resize template if necessary
        #if template.shape[0] > image.shape[0] or template.shape[1] > image.shape[1]:
        #scale_percent = min(image.shape[0] / template.shape[0], image.shape[1] / template.shape[1])
        #width = int(template.shape[1] * scale_percent)
        #height = int(template.shape[0] * scale_percent)
        #dim = (width, height)
        #template = cv2.resize(template, dim, interpolation=cv2.INTER_AREA)
        #match template
        img=cv2.imread("temp.jpg")
        img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        res = cv2.matchTemplate(img, template, cv2.TM_SQDIFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        bottom_right = (top_left[0] + template.shape[1], top_left[1] + template.shape[0])
        img_copy = image.copy()
        cv2.rectangle(img_copy, top_left, bottom_right, (0, 255, 0), 2)
        '''
        show the source and the result
        plt.subplot(121)
        plt.imshow(cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB))
        plt.title('Detected Point')
        plt.xticks([]), plt.yticks([])
        plt.subplot(122)
        plt.imshow(cv2.cvtColor(template, cv2.COLOR_BGR2RGB))
        plt.title('Template')
        plt.xticks([]), plt.yticks([])
        plt.show()
	'''
        if min_val < best_match_value:
            best_match_value = min_val
            best_match = template_name
    return best_match

def recognize_card(image_path, template_folder):
    print("Start recognize")
    image = cv2.imread(image_path)
    show(image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 110, 60])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 110, 60])
    upper_red2 = np.array([180, 255, 255])
    
    # 假設牌的左上角包含數字和花色
    rank_roi = hsv[y:y+h,x:x+w]
    show(rank_roi)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, black_value])
    mask_red1 = cv2.inRange(rank_roi, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(rank_roi, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_black = cv2.inRange(rank_roi, lower_black, upper_black)

    # Combine the masks for red and black regions
    mask_combined = cv2.bitwise_or(mask_red, mask_black)

    # Invert the mask to get regions that are not red and not black
    mask_not_red_black = cv2.bitwise_not(mask_combined)

    # Create a white image
    white_image_rank = np.ones_like(rank_roi) * 255

    # Create a black image
    black_image_rank = np.zeros_like(rank_roi)

    # Apply the inverted mask to make the non-red and non-black regions white
    result_rank = np.where(mask_not_red_black[:, :, np.newaxis] == 255, white_image_rank, rank_roi)

    # Apply the red mask to make the red regions black
    rank_roi = np.where(mask_combined[:, :, np.newaxis] == 255, black_image_rank, result_rank)
    show(rank_roi)
    #rank_roi = tuple(index.astype(np.unit8) for index in rank_roi)

    suit_roi = hsv[y_:y_+h_,x_:x_+w_]
    upper_black = np.array([180, 255, black_value_suit])
    show(suit_roi)

    mask_red1 = cv2.inRange(suit_roi, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(suit_roi, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    # Create mask for the black color
    mask_black = cv2.inRange(suit_roi, lower_black, upper_black)

    # Combine the masks for red and black regions
    mask_combined = cv2.bitwise_or(mask_red, mask_black)

    # Invert the mask to get regions that are not red and not black
    mask_not_red_black = cv2.bitwise_not(mask_combined)

    # Create a white image
    white_image_suit = np.ones_like(suit_roi) * 255

    # Create a black image
    black_image_suit = np.zeros_like(suit_roi)

    # Apply the inverted mask to make the non-red and non-black regions white
    result_suit = np.where(mask_not_red_black[:, :, np.newaxis] == 255, white_image_suit, suit_roi)

    # Apply the red mask to make the red regions black
    suit_roi = np.where(mask_combined[:, :, np.newaxis] == 255, black_image_suit, result_suit)
    show(suit_roi)
    
    # 加載模板
    print("load the templates")
    templates = load_templates(template_folder)

    #if len(rank_roi.shape) == 2 or rank_roi.shape[2] == 1:
        #rank_roi = cv2.cvtColor(rank_roi, cv2.COLOR_GRAY2BGR)
    #if len(suit_roi.shape) == 2 or suit_roi.shape[2] == 1:
        #suit_roi = cv2.cvtColor(suit_roi, cv2.COLOR_GRAY2BGR)
    # 匹配數字和花色
    print("Seperate rand and suit")
    cv2.imwrite("temp.jpg",rank_roi)
    rank = match_template(rank_roi, {k: v for k, v in templates.items() if 'rank' in k})
    cv2.imwrite("temp.jpg",suit_roi)
    suit = match_template(suit_roi, {k: v for k, v in templates.items() if 'suit' in k})

    return rank, suit

def card_identify():
    picam2.configure(camera_config)
    picam2.start()
    picam2.capture_file("test.jpg")
    picam2.stop()

    path = os.path.dirname(os.path.abspath(__file__))
    print(path)
    image_path = os.path.join(path, 'test.jpg')
    print(image_path)

    template_folder = path + '/Card_Imgs'
    rank, suit = recognize_card(image_path, template_folder)
    r=0
    if rank == "rank_ace":
        r=1
    elif rank == "rank_two":
        r=2
    elif rank == "rank_three":
        r=3
    elif rank == "rank_four":
        r=4
    elif rank == "rank_five":
        r=5
    elif rank == "rank_six":
        r=6
    elif rank == "rank_seven":
        r=7
    elif rank == "rank_eight":
        r=8
    elif rank == "rank_nine":
        r=9
    elif rank == "rank_ten":
        r=10
    elif rank == "rank_jack":
        r=11
    elif rank == "rank_queen":
        r=12
    else:
        r=13
    s=0
    if suit == "suit_heart":
        s=0
    elif suit == "suit_diamond":
        s=1
    elif suit == "suit_club":
        s=2
    else:
        s=3 
    print(f'Recognized card: {rank} of {suit}')
    #cv2.destroyAllWindows()
    return 13*s+r-1


card_identify()
