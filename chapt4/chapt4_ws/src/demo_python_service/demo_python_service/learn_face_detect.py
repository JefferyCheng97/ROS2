# 用于检测图像中人脸的位置、识别人脸的特征以及进行人脸比对等操作
import face_recognition
import cv2
# 获取功能包share目录的绝对路径
from ament_index_python.packages import get_package_share_directory
# 提供与操作系统交互的各种功能，像文件和目录操作等
import os

def main():
    # 方法1：获取图片绝对路径
    # default_image_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
    # 方法2：使用os，后面不能是/resource/default.jpg，不加/
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')

    # 使用cv2加载图片
    image = cv2.imread(default_image_path)

    # 检测人脸，返回人脸坐标值
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample = 1, model = 'hog')

    # 绘制人脸边框，通过左上角顶点坐标和右下角顶点坐标
    for top,right,bottom,left in face_locations:
        cv2.rectangle(image, (left, top), (right,bottom), (255,0,0), 4)
    
    # 显示结果
    cv2.imshow('Face detect result', image)
    
    # 等待用户按键操作，参数0表示程序会一直等待，直到用户按下任意键为止
    cv2.waitKey(0)