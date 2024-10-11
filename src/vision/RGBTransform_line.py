import os, sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2

from realsense.realsense_camera import DepthCamera


resolution_width, resolution_height = (1280,  720)


def main():
    rs = DepthCamera(resolution_width, resolution_height)

    while True:

        ret, depth_raw_frame, color_raw_frame = rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())

        cv2.imshow('RealSense Camera', color_frame)
    
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv)

        equalizedV = cv2.equalizeHist(v)

        hsv2 = cv2.merge([h,s,equalizedV])

        hsvDst = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

        yCrCb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)

        y, Cr, Cb = cv2.split(yCrCb)

        

        key = cv2.waitKey(1)

        if key == 27:  # ESC를 누르면 종료
            break

    rs.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

# 이미지 파일 불러오기
image_path = './data/imagecopy.png'  # 여기에 파일 경로를 입력하세요
image = cv2.imread(image_path)
image = cv2.resize(image, (480,640))

# hsv 컬러 형태로 변형합니다.
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# h, s, v로 컬러 영상을 분리 합니다. 
h, s, v = cv2.split(hsv)
# v값을 히스토그램 평활화를 합니다.
equalizedV = cv2.equalizeHist(v)
# h,s,equalizedV를 합쳐서 새로운 hsv 이미지를 만듭니다.
hsv2 = cv2.merge([h,s,equalizedV])
# 마지막으로 hsv2를 다시 BGR 형태로 변경합니다.
hsvDst = cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

# YCrCb 컬러 형태로 변환합니다.
yCrCb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
# y, Cr, Cb로 컬러 영상을 분리 합니다.
y, Cr, Cb = cv2.split(yCrCb)
# y값을 히스토그램 평활화를 합니다.
equalizedY = cv2.equalizeHist(y)
# equalizedY, Cr, Cb를 합쳐서 새로운 yCrCb 이미지를 만듭니다.
yCrCb2 = cv2.merge([equalizedY, Cr, Cb])
# 마지막으로 yCrCb2를 다시 BGR 형태로 변경합니다.
yCrCbDst = cv2.cvtColor(yCrCb2, cv2.COLOR_YCrCb2BGR)
blurred = cv2.GaussianBlur(yCrCbDst, (5, 5), 0)
yCrCbDst = cv2.Canny(blurred, 50, 100)


# src, hsv, YCrCb 각각을 출력합니다.
cv2.imshow('src', image)
cv2.imshow('hsv dst', hsvDst)
cv2.imshow('YCrCb dst', yCrCbDst)
cv2.waitKey()
cv2.destroyAllWindows()

# Hough Line Transform을 사용하여 가로선 검출
lines = cv2.HoughLinesP(yCrCbDst, rho=1, theta=np.pi/180, threshold=50, minLineLength=100, maxLineGap=10)

# 검출된 선을 원본 이미지에 그리기
for line in lines:
    x1, y1, x2, y2 = line[0]
    # y1 == y2인 경우 가로선임을 의미함
    if abs(y1 - y2) < 5:
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

# 결과 시각화
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Horizontal Lines Detected')
plt.show()