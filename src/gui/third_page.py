#!/usr/bin/env python3

from tkinter import Tk, Canvas, Button, PhotoImage, Frame, Label
from pathlib import Path
import second_page
import subprocess
from time import sleep
import rospy
from std_msgs.msg import String
from selected_menu import menu_index
# from macstouch.msg import order
from PIL import Image, ImageTk


#변수 선언
tempo_menu = []

price_text_id = None
num_text_id = None
menu_text_id = None
total_text_id = None

menu_y_position = 1446.0  # 메뉴가 출력될 y 좌표의 시작 위치
menu_offset = 30
max_menu_limit = 9

custum_menu_list = []

menu_list2button = ["빵", "패티", "치즈", "피클", "양파", "소스", "토마토", "양상추", '']
price_list2button = [500, 1000, 500, 500, 500, 500, 1000, 500, ''] 


# 경로 설정
OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH_2 = OUTPUT_PATH / Path(r"/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/assets/frame0")
ASSETS_PATH_1 = OUTPUT_PATH / Path(r"/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/assets")

def relative_to_assets_1(path: str) -> Path:
    return ASSETS_PATH_1 / Path(path)

from menu_info import menu_list, price_list 

def relative_to_assets_2(path: str) -> Path:
    return ASSETS_PATH_2 / Path(path)

def open_second_page(window):
    subprocess.Popen(['python', '/home/seojin/catkin_ws/src/macstouch/src/gui/second_page.py'])  # 세 번째 페이지 실행
    sleep(1)
    window.destroy()

def open_buy(window):
    subprocess.Popen(['python', '/home/seojin/catkin_ws/src/macstouch/src/gui/fourth_page.py'])  # 구매 페이지 실행
    sleep(1)
    window.destroy() 

def resize_image(image_path, scale=2/3):
    """이미지의 크기를 scale 비율만큼 줄임"""
    img = Image.open(image_path)
    new_size = (int(img.width * scale), int(img.height * scale))
    img = img.resize(new_size, Image.Resampling.LANCZOS)  # ANTIALIAS 대신 Image.Resampling.LANCZOS 사용
    return ImageTk.PhotoImage(img)


def create_third_page():
    window = Tk()
    window.geometry("1080x1920")
    window.configure(bg="#FFFFFF")

    # 두 번째 페이지 Frame
    page2 = Frame(window, bg="#FFFFFF")
    page2.place(x=0, y=0, width=1080, height=1920)

    canvas2 = Canvas(
        page2,
        bg="#FFFFFF",
        height=1920,
        width=1080,
        bd=0,
        highlightthickness=0,
        relief="ridge"
    )
    canvas2.place(x=0, y=0)

    # 두 번째 페이지 구성 요소
    image_image_2 = PhotoImage(file=relative_to_assets_2("image_1.png"))
    canvas2.create_image(
        540.0,
        1100.738525390625,
        image=image_image_2
    )

    image_image_3 = PhotoImage(file=relative_to_assets_2("image_2.png"))
    canvas2.create_image(
        540.0,
        1618.0,
        image=image_image_3
    )

    image_image_4 = PhotoImage(file=relative_to_assets_2("image_3.png"))
    canvas2.create_image(
        540.0,
        147.0,
        image=image_image_4
    )

    ########################## 햄버거 재료 버튼 ###############################

    # 이미지 리사이즈 (2/3 크기)
    # button_image = resize_image(relative_to_assets_2("button_1.png"))

    # 버튼 크기 및 배치 설정
    button_width = 340  # 버튼 너비 (화면 크기에 맞춰 조정)
    button_height = 335  # 버튼 높이 (화면 크기에 맞춰 조정)
    padding_x = 30  # 좌우 여백
    padding_y = 30  # 위아래 여백

    positions = [
        (30, 316),  # 첫 번째 줄 첫 번째 버튼 좌표
        (375, 316),  # 첫 번째 줄 두 번째 버튼 좌표
        (720, 316),  # 첫 번째 줄 세 번째 버튼 좌표
        (30, 661),  # 두 번째 줄 첫 번째 버튼 좌표
        (375, 661),  # 두 번째 줄 두 번째 버튼 좌표
        (720, 661),  # 두 번째 줄 세 번째 버튼 좌표
        (30, 1006),  # 세 번째 줄 첫 번째 버튼 좌표
        (375, 1006),  # 세 번째 줄 두 번째 버튼 좌표
        (720, 1006)  # 세 번째 줄 세 번째 버튼 좌표
    ]

    button_images = []  # 이미지 객체를 저장할 리스트

    for i in range(9):
        # 버튼 생성
        button_image_original = Image.open(relative_to_assets_1(f"download({i}).png"))
        button_image_resized = button_image_original.resize((335, 335))  # 원하는 크기로 리사이즈
        button_image = ImageTk.PhotoImage(button_image_resized)

        # 이미지 객체를 리스트에 저장하여 메모리에서 유지
        button_images.append(button_image)

        button = Button(
            window,
            image=button_image,
            borderwidth=0,
            highlightthickness=0,
            command=lambda i=i: [print(f"button {i+1} clicked"), print_menu(i)],
            relief="flat"
        )
        button.place(
            x=positions[i][0],
            y=positions[i][1],
            width=button_width,
            height=button_height
        )

    # window.resizable(False, False)

    ###############################################

    button_image_6 = PhotoImage(file=relative_to_assets_2("button_5.png"))
    button_6 = Button(
        page2,
        image=button_image_6,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: open_second_page(window),
        relief="flat"
    )
    button_6.place(
        x=114.0,
        y=92.0,
        width=360.0,
        height=129.0
    )

    button_image_7 = PhotoImage(file=relative_to_assets_2("button_6.png"))
    button_7 = Button(
        page2,
        image=button_image_7,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: print("button_6 clicked"),
        relief="flat"
    )
    button_7.place(
        x=587.0,
        y=92.0,
        width=371.0,
        height=129.0
    )

    button_image_8 = PhotoImage(file=relative_to_assets_2("button_7.png"))
    button_8 = Button(
        page2,
        image=button_image_8,
        borderwidth=0,
        highlightthickness=0,
        command=lambda:  [open_buy(window)],
        relief="flat"
    )
    button_8.place(
        x=673.0,
        y=1760.0,
        width=365.0,
        height=120.0
    )

    image_image_5 = PhotoImage(file=relative_to_assets_2("image_4.png"))
    canvas2.create_image(
        542.0,
        1561.0,
        image=image_image_5
    )

    def print_menu(menu_num):
        global tempo_menu, price_text_id, num_text_id, menu_text_id, total_text_id
        global max_menu_limit, menu_offset, menu_y_position, custum_menu_list

        custum_menu_list.append(menu_num)

        tempo_menu.append(price_list2button[menu_num])
        print(tempo_menu)
        # canvas2.delete()

        if len(tempo_menu) > max_menu_limit:
            print("최대 메뉴 개수(8개)를 초과할 수 없습니다.")
            return
        
        if total_text_id:
            canvas2.delete(total_text_id)
            
        # price list 
        price_text_id = canvas2.create_text(
            754.0,
            menu_y_position,
            anchor="nw",
            text=price_list2button[menu_num],
            fill="#000000",
            font=("Inter", 24 * -1)
        )

        # num
        num_text_id = canvas2.create_text(
            544.0,
            menu_y_position,
            anchor="nw",
            text='1',
            fill="#000000",
            font=("Inter", 24 * -1)
        )

        # menu list
        menu_text_id = canvas2.create_text(
            81.0,
            menu_y_position,
            anchor="nw",
            text=menu_list2button[menu_num],
            fill="#000000",
            font=("Inter", 24 * -1)
        )

        total_text_id = canvas2.create_text(
            81.0,
            1800.0,
            anchor="nw",
            text="합계 : {}원".format(sum(tempo_menu)),
            fill="#000000",
            font=("Inter", 35 * -1)
        )

        with open('/home/seojin/catkin_ws/src/macstouch/src/gui/selected_menu.py', 'w') as file:
            file.write(f"menu_index = {custum_menu_list}\n")

            menu_y_position += menu_offset

    window.resizable(False, False)
    window.mainloop()

# 두 번째 페이지 생성 실행
if __name__ == "__main__":
    create_third_page()
