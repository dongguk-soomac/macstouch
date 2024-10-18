#!/usr/bin/env python3

from tkinter import Tk, Canvas, Button, PhotoImage, Frame
from pathlib import Path
import second_page
import subprocess
from time import sleep
import rospy
from std_msgs.msg import String
from selected_menu import menu_index
# from macstouch.msg import order

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

# 경로 설정
OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH_2 = OUTPUT_PATH / Path(r"/home/mac/catkin_ws/src/macstouch/src/gui/build_figma/assets/frame0")

from menu_info import menu_list, price_list 

def relative_to_assets_2(path: str) -> Path:
    return ASSETS_PATH_2 / Path(path)

def open_second_page(window):
    subprocess.Popen(['python', '/home/mac/catkin_ws/src/macstouch/src/gui/second_page.py'])  # 세 번째 페이지 실행
    sleep(1)
    window.destroy()

def open_buy(window):
    subprocess.Popen(['python', '/home/mac/catkin_ws/src/macstouch/src/gui/fourth_page.py'])  # 구매 페이지 실행
    sleep(1)
    window.destroy() 

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

    button_image_2 = PhotoImage(file=relative_to_assets_2("button_1.png"))
    button_2 = Button(
        page2,
        image=button_image_2,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: [print_menu(4), print("button2")],
        relief="flat"
    )
    button_2.place(
        x=27.0,
        y=316.0,
        width=505.0,
        height=502.0
    )

    button_image_3 = PhotoImage(file=relative_to_assets_2("button_2.png"))
    button_3 = Button(
        page2,
        image=button_image_3,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: [print_menu(5), print("button3")],
        relief="flat"
    )
    button_3.place(
        x=27.0,
        y=831.0,
        width=505.0,
        height=502.0
    )

    button_image_4 = PhotoImage(file=relative_to_assets_2("button_3.png"))
    button_4 = Button(
        page2,
        image=button_image_4,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: [print_menu(6), print("button4")],
        relief="flat"
    )
    button_4.place(
        x=547.0,
        y=831.0,
        width=505.0,
        height=502.0
    )

    button_image_5 = PhotoImage(file=relative_to_assets_2("button_4.png"))
    button_5 = Button(
        page2,
        image=button_image_5,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: [print_menu(7), print("button5")],
        relief="flat"
    )
    button_5.place(
        x=547.0,
        y=316.0,
        width=505.0,
        height=502.0
    )

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

        tempo_menu.append(price_list[menu_num])
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
            text=price_list[menu_num],
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
            text=menu_list[menu_num],
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
