from tkinter import Tk, Canvas, Button, PhotoImage, Frame
from pathlib import Path
import subprocess
from time import sleep
import rospy
from std_msgs.msg import String
from selected_menu import menu2pub


# 경로 설정
OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH_2 = OUTPUT_PATH / Path(r"/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/assets/frame0")

from menu_info import menu_list, price_list 


def relative_to_assets_2(path: str) -> Path:
    return ASSETS_PATH_2 / Path(path)

def open_third_page(window):
    subprocess.Popen(['python', '/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/third_page.py'])  # 세 번째 페이지 실행
    sleep(1)
    window.destroy()

def open_buy(window):
    subprocess.Popen(['python', '/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/fourth_page.py'])  # 구매 페이지 실행
    sleep(1)
    window.destroy()

def publish_order(menu_index):
    # Initialize the ROS node (name it whatever you want)
    rospy.init_node('order_publisher_node', anonymous=True)
    
    # Create a publisher object, specify the topic name and the message type
    order_pub = rospy.Publisher('/order', String, queue_size=10)
    
    # Wait for connections to establish (optional, for smooth operation)
    rospy.sleep(1)
    
    # Publish a message (you can customize the message as needed)
    order_msg = str(menu_index)
    order_pub.publish(order_msg)
    
    # Ensure all ROS communication is complete before exiting (optional)
    rospy.loginfo(f"Published message: {order_msg}")


# 두 번째 페이지 구성
def create_second_page():
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
        command=lambda: print_menu(0),
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
        command=lambda: print_menu(1),
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
        command=lambda: print_menu(2),
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
        command=lambda: print_menu(3),
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
        command=lambda: print("button_5 clicked"),
        relief="flat"
    )
    button_6.place(
        x=114.0,
        y=92.0,
        width=360.0,
        height=129.0
    )

    button_image_7 = PhotoImage(file=relative_to_assets_2("button_6.png"))  # custom menu button
    button_7 = Button(
        page2,
        image=button_image_7,
        borderwidth=0,
        highlightthickness=0,
        command=lambda: open_third_page(window),  # 기존 창 닫고 세 번째 페이지로 이동
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
        command=lambda: [open_buy(window), publish_order(menu2pub)] ,  # 기존 창 닫고 구매 페이지로 이동
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
        with open('/home/seojin/catkin_ws/src/macstouch/src/gui/build_figma/selected_menu.py', 'w') as file:
            file.write(f"menu2pub = [{menu_num}]\n")
            
        # price list 
        canvas2.create_text(
            754.0,
            1446.0,
            anchor="nw",
            text=price_list[menu_num],
            fill="#000000",
            font=("Inter", 24 * -1)
        )

        # num
        canvas2.create_text(
            544.0,
            1446.0,
            anchor="nw",
            text='1',
            fill="#000000",
            font=("Inter", 24 * -1)
        )

        # menu list
        canvas2.create_text(
            81.0,
            1446.0,
            anchor="nw",
            text=menu_list[menu_num],
            fill="#000000",
            font=("Inter", 24 * -1)
        )

    window.resizable(False, False)
    window.mainloop()

# 두 번째 페이지 생성 실행
if __name__ == "__main__":
    create_second_page()

# sdfsdf