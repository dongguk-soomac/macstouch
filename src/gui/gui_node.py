import customtkinter as ctk

# 기본 테마와 색상 설정
ctk.set_appearance_mode("light") 
ctk.set_default_color_theme("blue") 

# 메인 앱 클래스
class KioskApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("MAC'S TOUCH")
        self.geometry("500x480")  # 화면 크기 설정

        # 첫 번째 화면 생성
        self.start_frame = ctk.CTkFrame(self)
        self.start_frame.pack(expand=True, fill="both", padx=10, pady=10)

        self.touch_button = ctk.CTkButton(self.start_frame, text="click me!", font=("Arial", 24), command=self.show_menu_screen)
        self.touch_button.pack(expand=True, padx=10, pady=10)

        # 두 번째 화면 (메뉴 선택) 생성
        self.main_frame = ctk.CTkFrame(self)

        # 상단 카테고리 버튼들
        self.category_frame = ctk.CTkFrame(self.main_frame)
        self.category_frame.pack(side="top", fill="x")

        self.categories = ["추천메뉴", "커스텀 메뉴", "음료"]
        for category in self.categories:
            ctk.CTkButton(self.category_frame, text=category, font=("Arial", 14)).pack(side="left", padx=5, pady=5, expand=True)

        # 메뉴 항목 리스트
        self.menu_frame = ctk.CTkFrame(self.main_frame)
        self.menu_frame.pack(expand=True, fill="both", padx=10, pady=10)

        self.menu_items = [
            ("불고기 버거", "3,000원"), 
            ("치즈 왕창 버거", "3,000원"), 
            ("패티 듬뿍 버거", "3,000원"), 
            ("비건 버거", "3,000원")
        ]
        for item, price in self.menu_items:
            ctk.CTkButton(self.menu_frame, text=f"{item} - {price}", font=("Arial", 16)).pack(fill="x", padx=5, pady=5)

        # 하단 주문 정보와 결제 버튼
        self.bottom_frame = ctk.CTkFrame(self.main_frame)
        self.bottom_frame.pack(side="bottom", fill="x")

        self.order_info = ctk.CTkLabel(self.bottom_frame, text="주문상품: 0개 | 주문금액: 0원", font=("Arial", 14))
        self.order_info.pack(side="left", padx=10)

        self.payment_button = ctk.CTkButton(self.bottom_frame, text="결제하기", font=("Arial", 16), fg_color="red")
        self.payment_button.pack(side="right", padx=10, pady=10)

    # 메뉴 화면 전환 함수
    def show_menu_screen(self):
        self.start_frame.pack_forget()  # 첫 번째 화면 숨기기
        self.main_frame.pack(expand=True, fill="both", padx=10, pady=10)  # 두 번째 화면 표시

if __name__ == "__main__":
    app = KioskApp()
    app.mainloop()
