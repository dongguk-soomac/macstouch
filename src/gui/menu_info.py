menu_list = ["제로 버거", "치즈 버거", "불고기 버거", "더블 패티 버거", "치즈 두 배(+500)", "불고기 두 배(+1000)", "양상추 두 배(+500)", "토마토 두 배(+500)"]
price_list = [7000, 6000, 5000, 6500, 1000, 500, 1000, 500]

# 리스트 = [재료1의 개수, 재료2의 개수, 재료3의 개수 ...] ==> macstouch config 참고
# MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "lettuce"] 여기에 대응되게 -> 필요 없는 재료는 0으로 채우기

# /msg/order.msg 파일에서 id 가 주문번호 계속 +1, menu가 리스트

# 불고기 버거 = [2, 1, 0, 3, 8, 1, 1, 3]
# 제로 버거 = [2, 0, 0, 3, 8, 1, 1, 3]
# 치즈 버거 = [2, 1, 1, 3, 8, 1, 1, 3]
# 더블 치즈 버거 = [2, 1, 2, 3, 8, 1, 1, 3]
