#!/usr/bin/python
# -*- coding: utf-8 -*-

## 1. 초기 설정① #######################################
# 라이브러리 가져오기
from i611_MCS import *
from i611_extend import *
from i611_io import *
from teachdata import *
from rbsys import *
from i611_common import *
from i611shm import * 

def main():
    ## 2. 초기 설정② ####################################
    # i611 로봇 생성자
    rb = i611Robot()
    # 좌표계의 정의
    _BASE = Base()
    # 로봇과 연결 시작 초기화
    rb.open()
    # I/O 입출력 기능의 초기화 (I/O 미사용시 생략 가능 )
    IOinit( rb )
    
    dout(48, '000')

    ## dout ##############################
    rb.sleep(1)
    dout(48, '1**') # 그리퍼 열기
    rb.sleep(1)
    dout(48, '0**')
    rb.sleep(1)
    dout(48, '**1') # 그리퍼 닫기

    rb.home()
    rb.sleep(1)

    # shm = shm_read( 0x0104, 1 )
    # print(shm)

 
    ## 6. 종료 ######################################
    # 로봇과의 연결을 종료
    rb.close()

if __name__ == '__main__':
    main()




