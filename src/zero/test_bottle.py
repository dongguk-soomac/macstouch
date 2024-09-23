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
    
    # MDO 설정
    # distance 수정 필요
    rb.set_mdo( mdoid=1, portno=48, value=1, kind=1, distance=50 ) # portno 48(그리퍼 닫기) HIGH
    rb.set_mdo( mdoid=2, portno=48, value=0, kind=1, distance=50 ) # portno 48(그리퍼 닫기) LOW
    rb.set_mdo( mdoid=3, portno=50, value=1, kind=1, distance=80 ) # portno 50(그리퍼 열기) HIGH
    rb.set_mdo( mdoid=4, portno=50, value=0, kind=1, distance=50 ) # portno 50(그리퍼 열기) LOW
    rb.set_mdo( mdoid=5, portno=48, value=1, kind=2, distance=200 ) # portno 48(그리퍼 닫기) HIGH && 동작 끝나면서 수행

  
    ## 3. 교시 포인트 설정 ######################
    before_pick_jnt = Joint(0.0, -10.0, -120.0, 0.0, -50.0, -90.0)
    before_pick_pos = rb.Joint2Position(before_pick_jnt)
    mod_bfpck_pos = before_pick_pos.offset(dz=20)
    mod_bfpck_jnt = rb.Position2Joint(mod_bfpck_pos)
    pick_bottle_pos = mod_bfpck_pos.offset(dz=-33)
    pick_bottle_jnt = rb.Position2Joint(pick_bottle_pos)
    flip_jnt = Joint(0.0, 20.0, -70.0, 0.0, 0.0, -90.0)
   

    
    ## 4. 동작 조건 설정 ##################################
    #MotionParam 생성자에서 동작 조건 설정
    m = MotionParam( jnt_speed=10, lin_speed=70 )
    m_fast = MotionParam( jnt_speed=100, lin_speed=100, pose_speed=100, overlap = 0, acctime=0.2, dacctime=0.2)
    #MotionParam 형으로 동작 조건 설정
    
    ## 5. 로봇 동작을 정의 ##############################
    # 작업 시작
    rb.motionparam( m )
    dout(48, '000') # 그리퍼 초기화

    rb.home()

    print("before_pick_jnt = ", mod_bfpck_jnt.jnt2list())
    rb.move( mod_bfpck_jnt )
    rb.sleep(1)
    dout(48, '1**') # 그리퍼 열기
    rb.sleep(2)
    dout(48, '0**') # 그리퍼 닫기를 위해 0으로 할당
    print("pick_bottle_jnt = ", pick_bottle_jnt.jnt2list())
    rb.line(pick_bottle_jnt)
    rb.sleep(1)
    dout(48, '**1') # 그리퍼 닫기
    rb.sleep(2)

    rb.motionparam( m_fast )
    print("flip_jnt = ", flip_jnt.jnt2list())
    # 여기에 MDO를 이용하여 던짐과 동시에 그리퍼 열기 동작
    rb.enable_mdo( bitfield=6 ) # 그리퍼 닫힘 LOW && 그리퍼 열림 HIGH
    rb.move(flip_jnt)
    rb.sleep(1)
    

    rb.motionparam( m )

    rb.enable_mdo( bitfield=24 )
    rb.home() # Home 위치로 이동
    # mdo 종료
    rb.disable_mdo( bitfield=6 ) 
    rb.disable_mdo( bitfield=24 )
 
    ## 6. 종료 ######################################
    # 로봇과의 연결을 종료
    rb.close()

if __name__ == '__main__':
    main()