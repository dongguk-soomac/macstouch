# 2024 R-Biz Challenge Soomac ZeroBurger
  
1. issue 생성 -> 구현할 기능 별로 체크 박스, 해당 작업에 필요한 인원 Assignees 추가, 상황에 따라 Labels 추가
2. Develope_#(issue번호) 이름으로 branch 생성 -> 반드시 pull 먼저 받고 하기
3. branch 내에서 작업 완료
4. pull request 생성 -> 관련 인원 Reviewers(코드를 봐야 하는 인원), Assignees(코드를 짠 인원) 추가, Labels 추가, 디코 또는 톡방에 알리기
5. Reviewers에 추가된 인원은 최대한 빠르게 리뷰 작성
6. 위 단계 완료 시 merge, issue close 진행 -> 코드 병합 시 문제 발생 최소화를 위해 한 명이 담당(윤지)
  
src  
    /control  
    /gui  
    /vision  
    /workspace -> 워크스페이스 아두이노 코드 및 ros node 코드  
    /zero -> robot controller 쪽 코드  
  
* 코드 작성 시 모듈화 신경 써주세요. 코드가 너무 길어지면 보는 것도 힘들고 디버깅하는 것도 힘듭니다.
