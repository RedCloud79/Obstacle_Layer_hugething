# Obstacle_Layer_hugething

---
- 포인트에 차량이나 큰 쿨건이 생성이된 경우 work 상태에서 비어있는 다른 구간으로 새로운 path 생성하면서 도착 못해도 계속 시도하는 현상

- 포인트에 차량이나 큰 물건이 있으면 해당 포인트 몇번 시도 후 다음 포인트로 넘어가는 동작이 필요

---

* 거리 기능 (총 이동거리 측정 하는거) 회피 기능에서 거리 추출기능 활용하여 추가가

* 단순히 거리(0.5m 이내)만으로 도착 판정하지 않음
* 일정 시간(3초) 동안 로봇이 실제로 목표 방향으로 이동 중인지 확인
* 목표 방향으로 계속 가까워지고 있으면 정상 이동으로 판단
* 일정 시간 동안 거리 변화가 없으면 장애물로 인해 막힌 것으로 판단 & 다음 포인트로 이동

distance = path 정보를 받을 수 있으면 해당 정보를 통해서 거리로 지정
speed = imu, odom 값을 subscriber로 추출 해당 값 지정