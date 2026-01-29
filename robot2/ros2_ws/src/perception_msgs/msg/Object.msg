# 단일 감지 객체
# Perception Bridge가 YOLO + LiDAR 융합으로 추정한 객체 정보

# 로봇 기준 상대 좌표 (미터)
# rel_x > 0: 로봇 전방
# rel_y > 0: 로봇 좌측
float32 rel_x
float32 rel_y

# 객체 클래스
# 0 = 빈 슬롯 (고정 배열에서 사용되지 않는 원소)
# 1 = 장애물 (보행자, 휠체어, 침대 등 피해야 하는 객체)
# 2 = 미션 객체 (엘리베이터 버튼 등)
uint8 object_class

# 클래스 상수 정의
uint8 CLASS_EMPTY = 0
uint8 CLASS_OBSTACLE = 1
uint8 CLASS_MISSION = 2
