# 감지된 객체 배열 (고정 크기 10개)
# Perception Bridge가 발행하는 메인 출력 메시지

# 타임스탬프 및 좌표계 정보
std_msgs/Header header

# 감지된 객체 배열 (거리순 정렬, 가까운 것부터)
# 객체가 10개 미만이면 나머지는 빈 슬롯 (object_class=0)
Object[10] objects
