"""유틸리티 함수들"""

import numpy as np
from perception_msgs.msg import Object, ObjectArray


def compute_distance(rel_x: float, rel_y: float) -> float:
    """로봇 기준 상대 좌표에서 거리 계산"""
    return np.sqrt(rel_x * rel_x + rel_y * rel_y)


def sort_objects_by_distance(objects: list) -> list:
    """객체 리스트를 거리순으로 정렬 (가까운 것부터)

    Args:
        objects: (rel_x, rel_y, object_class) 튜플의 리스트

    Returns:
        거리순 정렬된 리스트
    """
    return sorted(objects, key=lambda obj: compute_distance(obj[0], obj[1]))


def create_empty_object() -> Object:
    """빈 슬롯 Object 생성"""
    obj = Object()
    obj.rel_x = 0.0
    obj.rel_y = 0.0
    obj.object_class = Object.CLASS_EMPTY
    return obj


def create_object(rel_x: float, rel_y: float, object_class: int) -> Object:
    """Object 메시지 생성"""
    obj = Object()
    obj.rel_x = float(rel_x)
    obj.rel_y = float(rel_y)
    obj.object_class = int(object_class)
    return obj


def build_object_array(objects: list, header) -> ObjectArray:
    """ObjectArray 메시지 생성

    Args:
        objects: (rel_x, rel_y, object_class) 튜플의 리스트
        header: std_msgs/Header

    Returns:
        ObjectArray 메시지 (항상 10개, 거리순 정렬, 빈 슬롯 채움)
    """
    msg = ObjectArray()
    msg.header = header

    # 거리순 정렬
    sorted_objects = sort_objects_by_distance(objects)

    # 최대 10개까지만 사용
    sorted_objects = sorted_objects[:10]

    # Object 메시지 배열 생성
    obj_list = []
    for rel_x, rel_y, obj_class in sorted_objects:
        obj_list.append(create_object(rel_x, rel_y, obj_class))

    # 10개 미만이면 빈 슬롯으로 채움
    while len(obj_list) < 10:
        obj_list.append(create_empty_object())

    msg.objects = obj_list
    return msg
