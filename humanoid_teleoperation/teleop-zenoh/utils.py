import numpy as np
import time
import rerun as rr
import quaternionic
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor


def log_hands(r):
    left_fingers = r["left_wrist"][0] @ r["left_fingers"]
    right_fingers = r["right_wrist"][0] @ r["right_fingers"]

    start = time.perf_counter()
    ex = ThreadPoolExecutor(max_workers=16)
    # ex.submit(
    #     rr.log,
    #     "hand_tracking/left_hand",
    #     rr.Points3D(
    #         left_fingers[:, 0:3, 3]
    #     ),
    # )
    # ex.submit(
    #     rr.log,
    #     "hand_tracking/right_hand",
    #     rr.Points3D(
    #         right_fingers[:, 0:3, 3]
    #     ),
    # )

    left_finger_quats = []

    for i in range(left_fingers.shape[0]):
        left_finger_quats.append(
            quaternionic.array.from_rotation_matrix(left_fingers[i][0:3, 0:3])
        )

    ex.submit(
        rr.log,
        "hand_tracking/left_hand",
        rr.Boxes3D(
            centers=left_fingers[:, 0:3, 3],
            half_sizes=np.ones((left_fingers.shape[0], 3)) * 0.001,
            rotations=left_finger_quats,
        ),
    )

    right_finger_quats = []

    for i in range(right_fingers.shape[0]):
        right_finger_quats.append(
            quaternionic.array.from_rotation_matrix(right_fingers[i][0:3, 0:3])
        )

    ex.submit(
        rr.log,
        "hand_tracking/right_hand",
        rr.Boxes3D(
            centers=right_fingers[:, 0:3, 3],
            half_sizes=np.ones((right_fingers.shape[0], 3)) * 0.001,
            rotations=right_finger_quats,
        ),
    )
    # ex.submit(
    #     rr.log,
    #     "hand_tracking/left_hand_root",
    #     rr.Transform3D(
    #         translation=r['left_wrist'][0][0:3, 3],
    #         mat3x3=r['left_wrist'][0][0:3, 0:3]
    #     ),
    # )
    # for i in range(left_fingers.shape[0]):
    #     ex.submit(
    #         rr.log,
    #         f"hand_tracking/left_finger_{i}",
    #         rr.Transform3D(
    #             translation=left_fingers[i][0:3, 3],
    #             mat3x3=left_fingers[i][0:3, 0:3]
    #         ),
    #     )

    # ex.submit(
    #     rr.log,
    #     "hand_tracking/right_hand_root",
    #     rr.Transform3D(
    #         translation=r['right_wrist'][0][0:3, 3],
    #         mat3x3=r['right_wrist'][0][0:3, 0:3]
    #     ),
    # )
    # for i in range(right_fingers.shape[0]):
    #     ex.submit(
    #         rr.log,
    #         f"hand_tracking/right_finger_{i}",
    #         rr.Transform3D(
    #             translation=right_fingers[i][0:3, 3],
    #             mat3x3=right_fingers[i][0:3, 0:3]
    #         ),
    #     )
    ex.shutdown(wait=False)

    print(f"log time: {time.perf_counter() - start:.5f}")


def calculate_angle_between_vectors(v1, v2):
    assert type(v1).__module__ == np.__name__
    assert type(v2).__module__ == np.__name__
    assert v1.shape == (3,)
    assert v2.shape == (3,)

    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    angle_radians = np.arccos(cos_theta)
    return angle_radians


# counterclockwise angle from vector_a to vector_b
def calculate_counterclockwise_angle(v1, v2):
    assert type(v1).__module__ == np.__name__
    assert type(v2).__module__ == np.__name__
    assert v1.shape == (3,)
    assert v2.shape == (3,)

    dot_product = np.dot(v1, v2)
    cross_product = np.cross(v1, v2)
    magnitude_cross = np.linalg.norm(cross_product)
    angle_radians = np.arctan2(magnitude_cross, dot_product)
    if cross_product[2] < 0:
        angle_radians = -angle_radians

    angle_degrees = np.degrees(angle_radians)

    return angle_degrees
