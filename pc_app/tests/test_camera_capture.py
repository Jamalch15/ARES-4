import numpy as np

from app.vision import CameraCapture


def test_camera_x_flip_reverses_image_columns() -> None:
    image = np.array(
        [
            [[1, 1, 1], [2, 2, 2]],
            [[3, 3, 3], [4, 4, 4]],
        ],
        dtype=np.uint8,
    )

    oriented = CameraCapture._orient_image(image, {"display": {"flip_x": True}})

    np.testing.assert_array_equal(oriented, image[:, ::-1])


def test_camera_x_flip_is_disabled_by_default() -> None:
    image = np.array([[[1, 2, 3]]], dtype=np.uint8)

    oriented = CameraCapture._orient_image(image, {})

    assert oriented is image
