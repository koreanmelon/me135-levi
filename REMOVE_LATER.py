def extract_bounding_box(color_frame: npt.NDArray, bb_config: dict):
    frame = color_frame.copy()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ...

    mask = mask_right + mask_left  # type: ignore
    frame[np.where(mask == 0)] = 0

    # Post processing
    frame = cv2.blur(
        src=frame,
        ksize=bb_config["blur_size"]
    )
    _, frame = cv2.threshold(
        src=frame,
        thresh=bb_config["thresh"],
        maxval=255,
        type=cv2.THRESH_BINARY
    )
    frame = cv2.cvtColor(
        src=frame,
        code=cv2.COLOR_RGB2GRAY
    )

    # Find contours
    contours, _ = cv2.findContours(
        image=frame,
        mode=cv2.RETR_LIST,
        method=cv2.CHAIN_APPROX_NONE
    )

    if len(contours) >= 2:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

        # index 0 is always ORB, index 1 is always CAR
        contours = sorted(contours, key=circle_like)

        ...

        return bb_orb, bb_car

    return None
