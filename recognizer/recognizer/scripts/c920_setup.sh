#!/bin/bash

CAMERAS=(
    "/dev/camera_front"
    "/dev/camera_left"
    "/dev/camera_right"
)

for DEVICE in "${CAMERAS[@]}"; do

    echo "======================================"
    echo " Configuring CAMERA"
    echo " Device: ${DEVICE}"
    echo "======================================"

    # カメラが認識されるまで少し待つ
    sleep 1

    # カメラが存在するか確認
    if [ ! -e "${DEVICE}" ]; then
        echo "ERROR: ${DEVICE} not found"
        continue
    fi

    # 屋外用設定
    v4l2-ctl -d "${DEVICE}" \
        --set-ctrl=brightness=128,\
contrast=128,\
saturation=128,\
white_balance_automatic=1,\
sharpness=110,\
backlight_compensation=0,\
power_line_frequency=1,\
auto_exposure=3,\
exposure_dynamic_framerate=0,\
focus_automatic_continuous=1

    if [ $? -eq 0 ]; then
        echo "${DEVICE}: configuration completed."
    else
        echo "${DEVICE}: configuration FAILED."
    fi

    echo ""
done

echo "======================================"
echo " CAMERA configuration finished"
echo "======================================"
