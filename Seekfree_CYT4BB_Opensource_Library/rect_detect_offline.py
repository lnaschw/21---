# bilibili搜索学不会电磁场看教程
# 第六课 - 离线运行版（外接 ST7701 LCD 屏幕）
# 矩形识别，可以用来找矩形的四个角的坐标

import time
import os
import sys

from media.sensor import *
from media.display import *
from media.media import *

sensor = None

try:
    print("camera_test")

    sensor = Sensor(width=640, height=640)
    sensor.reset()

    sensor.set_framesize(width=640, height=640)
    sensor.set_pixformat(Sensor.RGB565)

    # ★ 离线模式：to_ide=False，画面输出到物理 LCD 屏幕
    Display.init(Display.ST7701, width=800, height=480, to_ide=False)

    MediaManager.init()
    sensor.run()
    clock = time.clock()

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot()

        # 矩形识别
        img_rect = img.to_grayscale(copy=True)
        img_rect = img_rect.binary([(82, 212)])
        rects = img_rect.find_rects(threshold=10000)

        if rects:
            for rect in rects:
                corner = rect.corners()
                # 画四条边
                img.draw_line(corner[0][0], corner[0][1],
                              corner[1][0], corner[1][1],
                              color=(0, 255, 0), thickness=3)
                img.draw_line(corner[2][0], corner[2][1],
                              corner[1][0], corner[1][1],
                              color=(0, 255, 0), thickness=3)
                img.draw_line(corner[2][0], corner[2][1],
                              corner[3][0], corner[3][1],
                              color=(0, 255, 0), thickness=3)
                img.draw_line(corner[0][0], corner[0][1],
                              corner[3][0], corner[3][1],
                              color=(0, 255, 0), thickness=3)

        # 显示 FPS
        img.draw_string_advanced(10, 10, 40,
                                 "fps: {:.1f}".format(clock.fps()),
                                 color=(255, 0, 0))

        Display.show_image(img)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
