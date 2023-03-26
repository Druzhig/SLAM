import numpy as np
import cv2
from robotics_toolbox import Camera, OccupancyGridMap, SimRobot, LaserRangeFinder
from robotics_toolbox.mapping import FastSLAM

# создание объекта камеры
camera = Camera()

# создание объекта лидара
lidar = LaserRangeFinder()

# создание объекта карты помещения
map_size = (200, 200)  # размер карты (в пикселях)
map_resolution = 0.1  # разрешение карты (в метрах на пиксель)
map = OccupancyGridMap(map_size, map_resolution)

# создание объекта SLAM
slam = FastSLAM(map, camera, lidar)

# создание объекта симулятора робота
robot = SimRobot()

# главный цикл программы
while True:
    # получение изображения с камеры
    img = camera.getImage()

    # получение данных лидара
    ranges = lidar.getRanges()

    # запуск алгоритма SLAM
    slam.update(img, ranges)

    # получение карты помещения
    map_data = slam.getMap()

    # отображение карты помещения
    map_img = np.zeros((map_size[1], map_size[0], 3), dtype=np.uint8)
    map_img[map_data > 0] = (255, 255, 255)
    cv2.imshow('Map', map_img)
    cv2.waitKey(1)

    # перемещение робота
    robot.moveForward(0.5)  # движение вперед на 0.5 метра
    robot.turn(np.random.uniform(-np.pi/4, np.pi/4))  # поворот на случайный угол

    # проверка на столкновение с препятствием
    if map.isOccupied(robot.getPosition()):
        robot.stop()  # остановка робота, если он столкнулся с препятствием
