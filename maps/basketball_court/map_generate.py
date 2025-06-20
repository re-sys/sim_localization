#!/usr/bin/env python3
import numpy as np
import cv2

# 可配置参数
MAP_WIDTH = 15.0  # 米 (内部可用区域)
MAP_HEIGHT = 7.0  # 米 (内部可用区域)
RESOLUTION = 0.05  # 米/像素
BORDER_WIDTH = 0.2  # 边框宽度(米)
BASKETBALL_RADIUS = 0.25  # 篮球架半径(米)
BASKETBALL_POSITION = (2.0, 3.5)  # 篮球架中心位置(米)
EXTENSION = 5.0  # 向外扩展的距离(米)

# 计算总地图尺寸(包括扩展区域)
total_width = MAP_WIDTH + 2 * EXTENSION
total_height = MAP_HEIGHT + 2 * EXTENSION

# 计算像素尺寸
width_px = int(total_width / RESOLUTION)
height_px = int(total_height / RESOLUTION)
border_px = int(BORDER_WIDTH / RESOLUTION)
radius_px = int(BASKETBALL_RADIUS / RESOLUTION)

# 创建空白地图 (0=自由空间, 100=占用空间)
map_data = np.zeros((height_px, width_px), dtype=np.uint8)

# 修正后的内部区域边界计算
inner_left = int(EXTENSION / RESOLUTION)
inner_right = int((EXTENSION + MAP_WIDTH) / RESOLUTION)
inner_bottom = height_px - int(EXTENSION / RESOLUTION)  # 从底部向上计算
inner_top = inner_bottom - int(MAP_HEIGHT / RESOLUTION)  # 从inner_bottom向上减去高度

# 添加边框 (占用空间)
# 外边框(整个地图边界)
map_data[:border_px, :] = 100  # 上边框
map_data[-border_px:, :] = 100  # 下边框
map_data[:, :border_px] = 100  # 左边框
map_data[:, -border_px:] = 100  # 右边框

# 内部可用区域的边框
map_data[inner_top-border_px:inner_top, inner_left:inner_right] = 100  # 上边框
map_data[inner_bottom:inner_bottom+border_px, inner_left:inner_right] = 100  # 下边框
map_data[inner_top:inner_bottom, inner_left-border_px:inner_left] = 100  # 左边框
map_data[inner_top:inner_bottom, inner_right:inner_right+border_px] = 100  # 右边框

# 添加篮球架 (圆形障碍物)
# 调整篮球架位置(相对于内部区域)
center_x = inner_left + int(BASKETBALL_POSITION[0] / RESOLUTION)
center_y = inner_top + int(BASKETBALL_POSITION[1] / RESOLUTION)  # 从inner_top向下计算
cv2.circle(map_data, (center_x, center_y), radius_px, 100, -1)

# 保存为PGM文件
with open('map.pgm', 'wb') as f:
    # PGM头部
    f.write(b'P5\n')
    f.write(f'{width_px} {height_px}\n'.encode())
    f.write(b'255\n')
    # 图像数据
    f.write(map_data.tobytes())

print(f"地图已生成: {width_px}x{height_px} 像素")
print(f"总尺寸: {total_width}x{total_height} 米")
print(f"内部可用区域: {MAP_WIDTH}x{MAP_HEIGHT} 米")
print(f"左下角坐标: {-EXTENSION}, {-EXTENSION}")