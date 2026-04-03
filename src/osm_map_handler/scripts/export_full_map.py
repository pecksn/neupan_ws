import os
import cv2
import numpy as np
import glob

def export_full_map():
    map_dir = "/home/nuc/lxw/ros_maps/20"
    output_path = "/home/nuc/lxw/full_osm_stitch.jpg"
    
    # 1. 扫描所有存在的瓦片
    all_files = glob.glob(os.path.join(map_dir, "*/*.png"))
    if not all_files:
        print("Error: No tile files found in " + map_dir)
        return

    # 提取所有 x 和 y 坐标
    xs = []
    ys = []
    for f in all_files:
        try:
            parts = f.split('/')
            xs.append(int(parts[-2]))
            ys.append(int(parts[-1].split('.')[0]))
        except:
            continue
            
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    width_tiles = max_x - min_x + 1
    height_tiles = max_y - min_y + 1
    
    print(f"Detected bounding box: X[{min_x} to {max_x}], Y[{min_y} to {max_y}]")
    print(f"Total grid size: {width_tiles}x{height_tiles} tiles.")
    
    # 2. 创建大画布 (初始化为中灰色)
    # 限制最大尺寸防止内存溢出，如果太大则分层处理或提示
    if width_tiles > 100 or height_tiles > 100:
        print("Warning: Map is very large, limit to 100x100 tiles near center.")
        # 取中心
        cx, cy = (min_x + max_x)//2, (min_y + max_y)//2
        min_x, max_x = cx - 40, cx + 40
        min_y, max_y = cy - 40, cy + 40
        width_tiles, height_tiles = 81, 81
    
    full_canvas = np.zeros((height_tiles * 256, width_tiles * 256, 3), dtype=np.uint8) + 128
    
    # 3. 填充图片
    count = 0
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            tile_path = os.path.join(map_dir, str(x), f"{y}.png")
            if os.path.exists(tile_path):
                img = cv2.imread(tile_path)
                if img is not None:
                    # 计算在画布上的位置
                    y_offset = (y - min_y) * 256
                    x_offset = (x - min_x) * 256
                    full_canvas[y_offset:y_offset+256, x_offset:x_offset+256] = img
                    count += 1
    
    # 4. 保存
    print(f"Stitched {count} tiles. Saving to {output_path}...")
    cv2.imwrite(output_path, full_canvas)
    print("Export Success!")

if __name__ == "__main__":
    export_full_map()
