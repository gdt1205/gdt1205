import matplotlib.pyplot as plt

def draw_segments(segments):
    # 设置中文字体
    plt.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
    
    # 创建图形和坐标轴
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 绘制每条线段
    for i, segment in enumerate(segments):
        start = segment["start"]
        end = segment["end"]
        ax.plot([start[0], end[0]], [start[1], end[1]], 'b-', linewidth=2, label=f'线段 {i+1}' if i == 0 else "")
    
    # 标记起始点
    # ax.plot(start_point[0], start_point[1], 'ro', markersize=8, label='起始点')
    
    # 设置坐标轴范围
    all_x = [p for seg in segments for p in [seg["start"][0], seg["end"][0]]] 
    all_y = [p for seg in segments for p in [seg["start"][1], seg["end"][1]]]
    
    x_min, x_max = min(all_x) - 1, max(all_x) + 1
    y_min, y_max = min(all_y) - 1, max(all_y) + 1
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    # 添加网格、标签和标题
    ax.grid(True)
    ax.set_xlabel('X坐标')
    ax.set_ylabel('Y坐标')
    ax.set_title('线段图可视化')
    ax.legend()
    
    # 显示图形
    plt.show()

if __name__ == "__main__":
    # 输入数据
    data = {
        "segments": [
            {"start": [0, 0], "end": [0, 8]},
            {"start": [0, 8], "end": [8, 8]},
            {"start": [0, 0], "end": [8, 0]},
            {"start": [8, 0], "end": [8, 8]},
            {"start": [0, 6], "end": [2, 6]},
            {"start": [4, 0], "end": [4, 2]},
            {"start": [2, 2], "end": [2, 4]},
            {"start": [2, 4], "end": [4, 4]},
            {"start": [4, 4], "end": [4, 6]},
            {"start": [4, 6], "end": [8, 6]},
            {"start": [6, 2], "end": [6, 6]},
        ],
      }
    
    # 绘制线段
    draw_segments(data["segments"])
