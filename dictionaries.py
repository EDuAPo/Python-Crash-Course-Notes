# 字典操作
alien = {'color': 'green', 'points': 5}
print(alien['color'])  # 访问值
alien['x_position'] = 0  # 添加键值对
for key, value in alien.items():
    print(f"{key}: {value}")