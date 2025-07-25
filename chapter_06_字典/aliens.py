# 在Python中，字典 是一系列键值对 。每个键 都与一个值相关联
# 与键相关联的值可以是数字、字符串、列表
# 在Python中，字典用放在花括号（{} ）中的一系列键值对表示
# 键和值之间用冒号分隔，而键值对之间用逗号分隔
# 字典是一种动态结构
# 要添加键值对，可依次指定字典名、用方括号括起的键和相关联的值

# Make an empty list for storing aliens.
aliens = []

# Make 30 green aliens.
for alien_number in range(30):
    new_alien = {'color': 'green', 'points': 5, 'speed': 'slow'}
    aliens.append(new_alien)

for alien in aliens[:3]:
    if alien['color'] == 'green':                                   # 修改字典中的值，可依次指定字典名、用方括号括起的键，以及与该键相关联的新值
        alien['color'] = 'yellow'
        alien['speed'] = 'medium'
        alien['points'] = 10
    
# Show the first 5 aliens.
for alien in aliens[:5]:
    print(alien)
print("...")


# 嵌套


aliens = []                                     # 创建一个用于存储外星人的空列表

# 创建30个绿色的外星人。
for alien_number in range(30):                  # 告诉Python要重复这个循环多少次，每次执行这个循环时，都创建一个外星人，并将其附加到列表aliens 末尾
     new_alien = {'color': 'green', 'points': 5, 'speed': 'slow'}
     aliens.append(new_alien)

# 显示前5个外星人。
for alien in aliens[:5]:                        # 使用一个切片来打印前5个外星人
      print(alien)
print("...")

# 显示创建了多少个外星人。
print(f"Total number of aliens: {len(aliens)}")