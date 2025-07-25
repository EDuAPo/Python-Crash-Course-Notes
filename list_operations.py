# 列表排序和切片
cars = ['bmw', 'audi', 'toyota', 'subaru']
cars.sort()  # 永久排序
print(cars)
print(cars[1:3])  # 切片
squares = [value**2 for value in range(1, 6)]  # 列表推导式
print(squares)


# 切片
# 含义：对操作的对象截取一部分的操作
# 语法：[开始位置：结束位置：步长]
# 包前不包后：即从起始位置开始，到结束位置的前一位结束（不包含结束位置本身）
st = "abcdefghjik"
print(st[0:4])
print(st[4:7])
print(st[3:])
print(st[:7])
print(st[-1:])
print(st[:-1])
print(st[-1:-5])
print(st[0:7:3])

# 步长：表示选取间隔，不写步长，默认为1
# 步长的绝对值决定切取的间隔，正负号决定切取方向
# 正数表示从左往右取值，负数表示从右往左取值


