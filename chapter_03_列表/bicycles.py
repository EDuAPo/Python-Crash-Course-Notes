bicycles = ['trek', 'cannondale', 'redline', 'specialized']
message = f"My first bicycle was a {bicycles[0].title()}."

print(message)

# 在Python中，用方括号（[] ）表示列表，并用逗号分隔其中的元素
# 列表通常包含多个元素，因此给列表指定一个表示复数的名称（如letters 、digits 或names ）是个不错的主意。
# 列表是有序集合，因此要访问列表的任意元素，只需将该元素的位置（索引 ）告诉Python即可
# 要访问列表元素，可指出列表的名称，再指出元素的索引，并将后者放在方括号内
# 在Python中，第一个列表元素的索引为0，而不是1,要访问列表的任何元素，都可将其位置减1
# Python为访问最后一个列表元素提供了一种特殊语法。通过将索引指定为-1
bicycles = ['trek', 'cannondale', 'redline', 'specialized']
print(bicycles[0])
print(bicycles[0].title())
print(bicycles[-1].title())

# 方法append() 将元素添加到列表末尾,不影响列表中的其他所有元素
motorcycles = ['honda', 'yamaha', 'suzuki']
print(motorcycles)
motorcycles.append('ducati')
print(motorcycles)


motorcycles = []
motorcycles.append('honda')
motorcycles.append('yamaha')
motorcycles.append('suzuki')
print(motorcycles)


# 方法insert() 可在列表的任何位置添加新元素,需要指定新元素的索引和值
motorcycles = ['honda', 'yamaha', 'suzuki']
motorcycles.insert(0,'ducati')
print(motorcycles)

# del删除任意位置处的元素
del motorcycles[0]
print(motorcycles)

#方法pop() 删除列表末尾的元素，并让你能够接着使用它。术语弹出 （pop）源自这样的类比
popped_motorcycle = motorcycles.pop()
print(motorcycles)
print(popped_motorcycle)

# 使用方法remove()要删除的元素的值
motorcycles.remove('honda')
print(motorcycles)


# 使用方法sort() 对列表进行永久排序
cars = ['bmw', 'audi', 'toyota', 'subaru']
cars.sort()
print(cars)


cars = ['bmw', 'audi', 'toyota', 'subaru']
cars.sort(reverse=True)               # 可以按与字母顺序相反的顺序排列列表元素，只需向sort() 方法传递参数reverse=True 即可
print(cars)


# 使用函数sorted() 对列表临时排序。能够按特定顺序显示列表元素，同时不影响它们在列表中的原始排列顺序
cars = ['bmw', 'audi', 'toyota', 'subaru']

print("Here is the original list:")
print(cars)

print("\nHere is the sorted list:")
print(sorted(cars))

print("\nHere is the original list again:")
print(cars)



# 要反转列表元素的排列顺序，可使用方法reverse()
cars = ['bmw', 'audi', 'toyota', 'subaru']
print(cars)

cars.reverse()            # 注意，reverse() 不是按与字母顺序相反的顺序排列列表元素，而只是反转列表元素的排列顺序
print(cars)



# 使用函数len() 可快速获悉列表的长度
cars = ['bmw', 'audi', 'toyota', 'subaru']
print(len(cars))
