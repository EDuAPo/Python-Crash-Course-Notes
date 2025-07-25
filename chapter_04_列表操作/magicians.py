# 假设我们有一个魔术师名单，需要将其中每个魔术师的名字都打印出来。为此，可以分别获取名单中的每个名字，
# 但这种做法会导致很多问题。例如，如果名单很长，将包含大量重复的代码。
# 另外，每当名单的长度发生变化时，都必须修改代码。通过使用for 循环，可以让Python去处理这些问题

magicians = ['alice', 'david', 'carolina'] 
for magician in magicians: 
    print(f"{magician.title()}, that was a great trick!")                   # 在代码行for magician in magicians 后面，每个缩进的代码行都是循环的一部分，将针对列表中的每个值都执行一次
    print(f"I can't wait to see your next trick, {magician.title()}.\n")    # 调用print() 中的换行符"\n" 在每次迭代结束后都插入一个空行
    
print("Thank you, everyone. That was a great magic show!")                  # 在for 循环后面，没有缩进的代码都只执行一次，不会重复执行         



# ython函数range() 让你能够轻松地生成序列数
for value in range(1, 5):
    print(value)
# 调用函数range() 时，也可以只指定一个参数，这样就会从0开始。例如，range(6) 返回数字0～5
# 可以使用函数list() 将range() 结果直接转换为列表
numbers = list(range(1, 6))
print(numbers)

# 使用函数range() 时，指定步长。因此，可以给这个函数指定第三个参数
even_numbers = list(range(2, 11, 2))
print(even_numbers)
# 有几个专门用于处理数字列表的Python函数。例如，您可以轻松地查找数字列表的简便、简单和总和
digits = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0]
print(min(digits))
print(max(digits))
print(sum(digits))

# 将前10个整数的平方加入一个列表中：
squares = []
for value in range(1, 11):
    square = value ** 2
    squares.append(square)

print(squares)

# 为了让代码更简洁，可以不使用临时变量square ，而直接将每个计算得到的值附加到列表中
squares = []
for value in range(1,11):
    squares.append(value**2)

print(squares)

# 前面介绍的生成列表squares 的方式包含三四行代码，而列表解析只需编写一行代码就可以生成这样的列表
squares = [value**2 for value in range(1, 11)]        # 要使用这种语法，首先指定一个描述性的列表名，例如squares;然后，指定一个左方表达式 value**2,接下来，编写一个for 循环，用于给表达式提供值
print(squares)                                        # 注意，这里的for 语句并没有冒号