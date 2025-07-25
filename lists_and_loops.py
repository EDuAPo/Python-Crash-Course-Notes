# 第 3-4 章：列表和循环
# guests = ["Alice", "Bob", "Charlie"]
# for guest in guests:
#     print(f"Welcome, {guest}!")

# 添加和删除元素
# guests.append("David")
# guests.remove("Bob")
# print(f"Updated guests: {guests}")



# str = '1234'          #定义一个字符串
# for i in str:           i是临时变量，可以随便写，i是常规写法
    # print(i)


# range ()  用来记录循环次数，相当于一个计数器
# for i in range(1,6):          #从1开始，到6结束，遵循包前不包后规则 【）
#     print(i)
# range()里面只写一个数。这个数就是循环次数，默认从0开始；写2个数，前面的数字代表开始位置，后面的数字代表结束位置


# continue
# 作用：退出本次循环，下一次循环继续执行
# i =1
# while i <= 5:
#   print (f"小明在吃第{i}个苹果")
#   if i == 3:
#     print(f"吃到了一条大虫子，第{i}个苹果不吃了")
#   i += 1
#   continue              #在continue之前一定要修改计数器，否则会进入死循环


# break
# for i in range (5):
#   if i == 2:
#     # break         # 结束本次循环
#     # continue      #跳过3,结束了在2时的循环，继续执行下一次循环
#   i += 1
#   print(i)








