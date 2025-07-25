name = "Ada Lovelace"
print(name.upper())
print(name.lower())


first_name = "ada"
last_name = "lovelace"
full_name = f"{first_name} {last_name}"         # 要在字符串中插入变量的值，可在前引号前加上字母f,再将要插入的变量放在花括号内,这样，当Python显示字符串时，将把每个变量都替换为其值。
print(f"Hello, {full_name.title()}!")




# 要在字符串中添加换行符，可使用字符组合\n ：
print("Languages:\nPython\nC\nJavaScript")

# 还可在同一个字符串中同时包含制表符和换行符
print("Languages:\n\tPython\n\tC\n\tJavaScript")



# 在Python中，可对整数执行加（+ ）减（- ）乘（* ）除（/ ）运算
# Python使用两个乘号表示乘方运算
# 将任意两个数相除时，结果总是浮点数，即便这两个数都是整数且能整除
# 书写很大的数时，可使用下划线将其中的数字分组
# universe_age = 14_000_000_000    python不会打印其中的下划线

# 可在一行代码中给多个变量赋值，这有助于缩短程序并提高其可读性
x, y, z = 0, 0, 0   # 这样做时，需要用逗号将变量名分开；对于要赋给变量的值，也需同样处理


# 在Python中，注释用井号（# ）标识