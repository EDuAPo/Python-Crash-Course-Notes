# # 注释
# # 单行注释 #
# # 多行注释 '''  '''



# # 变量命名规则
# # 数字 字母 下划线组成
# # 严格区分大小写
# # 不能以数字开头
# # 不能是关键字
# # 建议规范命名：代表含义，驼峰
# 变量名应既简短又具有描述性




# # 数值类型
# # int 整型
# # float 浮点数
# # bool 布尔型： True False布尔值可以当作整型对待，1,0
# print(True+False)
# print(type(True))   #type()函数查看变量类型


# 用引号括起的都是字符串，其中的引号可以是单引号，也可以是双引号
# "This is a string."
# 'This is also a string.'
# 方法title() 以首字母大写的方式显示每个单词，即将每个单词的首字母都改为大写



# # 算术运算符
# # 加减乘除 +-*/
# # 整除//  取余数%  幂 2**3=8

# # 赋值运算符
# # 赋值运算符必须连着写，中间不能有空格，否则会报错

# # 比较运算符
# # ==  !=  >  <=   >=

# # 逻辑运算符
# # and与  or或  not非




# # 转义字符 
# # \t 制表符 通常缩进四个字符
# # \n 换行符 表示将当前位置移到下一行开头
# print("嘻嘻\t哈哈")
# print("嘻嘻\n哈哈")




# # 输入函数
# # input(prompt)  prompt是提示，会在控制台中显示
# pwd = input("请输入密码：")
# print(pwd)




# # 字符串操作
# name = "ada lovelace"
# print(name.title())  # 首字母大写
# print(name.upper())  # 全大写
# message = f"Hello, {name.title()}!"
# print(message)




# # 格式化输出（ d%d,%s,%f)
# age=18
# print("我的名字:%s,年龄:%d" % (name,age)) 
# print(f"我的名字：{name},年龄：{age}")







