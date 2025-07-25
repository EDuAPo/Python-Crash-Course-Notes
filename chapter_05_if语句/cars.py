# if 条件语句
# 每条if 语句的核心都是一个值True 或False 的表达式
# Python根据条件测试的值True 还是False 来决定是否执行if 语句中的代码
# 如果条件测试的值True ，Python就执行紧跟在if 语句后面的代码；如果为False ，Python就忽略这些代码

cars = ['audi', 'bmw', 'subaru', 'toyota']

for car in cars:
    if car == 'bmw':
        print(car.upper())
    else:
        print(car.title())
