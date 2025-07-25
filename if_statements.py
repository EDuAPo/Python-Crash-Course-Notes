# # if判断
# age = 10
# if age < 18:
#     print("未成年")
# if age >= 18:
#     print("成年人")




# # if-else  #二选一
# age =10
# if age < 18:
#     print("未成年")
# else:                       # else后面不需要添加任何条件
#     print("成年人")




# # if-elif-else 多选一
# age = 19
# if age < 4:
#     price = 0
# elif age < 18:
#     price = 25
# else:                       # else后面不需要添加任何条件
#     price = 40
# print(f"Your admission cost is ${price}.")


# score =input("请输入你的成绩:\n")
# if '85' <= score <= '100':
#     print("优秀")
# elif '60' <= score <= '85':
#     print("及格")
# elif '0' <= score < '60':
#     print("不及格")
# else:                       # else可以表示所有条件都不符合时的一个情况
#     print("分数无效")

 


# # if嵌套
# ticket =True
# temp = input("请输入你的体温：\n")
# if ticket == True:
#     print("有票可以进站",end="")
#     if '36.3'< temp < '37.2':
#         print("体温正常，安心回家")
#     else:
#         print("但体温异常，需要被抓去隔离")
# else:
#     print("没票不能进站，哭唧唧")








# # 三目运算
# # 基本格式：   为真结果 if判断条件 else为假结果
# a = 5
# b = 8
# if a<b:
#     print("a比b小")     #为真结果
# if a>b:
#     print("a比b大")     #为假结果
# print("a比b小")if a<=b else print("a比b小")
# print("测试结果:", 'PASS' if result else 'FAIL')





