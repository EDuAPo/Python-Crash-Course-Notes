prompt = "\nPlease enter the name of a city you have visited:"
prompt += "\n(Enter 'quit' when you are finished.) "                      # 运算符+= 在前面赋给变量prompt 的字符串末尾附加一个字符串

while True:
    city = input(prompt)
    
    if city == 'quit':
        break
    else:
        print(f"I'd love to go to {city.title()}!")
