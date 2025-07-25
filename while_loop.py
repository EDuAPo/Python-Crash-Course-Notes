# 用户输入和while循环
prompt = "Tell me something, and I will repeat it back to you: "
message = ""
while message != 'quit':
    message = input(prompt)
    if message != 'quit':
        print(message)