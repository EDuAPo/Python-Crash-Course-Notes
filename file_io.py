# 读取文件并处理异常
try:
    with open('pi_digits.txt', 'r') as file:
        contents = file.read()
    print(contents)
except FileNotFoundError:
    print("Sorry, the file was not found.")