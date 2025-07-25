favorite_languages = {                                        # 确定需要使用多行来定义字典时，要在输入左花括号后按回车键。在下一行缩进四个空格，指定第一个键值对，并在后面加上一个空格
    'jen': 'python',
    'sarah': 'c',
    'edward': 'ruby',
    'phil': 'python',
    }                                                         # 定义好字典后，在最后一个键值对的下一行添加一个右花括号，并缩进四个空格

friends = ['phil', 'sarah']
for name in favorite_languages.keys():
    print(name.title())
    
    if name in friends:
        language = favorite_languages[name].title()
        print(f"\t{name.title()}, I see you love {language}!")



for name, language in favorite_languages.items():             # 遍历字典
    print(f"{name.title()}'s favorite language is {language.title()}.")


 

