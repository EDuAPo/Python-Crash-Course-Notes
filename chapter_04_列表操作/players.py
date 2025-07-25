# 切片

players = ['charles', 'martina', 'michael', 'florence', 'eli'] 

print("Here are the first three players on my team:")
for player in players[:3]:
    print(player.title())



players = ['charles', 'martina', 'michael', 'florence', 'eli']
print(players[0:3])



# 复制列表        方法是同时删除起始索引和终止索引（[:] ）
my_foods = ['pizza', 'falafel', 'carrot cake']
friend_foods = my_foods[:]

print("My favorite foods are:")
print(my_foods)

print("\nMy friend's favorite foods are:")
print(friend_foods)