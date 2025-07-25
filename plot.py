# 绘制折线图
import matplotlib.pyplot as plt

squares = [1, 4, 9, 16, 25]
plt.plot(range(1, 6), squares, linewidth=2)
plt.title("Square Numbers", fontsize=24)
plt.xlabel("Value", fontsize=14)
plt.ylabel("Square", fontsize=14)
plt.show()