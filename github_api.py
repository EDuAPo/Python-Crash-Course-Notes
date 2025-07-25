# 读取CSV并绘图
import csv
import matplotlib.pyplot as plt

dates, highs = [], []
with open('sitka_weather_07-2018_simple.csv') as f:
    reader = csv.reader(f)
    next(reader)  # 跳过标题行
    for row in reader:
        dates.append(row[2])
        highs.append(int(row[5]))

plt.plot(dates, highs, c='red')
plt.title("Daily High Temperatures, July 2018")
plt.xticks(rotation=45)
plt.show()