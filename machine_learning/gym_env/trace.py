import matplotlib,sys
matplotlib.rcParams["backend"] = "TkAgg"
import csv
from matplotlib import pyplot as plt
import seaborn as sns

f = open("reward_history.csv")
reader = csv.reader(f, delimiter=",")

r = [float(row[0]) for row in reader]

print("Avg: %s, episodes: %s" %(sum(r)/len(r), len(r)))

plt.plot(range(len(r)), r)
plt.show()
