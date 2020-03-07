import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "IPAexGothic"

raw = np.loadtxt('./build/measurement.csv', delimiter=',',
                 dtype='unicode', encoding='utf-8')
raw = np.flipud(raw)

# show step
fig, ax = plt.subplots()
year = raw[:, 0]
year = [l[4:8] for l in year]
step = raw[:, [3, 4, 5, 6, 7]].astype(np.int)
p_s_f = plt.bar(year, step[:, 1])
p_s_l = plt.bar(year, step[:, 2], bottom=step[:, 1])
p_s_r = plt.bar(year, step[:, 3], bottom=step[:, 1]+step[:, 2])
p_s_b = plt.bar(year, step[:, 4], bottom=step[:, 1]+step[:, 2]+step[:, 3])
plt.legend(reversed([p_s_f[0], p_s_l[0], p_s_r[0], p_s_b[0]]),
           reversed(["Forward", "Left", "Right", "Back"]))
plt.title("迷路探索に要した歩数")
plt.xlabel("全日本マイクロマウス大会マイクロマウス競技決勝 [年]")
plt.ylabel("歩数 [歩]")

labels = step[:, 0]
for i, v in enumerate(labels):
    ax.text(i, labels[i], labels[i], ha='center', va='bottom')

plt.tight_layout()

plt.savefig('build/step.png')
plt.savefig('build/step.pdf')
plt.show()
