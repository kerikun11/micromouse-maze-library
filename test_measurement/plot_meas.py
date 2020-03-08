import numpy as np
import matplotlib.pyplot as plt
# enable japanese
plt.rcParams["font.family"] = "IPAexGothic"

# load measurement.csv
raw = np.loadtxt('./build/measurement.csv', delimiter=',',
                 dtype='unicode', encoding='utf-8')
raw = np.flipud(raw)

# show step
# fig, ax = plt.subplots(figsize=(6, 6))
fig, ax = plt.subplots()
year = [l[4:8] for l in raw[:, 0]]
step = raw[:, [3, 4, 5, 6, 7]].astype(np.int)
bottom = 0 * step[:, 1]
p_s_f = plt.bar(year, step[:, 1], bottom=bottom, label='Forward')
bottom += step[:, 1]
p_s_l = plt.bar(year, step[:, 2], bottom=bottom, label='Left')
bottom += step[:, 2]
p_s_r = plt.bar(year, step[:, 3], bottom=bottom, label='Right')
bottom += step[:, 3]
p_s_b = plt.bar(year, step[:, 4], bottom=bottom, label='Back')
plt.title("最短経路探索に要した総歩数の比較")
# plt.title("全探索に要した総歩数の比較")
plt.xlabel("全日本マイクロマウス大会 32x32迷路 [年]")
plt.ylabel("歩数 [歩]")

labels = step[:, 0]
for i, v in enumerate(labels):
    ax.text(i, labels[i], labels[i], ha='center', va='bottom')

b, t = plt.ylim()
plt.ylim(0, t * 1.08)
plt.legend(loc='best')

plt.tight_layout()

filename = 'build/step'
for ext in ['.png', '.pdf', '.svg']:
    plt.savefig(filename + ext)

# show estimated time
fig, ax = plt.subplots()
time_s = raw[:, 2].astype(np.float).astype(np.int)
time_str = raw[:, 1]
p_t = plt.bar(year, time_s)
labels = time_s
for i, v in enumerate(labels):
    ax.text(i, labels[i], time_str[i], ha='center', va='bottom')
plt.title("最短経路探索に要した時間の目安")
plt.xlabel("全日本マイクロマウス大会 32x32迷路 [年]")
plt.ylabel("見積探索時間 [秒]")

plt.tight_layout()

filename = 'build/time'
for ext in ['.png', '.pdf', '.svg']:
    plt.savefig(filename + ext)

plt.show()
