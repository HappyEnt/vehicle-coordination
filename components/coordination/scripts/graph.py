import matplotlib.pyplot as plt
import os
import json

directories = ["01", "02", "04", "08"];

data = []

for directory in directories:
    path = f"../evaluation/{directory}"
    files = os.listdir(path)

    file_contents = []
    for file in files:
        file_contents.extend(map(lambda elem: elem, filter(lambda elem: elem is not None, json.loads(open(f"{path}/{file}").read()))))
    data.append(file_contents)

# plt.xkcd()

plt.xlabel("Relative Maximum Speed")
plt.ylabel("Direction Offset [radians]")
plt.boxplot(data, showfliers = False)
plt.xticks([1, 2, 3, 4], ["0.1", "0.2", "0.4", "0.8"])
# plt.show()
plt.savefig("evaluation_relative.pdf", format="pdf")
