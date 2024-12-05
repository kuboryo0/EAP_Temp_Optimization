from itertools import product
n = 4
binary__list = list(product([0, 1], repeat=n)) 
all_patterns = list([binary__list[i],True] for i in range(len(binary__list))) #stepごとのパターン

print("all_patterns",all_patterns)