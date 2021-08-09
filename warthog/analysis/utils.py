import pandas as pd
from os import listdir
from os.path import join

domains = [
    #  "dao",
    #  "bgmaps",
    #  "starcraft",
    #  "mazes",
    #  "random10",
    #  "random20",
    #  "random30",
    #  "random40",
    #  "rooms",
    #  "street",
    "iron",
]

algos = ["jps", "jps2", "jps-prune", "jps2-prune"]

def load_all():
    data = {}
    for algo in algos:
        dfs = []
        for domain in domains:
            dir = "../output/{0}/{1}".format(algo, domain)
            files = [join(dir, f) for f in listdir(dir)]
            for file in files:
                df = pd.read_csv(file, sep='\t')
                df['domain'] = domain
                dfs.append(df)
        data[algo] = pd.concat(dfs)
    return data
