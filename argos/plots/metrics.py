import pandas as pd
import igraph as ig
import numpy as np
import matplotlib.pyplot as plt

def norm(vector):
    return (sum(x ** 2 for x in vector)) ** 0.5

def distance_between(pos1, pos2):
    return norm(pos1 - pos2)

totalResults = []
for seed in range(30):
    # if seed == 17:
    #     continue
    print("Dataframe " + str(seed))
    data = pd.read_csv ("../results/social_behavior/social_behavior_2022-10-25-11:06_3600_seconds/seed#" + str(seed+1) + "_kiloLOG.tsv", sep = '\t')
    columnsToDrop = []
    for i in range(25):
        columnsToDrop.append(4+i*4+i)
        columnsToDrop.append(4+i*4+i+1)
    data.drop(data.columns[columnsToDrop], axis=1, inplace=True)
    data.drop(data.columns[-1], axis=1, inplace=True)
    print(data)

    pop_size = 25
    cluster_metric_array = []
    for time in range(360):
        neighbors_table = [[] for i in range(pop_size)]
        for id1 in range(pop_size):
            for id2 in range(id1 + 1, pop_size):
                if distance_between(np.array([data.iloc[time*10,2+id1*2+id1], data.iloc[time*10,2+id1*2+id1+1]]).astype('float64'), np.array([data.iloc[time*10,2+id2*2+id2], data.iloc[time*10,2+id2*2+id2+1]]).astype('float64')) <= 0.1:
                    neighbors_table[id1].append(id2)
                    neighbors_table[id2].append(id1)

        # print(neighbors_table)
        edges = []
        for i in range(len(neighbors_table)):
            for j in range(len(neighbors_table[i])):
                edges.append((i, int(neighbors_table[i][j])))

        graph = ig.Graph(edges=edges)
        clusters = graph.clusters()
        max_val = 0
        cluster_count = len(clusters)
        for cluster in clusters:
            if(len(cluster) > max_val):
                max_val = len(cluster)
            if(len(cluster)<=1):
                cluster_count -= 1

        # print("Largest cluster size = %d" % max)
        cluster_metric = max_val / pop_size
        # print("Cluster metric = %f" % cluster_metric)
        cluster_metric_array.append(cluster_metric)

    df = pd.DataFrame(cluster_metric_array, columns=['cluster_metric'])
    totalResults.append(df)

df_concat = pd.concat(totalResults)
by_row_index = df_concat.groupby(df_concat.index)
df_medians = by_row_index.median()
df_quantile25 = by_row_index.quantile(q = 0.25)
df_quantile75 = by_row_index.quantile(q = 0.75)
print(df_medians.head())
print(df_quantile25.head())
print(df_quantile75.head())

ax = plt.gca()
df_medians.plot(kind='line', y='cluster_metric', color = 'blue', ax=ax, ylim=(0,1))
df_quantile25.plot(kind='line', y='cluster_metric', color = 'blue', ax=ax, ylim=(0,1), legend=False)
df_quantile75.plot(kind='line', y='cluster_metric', color = 'blue', ax=ax, ylim=(0,1), legend=False)
plt.fill_between(df_medians.index, df_quantile25.cluster_metric, df_quantile75.cluster_metric)
plt.show()

ax = plt.gca()
for dataframe in totalResults:
    dataframe.plot(kind='line', y='cluster_metric', ax=ax, ylim=(0,1))
plt.show()
