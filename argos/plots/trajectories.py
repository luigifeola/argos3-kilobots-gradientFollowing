import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob

totalResults = []
experimentTime = 3600
pop_size = 25

for path in glob.iglob("../results/heterogeneous/heterogeneous_2022-11-21-11:35_3600_seconds/" + "*.tsv"):
    print("Dataframe " + str(path))
    data = pd.read_csv (path, sep = '\t')
    columnsToDrop = []
    for i in range(25):
        columnsToDrop.append(4+i*4+i)
        columnsToDrop.append(4+i*4+i+1)
    data.drop(data.columns[columnsToDrop], axis=1, inplace=True)
    data.drop(data.columns[-1], axis=1, inplace=True)
    print(data)

    x = []
    y = []
    robotsOnBlack = [0] * experimentTime
    for robot in range(pop_size):
        x.append(data.iloc[experimentTime - 1,2+robot*2+robot])
        y.append(data.iloc[experimentTime - 1,2+robot*2+robot+1])
        for timeStep in range(experimentTime):
            if(data.iloc[timeStep,2+robot*2+robot] <= 0.5/3 and data.iloc[timeStep,2+robot*2+robot] >= -0.5/3 and data.iloc[timeStep,2+robot*2+robot+1] <= 0.5/3 and data.iloc[timeStep,2+robot*2+robot+1] >= -0.5/3):
                robotsOnBlack[timeStep] += 1


    plt.rcParams["figure.figsize"] = (40,20)
    plt.subplot(1, 2, 1)
    circle1 = plt.Circle((0, 0), 0.5/3, color='black', fill=False)
    plt.gca().add_patch(circle1)
    circle1 = plt.Circle((0, 0), 0.5*2/3, color='grey', fill=False)
    plt.gca().add_patch(circle1)
    plt.gca().set_title('End positions of the robots')
    plt.scatter(x,y)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)

    plt.subplot(1, 2, 2)
    xPlot = np.linspace(0,experimentTime - 1,num = experimentTime)
    plt.gca().set_title('Number of robots on the black spot over time')
    plt.plot(xPlot, robotsOnBlack)
    plt.ylim(0, pop_size)

    plt.show()
