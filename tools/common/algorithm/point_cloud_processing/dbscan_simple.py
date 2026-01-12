import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.datasets import make_moons

if __name__ == '__main__':
    # 1. 创建数据集
    X, y_true = make_moons(n_samples=300, noise=0.05, random_state=42)

    # 2. 可视化数据
    plt.scatter(X[:, 0], X[:, 1], s=50)
    plt.title("Input Data")
    plt.show()

    # 3. 应用 DBSCAN 算法
    eps = 0.2  # 邻域半径
    min_samples = 5  # 最小样本数
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    y_dbscan = dbscan.fit_predict(X)

    # 4. 可视化聚类结果
    plt.scatter(X[:, 0], X[:, 1], c=y_dbscan, cmap='plasma', s=50)
    plt.title(f"DBSCAN Clustering (eps={eps}, min_samples={min_samples})")
    plt.show()

    # 5. 打印聚类标签
    print("Cluster labels:", y_dbscan)

