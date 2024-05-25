import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load an image in grayscale
img = cv2.imread('~/Duckietown-Project/packages/lane_processing/src/lane.png', cv2.IMREAD_GRAYSCALE)

# Experiment with different thresholds
thresholds = [(50, 150), (100, 200), (150, 250)]
results = []

for t1, t2 in thresholds:
    edges = cv2.Canny(img, threshold1=t1, threshold2=t2)
    results.append(edges)

# Plot the results
plt.figure(figsize=(12, 8))
for i in range(len(results)):
    plt.subplot(1, 3, i + 1)
    plt.imshow(results[i], cmap='gray')
    plt.title(f'Thresholds: {thresholds[i][0]}, {thresholds[i][1]}')
    plt.axis('off')
plt.show()
