#!/usr/bin/env python3
"""
AS7263 Dataset Analyzer
- Reads CSV from data collection
- Computes channel index (average of 6 channels)
- Maps index -> moisture % via linear regression
- Visualizes distributions and outputs predicted moisture and suggested thresholds
"""

import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import numpy as np

# ------------------ USER SETTINGS ------------------
CSV_FILE = "dataset_gathering/data/as7263_dataset.csv"
# Optional: assign representative moisture values per class
CLASS_MOISTURE = {
    "overcooked": 5.0,  # <=5.9%, can refine later
    "standard": 6.5,    # 6-7%
    "raw": 7.5          # >=7.1%
}
# ---------------------------------------------------

# 1️⃣ Load CSV
df = pd.read_csv(CSV_FILE)

# 2️⃣ Compute average index of 6 channels
channel_cols = ["ch1", "ch2", "ch3", "ch4", "ch5", "ch6"]
df['index'] = df[channel_cols].mean(axis=1)

# 3️⃣ Assign numeric moisture target based on class
df['moisture_gt'] = df['label'].map(CLASS_MOISTURE)

# 4️⃣ Fit linear regression: index -> moisture
X = df[['index']].values
y = df['moisture_gt'].values
reg = LinearRegression().fit(X, y)
df['moisture_pred'] = reg.predict(X)

print(f"Linear regression: moisture% = {reg.coef_[0]:.4f}*index + {reg.intercept_:.4f}")

# 5️⃣ Compute simple thresholds
# Get predicted moisture per class
pred_by_class = df.groupby('label')['moisture_pred']
thresholds = {
    "overcooked_standard": (pred_by_class.get_group('overcooked').max() + 
                            pred_by_class.get_group('standard').min()) / 2,
    "standard_raw": (pred_by_class.get_group('standard').max() + 
                    pred_by_class.get_group('raw').min()) / 2
}
print("Suggested thresholds (predicted moisture %):", thresholds)

# 6️⃣ Optional: plot distributions
for lbl in df['label'].unique():
    plt.hist(df[df['label']==lbl]['moisture_pred'], bins=10, alpha=0.5, label=lbl)
plt.xlabel("Predicted Moisture %")
plt.ylabel("Frequency")
plt.title("AS7263 Predicted Moisture Distribution")
plt.legend()
plt.show()

# 7️⃣ Add classification column based on thresholds
def classify(m):
    if m <= thresholds['overcooked_standard']:
        return "overcooked"
    elif m <= thresholds['standard_raw']:
        return "standard"
    else:
        return "raw"

df['pred_class'] = df['moisture_pred'].apply(classify)

# 8️⃣ Show a quick confusion check
confusion = pd.crosstab(df['label'], df['pred_class'])
print("\nConfusion table (rows=actual, columns=predicted):")
print(confusion)

# 9️⃣ Save analyzed CSV
df.to_csv("dataset_gathering/data/as7263_dataset_analyzed.csv", index=False)
print("\nSaved analyzed CSV: data/as7263_dataset_analyzed.csv")