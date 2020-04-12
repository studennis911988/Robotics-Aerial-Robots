import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from sklearn import linear_model
from sklearn.model_selection import train_test_split

# read from file
file = '/home/dennis/ROS/aerial_robots_ws/src/Homework5/HW5-1.xls'
data = pd.read_excel(file)
# classified data
data.columns = ['input1', 'input2', 'output']
print(data.describe())

# X : input; Y: output
X = data.iloc[:, data.columns != 'output']
Y = data.iloc[:, 2]

X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state= 0)

# train
model = linear_model.LinearRegression()
model.fit(X_train, Y_train)

# get coefficient
coeff_df = pd.DataFrame(model.coef_, X.columns, columns=['Coefficient'])
print(coeff_df)

# # test model with testing data
# y_pred = model.predict(X_test)
# df = pd.DataFrame({'Actual': Y_test, 'Predicted': y_pred})
# print(df.head(10))