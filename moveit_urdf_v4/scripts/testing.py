# load and evaluate a saved model
#from google.colab import drive
#drive.mount('/content/drive')
import pandas as pd
import tensorflow as tf
from array import array
from tensorflow import keras
from tensorflow.keras import backend as K
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Dense
from sklearn import preprocessing
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import MinMaxScaler
from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers import *
from tensorflow.python.keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import MinMaxScaler
from scipy import io
from keras.models import load_model
from keras import backend as K
import pandas as pd
import tensorflow as tf
from array import array
from tensorflow import keras
from tensorflow.keras import backend as K
from tensorflow.keras.models import Sequential
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Dense
from sklearn import preprocessing
import matplotlib.pyplot as plt
import numpy as np
from keras.layers.normalization import BatchNormalization
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import MinMaxScaler
from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers import *
from tensorflow.python.keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import MinMaxScaler
from datetime import datetime
from pandas import DataFrame
from scipy.io import loadmat
from keras.utils.generic_utils import get_custom_objects

def custom_activation(x):
    return (K.tanh(x) * 2.718**(0.2*x))    
custom_obj = {}
custom_obj['custom_activation'] = custom_activation


####### Forward kinematics models #########
model11_forward = tf.keras.models.load_model("Forward_whole_pose_python2_11.h5",custom_objects = custom_obj)
model12_forward = tf.keras.models.load_model("Forward_whole_pose_python2_12.h5",custom_objects = custom_obj)
model13_forward = tf.keras.models.load_model("Forward_whole_pose_python2_13.h5",custom_objects = custom_obj)
model14_forward = tf.keras.models.load_model("Forward_whole_pose_python2_14.h5",custom_objects = custom_obj)

model21_forward = tf.keras.models.load_model("Forward_whole_pose_python2_21.h5",custom_objects = custom_obj)
model22_forward = tf.keras.models.load_model("Forward_whole_pose_python2_22.h5",custom_objects = custom_obj)
model23_forward = tf.keras.models.load_model("Forward_whole_pose_python2_23.h5",custom_objects = custom_obj)
model24_forward = tf.keras.models.load_model("Forward_whole_pose_python2_24.h5",custom_objects = custom_obj)

model31_forward = tf.keras.models.load_model("Forward_whole_pose_python2_31.h5",custom_objects = custom_obj)
model32_forward = tf.keras.models.load_model("Forward_whole_pose_python2_32.h5",custom_objects = custom_obj)
model33_forward = tf.keras.models.load_model("Forward_whole_pose_python2_33.h5",custom_objects = custom_obj)
model34_forward = tf.keras.models.load_model("Forward_whole_pose_python2_34.h5",custom_objects = custom_obj)

model41_forward = tf.keras.models.load_model("Forward_whole_pose_python2_41.h5",custom_objects = custom_obj)
model42_forward = tf.keras.models.load_model("Forward_whole_pose_python2_42.h5",custom_objects = custom_obj)
model43_forward = tf.keras.models.load_model("Forward_whole_pose_python2_43.h5",custom_objects = custom_obj)
model44_forward = tf.keras.models.load_model("Forward_whole_pose_python2_44.h5",custom_objects = custom_obj)

####### Inverse kinematics models #########
model11_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_11.h5",custom_objects = custom_obj)
model12_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_12.h5",custom_objects = custom_obj)
model13_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_13.h5",custom_objects = custom_obj)
model14_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_14.h5",custom_objects = custom_obj)

model21_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_21.h5",custom_objects = custom_obj)
model22_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_22.h5",custom_objects = custom_obj)
model23_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_23.h5",custom_objects = custom_obj)
model24_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_24.h5",custom_objects = custom_obj)

model31_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_31.h5",custom_objects = custom_obj)
model32_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_32.h5",custom_objects = custom_obj)
model33_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_33.h5",custom_objects = custom_obj)
model34_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_34.h5",custom_objects = custom_obj)

model41_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_41.h5",custom_objects = custom_obj)
model42_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_42.h5",custom_objects = custom_obj)
model43_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_43.h5",custom_objects = custom_obj)
model44_inverse = tf.keras.models.load_model("Inverse_whole_pose_python2_44.h5",custom_objects = custom_obj)

## ***************************************************3##
results_value=[]
forward = {
    11: model11_forward,12: model12_forward, 13: model13_forward,14: model14_forward,
    21: model21_forward,22: model22_forward,23: model23_forward,24: model24_forward,
    31: model31_forward,32: model32_forward,33: model33_forward,34: model34_forward,
    41: model41_forward,42: model42_forward,43: model43_forward,44: model44_forward,}

inverse = {
    11: model11_inverse,12: model12_inverse, 13: model13_inverse,14: model14_inverse,
    21: model21_inverse,22: model22_inverse,23: model23_inverse,24: model24_inverse,
    31: model31_inverse,32: model32_inverse,33: model33_inverse,34: model34_inverse,
    41: model41_inverse,42: model42_inverse,43: model43_inverse,44: model44_inverse,}

for n1 in range(1,5):
  for n2 in range(1,5):
    num1 = str(n1) 
    num2 = str(n2) 
    num3 = num1+num2 
    n12=int(num3)
    print(n12)
    name = "Data_{}{}.mat".format(n1, n2)
    df2 = loadmat(name)
    dataset=df2['data']
    X = dataset[:,7:13]
    Y = dataset[:,0:6]
    from sklearn.model_selection import train_test_split
    X_train, X_val_and_test, Y_train, Y_val_and_test = train_test_split(X, Y, test_size=0.01)
    X_val, X_test, Y_val, Y_test = train_test_split(X_val_and_test, Y_val_and_test, test_size=0.5)
    results = forward[n12].evaluate(X_test, Y_test, batch_size=128)
    results_value.append(results)
    print("test loss, test acc:", results)
print("############")
for n1 in range(1,5):
  for n2 in range(1,5):
    num1 = str(n1) 
    num2 = str(n2) 
    num3 = num1+num2 
    n12=int(num3)
    print(n12)
    name = "Data_{}{}.mat".format(n1, n2)
    df2 = loadmat(name)
    dataset=df2['data']
    Y = dataset[:,7:13]
    X = dataset[:,0:6]
    from sklearn.model_selection import train_test_split
    X_train, X_val_and_test, Y_train, Y_val_and_test = train_test_split(X, Y, test_size=0.01)
    X_val, X_test, Y_val, Y_test = train_test_split(X_val_and_test, Y_val_and_test, test_size=0.5)
    results = inverse[n12].evaluate(X_test, Y_test, batch_size=128)
    results_value.append(results)
    print("test loss, test acc:", results)
for i in results_value:
	print(i)