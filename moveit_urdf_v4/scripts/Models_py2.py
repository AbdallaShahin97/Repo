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

print("Uploading models: Done")
