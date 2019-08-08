import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Activation, LSTM, Dropout
from keras.layers import TimeDistributed, BatchNormalization
from keras.optimizers import Adam
from sklearn.metrics import f1_score, confusion_matrix, roc_auc_score, precision_score
from sklearn.metrics import recall_score, accuracy_score
from sklearn.preprocessing import normalize
import matplotlib.pyplot as plt
